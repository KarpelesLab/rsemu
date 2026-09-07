//! The long run: the interpreter and a translated engine, side by side, for as
//! much guest time as the caller will pay for.
//!
//! `tests/a64_engines.rs`, `tests/riscv_virt_engines.rs` and
//! `tests/x86_engines.rs` each run forty quanta of a six-instruction loop and
//! compare state hashes at ten checkpoints. That is the right shape for a test
//! on every commit and it is not a gate: two defects in the A64 translating
//! engine survived it, and survived a *twenty-second* Linux boot as well. One
//! first parted from the interpreter at **15.04 s** of guest time and the other
//! at **23.46 s**, where it desynchronised the guest permanently
//! (`docs/platforms/arm64-virt.md`). Both were found by hand.
//!
//! [`longrun`] is the automation, and this file is what it is pointed at.
//!
//! # What runs, and when
//!
//! | test | fixture | cost | when |
//! | --- | --- | --- | --- |
//! | [`the_harness_names_the_quantum_a_planted_divergence_appears_on`] | none | milliseconds | every `cargo test` |
//! | [`a_synthetic_a64_workload_agrees_across_the_engines`] | none | a few seconds | every `cargo test` |
//! | [`a_synthetic_riscv_workload_agrees_across_the_engines`] | none | a few seconds | every `cargo test` |
//! | [`a_tlbi_in_the_loop_agrees_across_the_engines`] | none | under a second | every `cargo test` |
//! | `a_real_arm64_linux_boot_agrees_across_the_engines` | a kernel | minutes | `--ignored`, nightly |
//!
//! `RSEMU_LONGRUN_SECONDS` lengthens the synthetic runs; the default is sized
//! so an ordinary `cargo test` does not notice them. `RSEMU_LONGRUN_ENGINES`
//! chooses which engines are compared against the oracle — `interp` is the
//! control leg, an interpreter against itself, which must always agree and is
//! the first thing to run when a failure here looks like a harness bug.
//!
//! # The kernel, and why it cannot be here
//!
//! A Debian arm64 kernel is a GPL-2.0 binary. Running one as an emulated guest
//! is ordinary use; committing one to this repository is redistribution under
//! its terms (`CLAUDE.md`, *Testing*). So it is fetched, like every other
//! fixture:
//!
//! ```text
//!   scripts/fetch-testdata.sh arm64-linux arm64-initramfs
//!
//!   RSEMU_ARM64_KERNEL=testdata/arm64/linux \
//!   RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \
//!   RSEMU_LONGRUN_SECONDS=120 \
//!       cargo test --release --features machine-arm64-virt,cpu-arm-a64-lift,jit,jit-x86 \
//!           --test engine_longrun -- --ignored --nocapture
//! ```
//!
//! With no kernel that test **skips loudly** and says the two lines above.
//! `.github/workflows/long-run.yml` fetches the kernel on a nightly schedule
//! and runs it, so the gate has somewhere it actually runs rather than being an
//! opt-in nobody opts into.
//!
//! # Can the synthetic workload replace the kernel?
//!
//! No, and the honest answer is worth writing down. [`A64_MAIN`] is *designed*
//! around the two mechanisms that broke — it leaves its page for an instruction
//! outside the lifted subset, it thrashes the software TLB, and it takes a
//! generic-timer interrupt in the middle of a long chain of lifted code — so it
//! reaches both in well under a second of guest time where the kernel needed
//! fifteen and twenty-three. That makes it a fast **regression** test and a
//! poor **discovery** one: it can only exercise the mechanisms its author
//! already thought of, and the two defects were found precisely because a real
//! kernel does things nobody designed for. A synthetic guest that reached
//! fifteen seconds of genuinely varied behaviour would be a kernel.
//!
//! So both exist, and they are different claims: the synthetic runs everywhere
//! and holds the ground already taken, and the kernel run is the gate that can
//! still find something new.

#![cfg(all(feature = "jit", feature = "std"))]

mod longrun;

#[cfg(feature = "cpu-arm-a64-lift")]
mod a64 {
    use std::sync::Arc;

    use rsemu::core::Captured;
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;
    use rsemu::cpu::arm::a64::Cpu;
    use rsemu::cpu::arm::a64::mmu::desc;
    use rsemu::cpu::arm::a64::sysreg::{cntctl, sctlr};
    use rsemu::machine::{Machine, build};

    /// A board with RAM, a clock and one core, and nothing else.
    ///
    /// Deliberately not `arm64-virt`: this one has no console, no interrupt
    /// controller and no virtio, so the core's generic timer drives the core's
    /// own `IRQ` directly (as `machines/a64-mini.machine` explains) and 8 MiB
    /// of RAM keeps [`Machine::state_hash`] cheap enough to take often.
    ///
    /// The clock numbers are `arm64-virt`'s, because the guest's timer period
    /// below is expressed in system-counter ticks and the two should mean the
    /// same thing on both boards.
    pub(crate) const SOURCE: &str = r#"
machine "a64-longrun" {
  param engine = "interp"
  param ram = 8M
  osc cpu = 1000000000 Hz
  space mem { width = 64 }
  object cpu0 "cpu.arm.a64" {
    clock  = cpu
    space  = mem
    engine = engine
    cpu    = "cortex-a53"
    reset  = 0x00000000
    cntfrq = 62500000
    cntdiv = 16
  }
  object dram "ram" { size = ram }
  map mem 0x00000000 size ram = dram
}
"#;

    /// Level 1 of the translation table.
    const L1: u64 = 0x8_0000;
    /// Level 2, reached from [`L1`] entry 0.
    const L2: u64 = 0x8_1000;
    /// `VBAR_EL1`. Must be 2 KiB aligned; the IRQ vector is at `+0x280`.
    const VBAR: u64 = 0x800;
    /// The offset of the "current EL with `SP_ELx`, IRQ" vector — **not**
    /// `0x200`, which is the synchronous one for `SP_EL0`.
    const IRQ_VECTOR: u64 = 0x280;
    /// Where [`A64_TWO`] sits: a different 4 KiB page from the main loop, so
    /// branching to it is a TLB miss whatever the mapping's granule.
    const PAGE_TWO: u64 = 0x1000;
    /// System-counter ticks between generic-timer interrupts. It must match
    /// the `MOVZ` in [`A64_IRQ`], which re-arms `CNTP_TVAL_EL0` with it.
    const TIMER_PERIOD: u64 = 5_000;

    /// The main loop, at the reset vector.
    ///
    /// Assembled from source with `llvm-mc -triple=aarch64`; the listing is in
    /// the comments. Written for two properties and nothing else:
    ///
    /// * a **store and a load to a different 4 KiB page every iteration**, so
    ///   the software TLB's load and store sets are under real pressure and a
    ///   compiled access has to miss its inlined probe and walk. (Its *fetch*
    ///   set is a third, separate array and nothing here evicts from it — only
    ///   [`Tlbi::Every256`] makes a code page cold.)
    /// * a **run of sixteen instructions that touch no memory**, so a warm
    ///   quantum here is one long chain with no store to end it — which is
    ///   exactly the window a generic timer fires in when a real guest is doing
    ///   work rather than parked in `WFI`.
    const A64_MAIN: [u32; 26] = [
        0xd2a0_002b, // movz x11, #1, lsl #16   ; the data window at 0x10000
        // loop:  (0x0004)
        0x9100_0400, // add  x0, x0, #1
        0x9240_1409, // and  x9, x0, #0x3f
        0xd374_cd29, // lsl  x9, x9, #12
        0x8b0b_0129, // add  x9, x9, x11        ; a different page every time
        0xf900_0120, // str  x0, [x9]
        0xf940_012a, // ldr  x10, [x9]
        0xca0a_0042, // eor  x2, x2, x10
        0x8b00_0042, // add  x2, x2, x0
        0x8b02_0084, // add  x4, x4, x2         ; sixteen instructions of pure
        0xca04_00a5, // eor  x5, x5, x4         ; ALU: the chain the timer has
        0xcb05_00c6, // sub  x6, x6, x5         ; to be able to fire inside
        0x8b06_00e7, // add  x7, x7, x6
        0xca07_0084, // eor  x4, x4, x7
        0x8b04_00a5, // add  x5, x5, x4
        0xcb05_00c6, // sub  x6, x6, x5
        0x8b06_00e7, // add  x7, x7, x6
        0xca07_0084, // eor  x4, x4, x7
        0x8b04_00a5, // add  x5, x5, x4
        0xcb05_00c6, // sub  x6, x6, x5
        0x8b06_00e7, // add  x7, x7, x6
        0xca07_0084, // eor  x4, x4, x7
        0x8b04_00a5, // add  x5, x5, x4
        0xcb05_00c6, // sub  x6, x6, x5
        0x8b06_00e7, // add  x7, x7, x6
        0x1400_03e7, // b    0x1000             ; onto the next page
    ];

    /// What sits on the second page: an `MRS` the frontend does not lift, and
    /// — in the [`Tlbi::Every256`] variant — a periodic `TLBI` so that the
    /// branch back is a cold walk.
    ///
    /// With the `TLBI` this is the shape defect 1 lived in: a chained boundary
    /// the frontend declines, on a page the TLB has just lost. **The `TLBI` is
    /// the only way to produce that shape here**, because `mmu::Tlb` keeps
    /// fetch, load and store translations in three separate 256-entry sets — so
    /// no amount of data-side pressure can evict the code page's *fetch* entry,
    /// and a guest that executed from 257 pages would be a different fixture.
    const A64_TWO: [u32; 7] = [
        0xd538_000c, // mrs  x12, midr_el1      ; outside the lifted subset
        0x8b0c_0042, // add  x2, x2, x12
        0x3640_0080, // tbz  w0, #8, back       ; the TLBI on half the passes
        0xd508_871f, // tlbi vmalle1
        0xd503_379f, // dsb  nsh
        0xd503_3fdf, // isb
        // back:
        0x17ff_fbfb, // b    0x0004
    ];

    /// The same page with the three barrier slots filled by `NOP`s.
    ///
    /// Everything else about the workload is unchanged: the MMU is still on,
    /// the data window still walks 64 pages, the `MRS` is still outside the
    /// lifted subset, and the generic timer still fires inside long chains.
    const A64_TWO_NO_TLBI: [u32; 7] = [
        0xd538_000c, // mrs  x12, midr_el1
        0x8b0c_0042, // add  x2, x2, x12
        0x3640_0080, // tbz  w0, #8, back
        0xd503_201f, // nop
        0xd503_201f, // nop
        0xd503_201f, // nop
        // back:
        0x17ff_fbfb, // b    0x0004
    ];

    /// Whether the guest flushes its own TLB in the loop.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(crate) enum Tlbi {
        /// The three barrier slots are `NOP`s.
        Never,
        /// `TLBI VMALLE1; DSB NSH; ISB` on half the passes round the loop.
        Every256,
    }

    /// The IRQ handler, at `VBAR + 0x280`.
    ///
    /// Re-arms the physical timer through `CNTP_TVAL_EL0` (which writes
    /// `CNTP_CVAL_EL0 = CNTPCT_EL0 + x13`, DDI 0487 D11.2.4), counts the
    /// interrupt in `x3`, and returns. `MSR` is outside the lifted subset, so
    /// the handler is also a second source of declined boundaries.
    const A64_IRQ: [u32; 4] = [
        0xd282_710d, // movz x13, #5000         ; == TIMER_PERIOD
        0xd51b_e20d, // msr  cntp_tval_el0, x13
        0x9100_0463, // add  x3, x3, #1
        0xd69f_03e0, // eret
    ];

    /// Build the board on `engine`, with the guest in RAM and the MMU on.
    pub(crate) fn board(engine: &str, tag: &str, tlbi: Tlbi) -> (Machine, Arc<Cpu>) {
        let cpus: Arc<Captured<Cpu>> = Arc::new(Captured::new());
        let kept = Arc::clone(&cpus);
        let mut bindings = rsemu::machine::catalog::bindings().expect("this build's bindings");
        bindings.replace("cpu.arm.a64", move |props| {
            let cpu = Arc::new(Cpu::from_props(props)?);
            kept.push(&cpu);
            Ok(cpu)
        });
        let options = rsemu::machine::BuildOptions::new()
            .with_classes(rsemu::machine::catalog::classes())
            .with_bindings(bindings)
            .with_param("engine", engine);
        let registry = rsemu::machine::catalog::registry().expect("this build's registry");
        let machine = build(&format!("a64-longrun.{tag}"), SOURCE, &registry, &options)
            .unwrap_or_else(|e| panic!("the board does not build with engine={engine}: {e}"));
        let cpu = cpus.take().expect("the binding captured the core");

        // `build` realizes and resets, and a cold reset zeroes RAM, so
        // everything below happens afterwards.
        let space = cpu.space().expect("the core has its space");
        let put = |addr: u64, width: Width, value: u64| {
            space
                .write(addr, width, value, MemAttrs::DEFAULT)
                .expect("inside RAM");
        };
        let two: &[u32] = match tlbi {
            Tlbi::Never => &A64_TWO_NO_TLBI[..],
            Tlbi::Every256 => &A64_TWO[..],
        };
        for (at, words) in [
            (0u64, &A64_MAIN[..]),
            (PAGE_TWO, two),
            (VBAR + IRQ_VECTOR, &A64_IRQ[..]),
        ] {
            for (n, word) in words.iter().enumerate() {
                put(at + 4 * n as u64, Width::U32, u64::from(*word));
            }
        }

        // A three-level hierarchy that identity-maps the first two mebibytes as
        // one block: `L1` entry 0 is a table, `L2` entry 0 is the block. Every
        // address this guest uses — its code, its data window, and the tables
        // themselves at 0x80000 — is inside it. DDI 0487 D5.
        put(L1, Width::U64, L2 | desc::VALID | desc::TABLE);
        put(L2, Width::U64, desc::VALID | desc::AF);

        let mut sys = cpu.sysregs();
        sys.ttbr0 = L1;
        // T0SZ = T1SZ = 25 (39-bit halves), TG1 = 0b10 (the 4 KiB granule).
        sys.tcr = 25 | (25 << 16) | (0b10 << 30);
        sys.sctlr |= sctlr::M;
        sys.vbar_el1 = VBAR;
        // Nothing masked: the timer below has to be able to reach the core.
        sys.daif = 0;
        sys.cntp_cval = TIMER_PERIOD;
        sys.cntp_ctl = cntctl::ENABLE;
        cpu.set_sysregs(sys);

        (machine, cpu)
    }

    /// Assert the guest actually did what it was written to do.
    ///
    /// Without this the run could be green because the core wedged on the
    /// first instruction: a comparison of two stopped machines agrees at every
    /// checkpoint. Each of these is a property the workload exists for.
    pub(crate) fn assert_the_workload_ran(cpu: &Cpu, engine: &str) {
        // `RSEMU_LONGRUN_ENGINES=interp` is the control leg — an interpreter
        // against itself, which must always agree — and an interpreted core has
        // no translation statistics to assert. Everything below it still
        // applies to that run and is still checked.
        if let Some(stats) = cpu.jit_stats() {
            assert!(
                stats.blocks > 0,
                "engine={engine} executed no translated block, so the run \
                 compared two interpreters"
            );
            assert!(
                stats.retired > stats.interpreted,
                "engine={engine} retired {} instructions inside blocks against \
                 {} interpreted, which is not a translated run",
                stats.retired,
                stats.interpreted
            );
        }
        let (hits, misses) = cpu.tlb_stats();
        assert!(
            misses > 0 && hits > 0,
            "engine={engine}: the software TLB saw {hits} hits and {misses} \
             misses, so the workload is not exercising the walk at all — the \
             MMU is off, or the data window stopped moving"
        );
        // `x3` is the handler's interrupt counter, and it is the whole of the
        // evidence that the generic timer fired at all — defect 2's mechanism.
        assert!(
            cpu.x(3) > 0,
            "engine={engine}: the generic timer never fired, so the run says \
             nothing about where a translated block notices one"
        );
    }
}

#[cfg(feature = "cpu-arm-a64-lift")]
mod a64_tests {
    use super::a64::{self, Tlbi};
    use super::longrun::{self, Options, What};

    /// How much guest time the synthetic runs cover by default.
    ///
    /// Small, because this runs on every commit. `RSEMU_LONGRUN_SECONDS`
    /// raises it, and the nightly job does.
    const DEFAULT_SECONDS: u64 = 2;

    fn seconds() -> u64 {
        std::env::var("RSEMU_LONGRUN_SECONDS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(DEFAULT_SECONDS)
    }

    /// Which translated engines to compare against the oracle.
    fn engines() -> Vec<String> {
        std::env::var("RSEMU_LONGRUN_ENGINES")
            .unwrap_or_else(|_| "jit,jit-host".to_string())
            .split(',')
            .filter(|s| !s.is_empty())
            .map(str::to_string)
            .collect()
    }

    /// Run the synthetic workload under every engine and compare, failing on
    /// the first quantum anything parts.
    fn run(tag: &str, tlbi: Tlbi, secs: u64) {
        for engine in engines() {
            let (mut oracle, _) = a64::board("interp", &format!("{tag}.oracle.{engine}"), tlbi);
            let (mut under_test, cpu) = a64::board(&engine, &format!("{tag}.{engine}"), tlbi);
            let opts = Options::to_guest_seconds(secs).hashing_every(2_000);
            match longrun::lockstep(tag, &mut oracle, &engine, &mut under_test, &opts) {
                Ok(summary) => eprintln!("{tag} engine={engine}: {summary}"),
                Err(d) => panic!("{d}"),
            }
            a64::assert_the_workload_ran(&cpu, &engine);
        }
    }

    #[test]
    fn a_synthetic_a64_workload_agrees_across_the_engines() {
        run("a64-longrun", Tlbi::Never, seconds());
    }

    /// The same workload with `TLBI VMALLE1` in the loop — **the regression
    /// test for the third defect of the class**, and it ran `#[ignore]`d for
    /// exactly as long as that defect was open.
    ///
    /// This is what the harness found the first time it was pointed at
    /// anything. It failed at **quantum 417** (0.417 s of guest time, the 52nd
    /// timer interrupt) with the interpreter on `ELR_EL1 = 0x1014` and both
    /// translated engines on `0x4`: the translated core ran two more guest
    /// instructions before noticing a timer the interpreter took immediately,
    /// and carried one extra cycle for it.
    ///
    /// The cause was `engine::leave_at`'s absence. `engine::admit` looks for a
    /// pending interrupt and *then* charges the entry translation, so a cold
    /// walk between the two can cross the comparator; `Exec::timer_edge`
    /// reports `u64::MAX` for a comparator already crossed, so the run took
    /// that for its edge and no boundary inside it left. It needs a cold
    /// instruction-fetch translation, which on this core only a `TLBI`
    /// produces — `mmu::Tlb` keeps fetch, load and store entries in separate
    /// sets — which is why `DSB`, `ISB` and `DSB; ISB` in the same slots were
    /// all fine and why the workload with either the timer or the `TLBI` alone
    /// agreed for six thousand quanta. `jit` and `jit-host` gave the *same*
    /// wrong answer because the edge is computed in the frontend's host, above
    /// both code generators.
    ///
    /// `docs/testing/long-run.md` and `docs/platforms/arm64-virt.md` have the
    /// long form.
    #[test]
    fn a_tlbi_in_the_loop_agrees_across_the_engines() {
        run("a64-longrun-tlbi", Tlbi::Every256, seconds());
    }

    /// The instrument's own calibration: plant a divergence and check that the
    /// harness names the quantum it was planted on, and the field.
    ///
    /// `CLAUDE.md` has nothing to say about this directly, but last round's
    /// standard does: a test that cannot fail is worse than no test. The two
    /// defects this file exists for are fixed, so the only way to know the
    /// harness is sensitive rather than merely green is to make it fail on
    /// purpose. Three plants, one per tier of the comparison.
    #[test]
    fn the_harness_names_the_quantum_a_planted_divergence_appears_on() {
        // Tier 1, a watched device: a system register nothing in the guest
        // reads, changed after four quanta, must be reported on the fifth and
        // named.
        {
            let (mut oracle, _) = a64::board("interp", "plant.a.oracle", Tlbi::Never);
            let (mut copy, cpu) = a64::board("interp", "plant.a.copy", Tlbi::Never);
            for _ in 0..4 {
                oracle.run_quantum().expect("runs");
                copy.run_quantum().expect("runs");
            }
            let mut sys = cpu.sysregs();
            sys.tpidr_el0 ^= 0xdead_beef;
            cpu.set_sysregs(sys);
            let opts = Options::to_guest_seconds(9_999).at_most(8).hashing_every(0);
            let d = longrun::lockstep("plant", &mut oracle, "planted", &mut copy, &opts)
                .expect_err("a planted divergence must be reported");
            assert_eq!(d.quantum, 1, "reported on the wrong quantum: {d}");
            match &d.what {
                What::Device { path, detail, .. } => {
                    assert_eq!(path, "cpu0");
                    assert!(
                        detail.contains("tpidr_el0"),
                        "the report does not name the field that moved: {detail}"
                    );
                }
                other => panic!("the wrong kind of divergence: {other:?}"),
            }
        }

        // Tier 2, RAM: a word in a page the guest never touches is invisible to
        // every watched chunk and must be caught by the periodic full hash —
        // and *only* then, which is what says the two tiers are both load
        // bearing.
        {
            use rsemu::core::space::MemAttrs;
            use rsemu::core::value::Width;

            let (mut oracle, _) = a64::board("interp", "plant.b.oracle", Tlbi::Never);
            let (mut copy, cpu) = a64::board("interp", "plant.b.copy", Tlbi::Never);
            cpu.space()
                .expect("the core has its space")
                .write(0x6_0000, Width::U64, 0x1234_5678, MemAttrs::DEFAULT)
                .expect("inside RAM");
            let opts = Options::to_guest_seconds(9_999)
                .at_most(10)
                .hashing_every(5);
            let d = longrun::lockstep("plant", &mut oracle, "planted", &mut copy, &opts)
                .expect_err("a planted RAM divergence must be reported");
            assert_eq!(
                d.quantum, 5,
                "a RAM-only divergence must be reported on the first quantum \
                 the full hash is taken on, and no earlier: {d}"
            );
            assert!(
                matches!(d.what, What::Hash { .. }),
                "the wrong kind of divergence: {:?}",
                d.what
            );
        }

        // Tier 0, the clock: two machines whose schedulers are on different
        // instants are not comparable at all, and that has to be said as
        // itself rather than as a state diff.
        {
            let (mut oracle, _) = a64::board("interp", "plant.c.oracle", Tlbi::Never);
            let (mut copy, _) = a64::board("interp", "plant.c.copy", Tlbi::Never);
            copy.run_quantum().expect("runs");
            let opts = Options::to_guest_seconds(9_999).at_most(1).hashing_every(0);
            let d = longrun::lockstep("plant", &mut oracle, "planted", &mut copy, &opts)
                .expect_err("two machines on different instants must be reported");
            assert!(
                matches!(d.what, What::Clock { .. }),
                "the wrong kind of divergence: {:?}",
                d.what
            );
        }
    }
}

// ---------------------------------------------------------------------------
// the gate: a real kernel
// ---------------------------------------------------------------------------

#[cfg(all(feature = "machine-arm64-virt", feature = "cpu-arm-a64-lift"))]
mod arm64_virt {
    use super::longrun::{self, Options};
    use rsemu::machine::{Machine, catalog};

    /// Build `arm64-virt` on `engine` with `kernel` and `initrd` in its slots.
    ///
    /// `console` and `power` are per-machine, because two of these run in one
    /// process and must not type at each other or stop each other.
    fn board(engine: &str, tag: &str, kernel: &[u8], initrd: &[u8]) -> Machine {
        let entry = catalog::machine("arm64-virt").expect("this build ships it");
        let options = catalog::build_options()
            .expect("the catalog agrees with itself")
            .with_media("kernel", kernel)
            .with_media("initrd", initrd)
            .with_media("disk", &[][..])
            .with_param("engine", engine)
            .with_param(
                "ram",
                std::env::var("RSEMU_ARM64_RAM").unwrap_or_else(|_| "512M".to_string()),
            )
            .with_param(
                "cmdline",
                "earlycon=pl011,0x9000000 console=ttyAMA0 rdinit=/init",
            )
            .with_param("console", format!("longrun.{tag}"))
            .with_param("power", format!("longrun.{tag}"));
        let registry = catalog::registry().expect("a registry");
        rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .unwrap_or_else(|e| panic!("arm64-virt does not build with engine={engine}: {e}"))
    }

    /// The gate the two September defects were found by, automated.
    ///
    /// `#[ignore]` because it needs a fetched kernel and takes minutes;
    /// `.github/workflows/long-run.yml` is where it runs unattended. Without
    /// the fixture it skips **loudly**: the two commands that would make it run
    /// are printed, because a silent skip is how a gate stops being one.
    #[test]
    #[ignore = "needs a fetched kernel (scripts/fetch-testdata.sh arm64-linux) and minutes of wall time"]
    fn a_real_arm64_linux_boot_agrees_across_the_engines() {
        let Ok(path) = std::env::var("RSEMU_ARM64_KERNEL") else {
            eprintln!(
                "\n  SKIPPED: RSEMU_ARM64_KERNEL is not set, so there is no kernel to boot.\n\
                 \n      scripts/fetch-testdata.sh arm64-linux arm64-initramfs\n\
                 \n      RSEMU_ARM64_KERNEL=testdata/arm64/linux \\\n\
                 \x20     RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \\\n\
                 \x20     RSEMU_LONGRUN_SECONDS=120 \\\n\
                 \x20         cargo test --release --test engine_longrun -- --ignored --nocapture\n\
                 \n  This is the only test in the tree that can find a divergence a \n\
                 designed workload was not written to provoke, and it did find two.\n"
            );
            return;
        };
        let kernel = std::fs::read(&path).unwrap_or_else(|e| {
            panic!("RSEMU_ARM64_KERNEL names `{path}`, which will not read: {e}")
        });
        let initrd = match std::env::var("RSEMU_ARM64_INITRD") {
            Ok(p) => {
                std::fs::read(&p).unwrap_or_else(|e| panic!("RSEMU_ARM64_INITRD names `{p}`: {e}"))
            }
            Err(_) => Vec::new(),
        };

        let secs: u64 = std::env::var("RSEMU_LONGRUN_SECONDS")
            .ok()
            .and_then(|v| v.parse().ok())
            // 30 s of guest time clears both known defects (15.04 s and
            // 23.46 s) with room; the nightly asks for 120, which is where the
            // hand bisect stopped.
            .unwrap_or(30);
        let engines: Vec<String> = std::env::var("RSEMU_LONGRUN_ENGINES")
            .unwrap_or_else(|_| "jit,jit-host".to_string())
            .split(',')
            .filter(|s| !s.is_empty())
            .map(str::to_string)
            .collect();

        for engine in engines {
            let mut oracle = board("interp", &format!("oracle.{engine}"), &kernel, &initrd);
            let mut under_test = board(&engine, &engine, &kernel, &initrd);
            // The full hash walks 512 MiB, so it is taken rarely; the
            // per-quantum device fingerprint is what finds a divergence first
            // and it costs nothing by comparison.
            let opts = Options::to_guest_seconds(secs)
                .hashing_every(20_000)
                .reporting_every(10_000);
            match longrun::lockstep("arm64-virt", &mut oracle, &engine, &mut under_test, &opts) {
                Ok(summary) => eprintln!("arm64-virt engine={engine}: {summary}"),
                Err(d) => panic!("{d}"),
            }
        }
    }
}

// ---------------------------------------------------------------------------
// the same shape on a second core
// ---------------------------------------------------------------------------

#[cfg(all(feature = "machine-riscv-virt", feature = "cpu-riscv-lift"))]
mod riscv {
    use super::longrun::{self, Options};
    use rsemu::machine::{Machine, catalog};

    /// The RV64I loop `tests/riscv_virt_engines.rs` and `tests/workload` both
    /// use: add, store, load and two shifts per iteration, closed by a `jalr`
    /// through a register the first two instructions compute. Source: *The
    /// RISC-V Instruction Set Manual, Volume I*, chapter 2.
    ///
    /// It is a plain loop rather than anything shaped like [`super::a64`]'s
    /// workload, and that is the honest state of this core: the two defects
    /// were A64's, the mechanisms that produced them (a declined chained
    /// boundary, a per-core timer reached from inside a block) exist here too,
    /// and nobody has yet written the guest that provokes them. What this
    /// gives is the harness pointed at a second core at the cost of forty
    /// lines, which is the point of the harness being core-agnostic.
    const PROGRAM: [u32; 12] = [
        0x0000_0f17, // auipc t5, 0        t5 = 0x80000000
        0x014f_0f13, // addi  t5, t5, 20   t5 = loop
        0x0000_1397, // auipc t2, 1        t2 = 0x80001008, a scratch word in DRAM
        0x0000_0293, // addi  t0, x0, 0
        0x0010_0313, // addi  t1, x0, 1
        0x0062_82b3, // loop: add t0, t0, t1
        0x0053_b023, // sd    t0, 0(t2)
        0x0003_be03, // ld    t3, 0(t2)
        0x003e_1e93, // slli  t4, t3, 3
        0x003e_de93, // srli  t4, t4, 3
        0x01d2_82b3, // add   t0, t0, t4
        0x000f_0067, // jalr  x0, 0(t5)
    ];

    fn board(engine: &str, tag: &str) -> Machine {
        let firmware: Vec<u8> = PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect();
        let entry = catalog::machine("riscv-virt").expect("this build ships riscv-virt");
        let mut options = catalog::build_options().expect("the catalog agrees with itself");
        options
            .realize
            .media
            .insert("firmware", firmware.as_slice());
        for slot in ["flash0", "flash1", "disk", "initrd"] {
            options.realize.media.insert(slot, &[][..]);
        }
        for (name, value) in [
            ("ram", "16M".to_string()),
            ("engine", engine.to_string()),
            ("console", format!("longrun.riscv.{tag}")),
            ("power", format!("longrun.riscv.{tag}")),
        ] {
            options.resolve.params.push((String::from(name), value));
        }
        let registry = catalog::registry().expect("a registry");
        rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .unwrap_or_else(|e| panic!("riscv-virt does not build with engine={engine}: {e}"))
    }

    #[test]
    fn a_synthetic_riscv_workload_agrees_across_the_engines() {
        let secs: u64 = std::env::var("RSEMU_LONGRUN_SECONDS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(2);
        // The same variable the A64 legs read, so `RSEMU_LONGRUN_ENGINES=interp`
        // is the control on every board rather than on one of them.
        let engines =
            std::env::var("RSEMU_LONGRUN_ENGINES").unwrap_or_else(|_| "jit,jit-host".to_string());
        for engine in engines.split(',').filter(|s| !s.is_empty()) {
            let mut oracle = board("interp", &format!("oracle.{engine}"));
            let mut under_test = board(engine, engine);
            let opts = Options::to_guest_seconds(secs).hashing_every(2_000);
            match longrun::lockstep("riscv-virt", &mut oracle, engine, &mut under_test, &opts) {
                Ok(summary) => eprintln!("riscv-virt engine={engine}: {summary}"),
                Err(d) => panic!("{d}"),
            }
        }
    }
}
