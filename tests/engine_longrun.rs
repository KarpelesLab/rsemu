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
//! | `a_synthetic_riscv_workload_agrees_across_the_engines` | none | ~3 s | every `cargo test` |
//! | [`a_tlbi_in_the_loop_agrees_across_the_engines`] | none | under a second | every `cargo test` |
//! | `a_synthetic_x86_workload_agrees_across_the_engines` | none | ~1.4 s | every `cargo test` |
//! | `a_real_arm64_linux_boot_agrees_across_the_engines` | an `Image` | minutes | `--ignored`, nightly |
//! | `a_real_x86_linux_boot_agrees_across_the_engines` | a `bzImage` | minutes | `--ignored`, nightly |
//! | `the_clint_advances_while_the_hart_is_running` | none | milliseconds | every `cargo test` |
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
//!   scripts/fetch-testdata.sh arm64-linux arm64-initramfs x86-linux initramfs-x86
//!
//!   RSEMU_ARM64_KERNEL=testdata/arm64/linux \
//!   RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \
//!   RSEMU_X86_KERNEL=testdata/x86/bzImage \
//!   RSEMU_X86_INITRD=testdata/x86/initramfs-x86.cpio \
//!   RSEMU_LONGRUN_SECONDS=120 \
//!       cargo test --release \
//!           --features machine-arm64-virt,cpu-arm-a64-lift,machine-pc64,cpu-x86-lift,jit,jit-x86 \
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
//!
//! The x86 leg at the bottom of this file is written to the same brief and
//! against a longer list, because `cpu::x86::engine` documents more seams than
//! the A64 one does — the tick allowance, `Flags::Eager`, `admit`'s exclusion
//! list, `Smc::EndBlock`, block chaining and the entry translation are each
//! reached by something the guest deliberately does. It found nothing on the
//! `master` it was written against, over twenty guest seconds, and that is the
//! honest result: what it *does* catch is in `docs/testing/long-run.md`'s
//! calibration table, and one of the four defects re-introduced there is
//! invisible to `tests/x86_engines.rs`.
//!
//! There is an x86 leg of the kernel gate now, and it is the last module in
//! this file: `pc64` with a stock `bzImage` in its slot, both engines, quantum
//! by quantum. `docs/platforms/pc64.md` had already recorded nine hundred
//! guest seconds of that board booting on either engine — what was missing was
//! anybody comparing the two anywhere but at the end. Measured on the Debian
//! installer kernel `scripts/fetch-testdata.sh x86-linux` fetches: 900 guest
//! seconds, 1 946 548 quanta, 292 million translated blocks, 1.52 billion
//! instructions retired inside them against 6.5 million interpreted — and
//! agreement on every one of those quanta.
//!
//! The RISC-V leg is no longer a plain loop either. It is a **board**: Sv39
//! paging, a machine-mode trap handler, and the CLINT arming its own
//! comparator — written seam by seam off `cpu::riscv::engine` the way the x86
//! synthetic is written off `cpu::x86::engine`. Writing it turned up something
//! the engine's documentation assumed and this board did not provide, which is
//! `riscv::the_clint_advances_while_the_hart_is_running`. It had two causes,
//! landed a round apart: the hart publishes its position, and
//! `Scheduler::arm_live_cursors` now converts across two crystals as well as
//! within one. It runs on every `cargo test` and its doc comment is the record
//! of both.

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
    use std::sync::Arc;

    use rsemu::core::Captured;
    use rsemu::cpu::riscv::Hart;
    use rsemu::machine::{Machine, catalog};

    /// One `addi x0, x0, 0`, which is what a seam this build turns off is
    /// replaced by.
    ///
    /// The canonical RISC-V `NOP` (*The RISC-V Instruction Set Manual, Volume
    /// I*, §2.4), and the point of it is that it is **four bytes**, like every
    /// instruction it stands in for: every branch displacement in the loop
    /// below is left exactly as it was, so a bisect changes one property and
    /// nothing else.
    const NOP: u32 = 0x0000_0013;

    /// The CLINT's `mtime`, and the hart's comparator beside it.
    ///
    /// `machines/riscv-virt.machine` maps the chip at 0x02000000, and the
    /// offsets are the SiFive CLINT layout every `virt`-shaped board uses:
    /// `msip` at 0, `mtimecmp` at 0x4000, `mtime` at 0xbff8.
    const CLINT_MTIME: u64 = 0x0200_bff8;
    const CLINT_MTIMECMP: u64 = 0x0200_4000;
    /// Where DRAM starts, which is where the boot ROM at 0x1000 jumps.
    const DRAM: u64 = 0x8000_0000;
    /// The Sv39 tables, in the first two mebibytes so the level-0 table below
    /// describes them.
    const PT_ROOT: u64 = 0x8000_2000;
    const PT_L1: u64 = 0x8000_3000;
    const PT_L0: u64 = 0x8000_4000;
    /// The word the atomic every thirty-second pass adds into.
    const SCRATCH: u64 = 0x8000_5000;
    /// The base of the sixty-four-page data window the loop walks.
    const DATA: u64 = 0x8010_0000;
    /// Ticks of the CLINT's 10 MHz counter between timer interrupts: 4 µs.
    ///
    /// It has to be **short against a quantum** and that is the whole design.
    /// `cpu::riscv::engine` states the seam exactly: the window is *"a
    /// comparator the guest moves into the round that is already running"* —
    /// `Scheduler::natural_target` was computed when the round began and does
    /// not move, so a comparator written inside the round is crossed by the
    /// next read of `mtime` rather than at a quantum boundary, and `mtip`
    /// rises between two instructions of a lifted block. A period longer than
    /// a quantum would put every edge on a boundary, where both engines see it
    /// in the same place and the run says nothing.
    const PERIOD: u64 = 40;

    /// The machine-mode half, at [`DRAM`]: physical-memory protection, the
    /// Sv39 tables, the timer, the trap handler, and the drop into supervisor
    /// mode.
    ///
    /// Assembled from source with `llvm-mc -triple=riscv64 -mattr=+m,+a,+f,+d`
    /// and disassembled back; the listing is in the third column. Encodings and
    /// semantics: *The RISC-V Instruction Set Manual, Volume I: Unprivileged
    /// ISA* and *Volume II: Privileged Architecture* — §3.7 for physical-memory
    /// protection, §4.4 for Sv39, §3.1.9 for `mie`, and §3.3.2 for what `MRET`
    /// does with `mstatus.MPP`.
    ///
    /// ```text
    ///         # PMP entry 0, TOR from zero. Without it every supervisor
    ///         # access fails: a hart that implements PMP and has no entry
    ///         # matching refuses anything below machine mode.
    ///         li t0, 0x1fffffffffffffff / csrw pmpaddr0, t0
    ///         li t0, 0x0f              / csrw pmpcfg0, t0     # TOR, X|W|R
    ///
    ///         # The level-0 table: 512 four-kibibyte identity pages over the
    ///         # first two mebibytes of DRAM. Four kibibytes rather than one
    ///         # megapage because SFENCE.VMA has to be able to cool this code
    ///         # page's fetch translation and the walk that replaces it must
    ///         # be a real three-level one. A and D are left clear, so the
    ///         # first access to a page after each flush takes the walk's
    ///         # accessed/dirty update as well.
    ///         li t0, PT_L0 / li t1, 0x80000000 / li t2, 512
    /// 0:      srli t3, t1, 12 / slli t3, t3, 10 / ori t3, t3, 0x0f
    ///         sd t3, 0(t0) / addi t0, t0, 8
    ///         li t4, 4096 / add t1, t1, t4 / addi t2, t2, -1 / bnez t2, 0b
    ///
    ///         # Level 1 entry 0 -> that table; root entry 2 -> level 1; and
    ///         # root entry 0 is a one-gibibyte leaf over 0-1 GiB, R|W and no
    ///         # X, which is how the supervisor loop reaches the CLINT.
    ///         ...
    ///         la t0, mtrap / csrw mtvec, t0
    ///
    ///         # The first comparator. Every later one is the handler's.
    ///         li t0, CLINT_MTIME / ld t1, 0(t0)
    ///         li t2, PERIOD / add t1, t1, t2
    ///         li t0, CLINT_MTIMECMP / sd t1, 0(t0)
    ///
    ///         # mie.MTIE. mstatus.MIE is irrelevant: a machine-mode interrupt
    ///         # is always enabled while the hart runs in a lower privilege.
    ///         li t0, 0x80 / csrw mie, t0              # <- TIMER_BODY
    ///
    ///         li t0, PT_ROOT / srli t0, t0, 12
    ///         li t1, 8 / slli t1, t1, 60 / or t0, t0, t1
    ///         csrw satp, t0 / sfence.vma
    ///
    ///         li t0, 0x1800 / csrc mstatus, t0        # MPP = 01, supervisor
    ///         li t0, 0x0800 / csrs mstatus, t0
    ///         la t0, smain / csrw mepc, t0 / mret
    ///
    /// # s5, s6, s7 and s8 belong to the handler and the supervisor loop never
    /// # touches them, so nothing is spilled: a handler that pushed a frame
    /// # would put a store — and therefore a block boundary — in the middle of
    /// # the one thing this workload exists to reach.
    /// mtrap:  csrr s5, mcause
    ///         bltz s5, mtimer                 # bit 63 set: an interrupt
    ///         li s6, 9 / beq s5, s6, mecall   # ECALL from supervisor mode
    ///         addi s0, s0, 1                  # anything else, counted
    /// mecall: csrr s6, mepc / addi s6, s6, 4 / csrw mepc, s6 / mret
    /// mtimer: li s6, CLINT_MTIME / ld s7, 0(s6)
    ///         li s5, PERIOD / add s7, s7, s5
    ///         li s6, CLINT_MTIMECMP / sd s7, 0(s6)
    ///         addi s8, s8, 1                  # how many fired
    ///         mret
    /// ```
    const MACHINE: [u32; 87] = [
        0xfff0_0293, //   0  0x80000000  li t0, -0x1
        0x0032_d293, //   1  0x80000004  srli t0, t0, 0x3
        0x3b02_9073, //   2  0x80000008  csrw pmpaddr0, t0
        0x00f0_0293, //   3  0x8000000c  li t0, 0xf
        0x3a02_9073, //   4  0x80000010  csrw pmpcfg0, t0
        0x2000_12b7, //   5  0x80000014  lui t0, 0x20001
        0x0022_9293, //   6  0x80000018  slli t0, t0, 0x2
        0x0010_0313, //   7  0x8000001c  li t1, 0x1
        0x01f3_1313, //   8  0x80000020  slli t1, t1, 0x1f
        0x2000_0393, //   9  0x80000024  li t2, 0x200
        0x00c3_5e13, //  10  0x80000028  srli t3, t1, 0xc
        0x00ae_1e13, //  11  0x8000002c  slli t3, t3, 0xa
        0x00fe_6e13, //  12  0x80000030  ori t3, t3, 0xf
        0x01c2_b023, //  13  0x80000034  sd t3, 0x0(t0)
        0x0082_8293, //  14  0x80000038  addi t0, t0, 0x8
        0x0000_1eb7, //  15  0x8000003c  lui t4, 0x1
        0x01d3_0333, //  16  0x80000040  add t1, t1, t4
        0xfff3_8393, //  17  0x80000044  addi t2, t2, -0x1
        0xfe03_90e3, //  18  0x80000048  bnez t2, 0x28 <_start+0x28>
        0x0008_02b7, //  19  0x8000004c  lui t0, 0x80
        0x0032_8293, //  20  0x80000050  addi t0, t0, 0x3
        0x00c2_9293, //  21  0x80000054  slli t0, t0, 0xc
        0x2000_1337, //  22  0x80000058  lui t1, 0x20001
        0x0023_1313, //  23  0x8000005c  slli t1, t1, 0x2
        0x00c3_5313, //  24  0x80000060  srli t1, t1, 0xc
        0x00a3_1313, //  25  0x80000064  slli t1, t1, 0xa
        0x0013_6313, //  26  0x80000068  ori t1, t1, 0x1
        0x0062_b023, //  27  0x8000006c  sd t1, 0x0(t0)
        0x4000_12b7, //  28  0x80000070  lui t0, 0x40001
        0x0012_9293, //  29  0x80000074  slli t0, t0, 0x1
        0x0008_0337, //  30  0x80000078  lui t1, 0x80
        0x0033_0313, //  31  0x8000007c  addi t1, t1, 0x3
        0x00c3_1313, //  32  0x80000080  slli t1, t1, 0xc
        0x00c3_5313, //  33  0x80000084  srli t1, t1, 0xc
        0x00a3_1313, //  34  0x80000088  slli t1, t1, 0xa
        0x0013_6313, //  35  0x8000008c  ori t1, t1, 0x1
        0x0062_b823, //  36  0x80000090  sd t1, 0x10(t0)
        0x0c70_0313, //  37  0x80000094  li t1, 0xc7
        0x0062_b023, //  38  0x80000098  sd t1, 0x0(t0)
        0x0000_0297, //  39  0x8000009c  auipc t0, 0x0
        0x0782_8293, //  40  0x800000a0  addi t0, t0, 0x78
        0x3052_9073, //  41  0x800000a4  csrw mtvec, t0
        0x0200_c2b7, //  42  0x800000a8  lui t0, 0x200c
        0xff82_8293, //  43  0x800000ac  addi t0, t0, -0x8
        0x0002_b303, //  44  0x800000b0  ld t1, 0x0(t0)
        0x0280_0393, //  45  0x800000b4  li t2, 0x28
        0x0073_0333, //  46  0x800000b8  add t1, t1, t2
        0x0200_42b7, //  47  0x800000bc  lui t0, 0x2004
        0x0062_b023, //  48  0x800000c0  sd t1, 0x0(t0)
        0x0800_0293, //  49  0x800000c4  li t0, 0x80
        0x3042_9073, //  50  0x800000c8  csrw mie, t0
        0x4000_12b7, //  51  0x800000cc  lui t0, 0x40001
        0x0012_9293, //  52  0x800000d0  slli t0, t0, 0x1
        0x00c2_d293, //  53  0x800000d4  srli t0, t0, 0xc
        0x0080_0313, //  54  0x800000d8  li t1, 0x8
        0x03c3_1313, //  55  0x800000dc  slli t1, t1, 0x3c
        0x0062_e2b3, //  56  0x800000e0  or t0, t0, t1
        0x1802_9073, //  57  0x800000e4  csrw satp, t0
        0x1200_0073, //  58  0x800000e8  sfence.vma
        0x0030_0293, //  59  0x800000ec  li t0, 0x3
        0x00b2_9293, //  60  0x800000f0  slli t0, t0, 0xb
        0x3002_b073, //  61  0x800000f4  csrc mstatus, t0
        0x0010_0293, //  62  0x800000f8  li t0, 0x1
        0x00b2_9293, //  63  0x800000fc  slli t0, t0, 0xb
        0x3002_a073, //  64  0x80000100  csrs mstatus, t0
        0x0000_1297, //  65  0x80000104  auipc t0, 0x1
        0xefc2_8293, //  66  0x80000108  addi t0, t0, -0x104
        0x3412_9073, //  67  0x8000010c  csrw mepc, t0
        0x3020_0073, //  68  0x80000110  mret
        0x3420_2af3, //  69  0x80000114  csrr s5, mcause
        0x020a_c063, //  70  0x80000118  bltz s5, 0x138 <mtimer>
        0x0090_0b13, //  71  0x8000011c  li s6, 0x9
        0x016a_8463, //  72  0x80000120  beq s5, s6, 0x128 <mecall>
        0x0014_0413, //  73  0x80000124  addi s0, s0, 0x1
        0x3410_2b73, //  74  0x80000128  csrr s6, mepc
        0x004b_0b13, //  75  0x8000012c  addi s6, s6, 0x4
        0x341b_1073, //  76  0x80000130  csrw mepc, s6
        0x3020_0073, //  77  0x80000134  mret
        0x0200_cb37, //  78  0x80000138  lui s6, 0x200c
        0xff8b_0b13, //  79  0x8000013c  addi s6, s6, -0x8
        0x000b_3b83, //  80  0x80000140  ld s7, 0x0(s6)
        0x0280_0a93, //  81  0x80000144  li s5, 0x28
        0x015b_8bb3, //  82  0x80000148  add s7, s7, s5
        0x0200_4b37, //  83  0x8000014c  lui s6, 0x2004
        0x017b_3023, //  84  0x80000150  sd s7, 0x0(s6)
        0x001c_0c13, //  85  0x80000154  addi s8, s8, 0x1
        0x3020_0073, //  86  0x80000158  mret
    ];

    /// The supervisor half, one page above the machine-mode half — its own
    /// 4 KiB page on purpose, so `SFENCE.VMA` cools *this* translation and the
    /// handler's page is untouched.
    ///
    /// Everything below runs translated, and every line of it is a paragraph
    /// of `cpu::riscv::engine`'s own documentation turned into guest code:
    ///
    /// | seam in `cpu::riscv::engine` | what the loop does about it |
    /// | --- | --- |
    /// | `lift::MAX_INSNS` = 64, `CHAIN` = 16 | sixty-four consecutive lifted ALU instructions, so one `advance` is a chain rather than a block |
    /// | *"a store still ends the block"* | a store and a load to a different 4 KiB page every pass, sixty-four of them |
    /// | `IrHost::load` and a **lazily-advanced device** | `ld` of the CLINT's `mtime` **from inside a lifted block**, every pass — the load that catches the chip up to the hart's live position and can raise `mtip` between two instructions |
    /// | `admit`'s exclusion list | `SFENCE.VMA` every sixteenth pass, a supervisor CSR round trip every eighth, an `amoadd.d` every thirty-second, an `ECALL` every two hundred and fifty-sixth — every one of them outside the lifted subset and therefore a declined boundary |
    /// | the entry translation in `admit` | that same `SFENCE.VMA`, which throws away this page's *fetch* translation: the RISC-V analogue of the `TLBI` that found the third A64 defect, and the only thing on this hart that cools a fetch entry |
    /// | the data-side walk and its accessed and dirty bits | the tables leave `A` and `D` clear, so the first touch of each page after each flush takes the update |
    ///
    /// `mtime` is folded into the **first** of the sixty-four ALU
    /// instructions, and that is deliberate: a divergence about *when* a block
    /// noticed the timer becomes an arithmetic difference in a named register
    /// on the very next pass, rather than something only `debt` and `pc` carry.
    ///
    /// ```text
    /// smain:  li s1, DATA / li s3, CLINT_MTIME / li s4, SCRATCH
    ///         li s2, 0
    ///         li a0..a7, s9, s10, s11, t4, t5, t6, 0
    /// sloop:  addi s2, s2, 1
    ///         andi t0, s2, 63 / slli t0, t0, 12 / add t0, t0, s1
    ///         sd s2, 0(t0)                    ; a store ends its block
    ///         ld t1, 0(t0)
    ///         ld t2, 0(s3)                    ; <- CLINT_BODY: mtime
    ///         .rept 4                         ; sixty-four ALU instructions
    ///         add a0,a0,t2 / xor a1,a1,a0 / sub a2,a2,a1 / add a3,a3,a2
    ///         slli a4,a3,1 / xor a5,a5,a4 / add a6,a6,a5 / srli a7,a6,3
    ///         add s9,s9,a7 / xor s10,s10,s9 / sub s11,s11,s10 / add t4,t4,s11
    ///         xor t5,t5,t4 / addw t6,t6,t5 / sllw t4,t6,2 / add a0,a0,t4
    ///         .endr
    ///         andi t3, s2, 15  / bnez t3, 1f
    ///         sfence.vma                      ; <- SFENCE_BODY
    /// 1:      andi t3, s2, 7   / bnez t3, 2f
    ///         csrr t3, sscratch / csrw sscratch, s2   ; <- CSR_BODY
    /// 2:      andi t3, s2, 31  / bnez t3, 3f
    ///         amoadd.d t3, s2, (s4)           ; <- AMO_BODY
    /// 3:      andi t3, s2, 255 / bnez t3, 4f
    ///         ecall                           ; <- ECALL_BODY
    /// 4:      j sloop
    /// ```
    const SUPERVISOR: [u32; 107] = [
        0x0080_14b7, //   0  0x80001000  lui s1, 0x801
        0x0084_9493, //   1  0x80001004  slli s1, s1, 0x8
        0x0200_c9b7, //   2  0x80001008  lui s3, 0x200c
        0xff89_8993, //   3  0x8000100c  addi s3, s3, -0x8
        0x0008_0a37, //   4  0x80001010  lui s4, 0x80
        0x005a_0a13, //   5  0x80001014  addi s4, s4, 0x5
        0x00ca_1a13, //   6  0x80001018  slli s4, s4, 0xc
        0x0000_0913, //   7  0x8000101c  li s2, 0x0
        0x0000_0513, //   8  0x80001020  li a0, 0x0
        0x0000_0593, //   9  0x80001024  li a1, 0x0
        0x0000_0613, //  10  0x80001028  li a2, 0x0
        0x0000_0693, //  11  0x8000102c  li a3, 0x0
        0x0000_0713, //  12  0x80001030  li a4, 0x0
        0x0000_0793, //  13  0x80001034  li a5, 0x0
        0x0000_0813, //  14  0x80001038  li a6, 0x0
        0x0000_0893, //  15  0x8000103c  li a7, 0x0
        0x0000_0c93, //  16  0x80001040  li s9, 0x0
        0x0000_0d13, //  17  0x80001044  li s10, 0x0
        0x0000_0d93, //  18  0x80001048  li s11, 0x0
        0x0000_0e93, //  19  0x8000104c  li t4, 0x0
        0x0000_0f13, //  20  0x80001050  li t5, 0x0
        0x0000_0f93, //  21  0x80001054  li t6, 0x0
        0x0019_0913, //  22  0x80001058  addi s2, s2, 0x1
        0x03f9_7293, //  23  0x8000105c  andi t0, s2, 0x3f
        0x00c2_9293, //  24  0x80001060  slli t0, t0, 0xc
        0x0092_82b3, //  25  0x80001064  add t0, t0, s1
        0x0122_b023, //  26  0x80001068  sd s2, 0x0(t0)
        0x0002_b303, //  27  0x8000106c  ld t1, 0x0(t0)
        0x0009_b383, //  28  0x80001070  ld t2, 0x0(s3)
        0x0075_0533, //  29  0x80001074  add a0, a0, t2
        0x00a5_c5b3, //  30  0x80001078  xor a1, a1, a0
        0x40b6_0633, //  31  0x8000107c  sub a2, a2, a1
        0x00c6_86b3, //  32  0x80001080  add a3, a3, a2
        0x0016_9713, //  33  0x80001084  slli a4, a3, 0x1
        0x00e7_c7b3, //  34  0x80001088  xor a5, a5, a4
        0x00f8_0833, //  35  0x8000108c  add a6, a6, a5
        0x0038_5893, //  36  0x80001090  srli a7, a6, 0x3
        0x011c_8cb3, //  37  0x80001094  add s9, s9, a7
        0x019d_4d33, //  38  0x80001098  xor s10, s10, s9
        0x41ad_8db3, //  39  0x8000109c  sub s11, s11, s10
        0x01be_8eb3, //  40  0x800010a0  add t4, t4, s11
        0x01df_4f33, //  41  0x800010a4  xor t5, t5, t4
        0x01ef_8fbb, //  42  0x800010a8  addw t6, t6, t5
        0x002f_9e9b, //  43  0x800010ac  slliw t4, t6, 0x2
        0x01d5_0533, //  44  0x800010b0  add a0, a0, t4
        0x0075_0533, //  45  0x800010b4  add a0, a0, t2
        0x00a5_c5b3, //  46  0x800010b8  xor a1, a1, a0
        0x40b6_0633, //  47  0x800010bc  sub a2, a2, a1
        0x00c6_86b3, //  48  0x800010c0  add a3, a3, a2
        0x0016_9713, //  49  0x800010c4  slli a4, a3, 0x1
        0x00e7_c7b3, //  50  0x800010c8  xor a5, a5, a4
        0x00f8_0833, //  51  0x800010cc  add a6, a6, a5
        0x0038_5893, //  52  0x800010d0  srli a7, a6, 0x3
        0x011c_8cb3, //  53  0x800010d4  add s9, s9, a7
        0x019d_4d33, //  54  0x800010d8  xor s10, s10, s9
        0x41ad_8db3, //  55  0x800010dc  sub s11, s11, s10
        0x01be_8eb3, //  56  0x800010e0  add t4, t4, s11
        0x01df_4f33, //  57  0x800010e4  xor t5, t5, t4
        0x01ef_8fbb, //  58  0x800010e8  addw t6, t6, t5
        0x002f_9e9b, //  59  0x800010ec  slliw t4, t6, 0x2
        0x01d5_0533, //  60  0x800010f0  add a0, a0, t4
        0x0075_0533, //  61  0x800010f4  add a0, a0, t2
        0x00a5_c5b3, //  62  0x800010f8  xor a1, a1, a0
        0x40b6_0633, //  63  0x800010fc  sub a2, a2, a1
        0x00c6_86b3, //  64  0x80001100  add a3, a3, a2
        0x0016_9713, //  65  0x80001104  slli a4, a3, 0x1
        0x00e7_c7b3, //  66  0x80001108  xor a5, a5, a4
        0x00f8_0833, //  67  0x8000110c  add a6, a6, a5
        0x0038_5893, //  68  0x80001110  srli a7, a6, 0x3
        0x011c_8cb3, //  69  0x80001114  add s9, s9, a7
        0x019d_4d33, //  70  0x80001118  xor s10, s10, s9
        0x41ad_8db3, //  71  0x8000111c  sub s11, s11, s10
        0x01be_8eb3, //  72  0x80001120  add t4, t4, s11
        0x01df_4f33, //  73  0x80001124  xor t5, t5, t4
        0x01ef_8fbb, //  74  0x80001128  addw t6, t6, t5
        0x002f_9e9b, //  75  0x8000112c  slliw t4, t6, 0x2
        0x01d5_0533, //  76  0x80001130  add a0, a0, t4
        0x0075_0533, //  77  0x80001134  add a0, a0, t2
        0x00a5_c5b3, //  78  0x80001138  xor a1, a1, a0
        0x40b6_0633, //  79  0x8000113c  sub a2, a2, a1
        0x00c6_86b3, //  80  0x80001140  add a3, a3, a2
        0x0016_9713, //  81  0x80001144  slli a4, a3, 0x1
        0x00e7_c7b3, //  82  0x80001148  xor a5, a5, a4
        0x00f8_0833, //  83  0x8000114c  add a6, a6, a5
        0x0038_5893, //  84  0x80001150  srli a7, a6, 0x3
        0x011c_8cb3, //  85  0x80001154  add s9, s9, a7
        0x019d_4d33, //  86  0x80001158  xor s10, s10, s9
        0x41ad_8db3, //  87  0x8000115c  sub s11, s11, s10
        0x01be_8eb3, //  88  0x80001160  add t4, t4, s11
        0x01df_4f33, //  89  0x80001164  xor t5, t5, t4
        0x01ef_8fbb, //  90  0x80001168  addw t6, t6, t5
        0x002f_9e9b, //  91  0x8000116c  slliw t4, t6, 0x2
        0x01d5_0533, //  92  0x80001170  add a0, a0, t4
        0x00f9_7e13, //  93  0x80001174  andi t3, s2, 0xf
        0x000e_1463, //  94  0x80001178  bnez t3, 0x1180 <sfence_end>
        0x1200_0073, //  95  0x8000117c  sfence.vma
        0x0079_7e13, //  96  0x80001180  andi t3, s2, 0x7
        0x000e_1663, //  97  0x80001184  bnez t3, 0x1190 <csr_end>
        0x1400_2e73, //  98  0x80001188  csrr t3, sscratch
        0x1409_1073, //  99  0x8000118c  csrw sscratch, s2
        0x01f9_7e13, // 100  0x80001190  andi t3, s2, 0x1f
        0x000e_1463, // 101  0x80001194  bnez t3, 0x119c <amo_end>
        0x012a_3e2f, // 102  0x80001198  <unknown>
        0x0ff9_7e13, // 103  0x8000119c  zext.b t3, s2
        0x000e_1463, // 104  0x800011a0  bnez t3, 0x11a8 <ecall_end>
        0x0000_0073, // 105  0x800011a4  ecall
        0xeb1f_f06f, // 106  0x800011a8  j 0x1058 <sloop>
    ];

    /// Where the supervisor half sits, relative to [`DRAM`]: its own page.
    const SUPERVISOR_AT: usize = 0x1000;

    /// The word index of `sloop` in [`SUPERVISOR`], for the self-check below.
    const SLOOP: usize = 22;
    /// The two instructions that arm `mie.MTIE`, in [`MACHINE`].
    const TIMER_BODY: (usize, usize) = (49, 51);
    /// `ld t2, 0(s3)` — the CLINT read, in [`SUPERVISOR`].
    const CLINT_BODY: (usize, usize) = (28, 29);
    /// `sfence.vma`.
    const SFENCE_BODY: (usize, usize) = (95, 96);
    /// `csrr t3, sscratch` and `csrw sscratch, s2`.
    const CSR_BODY: (usize, usize) = (98, 100);
    /// `amoadd.d t3, s2, (s4)`.
    const AMO_BODY: (usize, usize) = (102, 103);
    /// `ecall`.
    const ECALL_BODY: (usize, usize) = (105, 106);

    /// Which of the workload's seams this build of the guest reaches.
    ///
    /// Every field off is still a valid guest — each window is replaced by
    /// [`NOP`]s in place and every branch around it keeps its displacement —
    /// so a bisect changes one property at a time and nothing else.
    /// [`Seams::ALL`] is what the committed test runs.
    ///
    /// This is the x86 leg's `Stress` on a second core, and it is here for the
    /// same reason: *"the same workload with either one alone agrees for six
    /// thousand quanta"* is the sentence that took the third A64 defect from a
    /// hash mismatch to a named function, and producing it should not need an
    /// edit.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(crate) struct Seams {
        /// Arm `mie.MTIE`, so the CLINT can interrupt the hart at all. With it
        /// off the chip still counts and `mtip` still rises on the wire; the
        /// hart simply never traps, and nothing re-arms the comparator.
        pub(crate) timer: bool,
        /// Keep the `ld` of `mtime` from inside the lifted loop — the load
        /// that catches a lazily-advanced device up to the hart's live
        /// position.
        pub(crate) clint: bool,
        /// Keep `SFENCE.VMA`, which cools this code page's fetch translation.
        pub(crate) sfence: bool,
        /// Keep the supervisor CSR round trip.
        pub(crate) csr: bool,
        /// Keep `amoadd.d`, which is outside the lifted subset and reports its
        /// store through the interpreter's own log.
        pub(crate) amo: bool,
        /// Keep `ECALL`.
        pub(crate) ecall: bool,
    }

    impl Seams {
        /// Everything on: the committed workload.
        pub(crate) const ALL: Seams = Seams {
            timer: true,
            clint: true,
            sfence: true,
            csr: true,
            amo: true,
            ecall: true,
        };

        /// The seams named in `keep`, and nothing else.
        pub(crate) fn keeping(keep: &str) -> Seams {
            let has = |name: &str| keep.split(',').any(|s| s.trim() == name);
            Seams {
                timer: has("timer"),
                clint: has("clint"),
                sfence: has("sfence"),
                csr: has("csr"),
                amo: has("amo"),
                ecall: has("ecall"),
            }
        }
    }

    /// The firmware image: [`MACHINE`] at [`DRAM`], [`SUPERVISOR`] one page
    /// above it, and the seams `seams` turns off replaced by [`NOP`]s.
    ///
    /// One image rather than two writes into RAM after the build, because
    /// `riscv-virt` already has a loader for exactly this and a cold reset
    /// zeroes DRAM *before* the loaders run — so anything written by hand
    /// afterwards would be a second mechanism to keep in step with the first.
    fn program(seams: Seams) -> Vec<u8> {
        let mut machine = MACHINE;
        if !seams.timer {
            for word in &mut machine[TIMER_BODY.0..TIMER_BODY.1] {
                *word = NOP;
            }
        }
        let mut supervisor = SUPERVISOR;
        for (keep, (from, to)) in [
            (seams.clint, CLINT_BODY),
            (seams.sfence, SFENCE_BODY),
            (seams.csr, CSR_BODY),
            (seams.amo, AMO_BODY),
            (seams.ecall, ECALL_BODY),
        ] {
            if !keep {
                for word in &mut supervisor[from..to] {
                    *word = NOP;
                }
            }
        }
        let mut image: Vec<u8> = machine.iter().flat_map(|w| w.to_le_bytes()).collect();
        // The gap between the two halves is never executed and never read; it
        // is there so the supervisor loop gets a page of its own.
        image.resize(SUPERVISOR_AT, 0);
        image.extend(supervisor.iter().flat_map(|w| w.to_le_bytes()));
        image
    }

    /// The addresses and windows this module names are the ones the assembled
    /// words carry.
    ///
    /// A named constant nothing checks drifts from the bytes it describes, and
    /// hand-placed offsets into an assembled blob are exactly where that goes
    /// unnoticed: `SFENCE_BODY` off by one would blank the `bnez` in front of
    /// it and the loop would fall into the next test with a different
    /// displacement, so a bisect would blame the wrong seam and the guest
    /// would still look like it ran.
    #[test]
    fn the_guest_carries_the_windows_and_the_addresses_this_file_names() {
        // The immediates the two halves were assembled with, read back out of
        // the encodings. `lui`'s immediate is bits 31:12 of the instruction,
        // already in place; `addi`'s is a signed twelve-bit field at bit 20;
        // `slli`'s shift amount is the low six bits of the same field
        // (*Volume I*, §2.4 and §4.2).
        let lui = |word: u32| u64::from(word & 0xffff_f000);
        let imm12 = |word: u32| i64::from((word as i32) >> 20);
        let shamt = |word: u32| (word >> 20) & 0x3f;
        // Every address this module names, out of the instruction that
        // materialises it. A constant nothing reads back is a comment.
        assert_eq!(
            (imm12(MACHINE[7]) as u64) << shamt(MACHINE[8]),
            DRAM,
            "`li t1, 0x80000000`, the base of the identity map"
        );
        assert_eq!(
            lui(MACHINE[5]) << shamt(MACHINE[6]),
            PT_L0,
            "`li t0, PT_L0`"
        );
        assert_eq!(
            lui(MACHINE[19]).wrapping_add(imm12(MACHINE[20]) as u64) << shamt(MACHINE[21]),
            PT_L1,
            "`li t0, PT_L1`"
        );
        assert_eq!(
            lui(MACHINE[28]) << shamt(MACHINE[29]),
            PT_ROOT,
            "`li t0, PT_ROOT`"
        );
        assert_eq!(
            lui(MACHINE[42]).wrapping_add(imm12(MACHINE[43]) as u64),
            CLINT_MTIME,
            "`li t0, CLINT_MTIME`, in the machine-mode half"
        );
        assert_eq!(
            lui(MACHINE[47]),
            CLINT_MTIMECMP,
            "`li t0, CLINT_MTIMECMP`, in the machine-mode half"
        );
        assert_eq!(
            imm12(MACHINE[45]) as u64,
            PERIOD,
            "the first comparator's `li t2, PERIOD`"
        );
        assert_eq!(
            imm12(MACHINE[81]) as u64,
            PERIOD,
            "the handler re-arms with the same period it started with"
        );
        assert_eq!(
            lui(SUPERVISOR[2]).wrapping_add(imm12(SUPERVISOR[3]) as u64),
            CLINT_MTIME,
            "smain's `li s3, CLINT_MTIME`"
        );
        assert_eq!(
            lui(SUPERVISOR[0]) << shamt(SUPERVISOR[1]),
            DATA,
            "smain's `li s1, DATA`"
        );
        assert_eq!(
            lui(SUPERVISOR[4]).wrapping_add(imm12(SUPERVISOR[5]) as u64) << shamt(SUPERVISOR[6]),
            SCRATCH,
            "smain's `li s4, SCRATCH`"
        );
        for (name, (from, to), want) in [
            ("the CLINT read", CLINT_BODY, &[0x0009_b383u32][..]),
            ("sfence.vma", SFENCE_BODY, &[0x1200_0073]),
            ("the CSR round trip", CSR_BODY, &[0x1400_2e73, 0x1409_1073]),
            ("amoadd.d", AMO_BODY, &[0x012a_3e2f]),
            ("ecall", ECALL_BODY, &[0x0000_0073]),
        ] {
            assert_eq!(&SUPERVISOR[from..to], want, "the window for {name} moved");
        }
        assert_eq!(
            &MACHINE[TIMER_BODY.0..TIMER_BODY.1],
            &[0x0800_0293, 0x3042_9073],
            "the window for `li t0, 0x80; csrw mie, t0` moved"
        );
        assert_eq!(
            SUPERVISOR[SLOOP], 0x0019_0913,
            "`sloop` no longer starts with `addi s2, s2, 1`"
        );
        // The back edge really does close the loop: `j sloop` is a JAL with
        // rd = x0 and a twenty-bit displacement in the J immediate's scrambled
        // order (*Volume I*, §2.3).
        let jal = SUPERVISOR[ECALL_BODY.1];
        let disp = (((jal >> 31) & 1) << 20)
            | (((jal >> 12) & 0xff) << 12)
            | (((jal >> 20) & 1) << 11)
            | (((jal >> 21) & 0x3ff) << 1);
        let disp = ((disp as i32) << 11) >> 11;
        assert_eq!(
            (ECALL_BODY.1 as i32) * 4 + disp,
            SLOOP as i32 * 4,
            "the back edge does not land on `sloop`"
        );
        // Every seam off is still a guest of exactly the same length with the
        // same branches in the same places.
        assert_eq!(program(Seams::ALL).len(), program(Seams::keeping("")).len());
    }

    /// Build `riscv-virt` on `engine` with this workload in its firmware slot.
    ///
    /// The catalog's board rather than one written here, because the seam this
    /// leg exists for **is a device**: `cpu::riscv::engine` says so in as many
    /// words — nothing on this hart is driven off its own tick count the way
    /// A64's generic timer is, and the raiser is the CLINT on the other side
    /// of a load. A hand-written board would have to grow one anyway, and this
    /// one has it wired the way a real guest meets it.
    ///
    /// Sixteen mebibytes of DRAM: the full state hash walks all of it, and the
    /// guest's top address is `DATA` plus sixty-four pages.
    pub(crate) fn board(engine: &str, tag: &str, seams: Seams) -> (Machine, Arc<Hart>) {
        board_with(engine, tag, &program(seams))
    }

    /// [`board`], with any firmware image at all — the reproduction at the
    /// bottom of this module wants a different guest on the same board.
    fn board_with(engine: &str, tag: &str, firmware: &[u8]) -> (Machine, Arc<Hart>) {
        let harts: Arc<Captured<Hart>> = Arc::new(Captured::new());
        let kept = Arc::clone(&harts);
        let mut bindings = catalog::bindings().expect("this build's bindings");
        bindings.replace("cpu.riscv", move |props| {
            let hart = Arc::new(Hart::from_props(props)?);
            kept.push(&hart);
            Ok(hart)
        });
        let entry = catalog::machine("riscv-virt").expect("this build ships riscv-virt");
        let mut options = catalog::build_options()
            .expect("the catalog agrees with itself")
            .with_bindings(bindings);
        options.realize.media.insert("firmware", firmware);
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
        let machine = rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .unwrap_or_else(|e| panic!("riscv-virt does not build with engine={engine}: {e}"));
        let hart = harts.take().expect("the binding captured the hart");
        (machine, hart)
    }

    /// **A defect this leg found**, fixed in two halves a round apart: on
    /// `riscv-virt` the CLINT's `mtime` did not move while the hart was
    /// running, so a guest read the same value for a whole quantum.
    ///
    /// # What the guest does
    ///
    /// [`TICK_PROBE`] reads `mtime` in a tight loop and counts how many
    /// **distinct** values it sees. It is run for eight quanta and asked how
    /// many it found.
    ///
    /// # What it should find
    ///
    /// `riscv.clint` is a lazily-advanced device (`ROADMAP.md` §4.2) and its
    /// `Registers::read` calls `sync` before answering, precisely so that *"a
    /// guest load catches the chip up to the core's live position"* — the
    /// sentence is `cpu::riscv::engine`'s own. A hart on this board is given
    /// `SchedulerConfig::max_ticks_per_quantum` — ten thousand — of a 1 GHz
    /// domain per round, which is 10 µs, and `mtime` counts at 10 MHz, so a
    /// round's worth of execution spans about **a hundred** distinct `mtime`
    /// values. Eight quanta should therefore find several hundred.
    ///
    /// # What it used to find: seven. One per quantum.
    ///
    /// It finds **800** now. There were two causes and the fix needed both.
    ///
    /// **Cause 1 — the hart published nothing. Fixed.**
    /// `Scheduler::arm_live_cursors` builds each lazy device's live view on
    /// the running runnable's [`TickCursor`], and a catch-up reads the
    /// runnable's own tick counter out of it. `Hart::attach_cursor` used to
    /// keep the cursor's **exit flag** and drop the position half, saying so
    /// in as many words. It keeps both now, and `Exec::publish_position`
    /// publishes `State::cycles` immediately before every access that leaves
    /// for the address space — which is the one publication point every
    /// engine reaches identically, since the compiled fast path covers plain
    /// RAM and nothing else. `cpu::riscv::tests::
    /// a_running_hart_publishes_its_position_to_the_devices_it_reads` is that
    /// half's own gate and does not need a board.
    ///
    /// **Cause 2 — the CLINT is on another crystal. Fixed.**
    /// `machines/riscv-virt.machine` hangs `mtime` off `osc rtc`, a separate
    /// oscillator tree from `osc core`, and `arm_live_cursors` used to arm a
    /// live view only across slots that share a root — so the CLINT's slot was
    /// skipped whatever the hart published. Instrumented on this very test at
    /// the time: one lazy slot, on `OscillatorId(1)`, skipped in all sixteen
    /// arm calls of an eight-quantum run, once for the hart on
    /// `OscillatorId(0)` and once for the 16550 on `OscillatorId(2)`.
    ///
    /// Closing it meant giving `Live` a **cross-tree** ratio, which is what
    /// `ROADMAP.md` §4.2 already prescribes for two independent crystals —
    /// "reciprocal multiply + a per-root residual accumulator", error bounded
    /// below one tick and non-accumulating because the base is re-anchored
    /// from the forest every round. It is emphatically *not* "routing an
    /// intra-tree relationship through absolute time": the intra-tree path is
    /// exactly as exact as it was, and
    /// `core::sched::tests::an_intra_tree_ratio_is_still_exact_with_another_crystal_present`
    /// is that claim's own gate. This test now reports 800 distinct values
    /// over eight quanta — exactly the hundred a round predicts — and
    /// `riscv_virt_engines::every_engine_hashes_to_the_same_machine_at_every_checkpoint`
    /// stayed green.
    ///
    /// # Why it matters here rather than only as a clock-resolution nit
    ///
    /// It closes, at the board level, the one seam this whole leg was asked to
    /// stress. `cpu::riscv::engine` documents the window exactly — *"`mtip`
    /// then rises between two instructions of a lifted block, where
    /// `Exec::step` would have taken the trap at the next one"* — and
    /// `IrHost::load` is the fix for it. But a rise can only happen where the
    /// comparator is crossed, and on this board it is never crossed anywhere
    /// but `close_round`, which is a quantum boundary and where both engines
    /// agree by construction. Measured: the workload above takes **1 999 timer
    /// interrupts in 2 000 quanta** — exactly one each — with the seam knob
    /// `clint` on and with it off alike, which is the same statement from the
    /// other side.
    ///
    /// So `engine::tests::a_load_that_raises_an_interrupt_is_taken_where_the_
    /// interpreter_takes_it`, which builds a device that raises
    /// unconditionally, is the **only** coverage that seam has, and no guest
    /// on a shipped RISC-V board can reach it. That is worth knowing before
    /// somebody deletes `IrHost::load`'s hand-back as dead code.
    ///
    /// The assertion below was written against the fixed behaviour rather than
    /// against the bug while the bug was still there, so landing the second
    /// half turned it green without an edit — which is the whole reason to
    /// write an `#[ignore]`d test that way.
    #[test]
    fn the_clint_advances_while_the_hart_is_running() {
        /// `mtime` in a tight loop, counting distinct values into `t2`.
        ///
        /// ```text
        ///         li   t3, CLINT_MTIME
        ///         ld   t1, 0(t3)
        ///         li   t2, 0
        /// loop:   ld   t0, 0(t3)
        ///         beq  t0, t1, loop       ; the same instant: go round again
        ///         addi t2, t2, 1          ; a new one: count it
        ///         mv   t1, t0
        ///         j    loop
        /// ```
        const TICK_PROBE: [u32; 9] = [
            0x0200_ce37, // lui  t3, 0x200c
            0xff8e_0e13, // addi t3, t3, -0x8
            0x000e_3303, // ld   t1, 0x0(t3)
            0x0000_0393, // li   t2, 0x0
            0x000e_3283, // loop: ld t0, 0x0(t3)
            0xfe62_8ee3, // beq  t0, t1, loop
            0x0013_8393, // addi t2, t2, 0x1
            0x0002_8313, // mv   t1, t0
            0xff1f_f06f, // j    loop
        ];

        const QUANTA: u64 = 8;
        let image: Vec<u8> = TICK_PROBE.iter().flat_map(|w| w.to_le_bytes()).collect();
        let (mut machine, hart) = board_with("interp", "clint-probe", &image);
        for _ in 0..QUANTA {
            machine.run_quantum().expect("the machine runs");
        }
        // x7 is t2.
        let seen = hart.x(7);
        // A round is 10 000 ticks of a 1 GHz domain and `mtime` counts at
        // 10 MHz, so a hundred `mtime` values fall inside one — and the probe
        // loop is four instructions, far finer than the hundred core ticks
        // between two of them, so it sees essentially all of them. Half that
        // is the floor: it is an order of magnitude above the one-per-quantum
        // the bug produces, and well below what a correct catch-up gives.
        let want = QUANTA * 50;
        assert!(
            seen >= want,
            "the guest read `mtime` for {QUANTA} quanta and saw {seen} distinct \
             value(s), wanted at least {want} — so the CLINT is not being \
             caught up to the hart's live position and `mtip` can never rise \
             inside a block. See this test's doc comment for the diagnosis."
        );
    }

    /// Assert the guest actually did what it was written to do.
    ///
    /// Without this the run could be green because the hart faulted on its
    /// first instruction: a comparison of two stopped machines agrees at every
    /// checkpoint. Each of these is a property the workload exists for, and
    /// the first of them — `s0`, the machine-mode handler's count of traps
    /// that were neither the timer nor the `ECALL` — has caught a wrong page
    /// table twice while this file was being written.
    pub(crate) fn assert_the_workload_ran(hart: &Hart, engine: &str, seams: Seams) {
        // x8 is s0: anything the machine-mode handler could not account for.
        assert_eq!(
            hart.x(8),
            0,
            "engine={engine}: the machine-mode handler took {} trap(s) that \
             were neither the timer nor the guest's own ECALL — a page fault \
             out of the Sv39 tables, most likely, in which case the supervisor \
             loop is not running and this run compared two stopped harts",
            hart.x(8)
        );
        // x18 is s2, the pass counter.
        assert!(
            hart.x(18) > 1_000,
            "engine={engine}: the loop went round {} times, so the guest is \
             not running the workload at all",
            hart.x(18)
        );
        if seams.timer {
            // x24 is s8, incremented once per timer interrupt taken.
            assert!(
                hart.x(24) > 0,
                "engine={engine}: the CLINT never interrupted, so the run says \
                 nothing about where a translated block notices one"
            );
        }
        // `RSEMU_LONGRUN_ENGINES=interp` is the control leg — an interpreter
        // against itself — and an interpreted hart has no statistics.
        if let Some((blocks, host)) = hart.jit_stats() {
            assert!(
                blocks > 0,
                "engine={engine} executed no translated block, so the run \
                 compared two interpreters"
            );
            eprintln!(
                "riscv-virt engine={engine}: {} pass(es), {} timer interrupt(s), \
                 {blocks} block(s) of which {host} as host code",
                hart.x(18),
                hart.x(24)
            );
        }
    }
}

#[cfg(all(feature = "machine-riscv-virt", feature = "cpu-riscv-lift"))]
mod riscv_tests {
    use super::longrun::{self, Options};
    use super::riscv::{self, Seams};

    fn engines() -> Vec<String> {
        std::env::var("RSEMU_LONGRUN_ENGINES")
            .unwrap_or_else(|_| "jit,jit-host".to_string())
            .split(',')
            .filter(|s| !s.is_empty())
            .map(str::to_string)
            .collect()
    }

    /// Which seams this run keeps.
    ///
    /// Everything, unless `RSEMU_RISCV_LONGRUN_SEAMS` names a subset —
    /// `timer,clint,sfence,csr,amo,ecall` — which is what a bisect turns off
    /// one at a time once this test has failed. See [`Seams::keeping`].
    fn seams() -> Seams {
        match std::env::var("RSEMU_RISCV_LONGRUN_SEAMS") {
            Ok(keep) => {
                let seams = Seams::keeping(&keep);
                eprintln!("riscv-virt: RSEMU_RISCV_LONGRUN_SEAMS={keep} -> {seams:?}");
                seams
            }
            Err(_) => Seams::ALL,
        }
    }

    #[test]
    fn a_synthetic_riscv_workload_agrees_across_the_engines() {
        let secs: u64 = std::env::var("RSEMU_LONGRUN_SECONDS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(2);
        let seams = seams();
        for engine in engines() {
            let (mut oracle, _) = riscv::board("interp", &format!("oracle.{engine}"), seams);
            let (mut under_test, hart) = riscv::board(&engine, &engine, seams);
            let opts = Options::to_guest_seconds(secs).hashing_every(2_000);
            match longrun::lockstep("riscv-virt", &mut oracle, &engine, &mut under_test, &opts) {
                Ok(summary) => eprintln!("riscv-virt engine={engine}: {summary}"),
                Err(d) => panic!("{d}"),
            }
            riscv::assert_the_workload_ran(&hart, &engine, seams);
        }
    }
}

// ---------------------------------------------------------------------------
// the same shape on a third core, with a workload built for its seams
// ---------------------------------------------------------------------------

#[cfg(all(feature = "cpu-x86-lift", feature = "dev-pc"))]
mod x86 {
    use std::sync::Arc;

    use rsemu::core::Captured;
    use rsemu::core::space::{AddressSpace, MemAttrs};
    use rsemu::core::value::Width;
    use rsemu::cpu::x86::prot::{SegReg, Sys, ar, cr0, cr4, efer, sys_type};
    use rsemu::cpu::x86::{Reg, Regs, Variant, X86, flags, isa::seg};
    use rsemu::machine::{Machine, build};

    /// A board built for the engine comparison rather than taken from
    /// `machines/`.
    ///
    /// Every shipped x86 board is a board for software this repository does not
    /// contain — `pc-at` and `q35` want a firmware, `pc64` and `q35-linux` want
    /// a `bzImage` — and all four start in **real mode**, which
    /// `cpu::x86::lift::World::of` refuses by construction. A board that spent
    /// its first hundred thousand instructions there would compare two
    /// interpreters and pass, which is the argument `tests/x86_engines.rs`
    /// already makes for building its own machine.
    ///
    /// This is that machine plus the one thing it deliberately has not got: an
    /// **asynchronous interrupt source**. `pc64`'s own, so the path is the one
    /// a kernel takes — an 8254 counter 0 into a master 8259A into `INTR` —
    /// because the seam under test is a timer edge arriving in the middle of a
    /// chain of translated blocks, and a `Machine` with no device that
    /// interrupts cannot reach it. Nothing else is here: no console, no RTC, no
    /// slave controller, because each would be a second thing being tested and
    /// a bigger chunk to fingerprint every quantum.
    ///
    /// 100 MHz is `pc64`'s processor clock and 105000000/88 Hz is the 8254's
    /// real input (14.31818 MHz / 12), so a count in [`TIMER_COUNT`] means on
    /// this board what it would mean on that one.
    ///
    /// Four mebibytes of RAM, which is the smallest power of two above the
    /// guest's top address (`PT0` ends at 0x304000): the full state hash walks
    /// all of it and this harness takes one every two thousand quanta.
    pub(crate) const SOURCE: &str = r#"
machine "x86-longrun" {
  param engine = "interp"
  param ram = 4M

  osc cpu = 100000000 Hz
  osc pit = 105000000/88 Hz

  space mem  { width = 64, unassigned = read-as-ones }
  space port { width = 16, unassigned = read-as-ones }

  object cpu0 "cpu.x86" {
    clock   = cpu
    space   = mem
    iospace = "port"
    variant = "x86-64"
    engine  = engine
  }

  object dram "ram" { size = ram }

  object pic1 "pc.pic" { mode = "master" }
  object pit0 "pc.pit" { clock = pit }

  map mem  0x00000000 size ram    = dram
  map port 0x0020 size 0x0002 = pic1.regs
  map port 0x0040 size 0x0004 = pit0.regs

  wire pit0.out0 -> pic1.ir0
  wire pic1.int  -> cpu0.intr
}
"#;

    /// The global descriptor table. Three entries: null, a 64-bit code segment
    /// at `0x08` and a data segment at `0x10`.
    ///
    /// It has to be real, unlike `tests/x86_engines.rs`'s, because an interrupt
    /// in long mode **loads `CS` from the gate's selector** — the one thing on
    /// this board that reads a descriptor out of memory.
    const GDT: u64 = 0x0800;
    /// The interrupt descriptor table: 256 sixteen-byte gates.
    const IDT: u64 = 0x1000;
    /// The top of the stack the interrupt frame is pushed on. Its page is
    /// distinct from the code page, so the frame never dirties a translation.
    const STACK_TOP: u64 = 0x8000;
    /// Where [`SOFT`] counts itself. Read back by
    /// [`assert_the_workload_ran`], because a software interrupt that never
    /// happened would otherwise be invisible.
    const MARK: u64 = 0x9000;
    /// The main loop's page — also the page [`MAIN`] hands to `INVLPG` and the
    /// page it writes into.
    const CODE: u64 = 0xa000;
    /// The IRQ 0 handler.
    const IRQ_HANDLER: u64 = 0xb000;
    /// The `INT 0x30` handler.
    const SOFT_HANDLER: u64 = 0xb100;
    /// The base of the 64-page data window the loop walks.
    const DATA: u64 = 0x10_0000;

    /// The four levels of the identity map. They sit in the 2-4 MiB range,
    /// which [`PD`] entry 1 covers with a single large page, so the tables that
    /// describe the first two mebibytes are not themselves described by
    /// [`PT0`].
    const PML4: u64 = 0x30_0000;
    const PDPT: u64 = 0x30_1000;
    const PD: u64 = 0x30_2000;
    /// The one page table, mapping 0-2 MiB in **4 KiB pages**.
    ///
    /// Deliberately not a large page like the rest: `INVLPG` has to be able to
    /// throw away the code page's translation on its own, and a walk that
    /// re-establishes it must be a real four-level one. `tests/x86_engines.rs`
    /// maps everything with two 2 MiB pages because nothing there invalidates.
    const PT0: u64 = 0x30_3000;

    /// The 8259A vector IRQ 0 is remapped to. Not the reset default of 8,
    /// which collides with `#DF`.
    const IRQ0_VECTOR: u64 = 0x20;
    /// The vector [`MAIN`]'s `INT` immediate names.
    const SOFT_VECTOR: u64 = 0x30;

    const CODE_SEL: u16 = 0x08;
    const DATA_SEL: u16 = 0x10;

    /// The 8254 count [`MAIN`] programs counter 0 with.
    ///
    /// 100 ticks of a 1.193 MHz input is 83.8 µs, against a quantum of at most
    /// 10 000 ticks of a 100 MHz processor — 100 µs. So the timer edge lands
    /// **inside** a quantum rather than on its boundary, on nearly every
    /// quantum, which is the whole point: a boundary edge would be taken at the
    /// same instruction by any engine.
    const TIMER_COUNT: u16 = 100;

    /// The main loop, at [`CODE`].
    ///
    /// Hand-assembled with `nasm -f bin`; the listing is in the comments, and
    /// the source it was assembled from is reproduced here so the bytes can be
    /// regenerated. Encodings and semantics: *Intel 64 and IA-32 Architectures
    /// Software Developer's Manual*, volume 2 (the instruction set) and volume
    /// 3A §6.14 (long-mode interrupt delivery) and §4.5 (IA-32e paging).
    ///
    /// It is written around what `cpu::x86::engine` actually does, seam by
    /// seam, because the honest finding of the last round is that a plain loop
    /// is a regression test and not a discovery instrument:
    ///
    /// | seam | what reaches it |
    /// | --- | --- |
    /// | `MAX_INSNS` = 32 and `CHAIN` = 16 | twenty-four consecutive lifted instructions in the middle of the loop, so one `advance` is a chain rather than a block |
    /// | `FLAGS = Flags::Eager` | every one of those writes flags and five read them back (`ADC`, `SBB`, `SETB`, `CMOVZ`, and the `Jcc` chain below), so a quantum that ends mid-block ends where a flag is live |
    /// | `IrHost::spent` | the same run of instructions has no store in it, so nothing but the allowance can end a quantum inside it |
    /// | `admit`'s exclusion list | `PUSHFQ`/`POPFQ` every eighth pass and `CLI`/`STI` every thirty-second — `STI` leaves the interrupt shadow `admit` refuses on, and `PUSHFQ` observes the packed `EFLAGS` a block never assembles |
    /// | `Smc::EndBlock` | every sixteenth pass the loop **rewrites its own immediate**, from a `MOV` two hundred bytes further down the same page |
    /// | the entry translation in `admit` | every sixty-fourth pass, `INVLPG` on this very page, so the next `admit` charges a cold four-level walk — the x86 analogue of the `TLBI` that found the A64 defect |
    /// | interrupt delivery from inside a chain | the 8254 above, firing roughly once a quantum |
    /// | a synchronous entry from inside a chain | `INT 0x30` every two hundred and fifty-sixth pass |
    /// | the data-side walk, its accessed and dirty bits | a store and a load to a different 4 KiB page every pass, sixty-four of them |
    ///
    /// ```text
    ///         mov     ebx, 0x100000           ; the data window, 64 pages
    ///         mov     ebp, 0xa000             ; this code page, for INVLPG
    ///         mov     edi, patch              ; the immediate the loop rewrites
    ///         mov     esp, 0x8000
    ///         xor     ecx, ecx
    ///         xor     esi, esi
    ///         ; master 8259A: ICW1-ICW4, then a mask leaving only IR0 open
    ///         mov al,0x11 / out 0x20,al
    ///         mov al,0x20 / out 0x21,al       ; IRQ0 -> vector 0x20
    ///         mov al,0x04 / out 0x21,al
    ///         mov al,0x01 / out 0x21,al
    ///         mov al,0xfe / out 0x21,al
    ///         ; 8254 counter 0: mode 2, lobyte/hibyte, binary
    ///         mov al,0x34 / out 0x43,al
    ///         mov al,100  / out 0x40,al
    ///         mov al,0    / out 0x40,al
    ///         sti
    /// top:    inc     rsi
    ///         mov     edx, esi
    ///         and     edx, 0x3f
    ///         shl     edx, 12
    ///         add     rdx, rbx
    ///         mov     [rdx], rcx              ; a different page every pass
    ///         mov     r8, [rdx]
    /// patched:add     ecx, 1                  ; the immediate the guest rewrites
    ///         add r8,rcx / adc r9,r8 / sbb r10,r9 / xor r11,r10 / add r12,r11
    ///         rol r12,1 / add r13,1 / xor r14,r13 / rol r14,1 / add r8,r14
    ///         sar r9,3 / shl r10,2 / setb al / movzx eax,al / add rcx,rax
    ///         cmp rcx,r11 / cmovz rcx,r12 / imul rdx,r13,3 / add rcx,rdx
    ///         neg r11 / not r10 / dec r12 / sub rcx,r8 / inc r13
    ///         test    esi, 7
    ///         jnz     .no_flags
    ///         pushfq / popfq                  ; outside the lifted subset
    /// .no_flags:
    ///         test    esi, 0xf
    ///         jnz     .no_smc
    ///         mov al,[rdi] / xor al,3 / mov [rdi],al   ; self-modifying
    /// .no_smc:
    ///         test    esi, 0x1f
    ///         jnz     .no_cli
    ///         cli / inc rcx / sti             ; STI leaves an interrupt shadow
    /// .no_cli:
    ///         test    esi, 0x3f
    ///         jnz     .no_invlpg
    ///         invlpg  [rbp]                   ; this page's own translation
    /// .no_invlpg:
    ///         test    esi, 0xff
    ///         jnz     .no_int
    ///         int     0x30
    /// .no_int:
    ///         jmp     top
    /// patch   equ     patched + 2
    /// ```
    const MAIN: [u8; 221] = [
        0xbb, 0x00, 0x00, 0x10, 0x00, // mov ebx, 0x100000
        0xbd, 0x00, 0xa0, 0x00, 0x00, // mov ebp, 0xa000
        0xbf, 0x4f, 0xa0, 0x00, 0x00, // mov edi, patch    (0xa04f)
        0xbc, 0x00, 0x80, 0x00, 0x00, // mov esp, 0x8000
        0x31, 0xc9, // xor ecx, ecx
        0x31, 0xf6, // xor esi, esi
        0xb0, 0x11, // mov al, 0x11
        0xe6, 0x20, // out 0x20, al
        0xb0, 0x20, // mov al, 0x20
        0xe6, 0x21, // out 0x21, al
        0xb0, 0x04, // mov al, 0x04
        0xe6, 0x21, // out 0x21, al
        0xb0, 0x01, // mov al, 0x01
        0xe6, 0x21, // out 0x21, al
        0xb0, 0xfe, // mov al, 0xfe      <- MASK_IMM
        0xe6, 0x21, // out 0x21, al
        0xb0, 0x34, // mov al, 0x34
        0xe6, 0x43, // out 0x43, al
        0xb0, 0x64, // mov al, 100
        0xe6, 0x40, // out 0x40, al
        0xb0, 0x00, // mov al, 0
        0xe6, 0x40, // out 0x40, al
        0xfb, // sti
        // top:  (CODE + 0x39)
        0x48, 0xff, 0xc6, // inc rsi
        0x89, 0xf2, // mov edx, esi
        0x83, 0xe2, 0x3f, // and edx, 0x3f
        0xc1, 0xe2, 0x0c, // shl edx, 12
        0x48, 0x01, 0xda, // add rdx, rbx
        0x48, 0x89, 0x0a, // mov [rdx], rcx
        0x4c, 0x8b, 0x02, // mov r8, [rdx]
        0x83, 0xc1, 0x01, // add ecx, 1        <- the patched immediate
        0x49, 0x01, 0xc8, // add r8, rcx
        0x4d, 0x11, 0xc1, // adc r9, r8
        0x4d, 0x19, 0xca, // sbb r10, r9
        0x4d, 0x31, 0xd3, // xor r11, r10
        0x4d, 0x01, 0xdc, // add r12, r11
        0x49, 0xd1, 0xc4, // rol r12, 1
        0x49, 0x83, 0xc5, 0x01, // add r13, 1
        0x4d, 0x31, 0xee, // xor r14, r13
        0x49, 0xd1, 0xc6, // rol r14, 1
        0x4d, 0x01, 0xf0, // add r8, r14
        0x49, 0xc1, 0xf9, 0x03, // sar r9, 3
        0x49, 0xc1, 0xe2, 0x02, // shl r10, 2
        0x0f, 0x92, 0xc0, // setb al
        0x0f, 0xb6, 0xc0, // movzx eax, al
        0x48, 0x01, 0xc1, // add rcx, rax
        0x4c, 0x39, 0xd9, // cmp rcx, r11
        0x49, 0x0f, 0x44, 0xcc, // cmovz rcx, r12
        0x49, 0x6b, 0xd5, 0x03, // imul rdx, r13, 3
        0x48, 0x01, 0xd1, // add rcx, rdx
        0x49, 0xf7, 0xdb, // neg r11
        0x49, 0xf7, 0xd2, // not r10
        0x49, 0xff, 0xcc, // dec r12
        0x4c, 0x29, 0xc1, // sub rcx, r8
        0x49, 0xff, 0xc5, // inc r13
        0xf7, 0xc6, 0x07, 0x00, 0x00, 0x00, // test esi, 7
        0x75, 0x02, // jnz .no_flags
        0x9c, // pushfq            <- FLAGS_BODY
        0x9d, // popfq
        0xf7, 0xc6, 0x0f, 0x00, 0x00, 0x00, // test esi, 0xf
        0x75, 0x06, // jnz .no_smc
        0x8a, 0x07, // mov al, [rdi]     <- SMC_BODY
        0x34, 0x03, // xor al, 3
        0x88, 0x07, // mov [rdi], al
        0xf7, 0xc6, 0x1f, 0x00, 0x00, 0x00, // test esi, 0x1f
        0x75, 0x05, // jnz .no_cli
        0xfa, // cli               <- SHADOW_BODY
        0x48, 0xff, 0xc1, // inc rcx
        0xfb, // sti
        0xf7, 0xc6, 0x3f, 0x00, 0x00, 0x00, // test esi, 0x3f
        0x75, 0x04, // jnz .no_invlpg
        0x0f, 0x01, 0x7d, 0x00, // invlpg [rbp]      <- INVLPG_BODY
        0xf7, 0xc6, 0xff, 0x00, 0x00, 0x00, // test esi, 0xff
        0x75, 0x02, // jnz .no_int
        0xcd, 0x30, // int 0x30          <- SOFTINT_BODY
        0xe9, 0x5c, 0xff, 0xff, 0xff, // jmp top
    ];

    /// The IRQ 0 handler, at [`IRQ_HANDLER`].
    ///
    /// `OUT` and `IRETQ` are both outside the lifted subset, so the handler is
    /// a second source of declined boundaries in its own right. `RAX` is saved
    /// and restored so a divergence in the main loop's arithmetic is the main
    /// loop's, rather than an interrupt landing one instruction later leaving a
    /// different `AL` behind.
    ///
    /// ```text
    ///         push    rax
    ///         mov     al, 0x20
    ///         out     0x20, al        ; EOI to the master 8259A
    ///         pop     rax
    ///         inc     r15             ; how many fired, for the assertion
    ///         iretq
    /// ```
    const IRQ: [u8; 11] = [
        0x50, // push rax
        0xb0, 0x20, // mov al, 0x20
        0xe6, 0x20, // out 0x20, al
        0x58, // pop rax
        0x49, 0xff, 0xc7, // inc r15
        0x48, 0xcf, // iretq
    ];

    /// The `INT 0x30` handler, at [`SOFT_HANDLER`].
    ///
    /// It counts into [`MARK`] rather than into a register because every
    /// register is already carrying something the main loop reads back, and
    /// because a counter in RAM is covered by the tier the periodic full hash
    /// checks rather than by the per-quantum fingerprint.
    ///
    /// ```text
    ///         inc     qword [0x9000]
    ///         iretq
    /// ```
    const SOFT: [u8; 10] = [
        0x48, 0xff, 0x04, 0x25, 0x00, 0x90, 0x00, 0x00, // inc qword [0x9000]
        0x48, 0xcf, // iretq
    ];

    /// Offsets into [`MAIN`] the bisecting variants blank out.
    ///
    /// A divergence found by the whole workload is a divergence found by
    /// *something* in it, and the first question is always which. The A64 leg
    /// answered that by keeping two hand-written copies of its second page;
    /// one array with a mask byte and five named windows is the same thing
    /// with no second copy to drift.
    const MASK_IMM: usize = 0x29;
    const FLAGS_BODY: (usize, usize) = (0xa5, 0xa7);
    const SMC_BODY: (usize, usize) = (0xaf, 0xb5);
    const SHADOW_BODY: (usize, usize) = (0xbd, 0xc2);
    const INVLPG_BODY: (usize, usize) = (0xca, 0xce);
    const SOFTINT_BODY: (usize, usize) = (0xd6, 0xd8);

    /// Which of the workload's seams this build of the guest reaches.
    ///
    /// Every field off is still a valid guest — the conditional bodies are
    /// replaced by `NOP`s and the branches around them keep their
    /// displacements — so a bisect changes one property at a time and nothing
    /// else. [`Stress::ALL`] is what the committed test runs.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(crate) struct Stress {
        /// Unmask IR0 on the 8259A, so the 8254 can interrupt at all.
        pub(crate) timer: bool,
        /// Keep `INVLPG [rbp]`, which makes this code page's next fetch a cold
        /// four-level walk.
        pub(crate) invlpg: bool,
        /// Keep the three instructions that rewrite the loop's own immediate.
        pub(crate) smc: bool,
        /// Keep `CLI`/`STI`, and with it the interrupt shadow `admit` refuses.
        pub(crate) shadow: bool,
        /// Keep `PUSHFQ`/`POPFQ`, which are outside the lifted subset.
        pub(crate) packed_flags: bool,
        /// Keep `INT 0x30`.
        pub(crate) soft_int: bool,
    }

    impl Stress {
        /// Everything on: the committed workload.
        pub(crate) const ALL: Stress = Stress {
            timer: true,
            invlpg: true,
            smc: true,
            shadow: true,
            packed_flags: true,
            soft_int: true,
        };

        /// The seams named in `keep`, and nothing else.
        ///
        /// This is the bisecting knob, and it is the reason the workload is one
        /// array with named windows rather than the two hand-written copies of
        /// a page the A64 leg keeps: *"the same workload with either one alone
        /// agrees for six thousand quanta"* is the sentence that took that
        /// defect from a hash mismatch to a named function, and producing it
        /// should not need an edit.
        pub(crate) fn keeping(keep: &str) -> Stress {
            let has = |name: &str| keep.split(',').any(|s| s.trim() == name);
            Stress {
                timer: has("timer"),
                invlpg: has("invlpg"),
                smc: has("smc"),
                shadow: has("shadow"),
                packed_flags: has("flags"),
                soft_int: has("int"),
            }
        }
    }

    /// [`MAIN`] with the seams `stress` turns off replaced by `NOP`s.
    fn program(stress: Stress) -> [u8; MAIN.len()] {
        let mut code = MAIN;
        if !stress.timer {
            // OCW1 = 0xff: every line masked, so the 8254 still counts and
            // still toggles `out0` and the processor never sees it.
            code[MASK_IMM] = 0xff;
        }
        for (keep, (from, to)) in [
            (stress.packed_flags, FLAGS_BODY),
            (stress.smc, SMC_BODY),
            (stress.shadow, SHADOW_BODY),
            (stress.invlpg, INVLPG_BODY),
            (stress.soft_int, SOFTINT_BODY),
        ] {
            if !keep {
                for byte in &mut code[from..to] {
                    *byte = 0x90;
                }
            }
        }
        code
    }

    /// The immediates hand-assembled into [`MAIN`] are the constants this
    /// module names, and the windows [`program`] blanks are the instructions it
    /// says they are.
    ///
    /// A named constant nothing checks drifts from the bytes it describes, and
    /// hand-assembled code is exactly where that goes unnoticed: `MASK_IMM` off
    /// by one would blank the `OUT` rather than the mask and the timer would
    /// still fire, so a bisect would blame the wrong seam. Every number below
    /// is read back out of the encoding.
    #[test]
    fn the_guest_carries_the_addresses_and_the_windows_this_file_names() {
        let imm32 = |at: usize| {
            u64::from(u32::from_le_bytes(
                MAIN[at..at + 4].try_into().expect("four bytes"),
            ))
        };
        assert_eq!(imm32(1), DATA, "mov ebx, <the data window>");
        assert_eq!(imm32(6), CODE, "mov ebp, <this code page>");
        assert_eq!(imm32(11), CODE + 0x4f, "mov edi, <the patched immediate>");
        assert_eq!(imm32(16), STACK_TOP, "mov esp, <the top of the stack>");
        assert_eq!(u64::from(MAIN[0x1d]), IRQ0_VECTOR, "ICW2, the vector base");
        assert_eq!(
            u16::from(MAIN[0x31]) | (u16::from(MAIN[0x35]) << 8),
            TIMER_COUNT,
            "the 8254 count, low byte then high"
        );
        assert_eq!(MAIN[0x4f], 1, "the rewritten immediate starts at one");
        assert_eq!(u64::from(MAIN[0xd7]), SOFT_VECTOR, "the INT immediate");
        assert_eq!(MAIN[MASK_IMM], 0xfe, "OCW1, with IR0 the only line open");
        for (name, (from, to), want) in [
            ("pushfq/popfq", FLAGS_BODY, &[0x9c, 0x9d][..]),
            (
                "the code rewrite",
                SMC_BODY,
                &[0x8a, 0x07, 0x34, 0x03, 0x88, 0x07],
            ),
            ("cli/inc/sti", SHADOW_BODY, &[0xfa, 0x48, 0xff, 0xc1, 0xfb]),
            ("invlpg [rbp]", INVLPG_BODY, &[0x0f, 0x01, 0x7d, 0x00]),
            ("int 0x30", SOFTINT_BODY, &[0xcd, 0x30]),
        ] {
            assert_eq!(&MAIN[from..to], want, "the window for {name} moved");
        }
        // The two handlers, likewise: `SOFT` carries `MARK` as an absolute
        // address, and nothing else in this file would notice if it did not.
        assert_eq!(
            imm32_of(&SOFT, 4),
            MARK,
            "inc qword [<the software-interrupt counter>]"
        );
    }

    /// A little-endian doubleword out of a hand-assembled blob.
    fn imm32_of(bytes: &[u8], at: usize) -> u64 {
        u64::from(u32::from_le_bytes(
            bytes[at..at + 4].try_into().expect("four bytes"),
        ))
    }

    /// One descriptor, as the two doublewords a table holds, packed into the
    /// quadword they are written as.
    ///
    /// A limit above 1 MiB is expressed in pages and the architecture rounds up
    /// to the page containing it, which is why `0xffff_ffff` and `0xffff_f000`
    /// produce the same descriptor. *SDM* volume 3A §3.4.5.
    fn descriptor(base: u64, limit: u32, rights: u32) -> u64 {
        let (limit, rights) = if limit > 0xf_ffff {
            (limit >> 12, rights | ar::GRANULAR)
        } else {
            (limit, rights)
        };
        let base = base as u32;
        let low = (limit & 0xffff) | (base << 16);
        let high = ((base >> 16) & 0xff) | rights | (limit & 0x000f_0000) | (base & 0xff00_0000);
        (u64::from(high) << 32) | u64::from(low)
    }

    /// Build the board on `engine`, place the core in long mode over a
    /// four-level identity map, and load the guest.
    ///
    /// `build` realizes and resets, and a cold reset zeroes RAM, so everything
    /// written into memory happens afterwards — the order
    /// `tests/x86_engines.rs` documents and for the same reason.
    pub(crate) fn board(engine: &str, tag: &str, stress: Stress) -> (Machine, Arc<X86>) {
        let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
        let kept = Arc::clone(&cpus);
        let mut bindings = rsemu::machine::catalog::bindings().expect("this build's bindings");
        bindings.replace("cpu.x86", move |props| {
            let cpu = Arc::new(X86::from_props_defaulting(props, Variant::X86_64)?);
            kept.push(&cpu);
            Ok(cpu)
        });
        let options = rsemu::machine::BuildOptions::new()
            .with_classes(rsemu::machine::catalog::classes())
            .with_bindings(bindings)
            .with_param("engine", engine);
        let registry = rsemu::machine::catalog::registry().expect("this build's registry");
        let machine = build(&format!("x86-longrun.{tag}"), SOURCE, &registry, &options)
            .unwrap_or_else(|e| panic!("the board does not build with engine={engine}: {e}"));
        let cpu = cpus.take().expect("the binding captured the core");

        // One step discharges the reset sequence, which is what clears
        // `reset_pending`; without it the first quantum would run the sequence
        // and throw away the `CS:RIP` written below.
        cpu.step();
        assert!(!cpu.reset_requested(), "the reset sequence did not run");

        let space = cpu.space().expect("the core has its space");
        let put = |addr: u64, width: Width, value: u64| {
            space
                .write(addr, width, value, MemAttrs::DEFAULT)
                .expect("inside RAM");
        };
        for (at, bytes) in [
            (CODE, &program(stress)[..]),
            (IRQ_HANDLER, &IRQ[..]),
            (SOFT_HANDLER, &SOFT[..]),
        ] {
            for (n, byte) in bytes.iter().enumerate() {
                put(at + n as u64, Width::U8, u64::from(*byte));
            }
        }

        // The descriptor table. Entry 1 is the 64-bit code segment every gate
        // in the table below names; entry 2 is what `DS`, `ES`, `SS`, `FS` and
        // `GS` are loaded with. Long mode ignores their bases and limits, but
        // `SS` has to be a writable data segment for `IRETQ` to accept it back.
        put(GDT, Width::U64, 0);
        put(
            GDT + 8,
            Width::U64,
            descriptor(
                0,
                0xffff_ffff,
                ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::L,
            ),
        );
        put(
            GDT + 16,
            Width::U64,
            descriptor(
                0,
                0xffff_ffff,
                ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::DB,
            ),
        );

        // Two sixteen-byte interrupt gates. Bytes 0-1 and 6-7 are the offset's
        // low thirty-two bits split around the selector and the access byte,
        // exactly as a 32-bit gate does it; bytes 8-11 are the offset's top
        // half and 12-15 are reserved. *SDM* volume 3A §6.14.1.
        for (vector, target) in [(IRQ0_VECTOR, IRQ_HANDLER), (SOFT_VECTOR, SOFT_HANDLER)] {
            let at = IDT + vector * 16;
            let low = (target as u32 & 0xffff) | (u32::from(CODE_SEL) << 16);
            let high = (target as u32 & 0xffff_0000)
                | ar::PRESENT
                | (u32::from(sys_type::INT_GATE32) << 8);
            put(at, Width::U32, u64::from(low));
            put(at + 4, Width::U32, u64::from(high));
            put(at + 8, Width::U32, target >> 32);
            put(at + 12, Width::U32, 0);
        }

        map_identity(&space);
        cpu.set_sys(system());

        let mut regs = Regs::new();
        regs.cs = CODE_SEL;
        for sr in [seg::SS, seg::DS, seg::ES, seg::FS, seg::GS] {
            regs.set_segment(sr, DATA_SEL);
        }
        regs.rip = CODE;
        // `IF` clear: the guest sets it itself, after it has programmed the
        // two chips. An interrupt before that would vector through a gate the
        // 8259A had not been told about.
        regs.eflags = flags::ALWAYS_SET;
        cpu.set_regs(regs);

        (machine, cpu)
    }

    /// The system registers for a long-mode guest over a four-level map.
    ///
    /// The *SDM*'s bring-up order minus the instructions that would perform it
    /// — `CR4.PAE`, `CR3`, `EFER.LME`, `CR0.PG`, at which point the processor
    /// sets `EFER.LMA` — because this builds the state rather than reaching it.
    /// `cpu::x86::tests` is where the transition is executed as real
    /// instructions.
    fn system() -> Sys {
        let mut sys = Sys::reset();
        sys.cr0 |= cr0::PE;
        sys.gdtr.base = GDT;
        sys.gdtr.limit = 0x17;
        sys.idtr.base = IDT;
        sys.idtr.limit = 0xfff;
        sys.segs[usize::from(seg::CS)] = SegReg {
            selector: CODE_SEL,
            base: 0,
            limit: 0xffff_ffff,
            ar: ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::L | ar::GRANULAR,
        };
        for index in [seg::DS, seg::ES, seg::SS, seg::FS, seg::GS] {
            sys.segs[usize::from(index)] = SegReg {
                selector: DATA_SEL,
                base: 0,
                limit: 0xffff_ffff,
                ar: ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::DB | ar::GRANULAR,
            };
        }
        sys.cr4 |= cr4::PAE;
        sys.cr3 = PML4;
        sys.efer |= efer::LME | efer::LMA;
        sys.cr0 |= cr0::PG;
        sys
    }

    /// Identity-map the first four mebibytes: 0-2 MiB in **4 KiB** pages, and
    /// 2-4 MiB as one large page.
    ///
    /// The split is the point. Everything the guest executes, stores to and
    /// pushes on is in the 4 KiB half, so `INVLPG` on the code page discards
    /// exactly one translation and the walk that replaces it reads four levels;
    /// the tables themselves are in the large half, where nothing invalidates
    /// them. *SDM* volume 3A §4.5.
    fn map_identity(space: &Arc<AddressSpace>) {
        /// Present and writable. No `U/S`: this guest never leaves ring 0.
        const PRESENT_RW: u64 = 0b11;
        /// `PS`, which makes a directory entry a 2 MiB page.
        const LARGE: u64 = 1 << 7;
        let put = |at: u64, value: u64| {
            space
                .write(at, Width::U64, value, MemAttrs::DEFAULT)
                .expect("the tables fit in RAM");
        };
        put(PML4, PDPT | PRESENT_RW);
        put(PDPT, PD | PRESENT_RW);
        put(PD, PT0 | PRESENT_RW);
        put(PD + 8, 0x20_0000 | LARGE | PRESENT_RW);
        for page in 0..512u64 {
            put(PT0 + 8 * page, (page << 12) | PRESENT_RW);
        }
    }

    /// Assert the guest actually did what it was written to do.
    ///
    /// Without this the run could be green because the core wedged on the
    /// first instruction: a comparison of two stopped machines agrees at every
    /// checkpoint. Each of these is a property the workload exists for, and
    /// each of them has failed at least once while this file was being written.
    pub(crate) fn assert_the_workload_ran(cpu: &X86, engine: &str, stress: Stress) {
        let space = cpu.space().expect("the core has its space");
        // `RSEMU_LONGRUN_ENGINES=interp` is the control leg — an interpreter
        // against itself — and an interpreted core has no translation
        // statistics to assert. Everything below still applies to that run.
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
            if stress.smc {
                assert!(
                    stats.invalidated > 0,
                    "engine={engine}: the guest rewrote its own code and not \
                     one translation was thrown away, so the self-modifying \
                     path is not being exercised"
                );
            }
        }
        assert!(
            cpu.reg(Reg::Rsi) > 1_000,
            "engine={engine}: the loop went round {} times, so the guest is \
             not running the workload at all",
            cpu.reg(Reg::Rsi)
        );
        if stress.timer {
            assert!(
                cpu.reg(Reg::R15) > 0,
                "engine={engine}: the 8254 never interrupted, so the run says \
                 nothing about where a translated block notices one"
            );
        }
        if stress.soft_int {
            let marks = space
                .read(MARK, Width::U64, MemAttrs::DEFAULT)
                .expect("inside RAM");
            assert!(
                marks > 0,
                "engine={engine}: `INT 0x30` never vectored, so the gate is \
                 wrong and a whole seam is unexercised"
            );
        }
    }
}

#[cfg(all(feature = "cpu-x86-lift", feature = "dev-pc"))]
mod x86_tests {
    use super::longrun::{self, Options};
    use super::x86::{self, Stress};

    /// What the per-commit run costs, and why it is a count of quanta rather
    /// than the guest seconds the other legs use.
    ///
    /// One guest second of this board is 24 818 quanta, 192 618 passes round
    /// the loop and about three seconds of wall time **per engine** — twice
    /// what the rest of this file costs put together — and effectively all of
    /// it is repetition: the workload reaches every seam it was written for
    /// inside the first two hundred quanta. Six thousand is 0.24 s of guest
    /// time and roughly 46 000 passes, which is still 2 900 code rewrites, 700
    /// `INVLPG`s, 2 800 timer interrupts and 180 software ones.
    ///
    /// `RSEMU_LONGRUN_SECONDS` removes the cap and asks for guest seconds
    /// instead, which is what `scripts/check.sh long` and the nightly do.
    const DEFAULT_QUANTA: u64 = 6_000;

    /// How far to run, and how often to take the full hash.
    fn budget() -> Options {
        match std::env::var("RSEMU_LONGRUN_SECONDS")
            .ok()
            .and_then(|v| v.parse::<u64>().ok())
        {
            Some(secs) => Options::to_guest_seconds(secs),
            None => Options::to_guest_seconds(u64::MAX).at_most(DEFAULT_QUANTA),
        }
        .hashing_every(2_000)
    }

    fn engines() -> Vec<String> {
        std::env::var("RSEMU_LONGRUN_ENGINES")
            .unwrap_or_else(|_| "jit,jit-host".to_string())
            .split(',')
            .filter(|s| !s.is_empty())
            .map(str::to_string)
            .collect()
    }

    /// Run the workload under every engine and compare, failing on the first
    /// quantum anything parts.
    pub(crate) fn run(tag: &str, stress: Stress, opts: &Options) {
        for engine in engines() {
            let (mut oracle, _) = x86::board("interp", &format!("{tag}.oracle.{engine}"), stress);
            let (mut under_test, cpu) = x86::board(&engine, &format!("{tag}.{engine}"), stress);
            match longrun::lockstep(tag, &mut oracle, &engine, &mut under_test, opts) {
                Ok(summary) => eprintln!("{tag} engine={engine}: {summary}"),
                Err(d) => panic!("{d}"),
            }
            x86::assert_the_workload_ran(&cpu, &engine, stress);
        }
    }

    /// Which seams this run keeps.
    ///
    /// Everything, unless `RSEMU_X86_LONGRUN_SEAMS` names a subset —
    /// `timer,invlpg,smc,shadow,flags,int` — which is what a bisect turns off
    /// one at a time once this test has failed. See [`Stress::keeping`].
    fn stress() -> Stress {
        match std::env::var("RSEMU_X86_LONGRUN_SEAMS") {
            Ok(keep) => {
                let stress = Stress::keeping(&keep);
                eprintln!("x86-longrun: RSEMU_X86_LONGRUN_SEAMS={keep} -> {stress:?}");
                stress
            }
            Err(_) => Stress::ALL,
        }
    }

    #[test]
    fn a_synthetic_x86_workload_agrees_across_the_engines() {
        run("x86-longrun", stress(), &budget());
    }
}

// ---------------------------------------------------------------------------
// the gate on a third core: a real x86-64 kernel
// ---------------------------------------------------------------------------

#[cfg(all(feature = "machine-pc64", feature = "cpu-x86-lift"))]
mod pc64 {
    use std::sync::Arc;

    use super::longrun::{self, Options};
    use rsemu::core::Captured;
    use rsemu::cpu::x86::{Variant, X86};
    use rsemu::host::chardev::CharPort;
    use rsemu::machine::{Machine, catalog};

    /// Build `pc64` on `engine` with `kernel` and `initrd` in its slots, and
    /// hand back its processor and the host end of its console.
    ///
    /// `console` is per-machine, because two of these run in one process and
    /// must not type at each other. There is no `power` parameter on this
    /// board — it has no system-control device — so there is nothing else to
    /// keep apart.
    ///
    /// `nokaslr` is not optional and `machines/pc64.machine` says why at
    /// length: this board has no firmware, so nothing has ever loaded a count
    /// into the 8254, and a kernel that draws randomisation entropy from the
    /// read-back command's null-count bit spins in the decompressor forever.
    /// `cryptomgr.notests` is what keeps a boot to a few hundred guest seconds
    /// rather than a few thousand.
    fn board(
        engine: &str,
        tag: &str,
        kernel: &[u8],
        initrd: &[u8],
    ) -> (Machine, Arc<X86>, Arc<CharPort>) {
        let port = format!("longrun.{tag}");
        let entry = catalog::machine("pc64").expect("this build ships it");
        // The catalog's own binding, with a hand kept on what it builds: a
        // translated run has to be able to say how many blocks it executed,
        // and `Machine` deliberately hands out a `dyn Device`.
        let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
        let kept = Arc::clone(&cpus);
        let mut bindings = catalog::bindings().expect("this build's bindings");
        bindings.replace("cpu.x86", move |props| {
            let cpu = Arc::new(X86::from_props_defaulting(props, Variant::X86_64)?);
            kept.push(&cpu);
            Ok(cpu)
        });
        let options = catalog::build_options()
            .expect("the catalog agrees with itself")
            .with_bindings(bindings)
            .with_media("kernel", kernel)
            .with_media("initrd", initrd)
            .with_param("engine", engine)
            .with_param(
                "extmem",
                std::env::var("RSEMU_X86_RAM").unwrap_or_else(|_| "256M".to_string()),
            )
            .with_param(
                "cmdline",
                std::env::var("RSEMU_X86_CMDLINE").unwrap_or_else(|_| {
                    "console=ttyS0,115200 earlyprintk=ttyS0,115200 nokaslr cryptomgr.notests"
                        .to_string()
                }),
            )
            .with_param("console", port.clone());
        let registry = catalog::registry().expect("a registry");
        let machine = rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .unwrap_or_else(|e| panic!("pc64 does not build with engine={engine}: {e}"));
        let console = rsemu::host::chardev::ports::open(&options.realize.hosts, &port)
            .expect("the 16550 opened this port under the same name");
        let cpu = cpus.take().expect("the binding captured the core");
        (machine, cpu, console)
    }

    /// The x86 half of the kernel gate: a stock `bzImage`, both engines,
    /// quantum by quantum.
    ///
    /// # Why this exists rather than the synthetic x86 leg alone
    ///
    /// The synthetic leg above is written seam by seam off `cpu::x86::engine`,
    /// and the honest limit of that is written down beside it: **it exercises
    /// what its author read off `engine.rs`.** The two A64 defects this whole
    /// file exists for were found because a real kernel does things nobody
    /// designed for, and they first appeared at 15.04 s and 23.46 s of boot.
    /// The A64 leg's calibration table has the exact shape of the argument:
    /// the declined-boundary defect needs a **cold instruction-fetch
    /// translation**, and no amount of data-side pressure produces one because
    /// `mmu::Tlb` keeps fetch, load and store entries in separate sets.
    ///
    /// The x86 analogue is the same fact from the other side. The synthetic
    /// guest reaches a cold fetch translation by executing `INVLPG` on its own
    /// code page — a thing its author wrote *because* he had read `admit`, on
    /// a guest whose whole text is one 221-byte page. A kernel gets there
    /// without anybody thinking of it: it maps and unmaps executable pages,
    /// reloads `CR3` on every context switch (which discards the whole
    /// non-global half of the translation caches at once, where `INVLPG`
    /// discards one page), and executes out of several thousand pages rather
    /// than one. It also reaches real mode's exit, the decompressor's
    /// `REP MOVS`, four-level walks over tables it builds itself, `SYSCALL`
    /// and `IRETQ`, and the packed-integer half of SSE2 — none of which the
    /// synthetic guest contains a single instruction of.
    ///
    /// # What it costs
    ///
    /// `docs/testing/long-run.md` has the measured table. `pc64` runs a
    /// 100 MHz processor and a quantum is at most 10 000 of its ticks, so a
    /// guest second is on the order of 10 000 quanta — far fewer than the
    /// synthetic x86 board's 24 818, because that guest leaves a block on
    /// nearly every pass and this one does not.
    ///
    /// # When the fixture is absent
    ///
    /// It skips loudly, printing the two commands that would make it run, the
    /// way the A64 leg does. `RSEMU_LONGRUN_REQUIRED` in `scripts/check.sh`
    /// turns that skip into a failure.
    #[test]
    #[ignore = "needs a fetched kernel (scripts/fetch-testdata.sh x86-linux initramfs-x86) and minutes of wall time"]
    fn a_real_x86_linux_boot_agrees_across_the_engines() {
        let Ok(path) = std::env::var("RSEMU_X86_KERNEL") else {
            eprintln!(
                "\n  SKIPPED: RSEMU_X86_KERNEL is not set, so there is no kernel to boot.\n\
                 \n      scripts/fetch-testdata.sh x86-linux initramfs-x86\n\
                 \n      RSEMU_X86_KERNEL=testdata/x86/bzImage \\\n\
                 \x20     RSEMU_X86_INITRD=testdata/x86/initramfs-x86.cpio \\\n\
                 \x20     RSEMU_LONGRUN_SECONDS=120 \\\n\
                 \x20         cargo test --release --test engine_longrun -- --ignored --nocapture\n\
                 \n  The synthetic x86 leg exercises the seams its author read off\n\
                 engine.rs. This one is the leg that can find something nobody\n\
                 designed for, which is how both A64 defects were found.\n"
            );
            return;
        };
        let kernel = std::fs::read(&path).unwrap_or_else(|e| {
            panic!("RSEMU_X86_KERNEL names `{path}`, which will not read: {e}")
        });
        // An empty value counts as unset, because `scripts/check.sh` passes one
        // when it has no ramdisk to offer — `env VAR=` sets the variable rather
        // than leaving it out, and this board boots without a root anyway (it
        // panics for want of one, which is still a complete boot).
        let initrd = match std::env::var("RSEMU_X86_INITRD") {
            Ok(p) if !p.is_empty() => {
                std::fs::read(&p).unwrap_or_else(|e| panic!("RSEMU_X86_INITRD names `{p}`: {e}"))
            }
            _ => Vec::new(),
        };
        eprintln!(
            "pc64: {} bytes of kernel, {} bytes of initramfs",
            kernel.len(),
            initrd.len()
        );

        let secs: u64 = std::env::var("RSEMU_LONGRUN_SECONDS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(30);
        let engines: Vec<String> = std::env::var("RSEMU_LONGRUN_ENGINES")
            .unwrap_or_else(|_| "jit,jit-host".to_string())
            .split(',')
            .filter(|s| !s.is_empty())
            .map(str::to_string)
            .collect();

        for engine in engines {
            let (mut oracle, _, left) =
                board("interp", &format!("oracle.{engine}"), &kernel, &initrd);
            let (mut under_test, cpu, right) = board(&engine, &engine, &kernel, &initrd);
            // The full hash walks the board's whole extended memory, so it is
            // taken rarely; the per-quantum device fingerprint is what finds a
            // divergence first and costs nothing beside a quantum.
            let opts = Options::to_guest_seconds(secs)
                .hashing_every(20_000)
                .reporting_every(10_000);
            let (mut said, mut heard) = (Vec::new(), Vec::new());
            let outcome = {
                let mut pump = || {
                    left.drain_into(&mut said);
                    right.drain_into(&mut heard);
                };
                longrun::lockstep_pumping(
                    "pc64",
                    &mut oracle,
                    &engine,
                    &mut under_test,
                    &opts,
                    &mut pump,
                )
            };
            match outcome {
                Ok(summary) => eprintln!("pc64 engine={engine}: {summary}"),
                Err(d) => panic!("{d}\n{}", console_report(&said, &heard)),
            }
            assert_the_kernel_ran(&cpu, &engine, &said, &heard);
        }
    }

    /// A boot that agreed at every checkpoint still says nothing if the guest
    /// did not run, and this is the one board in this file where that is a
    /// real possibility rather than a theoretical one: a `bzImage` the loader
    /// mis-parses, a command line without `nokaslr`, or an extended memory too
    /// small all end with a processor stopped early — and two processors
    /// stopped in the same place agree on every hash they are asked for.
    ///
    /// Three claims, and none of them is "the boot got as far as X". How far a
    /// kernel gets is a function of `RSEMU_LONGRUN_SECONDS` and of which
    /// `bzImage` somebody pointed at this, and a test that asserted a
    /// milestone would be asserting the budget:
    ///
    /// * the guest **said something**, so the processor reached the
    ///   decompressor's `earlyprintk` at the very least;
    /// * both machines said the **same bytes** — a fourth tier of comparison
    ///   the lockstep loop cannot make, because a drained console is a host
    ///   object rather than device state and only a byte still sitting in the
    ///   transmitter is in the per-quantum fingerprint;
    /// * and the engine under test **executed translated blocks**, and retired
    ///   more instructions inside them than outside, so what was compared is a
    ///   translated core rather than two interpreters. That one is
    ///   `x86::assert_the_workload_ran`'s first two assertions, and it is the
    ///   one that does not depend on the budget at all.
    fn assert_the_kernel_ran(cpu: &X86, engine: &str, said: &[u8], heard: &[u8]) {
        assert_eq!(
            said,
            heard,
            "engine={engine}: the two machines printed different bytes, which \
             the per-quantum fingerprint cannot see because a drained console \
             is a host object rather than device state\n{}",
            console_report(said, heard)
        );
        assert!(
            !said.is_empty(),
            "engine={engine}: the guest printed nothing at all, so it never \
             reached its own console and this run compared two kernels stopped \
             in the same place"
        );
        // `RSEMU_LONGRUN_ENGINES=interp` is the control leg — an interpreter
        // against itself — and an interpreted core has no statistics.
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
            eprintln!(
                "pc64 engine={engine}: {} blocks, {} instructions retired in \
                 them against {} interpreted, {} translations thrown away",
                stats.blocks, stats.retired, stats.interpreted, stats.invalidated
            );
        }
        let text = String::from_utf8_lossy(said);
        eprintln!(
            "pc64 engine={engine}: {} bytes on the console, identical on both \
             machines{}. The last of it:\n{}",
            said.len(),
            if text.contains("Linux version") {
                ", past `Linux version`"
            } else {
                ", still in the decompressor"
            },
            tail(&text)
        );
    }

    /// What each side printed, for a failure message.
    fn console_report(said: &[u8], heard: &[u8]) -> String {
        let (a, b) = (
            String::from_utf8_lossy(said),
            String::from_utf8_lossy(heard),
        );
        let mut out = format!("    the interpreter's console ({} bytes):\n", said.len());
        out.push_str(&tail(&a));
        if said != heard {
            out.push_str(&format!(
                "\n    the translated engine's console ({} bytes):\n",
                heard.len()
            ));
            out.push_str(&tail(&b));
        }
        out
    }

    /// The last few lines of a console log, indented.
    fn tail(text: &str) -> String {
        text.lines()
            .rev()
            .take(12)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .map(|line| format!("        {line}\n"))
            .collect()
    }
}
