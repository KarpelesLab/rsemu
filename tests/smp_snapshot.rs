//! Does a machine with **several processors** survive a snapshot — and does the
//! restored machine still run the same way? (`ROADMAP.md` §4.5, phase 9's gate.)
//!
//! Every stateful device has a round-trip test, and one two-processor board
//! already had a machine-level one (`tests/pc_at_smp.rs`). Neither shape is the
//! claim phase 9 needs, because **both are blind in exactly the same place**:
//!
//! ```text
//!   save() ── bytes ──► load() ── save() ──► the same bytes
//! ```
//!
//! is a tautology over the fields `save` writes. State that `save` *omits*
//! cannot make that comparison fail, however wrong the restored machine is —
//! and `Machine::state_hash` is a hash of `save` output, so it inherits the
//! blindness. The only thing that sees an omission is **running afterwards**:
//! take a machine that has only ever been reset, hand it the snapshot, run both
//! it and the original forward over the same span, and compare.
//!
//! `tests/frame_hash.rs` does that for the five workload boards, all of them
//! single-processor. This file does it for the four boards in `machines/` that
//! have two processors — `riscv-virt-smp`, `arm64-virt-smp`, `pc-at-smp`,
//! `q35-linux-smp` — and for `pc-at` on its own, as the control.
//!
//! # What it found
//!
//! The control is the point. `pc-at` and `pc-at-smp` both restored to a
//! byte-identical snapshot and then **diverged 200 µs later**, on the VGA
//! adapter's clock domain, by exactly one tick. The cause was not SMP and not
//! the VGA: `ClockForest::restore_ticks` anchored a restored domain's tick
//! counter at the tree position it was restored to, which sets the domain's
//! *sub-tick phase* to zero. A domain divided by *n* whose tree sat part way
//! into a tick came back at the start of one, so it ticked late — once, and
//! then forever, because the anchor stays put. §4.5 names sub-tick phase as the
//! thing a snapshot must not lose; this was the framework losing it on the
//! restore side rather than a device losing it on the save side.
//!
//! Nothing in the tree caught it because the workload boards' snapshot instants
//! happen to be tick-aligned for the domains that matter, and because every
//! other test compares only the two things that cannot see it.

#![cfg(feature = "std")]

use rsemu::core::clock::GlobalTime;
use rsemu::machine::Machine;

/// How far each leg runs. Long enough for a divided domain to tick many times,
/// short enough to keep four boards on every commit.
const SPAN: GlobalTime = GlobalTime::from_nanos(20_000_000);

/// Save `a`, restore into `b`, then run both and require they stay together.
///
/// The second half is what this file exists for; the first half is the check
/// every other snapshot test already makes, kept because when it fails it says
/// something different.
fn round_trip_and_resume(name: &str, mut a: Machine, mut b: Machine) {
    a.run_for(SPAN).expect("the machine runs");
    let saved = a.save().expect("it snapshots");
    b.load(&saved)
        .expect("a freshly built board takes the snapshot");
    assert_eq!(
        saved,
        b.save().expect("and snapshots again"),
        "{name}: the restored board does not re-save what it was given"
    );
    assert_eq!(a.now(), b.now(), "{name}: the restore moved virtual time");

    a.run_for(SPAN).expect("the original runs on");
    b.run_for(SPAN).expect("the restored one runs on");
    assert_eq!(
        a.state_hash().expect("a hash"),
        b.state_hash().expect("a hash"),
        "{name}: the restored board diverged from the one it was saved from \
         within {} ns, so the snapshot is missing state that execution reads",
        SPAN.as_nanos()
    );
}

// ---------------------------------------------------------------------------
// riscv-virt-smp
// ---------------------------------------------------------------------------

#[cfg(feature = "machine-riscv-virt")]
mod riscv {
    use super::*;

    /// The `tests/workload` RV64I loop: it stores, loads and shifts, so both
    /// harts have live registers and touched RAM at any instant.
    fn firmware() -> Vec<u8> {
        const PROGRAM: [u32; 12] = [
            0x0000_0f17,
            0x014f_0f13,
            0x0000_1397,
            0x0000_0293,
            0x0010_0313,
            0x0062_82b3,
            0x0053_b023,
            0x0003_be03,
            0x003e_1e93,
            0x003e_de93,
            0x01d2_82b3,
            0x000f_0067,
        ];
        PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    pub(super) fn board(engine: &str) -> Machine {
        use rsemu::machine::catalog;
        let entry = catalog::machine("riscv-virt-smp").expect("this build ships it");
        let options = catalog::build_options()
            .expect("the catalog agrees with itself")
            .with_media("firmware", firmware())
            .with_media("flash0", Vec::new())
            .with_media("flash1", Vec::new())
            .with_media("disk", Vec::new())
            .with_media("initrd", Vec::new())
            .with_param("ram", "16M".to_string())
            .with_param("engine", engine.to_string());
        let registry = catalog::registry().expect("a registry");
        rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .expect("the two-hart board realizes")
    }

    #[test]
    fn two_harts_round_trip_and_keep_running_the_same() {
        round_trip_and_resume("riscv-virt-smp", board("interp"), board("interp"));
    }

    /// A snapshot taken on one engine, restored onto another, on a **board** —
    /// `tests/riscv_virt_engines.rs` makes this claim for one hart.
    #[cfg(all(feature = "cpu-riscv-lift", feature = "jit"))]
    #[test]
    fn a_snapshot_crosses_engines_on_a_two_hart_board() {
        let mut interp = board("interp");
        interp.run_for(SPAN).expect("the interpreted board runs");
        let saved = interp.save().expect("it snapshots");

        let mut jit = board("jit");
        jit.load(&saved).expect("the compiled board takes it");
        assert_eq!(
            saved,
            jit.save().expect("and snapshots again"),
            "an engine change is not supposed to be visible in a chunk"
        );

        interp.run_for(SPAN).expect("the interpreter runs on");
        jit.run_for(SPAN).expect("the compiler runs on");
        assert_eq!(
            interp.state_hash().expect("a hash"),
            jit.state_hash().expect("a hash"),
            "the two engines parted company after the snapshot crossed between them"
        );

        // And back: a compiled board's snapshot has to be an interpreter's too.
        let saved = jit.save().expect("the compiled board snapshots");
        let mut back = board("interp");
        back.load(&saved).expect("the interpreter takes it");
        back.run_for(SPAN).expect("and runs on");
        interp.run_for(SPAN).expect("as does the original");
        assert_eq!(
            interp.state_hash().expect("a hash"),
            back.state_hash().expect("a hash"),
            "a snapshot taken under the compiler does not resume under the interpreter"
        );
    }
}

// ---------------------------------------------------------------------------
// arm64-virt-smp
// ---------------------------------------------------------------------------

#[cfg(feature = "machine-arm64-virt")]
mod a64 {
    use super::*;
    use rsemu::dev::arm::boot::asm;

    const KERNEL_ADDR: u64 = 0x4008_0000;
    const RELEASE: u64 = 0x4000_1000;
    const SECOND: u64 = 0x4008_1000;
    const COUNTER_A: u64 = 0x4000_1200;
    const COUNTER_B: u64 = 0x4000_1208;

    /// `tests/a64_smp.rs`'s `Image` header, which the board's loader insists on.
    fn image(words: &[u32]) -> Vec<u8> {
        const HEADER: usize = 0x40;
        let mut out = Vec::with_capacity(HEADER + words.len() * 4);
        out.extend_from_slice(&asm::b((HEADER / 4) as i32).to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes());
        out.extend_from_slice(&0u64.to_le_bytes()); // text_offset
        out.extend_from_slice(&((HEADER + words.len() * 4) as u64).to_le_bytes());
        out.extend_from_slice(&0u64.to_le_bytes()); // flags
        for _ in 0..3 {
            out.extend_from_slice(&0u64.to_le_bytes());
        }
        out.extend_from_slice(b"ARM\x64");
        out.extend_from_slice(&0u32.to_le_bytes());
        assert_eq!(out.len(), HEADER);
        for word in words {
            out.extend_from_slice(&word.to_le_bytes());
        }
        out
    }

    /// The boot processor releases the second one, then both count in RAM.
    ///
    /// Two counters rather than one: a snapshot taken at any instant then has
    /// live, *differing* state on each processor, so a restore that mixed the
    /// two up or dropped one would show.
    fn program() -> Vec<u8> {
        let mut words: Vec<u32> = Vec::new();
        words.extend_from_slice(&asm::load64(9, RELEASE + 8));
        words.extend_from_slice(&asm::load64(10, SECOND));
        words.push(asm::str_base(10, 9));
        words.extend_from_slice(&asm::load64(11, COUNTER_A));
        let spin = words.len() as i32;
        words.push(asm::ldr_base(12, 11));
        words.push(asm::add_imm(12, 12, 1));
        words.push(asm::str_base(12, 11));
        words.push(asm::b(spin - words.len() as i32));

        let at = (SECOND - KERNEL_ADDR) as usize / 4;
        assert!(words.len() <= at, "the first half ran into the second");
        words.resize(at, 0);
        words.extend_from_slice(&asm::load64(11, COUNTER_B));
        let spin = words.len() as i32;
        words.push(asm::ldr_base(12, 11));
        words.push(asm::add_imm(12, 12, 1));
        words.push(asm::str_base(12, 11));
        words.push(asm::b(spin - words.len() as i32));
        image(&words)
    }

    fn board(kernel: &[u8], engine: &str) -> Machine {
        use rsemu::machine::catalog;
        let entry = catalog::machine("arm64-virt-smp").expect("this build ships it");
        let options = catalog::build_options()
            .expect("the catalog agrees with itself")
            .with_media("kernel", kernel.to_vec())
            .with_media("initrd", Vec::new())
            .with_media("disk", Vec::new())
            .with_param("ram", "16M".to_string())
            .with_param("engine", engine.to_string());
        let registry = catalog::registry().expect("a registry");
        rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .expect("the two-core board realizes")
    }

    #[test]
    fn two_cores_round_trip_and_keep_running_the_same() {
        let k = program();
        round_trip_and_resume("arm64-virt-smp", board(&k, "interp"), board(&k, "interp"));
    }

    #[cfg(all(feature = "cpu-arm-a64-lift", feature = "jit"))]
    #[test]
    fn a_snapshot_crosses_engines_on_a_two_core_board() {
        let k = program();
        let mut interp = board(&k, "interp");
        interp.run_for(SPAN).expect("the interpreted board runs");
        let saved = interp.save().expect("it snapshots");

        let mut jit = board(&k, "jit");
        jit.load(&saved).expect("the compiled board takes it");
        assert_eq!(
            saved,
            jit.save().expect("and snapshots again"),
            "an engine change is not supposed to be visible in a chunk"
        );

        interp.run_for(SPAN).expect("the interpreter runs on");
        jit.run_for(SPAN).expect("the compiler runs on");
        assert_eq!(
            interp.state_hash().expect("a hash"),
            jit.state_hash().expect("a hash"),
            "the two engines parted company after the snapshot crossed between them"
        );
    }
}

// ---------------------------------------------------------------------------
// pc-at and pc-at-smp
// ---------------------------------------------------------------------------

#[cfg(all(feature = "machine-pc-at", feature = "fw-pcbios"))]
mod pcat {
    use super::*;
    use rsemu::machine::BuildOptions;

    fn options(file: &str, text: &str) -> BuildOptions {
        use rsemu::machine::catalog;
        let mut options = catalog::build_options().expect("this build's classes");
        options.realize.media.insert(
            "bios",
            rsemu::fw::pcbios::image_for_machine(file, text).expect("the board resolves"),
        );
        options.realize.media.insert("vgabios", Vec::new());
        options.realize.media.insert("optionrom", vec![0u8; 65536]);
        for slot in ["floppy", "disk", "hd0", "hd1", "hd2", "hd3", "cd0", "cd1"] {
            options.realize.media.insert(slot, Vec::new());
        }
        options
    }

    fn board(name: &str) -> Machine {
        use rsemu::machine::catalog;
        let entry = catalog::machine(name).expect("this build ships the board");
        let file = format!("{name}.machine");
        let options = options(&file, entry.source);
        let registry = catalog::registry().expect("this build's registry");
        rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .expect("the board realizes")
    }

    /// The control: the divergence this file was written to find was **not**
    /// about having two processors, and one processor is what proves it.
    #[test]
    fn one_processor_round_trips_and_keeps_running_the_same() {
        round_trip_and_resume("pc-at", board("pc-at"), board("pc-at"));
    }

    #[cfg(feature = "machine-pc-at-smp")]
    #[test]
    fn two_processors_round_trip_and_keep_running_the_same() {
        round_trip_and_resume("pc-at-smp", board("pc-at-smp"), board("pc-at-smp"));
    }
}

// ---------------------------------------------------------------------------
// q35-linux-smp
// ---------------------------------------------------------------------------

#[cfg(all(feature = "machine-q35-linux-smp", feature = "fw-pcbios"))]
mod q35 {
    use super::*;

    fn board(engine: &str) -> Machine {
        use rsemu::machine::catalog;
        let entry = catalog::machine("q35-linux-smp").expect("this build ships it");
        let mut options = catalog::build_options().expect("this build's classes");
        options.realize.media.insert(
            "bios",
            rsemu::fw::pcbios::image_for_machine("q35-linux-smp.machine", entry.source)
                .expect("the board resolves"),
        );
        options.realize.media.insert("vgabios", Vec::new());
        options.realize.media.insert("optionrom", vec![0u8; 65536]);
        for slot in [
            "disk", "hd0", "hd1", "hd2", "hd3", "cd0", "cd1", "floppy", "kernel", "initrd", "nvme0",
        ] {
            options.realize.media.insert(slot, Vec::new());
        }
        options
            .resolve
            .params
            .push((String::from("engine"), String::from(engine)));
        let registry = catalog::registry().expect("this build's registry");
        rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .expect("the board realizes")
    }

    #[test]
    fn two_processors_round_trip_and_keep_running_the_same() {
        round_trip_and_resume("q35-linux-smp", board("interp"), board("interp"));
    }
}
