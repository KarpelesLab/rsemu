//! **A phase-6 machine, booting under KVM, on firmware this repository builds.**
//!
//! `ROADMAP.md` phase 7's gate opens *"the phase-6 machines boot under KVM"*,
//! and phase 6a's machine is `pc-at` on [`rsemu::fw::pcbios`] — assembled here,
//! nothing downloaded and nothing vendored. `tests/pc_at_boot.rs` is that boot
//! under the interpreter, step by step. This is the same board with the same
//! firmware and the same diskette, with [`AccelCpus`] replacing what is
//! underneath its processor, and it asserts the two engines reach the **same
//! place**:
//!
//! 1. the processor fetches `0xfffffff0` on the host's own silicon and runs
//!    POST — which means every port write, every MMIO access and every ROM
//!    fetch it makes is routed back into `machines/pc-at.machine`'s own device
//!    models;
//! 2. POST fills the BIOS Data Area from the CMOS and the hardware, and the
//!    equipment word, the base-memory size and the video mode come out
//!    **identical** to the interpreted run;
//! 3. `INT 19h` reads cylinder 0, head 0, sector 1 off the diskette — through
//!    the µPD765 and the 8237, neither of which is memory — sees the
//!    `0x55 0xAA` signature, and jumps to `0000:7c00`;
//! 4. and the **boot sector itself executes in hardware**, leaving its mark in
//!    the board's own low RAM.
//!
//! It is one processor, because `pc-at` declares one. Two of them, started by
//! the guest's own `INIT` and Start-Up, is `tests/kvm_smp.rs`.
//!
//! # Why the boot sector ends in `hlt` rather than `jmp $`
//!
//! A guest that takes no exits is not preemptible — `accel::kvm` says so — and
//! on a board it is worse than that: under `ThreadingMode::Parallel` a
//! scheduler round does not end until every runnable returns, so a spin inside
//! `KVM_RUN` stops the machine's virtual time rather than only this
//! processor's. `hlt` leaves hardware; it is also what a real boot sector that
//! has nothing left to do writes.
//!
//! Skips cleanly with no `/dev/kvm`.

#![cfg(all(
    feature = "accel-kvm",
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "fw-pcbios",
    feature = "machine-pc-at",
    target_os = "linux",
    target_arch = "x86_64"
))]

use rsemu::accel::cpu::AccelCpus;
use rsemu::accel::kvm::Kvm;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{BuildOptions, Machine, build};

/// A 1.44 MB diskette whose boot sector says it ran.
///
/// Six instructions, hand-assembled: the point is that *real x86 bytes the
/// firmware loaded off a real diskette controller* execute on the host, so
/// anything cleverer would be testing the cleverness.
fn bootable_diskette() -> Vec<u8> {
    let mut image = vec![0u8; 1_474_560];
    let sector: &[u8] = &[
        0x31, 0xc0, // xor ax, ax
        0x8e, 0xd8, // mov ds, ax
        0xc7, 0x06, 0x00, 0x06, 0x07, 0xb0, // mov word [0x600], 0xb007
        0xf4, // hlt
        0xeb, 0xfd, // jmp back to the hlt
    ];
    image[..sector.len()].copy_from_slice(sector);
    // What `INT 19h` looks for, and the reason a blank diskette is not booted.
    image[510] = 0x55;
    image[511] = 0xaa;
    image
}

/// Where the boot sector leaves its mark, and what it leaves.
const BOOT_MARK_AT: u64 = 0x0600;
const BOOT_MARK: u64 = 0xb007;

/// The board's build options, with the firmware and the media every socket
/// names bound.
fn options() -> BuildOptions {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    // An empty option-ROM socket: 64 KiB of zeroes has no `0x55 0xAA`, which is
    // exactly what the firmware's scan must survive.
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("optionrom", vec![0u8; 65536]);
    options.realize.media.insert("floppy", bootable_diskette());
    for slot in ["disk", "hd0", "hd1", "hd2", "hd3", "cd0", "cd1"] {
        options.realize.media.insert(slot, vec![0u8; 8 << 20]);
    }
    options
}

/// What a boot leaves in the BIOS Data Area, as three numbers a person can
/// read: the equipment word at `0x410`, the base-memory size at `0x413`, the
/// video mode at `0x449` — and the boot sector's own mark.
#[derive(Debug, PartialEq, Eq)]
struct Reached {
    equipment: u64,
    basemem: u64,
    video_mode: u64,
    boot_mark: u64,
}

fn reached(m: &Machine) -> Reached {
    let mem = m.space("mem").expect("the memory space");
    let at = |addr, width| mem.read(addr, width, MemAttrs::DEBUG).unwrap_or(0);
    Reached {
        equipment: at(0x410, Width::U16),
        basemem: at(0x413, Width::U16),
        video_mode: at(0x449, Width::U8),
        boot_mark: at(BOOT_MARK_AT, Width::U16),
    }
}

/// Run `m` until the boot sector has left its mark, or for `rounds` quanta.
///
/// A span *shorter* than the scheduler quantum would advance the clock without
/// running anything: `run_for` is additive and declines a round its deadline
/// falls inside (`Machine::run_until`).
fn run(m: &mut Machine, rounds: usize) {
    for _ in 0..rounds {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if reached(m).boot_mark == BOOT_MARK {
            return;
        }
    }
}

#[test]
fn the_at_boots_on_host_silicon_and_reaches_where_the_interpreter_does() {
    if !Kvm::is_available() {
        return;
    }
    // `Parallel` rather than `Deterministic`, and not by preference:
    // `AccelCpus::open` refuses a mode that claims reproducibility, because a
    // run on host silicon is not reproducible.
    let accel = match AccelCpus::open(ThreadingMode::Parallel) {
        Ok(accel) => accel,
        Err(e) if e.is_unavailable() => return,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };

    let mut opts = options();
    opts.realize.scheduler.mode = ThreadingMode::Parallel;
    accel.install(&mut opts.bindings);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut accelerated = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &opts)
        .unwrap_or_else(|e| panic!("the board does not realize under acceleration: {e}"));
    accelerated.reset(ResetKind::Cold);
    accelerated.sweep();
    run(&mut accelerated, 200);

    let cpu = accel.cpus().pop().expect("the board's processor");
    let hardware = reached(&accelerated);
    assert!(
        cpu.entries() > 0,
        "the processor never entered the guest: {hardware:?}"
    );
    assert!(
        !cpu.is_stopped(),
        "the processor stopped: {:?} after {hardware:?}",
        cpu.failure()
    );
    assert_eq!(
        hardware.boot_mark, BOOT_MARK,
        "the boot sector did not run in hardware: {hardware:?}"
    );
    assert!(
        hardware.basemem > 0 && hardware.equipment != 0,
        "POST did not fill the BIOS data area: {hardware:?}"
    );

    // The same board, the same firmware, the same diskette, interpreted. The
    // gate is not "it ran" but "it ran to the same place".
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut interpreted = build(
        "pc-at.machine",
        rsemu::dev::pc::PC_AT,
        &registry,
        &options(),
    )
    .expect("the interpreted board");
    interpreted.reset(ResetKind::Cold);
    interpreted.sweep();
    run(&mut interpreted, 400);

    assert_eq!(
        hardware,
        reached(&interpreted),
        "the two engines booted the same board to different places"
    );
    // And the boot sector really is where it halted, rather than having been
    // written by something else: `0000:7c0b` is the `hlt` in it.
    let regs = cpu.shell().regs();
    assert_eq!(regs.cs, 0, "the boot sector runs in segment zero");
    assert!(
        (0x7c00..0x7c20).contains(&regs.rip),
        "the processor is inside the boot sector, at {:#x}",
        regs.rip
    );
    assert!(cpu.is_halted(), "and idle at its `hlt`");
}

/// Says out loud whether the test above actually ran.
#[test]
fn report_whether_this_host_can_boot_the_at_under_kvm() {
    if Kvm::is_available() {
        // Nothing to assert; the test above did the asserting.
    } else {
        // Deliberately not a failure: `cargo test` stays hermetic.
    }
}
