//! **Two processors on `q35-linux-smp`, and a Linux kernel that brings the
//! second one up.**
//!
//! [`tests/kvm_q35_linux.rs`](kvm_q35_linux.rs) boots a Gentoo `bzImage` to
//! userspace on `machines/q35-linux.machine` with one processor. This is
//! `machines/q35-linux-smp.machine` — the same board, the same kernel, the
//! same firmware-less loader, with a second processor added the way
//! `machines/pc-at-smp.machine` adds one — and the line it waits for is
//! Linux's own `smp: Brought up 1 node, 2 CPUs`.
//!
//! It reaches it, on the board's own command line, in under two seconds:
//!
//! ```text
//!   | [    0.254094] smp: Bringing up secondary CPUs ...
//!   | [    0.256147] smpboot: x86: Booting SMP configuration:
//!   | [    0.257053] .... node  #0, CPUs:      #1
//!   | [    0.260755] smp: Brought up 1 node, 2 CPUs
//!   | [    0.279455] smpboot: Total of 2 processors activated (15974.13 BogoMIPS)
//! ```
//!
//! and `nproc` in the initramfs shell says `2`.
//!
//! # What was in the way, and what it turned out to be
//!
//! This file was committed `#[ignore]`d as a *reproduction* rather than a
//! gate, because the board did not get there. What it measured then, on the
//! same kernel, was:
//!
//! | board | kernel's view | console lines before it stopped |
//! | --- | --- | --- |
//! | one `cpu.x86` | `nr_cpu_ids=1` | 300, on to userspace |
//! | two, `nosmp` on the command line | `nr_cpu_ids=1` | 301, on to userspace |
//! | two, `map … = lapic0.window` | `nr_cpu_ids=2` | **126**, then the machine stopped advancing |
//! | two, `map … = lapic0.regs` + `lapic1.regs` at `0xfef00000` | `nr_cpu_ids=2` | **126**, identically |
//!
//! The last two rows being identical is what said the APIC page was not the
//! problem, and the attribution that went with them was **virtual time not
//! advancing inside `KVM_RUN`** — `accel/` and `core::sched` rather than
//! `dev/pc/apic.rs`. That was right. The two things that fixed it are in
//! [`kvm_q35_linux.rs`](kvm_q35_linux.rs)'s module documentation at length:
//! [`ThreadingMode::Accel`], which reads a round's elapsed virtual time off
//! the host clock, and `accel::preempt`, which bounds a guest that takes no
//! exits at all. This test was running in
//! [`ThreadingMode::Parallel`](rsemu::core::sched::ThreadingMode::Parallel),
//! where neither applies; in `Accel` the same board and the same kernel reach
//! userspace with two processors.
//!
//! **The negative control now discriminates, and it did not before.** With
//! `RSEMU_SMP_NO_WINDOW=1` — each local APIC at its own address, which is what
//! `q35-linux` does with one — the same kernel still reaches userspace and
//! prints:
//!
//! ```text
//!   | [   10.260436] CPU1 failed to report alive state
//!   | [   10.267325] smp: Brought up 1 node, 1 CPU
//! ```
//!
//! because the application processor read the bootstrap processor's APIC ID.
//! So `lapic0.window` is load-bearing on this board after all; what hid that
//! was a machine whose clocks stood still, in which *neither* mapping got as
//! far as `smpboot`. Two facts, one behind the other, and the outer one had to
//! go first.
//!
//! # Running it
//!
//! Still `#[ignore]`d and still on `RSEMU_SMP_KERNEL` rather than
//! `RSEMU_KERNEL`, because it wants a kernel image and `cargo test` must not
//! start a run that depends on one:
//!
//! ```text
//! RSEMU_SMP_KERNEL=/boot/vmlinuz \
//! RSEMU_INITRD=testdata/x86/initramfs-x86.cpio \
//!     cargo test --release --features accel-kvm,machine-q35-linux-smp \
//!                --test kvm_q35_linux_smp -- --ignored --nocapture
//! ```
//!
//! `RSEMU_SMP_NO_WINDOW=1` puts the architectural page back the way `q35-linux`
//! has it, which is the control above. Nothing is vendored or downloaded: the
//! kernel is the host's own, run and never read (`ROADMAP.md` §1).
//!
//! # And from a command line
//!
//! The same board, without writing a test:
//!
//! ```text
//! rsemu run q35-linux-smp --media kernel=/boot/vmlinuz \
//!                         --media initrd=initramfs.cpio --accel kvm
//! ```
//!
//! `--accel kvm` is what selects the backend, and it implies
//! [`ThreadingMode::Accel`] for the reason this file is about.
//!
//! # What a run under this engine is not
//!
//! **Reproducible**, for every reason `kvm_q35_linux.rs` gives. Two runs take
//! different numbers of guest entries and reach `smpboot` at different virtual
//! instants; the console is the comparison that means something, which is why
//! it is what this file prints.

#![cfg(all(
    feature = "accel-kvm",
    feature = "cpu-x86",
    feature = "dev-q35",
    feature = "dev-nvme",
    feature = "dev-linuxboot",
    feature = "machine-q35-linux-smp",
    target_os = "linux",
    target_arch = "x86_64"
))]

mod x86boot;

use std::sync::Arc;

use rsemu::accel::cpu::AccelCpus;
use rsemu::accel::kvm::Kvm;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::host::chardev::CharPort;
use rsemu::machine::Machine;
use rsemu::machine::build;

use x86boot::Script;

/// The command line, which is the board's own with nothing added to it — no
/// `nosmp`, and no `no_timer_check` either. That second word is the one
/// `kvm_q35_linux.rs` needed before virtual time advanced inside `KVM_RUN`,
/// and this board does not need it any more than that one does.
const CMDLINE: &str = "console=ttyS0,115200 earlyprintk=ttyS0,115200 nokaslr";

/// What the guest has to print. Linux's own words, from `smp_init()`.
const BROUGHT_UP: &str = "Brought up 1 node, 2 CPUs";

/// How long to let it run, in virtual milliseconds.
///
/// Under [`ThreadingMode::Accel`] this is a ceiling on **wall clock too**, and
/// it is generous on purpose: the negative control below spends ten of these
/// seconds inside the kernel's own timeout for a secondary processor that
/// never answers.
const DEFAULT_MS: u64 = 60_000;

/// The board's text, as shipped, or with the architectural APIC page put back
/// the way `q35-linux` has it.
///
/// `RSEMU_SMP_NO_WINDOW=1` is the negative control: two local APICs at two
/// addresses, so `cpu1` reaching `0xfee00000` reaches `lapic0`. Everything
/// else is identical, which is what makes the difference attributable.
fn board_text() -> String {
    let text = String::from(rsemu::machine::catalog::Q35_LINUX_SMP.source);
    if std::env::var("RSEMU_SMP_NO_WINDOW").is_err() {
        return text;
    }
    const WINDOW: &str = "  map mem 0xfee00000 size 0x1000   = lapic0.window";
    assert!(text.contains(WINDOW), "the lapic0 mapping moved");
    text.replace(
        WINDOW,
        "  map mem 0xfee00000 size 0x1000   = lapic0.regs\n  \
         map mem 0xfef00000 size 0x1000   = lapic1.regs",
    )
}

/// A built board, its accelerator, and the console the 16550 opened.
type Built = (Machine, Arc<AccelCpus>, Arc<CharPort>);

/// Build the board with `mode`, and with the two `cpu.x86` objects it declares
/// running on the host's own silicon.
fn built(
    mode: ThreadingMode,
    kernel: Vec<u8>,
    initrd: Vec<u8>,
    params: &[(&str, String)],
) -> Result<Built, String> {
    let accel = AccelCpus::open(mode).map_err(|e| format!("{e}"))?;
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    // It must match what `AccelCpus::open` was given: that call is what decides
    // this engine's slice length and its preemption interval, and it has
    // already refused a mode claiming reproducibility.
    options.realize.scheduler.mode = mode;
    accel.install(&mut options.bindings);
    options = options.with_param("disk", "16777216");
    for (name, value) in params {
        options = options.with_param(*name, value.as_str());
    }
    options.realize.media.insert("kernel", kernel);
    options.realize.media.insert("initrd", initrd);
    options.realize.media.insert("nvme0", Vec::new());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("q35-linux-smp.machine", &board_text(), &registry, &options)
        .map_err(|e| format!("{e}"))?;
    // Deliberately **not** `set_host_clock`. `ThreadingMode::Accel` has no
    // other source of elapsed time, and `machine::realize` now installs the
    // host's monotonic clock when the mode asks for one — so a front end that
    // selects acceleration is not also required to know that rule. A round
    // that failed with `SchedError::NoHostClock` would end this run on its
    // first call to `run_for`, which is what makes this an assertion rather
    // than an omission.
    m.reset(ResetKind::Cold);
    m.sweep();
    let console = rsemu::host::chardev::ports::open(&options.realize.hosts, "console")
        .expect("the 16550 opened the board's console port");
    Ok((m, accel, console))
}

/// The shipped board is `q35-linux` plus exactly the six things it claims.
///
/// Cheap, hermetic, and the thing that catches an edit to one of the two files
/// that was not made to the other: they are meant to be read side by side, and
/// a diff nobody can see is how they stop being comparable.
#[test]
fn the_shipped_board_differs_from_q35_linux_in_the_six_stated_ways() {
    let smp = rsemu::machine::catalog::Q35_LINUX_SMP.source;
    #[cfg(feature = "machine-q35-linux")]
    {
        let one = rsemu::machine::catalog::Q35_LINUX.source;
        assert!(
            one.contains("size 0x1000   = lapic0.regs"),
            "the one-processor mapping"
        );
        assert!(!one.contains("cpu1"), "q35-linux grew a second processor");
    }
    for expected in [
        // a second processor, and its local APIC naming it
        "object cpu1 \"cpu.x86\"",
        "object lapic1 \"pc.lapic\"",
        "cpu = cpu0",
        "cpu = cpu1",
        // the I/O APIC out of lapic1's id, and the MADT told both numbers
        "object ioapic \"pc.ioapic\" { id = 2",
        "cpus       = 2",
        "ioapic-id  = 2",
        // the one mapping the whole board is about
        "size 0x1000   = lapic0.window",
        // and the second processor's two pins
        "wire lapic1.intr -> cpu1.intr",
        "wire lapic1.nmi  -> cpu1.nmi",
    ] {
        assert!(smp.contains(expected), "the board is missing `{expected}`");
    }
    // The *mapping*, not the word: the comment above it argues at length
    // about what `= lapic0.regs` would do, and that paragraph is the point of
    // the line rather than a thing to grep away.
    assert!(
        !smp.contains("size 0x1000   = lapic0.regs"),
        "the architectural page still decodes to one APIC for everybody"
    );
}

/// The board realizes with two accelerated processors, and the application
/// processor has not run.
///
/// This runs in an ordinary `cargo test`; the boot below does not.
#[test]
fn the_two_processor_q35_realizes() {
    if !Kvm::is_available() {
        println!("q35-linux-smp: no usable /dev/kvm on this host; skipping");
        return;
    }
    let (mut m, accel, _console) = match built(ThreadingMode::Accel, Vec::new(), Vec::new(), &[]) {
        Ok(built) => built,
        Err(e) => panic!("the two-processor q35 does not realize: {e}"),
    };
    assert_eq!(accel.cpus().len(), 2, "the board declares two processors");
    assert_eq!(
        accel.cpus()[1].entries(),
        0,
        "the application processor ran before anything started it"
    );
    // One round, which is what proves the host clock the mode needs was
    // installed by the build rather than by this test.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("an accelerated machine runs without being handed a host clock");
}

/// The boot: a stock Linux kernel brings up the second processor on host
/// silicon, on the board's own command line.
#[test]
#[ignore = "needs a Linux/x86 bzImage in RSEMU_SMP_KERNEL; pass --ignored to run it"]
fn a_linux_kernel_brings_up_a_second_processor_on_host_silicon() {
    if !Kvm::is_available() {
        println!("q35-linux-smp/kvm: no usable /dev/kvm on this host; skipping");
        return;
    }
    let Ok(path) = std::env::var("RSEMU_SMP_KERNEL") else {
        println!(
            "q35-linux-smp/kvm: set RSEMU_SMP_KERNEL to a Linux/x86 bzImage to bring up two \
             processors on host silicon; see the module docs"
        );
        return;
    };
    let kernel = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let initrd = std::env::var("RSEMU_INITRD")
        .ok()
        .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
        .unwrap_or_default();

    let cmdline = std::env::var("RSEMU_KERNEL_CMDLINE").unwrap_or_else(|_| CMDLINE.to_string());
    println!("q35-linux-smp/kvm: command line {cmdline:?}");
    let params = [("cmdline", cmdline)];
    let (mut m, accel, console) = match built(ThreadingMode::Accel, kernel, initrd, &params) {
        Ok(built) => built,
        Err(e) if e.contains("/dev/kvm") => {
            println!("q35-linux-smp/kvm: {e}; skipping");
            return;
        }
        Err(e) => panic!("the two-processor q35 does not realize: {e}"),
    };
    let cpus = accel.cpus();
    assert_eq!(cpus.len(), 2, "the board declares two processors");

    let ms: u64 = std::env::var("RSEMU_KERNEL_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_MS);
    // `RSEMU_KERNEL_STOP_AT` still wins, for a run being debugged; the default
    // is the line this test is about.
    let mut script = Script::from_env();
    if script.stop_at.is_empty() {
        script.stop_at = String::from(BROUGHT_UP);
    }
    println!("q35-linux-smp/kvm: what the guest wrote to its serial port at 0x3f8:");
    let started = std::time::Instant::now();
    let run = x86boot::run(
        &mut m,
        cpus[0].shell(),
        &console,
        GlobalTime::from_nanos(ms * 1_000_000),
        &script,
    );
    let wall = started.elapsed();
    println!(
        "q35-linux-smp/kvm: bsp {} entries, ap {} entries; {} console lines; {} ms of guest \
         time in {:.1} s of wall clock",
        cpus[0].entries(),
        cpus[1].entries(),
        run.text.lines().count(),
        run.at.as_nanos() / 1_000_000,
        wall.as_secs_f64()
    );

    assert!(
        !cpus[0].is_stopped(),
        "the bootstrap processor stopped: {:?}",
        cpus[0].failure()
    );
    assert!(
        !cpus[1].is_stopped(),
        "the application processor stopped: {:?}",
        cpus[1].failure()
    );
    assert!(run.long, "the kernel never reached long mode");
    // The negative control's own assertion, so that a run with
    // `RSEMU_SMP_NO_WINDOW=1` fails with what it *did* print rather than with
    // the absence of what it did not.
    if std::env::var("RSEMU_SMP_NO_WINDOW").is_ok() {
        assert!(
            run.text.contains("failed to report alive state"),
            "without the window the application processor is expected to go unheard from, and \
             this run said something else"
        );
        println!(
            "q35-linux-smp/kvm: the control ran; without `lapic0.window` the kernel gave up on \
             the second processor, which is the whole argument for the window"
        );
        return;
    }
    assert!(
        run.text.contains(BROUGHT_UP),
        "the kernel did not bring up the second processor"
    );
    assert!(
        cpus[1].entries() > 0,
        "the kernel says it brought up two processors but the second entered no guest"
    );
}
