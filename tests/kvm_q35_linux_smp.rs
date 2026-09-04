//! **Two processors on `q35-linux`, and the thing that is still in the way of a
//! Linux SMP boot.** A committed reproduction, not a gate.
//!
//! [`tests/kvm_q35_linux.rs`](kvm_q35_linux.rs) boots a Gentoo `bzImage` to
//! userspace on `machines/q35-linux.machine` with one processor, in about nine
//! seconds of wall clock. This is the same board, the same kernel and the same
//! firmware-less loader with a second processor added the way
//! `machines/pc-at-smp.machine` adds one — and the line it waits for is Linux's
//! own `smp: Brought up 1 node, 2 CPUs`.
//!
//! **It does not reach it, and the reason is not the APIC page.** What is
//! measured, on a Gentoo 6.6.67 `bzImage`, `--release`, `no_timer_check`:
//!
//! | board | kernel's view | console lines before it stops |
//! | --- | --- | --- |
//! | one `cpu.x86` | `nr_cpu_ids=1` | 300, on to userspace |
//! | two, `nosmp` on the command line | `nr_cpu_ids=1` | 301, on to userspace |
//! | two, `map … = lapic0.window` | `nr_cpu_ids=2` | **126**, then the machine stops advancing |
//! | two, `map … = lapic0.regs` + `lapic1.regs` at `0xfef00000` | `nr_cpu_ids=2` | **126**, identically |
//!
//! The last two rows are the control this file is built around, and they are
//! the point: **the same 126 lines, the same kernel line, with the window and
//! without it.** The board that stops is also the board that stops when its two
//! local APICs are addressed the old way, so whatever is in the way is not how
//! `0xfee00000` decodes. The second row rules out "two vCPUs on this board" on
//! its own — two vCPUs boot to userspace as long as the kernel does not intend
//! to use the second one. What is left is what happens once a kernel *believes*
//! it has two processors, which is a question about how this machine runs two
//! active accelerated processors, and lives in `accel/` and `core::sched`
//! rather than in `dev/pc/apic.rs`. The guest stops mid-`printk`, several
//! hundred lines before `smpboot` says anything, so it is not the bring-up
//! sequence itself.
//!
//! `tests/kvm_q35_linux.rs`'s own module documentation has the shape of the
//! most likely explanation, at length: **virtual time does not advance while a
//! vCPU is inside `KVM_RUN`**, and a scheduler round does not end until every
//! runnable returns. That file needed `no_timer_check` for it with one
//! processor. This is what the same fact may cost with two.
//!
//! # Running it
//!
//! `#[ignore]`d, and on `RSEMU_SMP_KERNEL` rather than `RSEMU_KERNEL`, so that
//! neither `cargo test` nor a developer following `kvm_q35_linux.rs`'s
//! instructions can start a run that does not come back:
//!
//! ```text
//! RSEMU_SMP_KERNEL=/boot/vmlinuz \
//!     cargo test --release --features accel-kvm,machine-q35-linux \
//!                --test kvm_q35_linux_smp -- --ignored --nocapture
//! ```
//!
//! `RSEMU_SMP_NO_WINDOW=1` puts the architectural page back the way it was —
//! each local APIC at its own address — which is how the third and fourth rows
//! of the table above were produced. Nothing is vendored or downloaded: the
//! kernel is the host's own, run and never read (`ROADMAP.md` §1).
//!
//! # What is patched into the board
//!
//! Five lines, exactly `machines/pc-at-smp.machine`'s five, plus the one thing
//! a q35 needs and an AT does not — its ACPI MADT is told how many processors
//! there are, because a processor is not a region and the survey cannot count
//! them (`dev::q35::acpi`):
//!
//! ```text
//! + object cpu1 "cpu.x86"     { … }               a second processor
//! + object lapic1 "pc.lapic"  { … cpu = cpu1 }    its local APIC
//! ~ object lapic0 "pc.lapic"  { … cpu = cpu0 }    and whose the first one is
//! ~ object ioapic "pc.ioapic" { id = 2 }          out of lapic1's way
//! ~ object acpi   "q35.acpi"  { cpus = 2, ioapic-id = 2 }
//! ~ map mem 0xfee00000 = lapic0.window            not lapic0.regs
//! + wire lapic1.intr -> cpu1.intr, lapic1.nmi -> cpu1.nmi
//! ```
//!
//! Patched rather than shipped because `machines/q35-linux.machine` is a
//! demonstration board and how many processors a Linux demonstration wants is a
//! question for whoever ships it — and because a board file is a promise, which
//! this one cannot yet keep. `pc-at-smp` is the board this change took, and
//! `tests/pc_at_smp.rs` is its proof.

#![cfg(all(
    feature = "accel-kvm",
    feature = "cpu-x86",
    feature = "dev-q35",
    feature = "dev-nvme",
    feature = "dev-linuxboot",
    feature = "machine-q35-linux",
    target_os = "linux",
    target_arch = "x86_64"
))]

mod x86boot;

use rsemu::accel::cpu::AccelCpus;
use rsemu::accel::kvm::Kvm;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::machine::build;

use x86boot::Script;

/// The command line, which is `tests/kvm_q35_linux.rs`'s with nothing added:
/// `no_timer_check` for the reason that file documents at length, and no
/// `nosmp`, which is the point.
const CMDLINE: &str = "console=ttyS0,115200 earlyprintk=ttyS0,115200 nokaslr no_timer_check";

/// What the guest has to print. Linux's own words, from `smp_init()`.
const BROUGHT_UP: &str = "Brought up 1 node, 2 CPUs";

/// How long to let it run, in virtual milliseconds.
const DEFAULT_MS: u64 = 60_000;

/// `machines/q35-linux.machine` with a second processor and the architectural
/// APIC page.
fn two_processor_q35() -> String {
    let mut text = String::from(rsemu::machine::catalog::Q35_LINUX.source);

    const CPU0: &str = "  object cpu0 \"cpu.x86\" {\n\
                        \x20   clock   = cpu\n\
                        \x20   space   = mem\n\
                        \x20   iospace = \"port\"\n\
                        \x20   variant = \"x86-64\"\n\
                        \x20   engine  = \"interp\"\n\
                        \x20 }\n";
    assert!(text.contains(CPU0), "the `cpu0` object moved");
    // Order matters: `accel::cpu` allocates one vCPU per `cpu.x86` in
    // declaration order, so `cpu0` stays first and stays vCPU 0.
    text = text.replace(CPU0, &format!("{CPU0}{}", CPU0.replace("cpu0", "cpu1")));

    const APICS: &str = "  object lapic0 \"pc.lapic\"  { clock = bus, id = 0, bus = \"apic\" }\n  \
                         object ioapic \"pc.ioapic\" { id = 1, bus = \"apic\" }";
    assert!(text.contains(APICS), "the APIC objects moved");
    text = text.replace(
        APICS,
        "  object lapic0 \"pc.lapic\"  { clock = bus, id = 0, bus = \"apic\", cpu = cpu0 }\n  \
         object lapic1 \"pc.lapic\"  { clock = bus, id = 1, bus = \"apic\", cpu = cpu1 }\n  \
         object ioapic \"pc.ioapic\" { id = 2, bus = \"apic\" }",
    );

    // The one line the whole change is about. `RSEMU_SMP_NO_WINDOW` puts the
    // board back the way it was — each APIC's own page at its own address — so
    // that a failure on this board can be attributed to the window or cleared
    // of it without editing anything.
    const LAPIC_MAP: &str = "  map mem 0xfee00000 size 0x1000   = lapic0.regs";
    assert!(text.contains(LAPIC_MAP), "the lapic0 mapping moved");
    text = if std::env::var("RSEMU_SMP_NO_WINDOW").is_ok() {
        text.replace(
            LAPIC_MAP,
            "  map mem 0xfee00000 size 0x1000   = lapic0.regs\n  \
             map mem 0xfef00000 size 0x1000   = lapic1.regs",
        )
    } else {
        text.replace(
            LAPIC_MAP,
            "  map mem 0xfee00000 size 0x1000   = lapic0.window",
        )
    };

    // The MADT says how many processors there are, because a processor is not
    // a region and the survey cannot count them (`dev::q35::acpi`).
    const ACPI: &str = "    cpus       = 1\n    ioapic-id  = 1";
    assert!(text.contains(ACPI), "the `acpi` object moved");
    text = text.replace(ACPI, "    cpus       = 2\n    ioapic-id  = 2");

    const WIRES: &str = "  wire lapic0.intr -> cpu0.intr\n  wire lapic0.nmi  -> cpu0.nmi";
    assert!(text.contains(WIRES), "the lapic0 wires moved");
    text = text.replace(
        WIRES,
        "  wire lapic0.intr -> cpu0.intr\n  \
         wire lapic0.nmi  -> cpu0.nmi\n  \
         wire lapic1.intr -> cpu1.intr\n  \
         wire lapic1.nmi  -> cpu1.nmi",
    );
    text
}

/// The board patch itself is asserted even without a kernel: five anchors in
/// `machines/q35-linux.machine`, and a two-processor board that realizes.
///
/// This runs in an ordinary `cargo test`; the boot below does not.
#[test]
fn the_two_processor_q35_realizes() {
    if !Kvm::is_available() {
        println!("q35-linux-smp: no usable /dev/kvm on this host; skipping");
        return;
    }
    let accel = match AccelCpus::open(ThreadingMode::Parallel) {
        Ok(accel) => accel,
        Err(e) if e.is_unavailable() => return,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.scheduler.mode = ThreadingMode::Parallel;
    accel.install(&mut options.bindings);
    options = options.with_param("disk", "16777216");
    options.realize.media.insert("kernel", Vec::new());
    options.realize.media.insert("initrd", Vec::new());
    options.realize.media.insert("nvme0", Vec::new());
    let text = two_processor_q35();
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("q35-linux-smp.machine", &text, &registry, &options)
        .unwrap_or_else(|e| panic!("the two-processor q35 does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    assert_eq!(accel.cpus().len(), 2, "the board declares two processors");
    assert_eq!(
        accel.cpus()[1].entries(),
        0,
        "the application processor ran before anything started it"
    );
}

/// The boot. See the module documentation: this **does not currently reach**
/// `smp: Brought up 1 node, 2 CPUs`, and the control in `two_processor_q35`
/// shows it does not reach it either way the local APIC page is mapped.
///
/// Kept and committed because a reproduction that says exactly what it measured
/// is worth more than a paragraph saying it was tried.
#[test]
#[ignore = "does not yet reach SMP bring-up; see the module docs, and pass --ignored to run it"]
fn a_linux_kernel_brings_up_a_second_processor_on_host_silicon() {
    if !Kvm::is_available() {
        println!("q35-linux-smp/kvm: no usable /dev/kvm on this host; skipping");
        return;
    }
    let Ok(path) = std::env::var("RSEMU_SMP_KERNEL") else {
        println!(
            "q35-linux-smp/kvm: set RSEMU_SMP_KERNEL to a Linux/x86 bzImage to try two \
             processors on host silicon; see the module docs"
        );
        return;
    };
    let kernel = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let initrd = std::env::var("RSEMU_INITRD")
        .ok()
        .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
        .unwrap_or_default();

    let accel = match AccelCpus::open(ThreadingMode::Parallel) {
        Ok(accel) => accel,
        Err(e) if e.is_unavailable() => return,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.scheduler.mode = ThreadingMode::Parallel;
    accel.install(&mut options.bindings);
    let cmdline = std::env::var("RSEMU_KERNEL_CMDLINE").unwrap_or_else(|_| CMDLINE.to_string());
    options = options.with_param("cmdline", cmdline.as_str());
    options = options.with_param("disk", "16777216");
    options.realize.media.insert("kernel", kernel);
    options.realize.media.insert("initrd", initrd);
    options.realize.media.insert("nvme0", Vec::new());

    let text = two_processor_q35();
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("q35-linux-smp.machine", &text, &registry, &options)
        .unwrap_or_else(|e| panic!("the two-processor q35 does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    let console = rsemu::host::chardev::ports::open(&options.realize.hosts, "console")
        .expect("the 16550 opened the board's console port");

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
    assert!(
        run.text.contains(BROUGHT_UP),
        "the kernel did not bring up the second processor"
    );
    assert!(
        cpus[1].entries() > 0,
        "the kernel says it brought up two processors but the second entered no guest"
    );
}
