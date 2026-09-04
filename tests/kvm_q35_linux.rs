//! **A stock Linux kernel booting to a shell on host silicon, and reading its
//! own disk.**
//!
//! [`tests/q35_linux.rs`](q35_linux.rs) is this board interpreted: a Gentoo
//! 6.6.67 `bzImage` reaches userspace and reads off its NVMe namespace in
//! **2,826 seconds of guest time and about sixteen minutes of wall clock**.
//! This is the same board, the same machine file, the same kernel, the same
//! initramfs and — now — the same command line, with [`AccelCpus`] replacing
//! what is underneath `cpu0`, and it reaches the same line in **about two and
//! a half seconds**, which is the whole point of `ROADMAP.md` §10. Run to run
//! that number moves, because nothing here is reproducible; the console is
//! what does not.
//!
//! Guest time and wall clock are the *same* two and a half seconds, and that
//! is not a coincidence: this board runs under
//! [`ThreadingMode::Accel`](rsemu::core::sched::ThreadingMode::Accel), where
//! virtual time is read off the host clock. On its own command line, with no
//! `no_timer_check` — see below for what that used to cost.
//!
//! ```text
//! RSEMU_KERNEL=/boot/vmlinuz \
//! RSEMU_INITRD=testdata/x86/initramfs-x86.cpio \
//! RSEMU_KERNEL_INPUT='rsemu# =>head -c 40 /dev/nvme0n1\n' \
//! RSEMU_KERNEL_STOP_AT='LBA 0' \
//!     cargo test --release --features accel-kvm,machine-q35-linux \
//!                --test kvm_q35_linux -- --nocapture
//! ```
//!
//! ```text
//!   | [    0.006528] nvme 0000:00:04.0: enabling device (0000 -> 0002)
//!   | [    0.008613] Run /init as init process
//!   |
//!   | rsemu initramfs on Linux 6.6.67-gentoo-x86_64 x86_64
//!   | rsemu# head -c 40 /dev/nvme0n1
//!   | rsemu q35-linux nvme namespace, LBA 0
//! ```
//!
//! Everything printed as evidence is a byte the guest itself wrote to its own
//! serial port; the image is run, never read (`ROADMAP.md` §1). Skips cleanly
//! with no `/dev/kvm`, and without `RSEMU_KERNEL` it says so and returns —
//! no kernel is vendored or downloaded by `cargo test`.
//!
//! # The two things that had to exist first
//!
//! Neither is a detail of this board and both are in
//! [`accel`](rsemu::accel) rather than here.
//!
//! 1. **A `CPUID` table.** A vCPU that has never been given one answers every
//!    leaf with zeros, and a kernel's own entry path checks the long-mode bit
//!    before it does anything else. `KVM_SET_CPUID2`, from
//!    `KVM_GET_SUPPORTED_CPUID` through
//!    [`board_cpuid`](rsemu::accel::kvm::board_cpuid), is what makes the
//!    processor a processor.
//! 2. **An interpreter for what hardware cannot fetch.** This board's reset
//!    vector is `x86.linuxboot`'s sixteen synthesised bytes at `0xfffffff0`, a
//!    *device region* — and a hypervisor cannot fetch through an MMIO exit. The
//!    far jump out of it now runs on the shell interpreter and everything after
//!    it runs in hardware, which is a thing an emulator with an interpreter can
//!    do and a hypervisor client cannot.
//!
//! # Time, and the word that is no longer on the command line
//!
//! The board's own command line used to boot interpreted and **panic
//! accelerated**, in `check_timer()`, and the reason was never the board's
//! interrupt tree — that is measurably correct, and the same tree carries the
//! tick here. It was time:
//!
//! > **Virtual time did not advance while a vCPU was inside `KVM_RUN`.**
//!
//! A scheduler round ends when every runnable returns, and an accelerated
//! processor returns when the *guest* exits. So a guest that ran without
//! exiting — a delay loop, which is exactly what `mdelay()` is — held the
//! round, and the board's clocks stood still for as long as it took. Both
//! failures on the stock line were that one fact:
//!
//! | what the kernel printed | what it did |
//! | --- | --- |
//! | `hpet: Counter not counting. HPET disabled` | read `HPET_COUNTER`, spun 200,000 TSC cycles, read it again. Both reads fell in one round, so the counter had not moved. |
//! | `..MP-BIOS bug: 8254 timer not connected to IO-APIC`, then the panic | `timer_irq_works()`: read `jiffies`, spin about forty milliseconds, read `jiffies`. The spin took no exits, so no timer fired. |
//!
//! Two changes, one level below `accel/` and one inside it, removed both:
//!
//! * [`ThreadingMode::Accel`](rsemu::core::sched::ThreadingMode::Accel) is
//!   implemented in `src/core/sched.rs`. A round's elapsed virtual time is
//!   read off the injected [`HostClock`](rsemu::core::sched::HostClock)
//!   instead of being taken from what the runnables claimed, which is §4.2's
//!   *"virtual time is slaved to the host clock"*. That is what makes the
//!   board's clocks move while the guest runs, and — because this engine's
//!   slice is **one guest exit long** — it makes every device access see the
//!   wall as of that access. `hpet_counting()` reads a counter that has moved.
//! * A **preemption interval** bounds a guest that takes no exits
//!   (`accel::preempt`): the vCPU's own thread asks the kernel for a periodic
//!   signal, whose delivery is what makes `KVM_RUN` return `EINTR`. That is
//!   what makes `timer_irq_works()` see a tick, and it is the one thing no
//!   signal-free mechanism could do — that module argues each alternative.
//!
//! What the kernel says now, on the same line, is the measurement that matters:
//!
//! ```text
//!   | [    0.033333] tsc: using HPET reference calibration
//!   | [    0.036666] tsc: Detected 3992.968 MHz processor
//!   | [    0.736685] hpet0: at MMIO 0xfed00000, IRQs 2, 8, 0
//!   | [    0.737097] hpet0: 3 comparators, 64-bit 10.000000 MHz counter
//! ```
//!
//! **3,992.968 MHz against a host that is 3,993,994 kHz.** Before this it
//! reported a **176,273 MHz** processor: it was measuring a real time-stamp
//! counter against a board whose clocks only moved when it stopped running, so
//! every delay it computed was wrong by about forty-four times. A guest's own
//! view of the clock is the honest test of whether the clock is right, and this
//! is it.
//!
//! # Do the two engines agree?
//!
//! `tests/riscv_virt_engines.rs` runs one guest under all three engines and
//! asserts an identical `Machine::state_hash` at every checkpoint — the whole
//! machine, not a console — and that is the standard. (It does *not* diff
//! console output; an earlier version of this comment said it diffed 265 lines
//! of console, which was never true and was quoted onward from here into
//! several task briefs before anyone checked.) This board cannot quite meet
//! that standard,
//! for a reason worth stating rather than averaging away: **an accelerated
//! processor is not the same part as an interpreted one.** `cpu::x86` answers
//! `CPUID` from its declared `variant`; a vCPU answers from the host's silicon.
//! So a kernel that boots on both takes different paths through its own feature
//! dispatch and says so.
//!
//! Measured, on the same board with the same kernel, the same initramfs and —
//! now that `no_timer_check` is gone — *literally the same command line*:
//! 2,826 seconds of guest time in 978 seconds of wall clock interpreted,
//! against 2.4 seconds of both here.
//!
//! * **282 of the accelerated run's 346 console lines are byte-identical** to
//!   the interpreted run's, in the same order, once the printk timestamp is
//!   removed.
//! * **Every milestone appears in both**, in order: the RSDP found by scanning
//!   `0xe0000`, the MADT, `IOAPIC[0]`, the switch to symmetric I/O mode, the
//!   PCI root bridge, `PCI: Using ACPI for IRQ routing`, `ttyS0`, the NVMe
//!   function at `0000:00:04.0`, its queue pair, `Run /init as init process`,
//!   the shell, and the signature read off the namespace.
//! * The 62 lines that differ are **all** downstream of who the processor is —
//!   the model line (`AMD 1a/08` against the interpreter's `Intel 06/0f`), the
//!   speculative-execution mitigations, the `XSAVE` feature list, the PMU, the
//!   TLB geometry, the `BogoMIPS` a correct calibration produces. Not one of
//!   them is a device answering differently, and **not one of them is a
//!   timekeeping failure any more**: the lines that used to say the HPET was
//!   not counting now say what its counter runs at.
//! * The interpreted run additionally prints five `soft lockup` backtraces and
//!   the 636 lines of stack that go with them, which is *its* artefact: the
//!   guest's own watchdog notices that its interpreter is slow.
//!
//! To reproduce, run this test and `tests/q35_linux.rs` with
//! `RSEMU_KERNEL_MS=3000000` on the interpreted one — no
//! `RSEMU_KERNEL_CMDLINE` on either, which is the point — and diff the `  | `
//! lines with the printk timestamp stripped.
//!
//! # What a run under this engine is not
//!
//! **Reproducible.** [`AccelCpus::open`] refuses a deterministic
//! [`ThreadingMode`], `Machine::set_recorder` refuses a non-deterministic one,
//! and `Machine::state_hash` over an `Accel` run is meaningless — more so than
//! over a `Parallel` one, because here the *clock itself* is the host's. Two
//! runs of this test take different numbers of guest entries and reach the
//! shell at different virtual instants. The console is the comparison that
//! means something, which is why it is what this file prints.

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

use std::sync::Arc;

use rsemu::accel::board::plan_space;
use rsemu::accel::cpu::{AccelCpu, AccelCpus};
use rsemu::accel::kvm::Kvm;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::host::chardev::CharPort;
use rsemu::machine::Machine;
use rsemu::machine::build;

use x86boot::Script;

/// How long to let the board run, in virtual milliseconds.
///
/// A ceiling, not a target, and under
/// [`ThreadingMode::Accel`](rsemu::core::sched::ThreadingMode::Accel) it is a
/// ceiling on **wall clock too**: virtual time is the host clock there, so 200
/// virtual seconds is 200 real ones. The boot needs about two and a half of
/// them, and the run ends early when the guest prints `RSEMU_KERNEL_STOP_AT`
/// or stops making progress.
const DEFAULT_MS: u64 = 200_000;

/// The command line an accelerated run gets.
///
/// **`machines/q35-linux.machine`'s own default, word for word**, repeated
/// here only because a machine-file parameter can only be replaced whole. It
/// used to carry a fourth word, `no_timer_check`, and this file's module
/// documentation is about why it does not any more.
const CMDLINE: &str = "console=ttyS0,115200 earlyprintk=ttyS0,115200 nokaslr";

/// What the test stamps over the front of a blank namespace, byte for byte the
/// same as the interpreted run's — so `head -c 40 /dev/nvme0n1` in the guest
/// reads exactly this back off the medium.
const SIGNATURE: &[u8] = b"rsemu q35-linux nvme namespace, LBA 0\n\0\0";

/// How big the namespace is when nothing is bound to it.
const DEFAULT_DISK: u64 = 16 * 1024 * 1024;

/// A built board and the handles a run needs.
type Built = (Machine, Arc<AccelCpus>, Arc<AccelCpu>, Arc<CharPort>);

/// Build `q35-linux` from its own machine file with every `cpu.x86` on KVM.
///
/// `Bindings::replace` is the whole of the interception: the machine file is
/// used verbatim, `engine = "interp"` and all, and what changes is the engine
/// underneath it (`accel::cpu`).
fn board(
    kernel: Vec<u8>,
    initrd: Vec<u8>,
    disk: Vec<u8>,
    params: &[(&str, String)],
) -> Result<Built, String> {
    let accel = AccelCpus::open(ThreadingMode::Accel).map_err(|e| format!("{e}"))?;
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    // `Accel`, and it must match what `AccelCpus::open` was given above: that
    // call is what decides this engine's slice length and its preemption
    // interval, and it has already refused a mode claiming reproducibility.
    options.realize.scheduler.mode = ThreadingMode::Accel;
    accel.install(&mut options.bindings);
    let capacity = format!("{}", (disk.len() as u64).max(DEFAULT_DISK));
    options = options.with_param("disk", capacity.as_str());
    for (name, value) in params {
        options = options.with_param(*name, value.as_str());
    }
    options.realize.media.insert("kernel", kernel);
    options.realize.media.insert("initrd", initrd);
    options.realize.media.insert("nvme0", disk);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = build(
        "q35-linux.machine",
        rsemu::machine::catalog::Q35_LINUX.source,
        &registry,
        &options,
    )
    .map_err(|e| format!("{e}"))?;
    // What `ThreadingMode::Accel` runs on. `machine::realize` now installs
    // exactly this clock when the mode asks for one — a machine without it
    // fails every round with `SchedError::NoHostClock`, and requiring every
    // caller to know that was what kept the mode inside tests. Kept here
    // anyway, and deliberately: it is the override path, and a build that
    // stopped honouring it would otherwise go unnoticed.
    machine.set_host_clock(Box::new(rsemu::host::clock::MonotonicClock::new()));
    machine.reset(ResetKind::Cold);
    machine.sweep();
    let console = rsemu::host::chardev::ports::open(&options.realize.hosts, "console")
        .expect("the 16550 opened the board's console port");
    let cpu = accel.cpus().pop().expect("the board's processor");
    Ok((machine, accel, cpu, console))
}

/// The board with nothing in any slot, which is what the hermetic tests want.
fn bare_board() -> Option<Built> {
    if !Kvm::is_available() {
        println!("q35-linux/kvm: no usable /dev/kvm on this host; skipping");
        return None;
    }
    match board(Vec::new(), Vec::new(), SIGNATURE.to_vec(), &[]) {
        Ok(built) => Some(built),
        Err(e) => panic!("the board does not realize under acceleration: {e}"),
    }
}

// ---------------------------------------------------------------------------
// what the board's memory map becomes, before anything runs on it
// ---------------------------------------------------------------------------

/// The three windows a hypervisor can run this board out of, and the one it
/// cannot.
///
/// `plan_space` is deliberately testable without a hypervisor
/// (`accel::board`), so this asserts the *decision* rather than the ioctl: RAM
/// and the ACPI table region become slots, and the reset vector — sixteen
/// bytes of `x86.linuxboot` at the top of the address space, a device region
/// because what lives at a board's reset vector here is a **loader** — cannot.
/// That last row is why [`AccelCpu`] has an interpreter in it.
#[test]
fn the_boards_ram_can_be_a_memory_slot_and_its_reset_vector_cannot() {
    let Some((m, _accel, _cpu, _console)) = bare_board() else {
        return;
    };
    let space = m.space("mem").expect("the memory space");
    let windows = plan_space(space, true);
    let bases: Vec<u64> = windows.slots.iter().map(|(base, _)| *base).collect();
    assert_eq!(
        bases,
        vec![0x0, 0xe0000, 0x100000],
        "base RAM, the ACPI tables and extended memory are the hardware-backed windows"
    );
    // And the reset vector is in none of them. A device region is left as
    // MMIO on purpose — that is what `FlatTarget::Io` *means* — so this is not
    // a complaint about the planner; it is the fact [`AccelCpu`] has an
    // interpreter for.
    let covered = |addr: u64| {
        windows
            .slots
            .iter()
            .any(|(base, window)| addr >= *base && addr - *base < window.len())
    };
    assert!(
        !covered(0xffff_fff0),
        "the reset vector is a device region, and hardware cannot fetch from one"
    );
    assert!(covered(0x1000), "and what it jumps to is hardware-backed");
}

/// The processor interprets its way off the reset vector and then runs in
/// hardware.
///
/// No kernel: `x86.linuxboot` with an empty socket still synthesises the far
/// jump, and what is at `0000:1000` afterwards is zeroed RAM — which is a
/// memory slot, so the guest executes it on the host's own silicon. The two
/// numbers are the assertion: a *handful* of interpreted instructions (the
/// jump, and the shell's own reset sequence), and guest entries after them.
#[test]
fn the_reset_vector_is_interpreted_and_everything_after_it_is_not() {
    let Some((mut m, _accel, cpu, _console)) = bare_board() else {
        return;
    };
    for _ in 0..8 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
    }
    assert!(
        !cpu.is_stopped(),
        "the processor stopped: {:?}",
        cpu.failure()
    );
    assert!(
        cpu.entries() > 0,
        "the processor never entered the guest ({} interpreted)",
        cpu.interpreted()
    );
    assert!(
        (1..=64).contains(&cpu.interpreted()),
        "the interpreter should run the reset vector and hand over, not the board: {} \
         instruction(s)",
        cpu.interpreted()
    );
    // And it really did leave the top of the address space.
    let regs = cpu.shell().regs();
    assert!(
        regs.rip < 0x10_0000,
        "the processor is in low memory, at {:04x}:{:#x}",
        regs.cs,
        regs.rip
    );
}

// ---------------------------------------------------------------------------
// and the kernel
// ---------------------------------------------------------------------------

#[test]
fn a_linux_kernel_boots_on_host_silicon_and_reads_the_disk() {
    if !Kvm::is_available() {
        println!("q35-linux/kvm: no usable /dev/kvm on this host; skipping");
        return;
    }
    let Ok(path) = std::env::var("RSEMU_KERNEL") else {
        println!(
            "q35-linux/kvm: set RSEMU_KERNEL to a Linux/x86 bzImage to boot one on host \
             silicon; see the module docs"
        );
        return;
    };
    let kernel = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let initrd = std::env::var("RSEMU_INITRD")
        .ok()
        .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
        .unwrap_or_default();
    let disk = match std::env::var("RSEMU_DISK") {
        Ok(p) => std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")),
        Err(_) => SIGNATURE.to_vec(),
    };
    println!(
        "q35-linux/kvm: {} bytes of kernel, {} bytes of initramfs, {} bytes into the namespace",
        kernel.len(),
        initrd.len(),
        disk.len()
    );

    let cmdline = std::env::var("RSEMU_KERNEL_CMDLINE").unwrap_or_else(|_| CMDLINE.to_string());
    println!("q35-linux/kvm: command line {cmdline:?}");
    let mut params: Vec<(&str, String)> = vec![("cmdline", cmdline)];
    if let Ok(extmem) = std::env::var("RSEMU_KERNEL_EXTMEM") {
        params.push(("extmem", extmem));
    }
    let (mut m, accel, cpu, console) = match board(kernel, initrd, disk, &params) {
        Ok(built) => built,
        Err(e) => panic!("the board does not realize under acceleration: {e}"),
    };
    if let Some(plan) = accel.plan() {
        println!("q35-linux/kvm: what runs in hardware:");
        for line in plan.describe().lines() {
            println!("  # {line}");
        }
    }
    let ms: u64 = std::env::var("RSEMU_KERNEL_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_MS);

    let script = Script::from_env();
    println!("q35-linux/kvm: what the guest wrote to its serial port at 0x3f8:");
    let started = std::time::Instant::now();
    let run = x86boot::run(
        &mut m,
        cpu.shell(),
        &console,
        GlobalTime::from_nanos(ms * 1_000_000),
        &script,
    );
    let wall = started.elapsed();
    x86boot::report("q35-linux/kvm", &m, cpu.shell(), &run, &script);
    // The numbers that say it was a hypervisor doing the work: entries into
    // the guest, exits routed back into this board's own device models, and
    // the handful of instructions the interpreter ran because a device region
    // cannot be fetched from.
    println!(
        "q35-linux/kvm: {:?}, {} interpreted, halted={}, stopped={} ({:?})",
        cpu.vcpu().map(|v| v.stats()),
        cpu.interpreted(),
        cpu.is_halted(),
        cpu.is_stopped(),
        cpu.failure()
    );
    println!(
        "q35-linux/kvm: {} ms of guest time in {:.1} s of wall clock; the interpreted run of \
         this board on this same command line measured 2,826,342 ms and 978 s",
        run.at.as_nanos() / 1_000_000,
        wall.as_secs_f64()
    );
    assert!(
        !cpu.is_stopped(),
        "the processor stopped: {:?}",
        cpu.failure()
    );
    assert!(run.long, "the kernel never reached long mode");
    x86boot::assert_booted(&run, &script);
}
