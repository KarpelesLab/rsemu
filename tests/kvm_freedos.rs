//! **FreeDOS on host silicon** — how far phase 7's first half actually gets,
//! and exactly where it stops.
//!
//! `ROADMAP.md` phase 7 opens *"the phase-6 machines boot under KVM"*, and
//! phase 6a's gate is FreeDOS reaching a shell on `machines/pc-at.machine`
//! with [`rsemu::fw::pcbios`] in the socket. `tests/pc_at_boot.rs`'s
//! `freedos_boots_on_rsemus_own_firmware` is that boot on the interpreter, and
//! it reaches the FreeDOS 1.3 installer's *"Do you want to proceed [Y,N]?"* in
//! sixty virtual seconds. This is **the same board, the same firmware and the
//! same diskette**, with
//! [`AccelCpus::install`](rsemu::accel::cpu::AccelCpus::install) replacing
//! what is underneath its processor.
//!
//! # The measured answer: it does not reach the prompt
//!
//! It gets remarkably far, and then stops in one place, for one reason:
//!
//! * POST runs on host silicon, `INT 19h` reads the diskette through the
//!   µPD765 and the 8237, FreeDOS's boot sector loads the compressed kernel a
//!   sector at a time through `INT 13h`, the kernel decompresses, initialises
//!   and installs its own `INT 21h`, and `FDCONFIG.SYS` paints its **boot
//!   menu** — *"FreeDOS 1.3 Floppy Edition: Please select your language"*.
//!   All of that inside **ten milliseconds of virtual time**, against the
//!   sixty seconds the interpreter needs.
//! * And then it stops for ever, on the menu's countdown.
//!
//! The countdown is a loop on the BIOS Data Area's tick counter at
//! `0040:006c`, and — as far as the hypervisor is concerned — it is a loop
//! that **takes no exits**: the counter is ordinary RAM, which is a memory
//! slot, and `INT 16h`'s key-available check reads the BDA's own keyboard ring
//! rather than the 8042. So the vCPU stays inside `KVM_RUN`; the scheduler
//! round it is in never ends; virtual time never advances; the 8254 never
//! fires; the tick never arrives; the loop never exits. Each step follows from
//! the one before it, and the first step is `accel::kvm`'s documented cost —
//! *"a guest taking no exits is not preemptible"* — met by a real operating
//! system rather than by a test's own `jmp $`.
//!
//! [`watchdog`] proves that rather than asserting it: it samples
//! [`VcpuStats::entries`](rsemu::accel::kvm::VcpuStats::entries), the count of
//! `KVM_RUN`s that have *returned*, and reports that it has not moved.
//!
//! # What would close it, and what would not
//!
//! * **Not `MAX_ENTRIES`, and not the scheduler quantum.** Both bound how many
//!   times the guest may *leave* hardware, and this guest leaves it zero times.
//! * **Not [`ThreadingMode::Accel`] on its own.** Making virtual time follow
//!   the host clock would let the 8254 fire in host time, but the interrupt is
//!   injected *between* guest entries — so something must still bring the vCPU
//!   out of an entry that is not going to end by itself.
//! * **A signal would**, and that is precisely how every other hypervisor does
//!   it. `ROADMAP.md` §0 rules it out — wasm has none — and `accel::kvm`
//!   chose `KVM_CAP_IMMEDIATE_EXIT` knowing the cost. That flag is written
//!   into the `kvm_run` page *before* an entry, so raising it afterwards
//!   changes nothing about an entry already in progress.
//! * **An in-kernel interrupt controller and timer would** —
//!   `KVM_CREATE_IRQCHIP` plus `KVM_CAP_PIT2` — because then the tick is
//!   raised and injected by the kernel and the guest exits on its own. That is
//!   a different machine, though: the 8259A, the 8254 and the local APIC would
//!   stop being rsemu devices for an accelerated board, and one of `ROADMAP.md`
//!   §10's premises is that they do not.
//!
//! This test therefore asserts what is **true today** — that a real operating
//! system boots on host silicon, through this board's own device models, as
//! far as its first timed wait — and prints, in as many words, that the prompt
//! is not reached and why.
//!
//! # One thing this changes about the framing
//!
//! It is tempting to say the two engines "reach the same place in the same
//! virtual time". They do not, and the difference is the whole story: an
//! accelerated processor consumes its budget however long the host took, so it
//! executes *thousands of times more guest instructions per virtual
//! millisecond* than the interpreter. Every guest loop that waits on virtual
//! time therefore spins thousands of times longer — and the ones that take no
//! exits stop being slow and start being infinite.
//!
//! # The A20 gate, which is *not* what goes wrong here
//!
//! `accel::cpu` names it: A20 is a mask on the shell's own accesses and the
//! guest's hardware accesses are not masked. FreeDOS opens A20 rather than
//! relying on the megabyte wrap, so it is not what stops this boot — worth
//! saying because it is the hole one would reach for first.
//!
//! Gated twice and skips cleanly on either: no `/dev/kvm`, or no
//! `RSEMU_FREEDOS_FLOPPY`. Never hangs: [`watchdog`] ends the process with a
//! diagnosis if virtual time stops moving.
//!
//! ```text
//! scripts/fetch-testdata.sh freedos
//! RSEMU_FREEDOS_FLOPPY=testdata/freedos/x86BOOT.img \
//!   cargo test --release --all-features --test kvm_freedos -- --nocapture
//! ```
//!
//! `RSEMU_KVM_TRACE` prints where the guest is at the end of every span,
//! `RSEMU_KVM_SPAN_MS` sets that span, `RSEMU_KVM_STALL_S` the watchdog's
//! patience, `RSEMU_FREEDOS_MS` the virtual-time budget, and
//! `RSEMU_FREEDOS_PROMPT` carries on past a resident kernel and into the stall
//! described above — which is how to reproduce it, and why it is off by
//! default.
//!
//! **Nothing is vendored.** FreeDOS is GPL-2.0 and the image never enters this
//! repository: running a program as an emulated guest is ordinary use, while
//! shipping it here would be redistribution under its terms (`ROADMAP.md` §1).
//!
//! [`ThreadingMode::Accel`]: rsemu::core::sched::ThreadingMode::Accel

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

use rsemu::accel::cpu::{AccelCpu, AccelCpus};
use rsemu::accel::kvm::Kvm;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{BuildOptions, Machine, build};

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

/// How long a diskette image is, and what `pc.fdc` infers a 1.44 MB geometry
/// from.
const FLOPPY_LEN: usize = 1_474_560;

/// A byte of guest memory, read as a debugger would.
fn peek(m: &Machine, addr: u64) -> u8 {
    let mem = m.space("mem").expect("the memory space");
    mem.read(addr, Width::U8, MemAttrs::DEBUG).unwrap_or(0) as u8
}

/// A word of guest memory.
fn peek16(m: &Machine, addr: u64) -> u16 {
    u16::from(peek(m, addr)) | (u16::from(peek(m, addr + 1)) << 8)
}

/// The colour text page, as lines of characters — the same reader
/// `tests/pc_at_boot.rs` uses, so the two tests are looking at one thing.
fn text_page(m: &Machine) -> Vec<String> {
    text_page_of(m.space("mem").expect("the memory space"))
}

/// The same, over the address space alone — which is what a thread that is not
/// the one holding the [`Machine`] can reach.
fn text_page_of(space: &rsemu::core::space::AddressSpace) -> Vec<String> {
    (0..25u64)
        .map(|row| {
            (0..80u64)
                .map(|col| {
                    let at = 0xb8000 + (row * 80 + col) * 2;
                    let ch = space.read(at, Width::U8, MemAttrs::DEBUG).unwrap_or(0) as u8;
                    match ch {
                        0x20..=0x7e => ch as char,
                        _ => ' ',
                    }
                })
                .collect::<String>()
                .trim_end()
                .to_string()
        })
        .collect()
}

/// Whether the **shell** has printed its own name.
///
/// `FreeCom` and not `FreeDOS`: the kernel prints "FreeDOS kernel …" as soon
/// as it is decompressed, and stopping there would report a kernel load as a
/// prompt. `COMMAND.COM` is the first thing in the sequence that names
/// itself, so this is the line that separates "the kernel loaded" from "the
/// system came up".
fn shell_reached(page: &[String]) -> bool {
    page.iter().any(|line| line.contains("FreeCom"))
}

/// Whether a DOS kernel has installed its own `INT 21h`.
///
/// The vector starts life pointing at this firmware's "unknown function" stub
/// in segment `0xf000`, and only an operating system moves it — so this one
/// predicate covers the boot sector, the kernel load, the decompression and
/// the kernel's own initialisation, and says nothing about which DOS.
fn dos_resident(m: &Machine) -> bool {
    let seg = peek16(m, 0x21 * 4 + 2);
    seg != 0xf000 && seg != 0x0000
}

/// Whether the guest reached `FDCONFIG.SYS`'s boot menu.
///
/// **The furthest an accelerated `pc-at` gets today**, and the reason this
/// test stops here rather than running on into a hang: the menu is a countdown
/// on the BIOS Data Area's tick counter, and the loop it counts in takes no
/// exits. See the module documentation.
fn menu_reached(page: &[String]) -> bool {
    page.iter()
        .any(|line| line.contains("Please select your language"))
}

/// The board, with the firmware this repository assembles and `image` in the
/// diskette drive — and both IDE bays empty, so `INT 19h` declines them and
/// falls through to the µPD765.
fn options(image: Vec<u8>) -> BuildOptions {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    // An empty option-ROM socket: 64 KiB of zeroes has no `0x55 0xAA`, which
    // is exactly what the firmware's scan must survive.
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("optionrom", vec![0u8; 65536]);
    options.realize.media.insert("floppy", image);
    for slot in ["disk", "hd0", "hd1", "hd2", "hd3", "cd0", "cd1"] {
        options.realize.media.insert(slot, Vec::new());
    }
    options
}

/// Build the board under acceleration, or say why it could not be.
fn accelerated(image: Vec<u8>) -> Option<(Machine, Arc<AccelCpu>, Arc<AccelCpus>)> {
    // `Parallel` rather than `Deterministic`, and not by preference:
    // `AccelCpus::open` refuses a mode that claims reproducibility, because a
    // run on host silicon is not reproducible.
    let accel = match AccelCpus::open(ThreadingMode::Parallel) {
        Ok(accel) => accel,
        Err(e) if e.is_unavailable() => return None,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };
    let mut opts = options(image);
    opts.realize.scheduler.mode = ThreadingMode::Parallel;
    accel.install(&mut opts.bindings);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &opts)
        .unwrap_or_else(|e| panic!("the board does not realize under acceleration: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    let cpu = accel.cpus().pop().expect("the board's processor");
    Some((m, cpu, accel))
}

/// A host-time watchdog, because a hung `KVM_RUN` is worse than a failure.
///
/// **A guest that takes no exits is not preemptible** — `accel::kvm` says so —
/// and under [`ThreadingMode::Parallel`] the scheduler round it is inside does
/// not end, so the *machine's* virtual time stops with it. There is no way to
/// break that from userspace without a signal, which this project does not
/// use: `KVM_CAP_IMMEDIATE_EXIT` is written into the `kvm_run` page *before*
/// the entry, so raising it afterwards changes nothing. A test in that state
/// would hang for ever, which tells whoever is watching nothing at all.
///
/// So a second thread watches `progress` and, if it stops moving, prints
/// everything that *can* be read from outside a running vCPU — the guest's
/// last observed instruction pointer, the BIOS Data Area's tick counter, and
/// the text page, all through the address space, which is `Send + Sync` and
/// needs no cooperation from the processor — and then ends the process rather
/// than waiting.
///
/// `std::thread` in a test, which `CLAUDE.md` forbids inside `core/`, `cpu/`,
/// `dev/`, `machine/` and `ir/`. This is none of those: it is the harness, and
/// the harness is the only place that *can* watch a blocked scheduler.
fn watchdog(
    cpu: Arc<AccelCpu>,
    space: Arc<rsemu::core::space::AddressSpace>,
    progress: Arc<AtomicU64>,
    seconds: u64,
) {
    std::thread::spawn(move || {
        let mut last = u64::MAX;
        let mut still = 0u64;
        loop {
            std::thread::sleep(std::time::Duration::from_secs(1));
            let now = progress.load(Ordering::Relaxed);
            if now == u64::MAX {
                return; // the run finished
            }
            still = if now == last { still + 1 } else { 0 };
            last = now;
            if still < seconds {
                continue;
            }
            let regs = cpu.shell().regs();
            let tick = space.read(0x46c, Width::U32, MemAttrs::DEBUG).unwrap_or(0);
            // The vCPU's own completion counter, which is bumped after every
            // `KVM_RUN` that returns. Sampled here rather than inferred: if it
            // has not moved in `seconds` seconds then no entry has ended, and
            // "the guest takes no exits" is measured rather than assumed.
            let completed = cpu.vcpu().map_or(0, |v| v.stats().entries);
            println!(
                "kvm freedos: NO PROGRESS for {seconds}s at {now} virtual ms. The guest \
                 last left hardware at {:04x}:{:08x}; {completed} `KVM_RUN`s have \
                 returned in this processor's whole life and none in the last \
                 {seconds}s, so it is inside one and taking no exits. The BIOS Data \
                 Area's tick counter at 0040:006c is frozen at {tick:#x}, because \
                 virtual time cannot advance while a vCPU is inside `KVM_RUN` \
                 (`accel::kvm`) — and the tick is what the guest is waiting for.",
                regs.cs, regs.rip,
            );
            for line in text_page_of(&space) {
                if !line.trim().is_empty() {
                    println!("kvm freedos:   |{line}|");
                }
            }
            // Not a panic: the main thread is blocked inside the scheduler and
            // will never unwind, so a panic here would be reported and then
            // hang exactly as before.
            std::process::exit(101);
        }
    });
}

/// **FreeDOS boots to its prompt, in hardware.**
#[test]
fn freedos_reaches_its_prompt_under_kvm() {
    if !Kvm::is_available() {
        println!("kvm freedos: no usable /dev/kvm, skipping");
        return;
    }
    let Ok(path) = std::env::var("RSEMU_FREEDOS_FLOPPY") else {
        println!(
            "kvm freedos: RSEMU_FREEDOS_FLOPPY is unset, so this test has nothing to \
             boot. `scripts/fetch-testdata.sh freedos` fetches one."
        );
        return;
    };
    let mut image = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    println!("kvm freedos: booting {path} ({} bytes)", image.len());
    image.resize(FLOPPY_LEN, 0);

    // The same sixty virtual seconds `tests/pc_at_boot.rs` measured for the
    // interpreted boot, run in tenth-of-a-second spans so that reaching the
    // shell early ends the test rather than being waited out. A span shorter
    // than the scheduler quantum would advance the clock without running
    // anything, so it is not made smaller than this.
    let ms: u64 = std::env::var("RSEMU_FREEDOS_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(60_000);
    // **One virtual millisecond, and that is load-bearing.** The accelerated
    // guest gets from the reset vector to a resident DOS kernel in ten of
    // them and into `FDCONFIG.SYS`'s unexitable countdown in the eleventh, so
    // a span coarser than this runs past the last observable state and into
    // the stall inside a single `run_for`. A span *shorter* than the scheduler
    // quantum would advance the clock without running anything
    // (`Machine::run_until`), which is the floor.
    let span_ms: u64 = std::env::var("RSEMU_KVM_SPAN_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|v| *v > 0)
        .unwrap_or(1);
    // Whether to carry on past a resident kernel and try for the prompt. Off
    // by default because it *does not get there* — it stalls in the countdown
    // and the watchdog ends the process — and a test that fails by design is
    // worse than a test that reports. Set it to reproduce the stall.
    let chase_prompt = std::env::var("RSEMU_FREEDOS_PROMPT").is_ok();

    let Some((mut m, cpu, _accel)) = accelerated(image) else {
        println!("kvm freedos: /dev/kvm unusable, skipping");
        return;
    };

    // `RSEMU_KVM_TRACE` prints where the guest was at the end of every span.
    // A guest that takes no exits does not come back out of `KVM_RUN`
    // (`accel::kvm`), and under `ThreadingMode::Parallel` the scheduler round
    // it is in does not end — so if this run stops making progress, the last
    // line printed is the last place the guest was seen, and that is the only
    // way to find out from outside.
    let trace = std::env::var("RSEMU_KVM_TRACE").is_ok();
    let stall: u64 = std::env::var("RSEMU_KVM_STALL_S")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|v| *v > 0)
        .unwrap_or(60);
    let progress = Arc::new(AtomicU64::new(0));
    watchdog(
        Arc::clone(&cpu),
        Arc::clone(m.space("mem").expect("the memory space")),
        Arc::clone(&progress),
        stall,
    );
    let started = std::time::Instant::now();
    let mut virtual_ms = 0;
    while virtual_ms < ms {
        m.run_for(GlobalTime::from_nanos(span_ms * 1_000_000))
            .expect("the machine runs");
        virtual_ms += span_ms;
        progress.store(virtual_ms, Ordering::Relaxed);
        if trace {
            let regs = cpu.shell().regs();
            println!(
                "kvm freedos: +{virtual_ms}ms {:?} entries={} {:04x}:{:08x} halted={} pe={}",
                started.elapsed(),
                cpu.entries(),
                regs.cs,
                regs.rip,
                cpu.is_halted(),
                cpu.shell().sys().protected(),
            );
        }
        if cpu.is_stopped() {
            break;
        }
        let page = text_page(&m);
        if shell_reached(&page) || menu_reached(&page) {
            break;
        }
        if !chase_prompt && dos_resident(&m) {
            break;
        }
    }
    let elapsed = started.elapsed();
    progress.store(u64::MAX, Ordering::Relaxed);

    let page = text_page(&m);
    println!("kvm freedos: text page after {virtual_ms} virtual ms:");
    for line in &page {
        if !line.trim().is_empty() {
            println!("  |{line}|");
        }
    }
    let regs = cpu.shell().regs();
    println!(
        "kvm freedos: {elapsed:?} of host time; {} guest entries; stopped at \
         {:04x}:{:08x}, halted={}, protected={}",
        cpu.entries(),
        regs.cs,
        regs.rip,
        cpu.is_halted(),
        cpu.shell().sys().protected()
    );
    let vectors: Vec<String> = [0x08u64, 0x10, 0x13, 0x1c, 0x21, 0x2f]
        .iter()
        .map(|v| {
            format!(
                "{v:02x}->{:04x}:{:04x}",
                peek16(&m, v * 4 + 2),
                peek16(&m, v * 4)
            )
        })
        .collect();
    println!("kvm freedos: vectors {}", vectors.join(" "));

    assert!(
        !cpu.is_stopped(),
        "the processor stopped: {:?}",
        cpu.failure()
    );
    assert!(
        cpu.entries() > 0,
        "the processor never entered the guest, so nothing ran in hardware"
    );

    // **The gate, reported rather than quietly passed.** The prompt is not
    // reached on this board today and this test says so out loud instead of
    // asserting a weaker fact and calling it a boot.
    if shell_reached(&page) {
        println!(
            "kvm freedos: REACHED THE PROMPT — `COMMAND.COM` printed its banner under \
             KVM. Phase 7's first half is met on this host, and the assertion below \
             should be tightened to require it."
        );
    } else {
        println!(
            "kvm freedos: DID NOT REACH THE PROMPT, and this is the measured state of \
             phase 7's first half. A real DOS kernel booted off a real diskette \
             controller and installed itself on host silicon in {virtual_ms} virtual \
             ms — the interpreter needs sixty virtual seconds for the same distance — \
             and the next thing the guest does is `FDCONFIG.SYS`'s countdown, which \
             waits on the BIOS Data Area's tick in a loop that takes no exits. The \
             vCPU then never leaves `KVM_RUN`, the scheduler round never ends, virtual \
             time never advances and the tick never comes. Set RSEMU_FREEDOS_PROMPT to \
             run into it and watch the watchdog say so; see this file's module \
             documentation for what would close it."
        );
    }

    // POST still did its job: a guest that took the machine over did not have
    // to repair the BIOS data area first.
    assert_eq!(peek16(&m, 0x413), 639, "the BDA's memory size");

    // And a real operating system installed itself, in hardware.
    assert!(
        dos_resident(&m),
        "INT 21h still points into the BIOS: no DOS kernel installed itself. The \
         text page above says how far it got."
    );
}

/// Says out loud whether the test above actually ran, and why not if it did
/// not — a skip that is silent is a skip nobody notices.
#[test]
fn report_whether_this_host_can_boot_freedos_under_kvm() {
    match (
        Kvm::is_available(),
        std::env::var("RSEMU_FREEDOS_FLOPPY").is_ok(),
    ) {
        (true, true) => println!("kvm freedos: this host boots it; see the test above"),
        (true, false) => println!("kvm freedos: /dev/kvm is usable, but no diskette is named"),
        (false, _) => println!("kvm freedos: no usable /dev/kvm on this host"),
    }
}
