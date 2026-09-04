//! Booting a real AArch64 Linux kernel on `arm64-virt`, to a shell.
//!
//! Gated on an environment variable and **skips cleanly** when it is unset, so
//! `cargo test` needs no download and no toolchain. The kernel and the busybox
//! inside the ramdisk are GPL-2.0 binaries: running one as an emulated guest is
//! ordinary use, and committing one here would be redistribution under its
//! terms (`ROADMAP.md` §1), so `scripts/fetch-testdata.sh` fetches them into an
//! ignored directory and nothing is vendored.
//!
//! ```text
//!   scripts/fetch-testdata.sh arm64-linux arm64-initramfs
//!
//!   RSEMU_ARM64_KERNEL=testdata/arm64/linux \
//!   RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \
//!       cargo test --release --features machine-arm64-virt \
//!           --test a64_linux -- --nocapture
//! ```
//!
//! It is a `--release` test in practice: a whole Linux boot is a few hundred
//! million interpreted instructions and takes about three minutes optimised.
//! `docs/platforms/arm64-virt.md` has the transcript and the ledger.
//!
//! # `RSEMU_ARM64_TRACE`
//!
//! Set it and the run steps in small slices and stops at the **first**
//! exception the guest takes, printing `ESR_EL1`, `ELR_EL1` and `FAR_EL1`.
//! That is the diagnostic that found both of the core bugs this board turned
//! up: once a core is looping in a vector table, every trip round overwrites
//! the syndrome with the second-order fault, and what is left says nothing.

#![cfg(all(feature = "machine-arm64-virt", feature = "std"))]

use std::sync::Arc;
use std::time::Instant;

use rsemu::dev::arm::power::{Request, Signal, signals};
use rsemu::host::chardev::{CharPort, ports};
use rsemu::machine::{Machine, catalog};

/// The kernel `Image`, if one was named.
fn kernel() -> Option<Vec<u8>> {
    let path = std::env::var("RSEMU_ARM64_KERNEL").ok()?;
    Some(
        std::fs::read(&path).unwrap_or_else(|e| {
            panic!("RSEMU_ARM64_KERNEL names `{path}`, which will not read: {e}")
        }),
    )
}

/// The ramdisk, if one was named.
fn initrd() -> Vec<u8> {
    match std::env::var("RSEMU_ARM64_INITRD") {
        Ok(path) => std::fs::read(&path)
            .unwrap_or_else(|e| panic!("RSEMU_ARM64_INITRD names `{path}`: {e}")),
        Err(_) => Vec::new(),
    }
}

/// An environment variable with a default.
fn env_or(name: &str, fallback: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| fallback.to_string())
}

struct Board {
    machine: Machine,
    console: Arc<CharPort>,
    power: Arc<Signal>,
}

fn board(kernel: &[u8], initrd: &[u8]) -> Board {
    let entry = catalog::machine("arm64-virt").expect("this build ships it");
    let options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_media("kernel", kernel)
        .with_media("initrd", initrd)
        .with_param("ram", env_or("RSEMU_ARM64_RAM", "1G"))
        .with_param(
            "cmdline",
            env_or(
                "RSEMU_ARM64_CMDLINE",
                // `earlycon` is what makes a failed boot legible: it prints
                // through the PL011 before the driver model exists, so a board
                // whose console driver never probes still says what happened.
                "earlycon=pl011,0x9000000 console=ttyAMA0 rdinit=/init",
            ),
        );
    let registry = catalog::registry().expect("a registry");
    let machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("arm64-virt does not build: {e}"),
    };
    Board {
        console: ports::open(&options.realize.hosts, "console").expect("the PL011 opened it"),
        power: signals::open(&options.realize.hosts, "power").expect("the controller opened it"),
        machine,
    }
}

/// What the core is doing, read out of its snapshot chunk.
///
/// There is no route from a `dyn Device` to a `Cpu` — `core::device` keeps
/// `Any` out of the supertrait chain deliberately — so this reads the register
/// file the same way `host::gdb` does: by asking the device to save itself and
/// counting to the fields. `cpu.arm.a64`'s chunk is X0-X30, the 32 SIMD&FP
/// registers as two words each, the program counter, three counters, two
/// flags, `PSTATE`, and then thirty system-register words in the order
/// `Cpu::sysreg_words` writes them.
#[derive(Debug, Clone, Copy)]
struct CpuState {
    pc: u64,
    esr: u64,
    elr: u64,
    far: u64,
    vbar: u64,
    sctlr: u64,
}

impl std::fmt::Display for CpuState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // `ESR_ELx.EC` is bits 31:26 and is the field that says *what* went
        // wrong; 0x00 is "unknown reason", which on this core means an
        // instruction or a system register it does not implement.
        write!(
            f,
            "pc {:#x}  esr {:#x} (ec {:#04x}, iss {:#x})  elr {:#x}  far {:#x}  \
             vbar {:#x}  sctlr {:#x}",
            self.pc,
            self.esr,
            (self.esr >> 26) & 0x3f,
            self.esr & 0x01ff_ffff,
            self.elr,
            self.far,
            self.vbar,
            self.sctlr
        )
    }
}

fn cpu_state(m: &Machine) -> CpuState {
    use rsemu::core::state::{MachineShape, Migrations, Source, StateReader, StateWriter};
    let entry = m.device("cpu").expect("the board has a cpu");
    let class = entry.device().class().name;
    let version = entry.device().class().version;
    let mut shape = MachineShape::new();
    shape.add_device("cpu", class).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", class, version).unwrap();
        entry.device().save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("cpu", class, version, &Migrations::new())
        .unwrap();
    let mut r = chunk.reader();
    for _ in 0..31 + 64 {
        r.read_u64().unwrap();
    }
    let pc = r.read_u64().unwrap();
    for _ in 0..3 {
        r.read_u64().unwrap(); // cycles, debt, faults
    }
    r.read_bool().unwrap(); // wfi
    if r.read_bool().unwrap() {
        r.read_u64().unwrap(); // the exclusive monitor's address
    }
    r.read_u32().unwrap(); // NZCV
    r.read_u64().unwrap(); // DAIF
    r.read_u8().unwrap(); // the exception level
    r.read_bool().unwrap(); // SPSel
    let mut sys = [0u64; 30];
    for slot in &mut sys {
        *slot = r.read_u64().unwrap();
    }
    CpuState {
        pc,
        esr: sys[13],
        elr: sys[12],
        far: sys[14],
        vbar: sys[15],
        sctlr: sys[2],
    }
}

/// What to type at the guest, and what to wait for before typing it.
struct Script {
    /// Type `send` the first time the console has printed this.
    after: &'static str,
    /// What to type, including its newline.
    send: &'static str,
}

/// Run until the guest stops the machine, until `quanta` have gone by, or
/// until nothing new has been printed for a long time — printing as it goes,
/// because the log *is* the result.
fn run(b: &mut Board, quanta: usize, script: Option<Script>) -> String {
    // Checking the core's state costs a whole snapshot per slice, so it is
    // opt-in: `RSEMU_ARM64_TRACE=1` is what a person debugging a fault sets,
    // and it stops at the first exception rather than at the log's end.
    let watch = std::env::var("RSEMU_ARM64_TRACE").is_ok();
    let mut out = Vec::new();
    let start = Instant::now();
    let mut last_len = 0;
    let mut idle = 0usize;
    let mut history: Vec<CpuState> = Vec::new();
    let mut typed = false;
    for round in 0..quanta {
        if b.power.peek().is_some() {
            break;
        }
        if watch {
            // A few thousand instructions at a time rather than a whole
            // quantum, so the *first* exception is the one reported.
            b.machine
                .run_for(rsemu::core::clock::GlobalTime::from_nanos(20_000))
                .expect("the machine advances");
        } else {
            b.machine.run_quantum().expect("the machine advances");
        }
        b.console.drain_into(&mut out);
        if watch {
            let st = cpu_state(&b.machine);
            // `ESR_EL1` is zero out of reset and is written only on exception
            // entry, so the first time it is not zero is the first exception
            // the guest took — and that is the one worth reporting.
            if st.esr != 0 {
                eprintln!("\n--- first exception, at round {round} ---");
                for (i, past) in history.iter().enumerate() {
                    eprintln!("    -{}: {past}", history.len() - i);
                }
                eprintln!("    now: {st}");
                break;
            }
            history.push(st);
            if history.len() > 6 {
                history.remove(0);
            }
        }
        if out.len() == last_len {
            idle += 1;
            // Sixty thousand slices with nothing printed is a guest that has
            // stopped moving, and the log up to that point is what the next
            // person needs.
            if idle > 60_000 {
                eprintln!("\n--- nothing printed for {idle} slices, at round {round} ---");
                break;
            }
        } else {
            idle = 0;
            last_len = out.len();
            if let Some(script) = &script
                && !typed
                && String::from_utf8_lossy(&out).contains(script.after)
            {
                typed = true;
                b.console.feed(script.send.as_bytes());
            }
        }
    }
    let text = String::from_utf8_lossy(&out).into_owned();
    eprintln!("{text}");
    eprintln!(
        "--- {} byte(s) of console in {:?}, power: {:?} ---\n    {}",
        out.len(),
        start.elapsed(),
        b.power.peek(),
        cpu_state(&b.machine)
    );
    text
}

fn quanta(fallback: &str) -> usize {
    env_or("RSEMU_ARM64_QUANTA", fallback)
        .parse()
        .expect("RSEMU_ARM64_QUANTA is a number of scheduler quanta")
}

#[test]
fn a_real_kernel_boots_and_runs_init() {
    let Some(image) = kernel() else {
        eprintln!(
            "RSEMU_ARM64_KERNEL is not set, so this test does nothing.\n\
             scripts/fetch-testdata.sh arm64-linux arm64-initramfs"
        );
        return;
    };
    let ramdisk = initrd();
    let mut b = board(&image, &ramdisk);
    let text = run(&mut b, quanta("600000"), None);

    // Each of these was broken at some point on the way here, so each is
    // asserted rather than merely printed: a regression should be a failure
    // and not a quieter log.
    assert!(
        text.contains("Booting Linux"),
        "the kernel did not reach its own banner"
    );
    assert!(
        text.contains("Machine model: rsemu arm64-virt"),
        "the kernel did not parse the generated device tree"
    );
    assert!(
        text.contains("Freeing unused kernel memory"),
        "the kernel did not finish its own initialisation"
    );
    if ramdisk.is_empty() {
        // Without a ramdisk there is nothing to be `/init`, and the kernel
        // says so rather than reaching userspace.
        return;
    }
    assert!(
        text.contains("Run /init as init process"),
        "the kernel did not reach userspace"
    );
    assert!(
        text.contains("rsemu initramfs on Linux"),
        "`/init` ran but did not get as far as its own banner"
    );
    assert!(
        text.contains("BusyBox"),
        "the shell did not start; the console said {text:?}"
    );
}

#[test]
fn a_poweroff_typed_at_the_shell_stops_the_machine() {
    // PSCI end to end, from userspace: busybox's `poweroff` asks the kernel to
    // power the machine off, the kernel's PSCI driver executes `SMC` with
    // `SYSTEM_OFF` in `x0`, `cpu.arm.a64` services it, the core pulses its
    // `poweroff` pin, and `arm.power` raises the host signal this asserts on.
    // Every one of those five is a thing this board grew.
    let Some(image) = kernel() else {
        return;
    };
    let ramdisk = initrd();
    if ramdisk.is_empty() {
        eprintln!("RSEMU_ARM64_INITRD is not set, so there is no shell to type at");
        return;
    }
    let mut b = board(&image, &ramdisk);
    let text = run(
        &mut b,
        quanta("900000"),
        Some(Script {
            after: "BusyBox",
            send: "poweroff -f\n",
        }),
    );
    assert!(text.contains("BusyBox"), "there was no shell to type at");
    assert_eq!(
        b.power.peek(),
        Some(Request::Poweroff),
        "the guest did not stop the machine"
    );
}
