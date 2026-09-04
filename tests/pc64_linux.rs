//! How far does a modern Linux kernel get on the `pc64` board?
//!
//! Gated on `RSEMU_KERNEL`, exactly as `tests/pc_at_firmware.rs` is gated on
//! `RSEMU_BIOS`: point it at a `bzImage` and it runs one, and without it the
//! test prints why and returns. No kernel is vendored, downloaded by
//! `cargo test`, or required for it (`CLAUDE.md`, Testing).
//!
//! ```text
//! RSEMU_KERNEL=/boot/vmlinuz \
//! RSEMU_INITRD=/boot/initramfs.img \
//! cargo test --release --features machine-pc64 --test pc64_linux -- --nocapture
//! ```
//!
//! | Variable | What it does |
//! | --- | --- |
//! | `RSEMU_KERNEL` | The `bzImage`. Without it the test skips. |
//! | `RSEMU_INITRD` | An initramfs. Optional; without one the kernel panics for want of a root, which is still a complete boot. |
//! | `RSEMU_KERNEL_MS` | How long to run, in virtual milliseconds. |
//! | `RSEMU_KERNEL_CMDLINE` | The command line, replacing the board's default. |
//! | `RSEMU_KERNEL_EXTMEM` | How much extended memory to give it, e.g. `512M`. |
//! | `RSEMU_KERNEL_INPUT` | Types at the guest: one `marker=>text` step per line. |
//! | `RSEMU_KERNEL_STOP_AT` | Ends the run when the guest prints this, rather than at the clock. |
//! | `RSEMU_ENGINE` | `interp`, `jit` or `jit-host`, overriding what the machine file's `engine` property said. The three must produce byte-identical output and the same virtual time; what differs is the wall clock. |
//!
//! `RSEMU_KERNEL_INPUT` is `src/dev/riscv/tests.rs`'s `RSEMU_RISCV_INPUT` on
//! this board, and it is here for the same reason: **a prompt that echoes what
//! is typed at it is the only proof that the console is bidirectional**, and
//! everything up to that point exercises the 16550's transmitter alone. Each
//! step waits for its marker in the guest's output and then feeds its text to
//! the console, so what triggers a keystroke is something the guest *said*
//! rather than elapsed time — the run stays deterministic and no wall clock is
//! consulted anywhere in the loop. `\n`, `\r`, `\t` and `\\` are the escapes.
//!
//! ```text
//! RSEMU_KERNEL_INPUT='rsemu# =>uname -srm\n'
//! RSEMU_KERNEL_STOP_AT='x86_64'
//! ```
//!
//! **Everything this test prints as evidence is a byte the guest itself wrote
//! to its own serial port.** That is reading what a program printed, which is
//! the most ordinary black-box observation there is (`ROADMAP.md` §1) — the
//! image is run, never read, and never vendored.
//!
//! # Reaching userspace
//!
//! Two things, neither of them a property of the board. **A root**: `pc64` has
//! no disk and no PCI, so an initramfs is the only one available —
//! `scripts/fetch-testdata.sh initramfs-x86` builds one around busybox. **And
//! time**: the kernel's crypto self-tests are hundreds of guest-seconds of
//! arithmetic that `cryptomgr.notests` removes outright. With both, a stock
//! 6.6 distribution kernel reaches `Run /init as init process` at about 490
//! virtual seconds and a shell prompt shortly after.
//!
//! ```text
//! RSEMU_KERNEL=/boot/vmlinuz \
//! RSEMU_INITRD=testdata/x86/initramfs-x86.cpio \
//! RSEMU_KERNEL_CMDLINE='console=ttyS0,115200 nokaslr cryptomgr.notests' \
//! RSEMU_KERNEL_MS=1200000 \
//! RSEMU_KERNEL_INPUT='rsemu# =>uname -srm\n' \
//! RSEMU_KERNEL_STOP_AT=x86_64 \
//! cargo test --release --features machine-pc64 --test pc64_linux -- --nocapture
//! ```
//!
//! # What a run proves
//!
//! More than it looks like. Before a kernel prints its first character it has
//! been entered in 32-bit protected mode, checked the processor with `CPUID`,
//! built identity page tables, set `CR4.PAE` and `EFER.LME`, turned paging on,
//! long-jumped into 64-bit mode, relocated itself, and decompressed several
//! megabytes with `REP MOVS` and the full 64-bit integer instruction set. A
//! single line of output is a large fraction of `ROADMAP.md` phase 6b's core.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-linuxboot",
    feature = "machine-pc64"
))]

mod x86boot;

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::host::chardev::CharPort;
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

use x86boot::Script;

/// How long to let the board run, in virtual milliseconds.
///
/// **Measured, not guessed, and the assertions below need every bit of it.**
/// A 12 MB distribution kernel spends about 250 virtual seconds getting from
/// the reset vector to `Linux version` — the decompressor alone is several
/// hundred million instructions — and everything interesting happens after
/// that. 900 costs four or five wall-clock minutes on a release build, which
/// is affordable because this whole file is gated on `RSEMU_KERNEL` and skips
/// when it is unset: `cargo test` never pays it.
///
/// A ceiling rather than a target: the run stops early when the processor
/// stops making progress.
const DEFAULT_MS: u64 = 900_000;

/// Everything the board needs to construct, with a `cpu.x86` that pushes what
/// it builds into `cpus`.
fn bindings(cpus: &Arc<Captured<X86>>) -> Bindings {
    let mut b = rsemu::machine::catalog::bindings().expect("this build's bindings");
    let kept = Arc::clone(cpus);
    b.replace("cpu.x86", move |props| {
        let cpu = Arc::new(x86boot::with_engine_from_env(X86::from_props_defaulting(
            props,
            Variant::X86_64,
        )?));
        kept.push(&cpu);
        Ok(cpu)
    });
    b
}

/// Build the board from its own machine file with a kernel in its slot.
fn board(
    kernel: Vec<u8>,
    initrd: Vec<u8>,
    params: &[(&str, String)],
) -> Result<(Machine, Arc<X86>, Arc<CharPort>), String> {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings(&cpus));
    for (name, value) in params {
        options = options.with_param(*name, value.as_str());
    }
    options.realize.media.insert("kernel", kernel);
    options.realize.media.insert("initrd", initrd);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = build(
        "pc64.machine",
        rsemu::machine::catalog::PC64.source,
        &registry,
        &options,
    )
    .map_err(|e| format!("{e}"))?;
    machine.reset(ResetKind::Cold);
    machine.sweep();
    let console = rsemu::host::chardev::ports::open(&options.realize.hosts, "console")
        .expect("the 16550 opened it");
    let cpu = cpus.take().expect("the constructor kept a handle");
    Ok((machine, cpu, console))
}

#[test]
fn a_linux_kernel_boots_on_the_pc64_board() {
    let Ok(path) = std::env::var("RSEMU_KERNEL") else {
        println!(
            "pc64: set RSEMU_KERNEL to a Linux/x86 bzImage to run one on this board; \
             see the module docs"
        );
        return;
    };
    let kernel = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let initrd = std::env::var("RSEMU_INITRD")
        .ok()
        .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
        .unwrap_or_default();
    println!(
        "pc64: {} bytes of kernel, {} bytes of initramfs",
        kernel.len(),
        initrd.len()
    );

    let mut params: Vec<(&str, String)> = Vec::new();
    if let Ok(cmdline) = std::env::var("RSEMU_KERNEL_CMDLINE") {
        params.push(("cmdline", cmdline));
    }
    if let Ok(extmem) = std::env::var("RSEMU_KERNEL_EXTMEM") {
        params.push(("extmem", extmem));
    }
    let (mut m, cpu, console) = match board(kernel, initrd, &params) {
        Ok(built) => built,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let ms: u64 = std::env::var("RSEMU_KERNEL_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_MS);

    let script = Script::from_env();
    println!("pc64: what the guest wrote to its serial port at 0x3f8:");
    let run = x86boot::run(
        &mut m,
        &cpu,
        &console,
        GlobalTime::from_nanos(ms * 1_000_000),
        &script,
    );
    x86boot::report("pc64", &m, &cpu, &run, &script);
    x86boot::assert_booted(&run, &script);
}
