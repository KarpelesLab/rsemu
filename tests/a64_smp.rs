//! Two processors on `arm64-virt-smp`.
//!
//! Two tests, and only one of them needs anything downloaded.
//!
//! [`the_second_processor_waits_for_the_first_to_release_it`] is **hermetic**:
//! a dozen hand-assembled instructions in an `Image` this file builds, where
//! the boot processor writes the other one's word of the release table and
//! then waits for it to answer. It runs under a plain `cargo test`, and what
//! it proves is the whole board mechanism — the reset vector reads
//! `MPIDR_EL1`, everything but affinity 0 goes to the parking loop, and a word
//! written into the table is a processor that starts executing.
//!
//! [`a_real_kernel_brings_up_both_processors`] is the same gate `a64_linux.rs`
//! runs, with the second core: it needs a kernel and a ramdisk, is skipped
//! when they are not named, and asserts the four lines a kernel prints when it
//! has actually started a second processor.
//!
//! ```text
//!   scripts/fetch-testdata.sh arm64-linux arm64-initramfs
//!
//!   RSEMU_ARM64_KERNEL=testdata/arm64/linux \
//!   RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \
//!       cargo test --release --features machine-arm64-virt \
//!           --test a64_smp -- --nocapture
//! ```
//!
//! The kernel and the busybox in the ramdisk are GPL-2.0 binaries: running one
//! as an emulated guest is ordinary use, committing one here would be
//! redistribution (`ROADMAP.md` §1), so nothing is vendored.

#![cfg(all(feature = "machine-arm64-virt", feature = "std"))]

use std::sync::Arc;
use std::time::Instant;

use rsemu::dev::arm::boot::asm;
use rsemu::dev::arm::power::{Request, Signal, signals};
use rsemu::host::chardev::{CharPort, ports};
use rsemu::machine::{Machine, catalog};

/// Where `machines/arm64-virt-smp.machine` puts the release table.
const RELEASE: u64 = 0x4000_1000;

/// Where the kernel — or, here, the test's own program — is entered.
const KERNEL_ADDR: u64 = 0x4020_0000;

/// Where the second processor's half of the test program starts.
const SECOND: u64 = KERNEL_ADDR + 0x200;

/// A word the second processor writes and the first one waits on.
const ANSWER: u64 = 0x4000_1200;

struct Board {
    machine: Machine,
    console: Arc<CharPort>,
    power: Arc<Signal>,
}

fn env_or(name: &str, fallback: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| fallback.to_string())
}

fn board(kernel: &[u8], initrd: &[u8], ram: &str) -> Board {
    let entry = catalog::machine("arm64-virt-smp").expect("this build ships it");
    let options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_media("kernel", kernel)
        .with_media("initrd", initrd)
        .with_media("disk", Vec::new())
        .with_param("ram", ram.to_string())
        .with_param(
            "cmdline",
            env_or(
                "RSEMU_ARM64_CMDLINE",
                "earlycon=pl011,0x9000000 console=ttyAMA0 rdinit=/init",
            ),
        );
    let registry = catalog::registry().expect("a registry");
    let machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("arm64-virt-smp does not build: {e}"),
    };
    Board {
        console: ports::open(&options.realize.hosts, "console").expect("the PL011 opened it"),
        power: signals::open(&options.realize.hosts, "power").expect("the controller opened it"),
        machine,
    }
}

/// Wrap `words` in the AArch64 `Image` header the board's loader insists on.
///
/// The fields are DDI 0487's and `src/dev/arm/loader.rs` reads them back:
/// two instruction words, `text_offset`, `image_size`, `flags`, three reserved
/// doublewords, the magic at 0x38 and a reserved word. Execution starts at
/// offset 0, so word 0 branches over the header to the code.
fn image(words: &[u32]) -> Vec<u8> {
    const HEADER: usize = 0x40;
    let mut out = Vec::with_capacity(HEADER + words.len() * 4);
    // `b .+0x40`, in instructions.
    out.extend_from_slice(&asm::b((HEADER / 4) as i32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&0u64.to_le_bytes()); // text_offset
    out.extend_from_slice(&((HEADER + words.len() * 4) as u64).to_le_bytes()); // image_size
    out.extend_from_slice(&0u64.to_le_bytes()); // flags: little-endian, 4 KiB
    for _ in 0..3 {
        out.extend_from_slice(&0u64.to_le_bytes()); // res2, res3, res4
    }
    out.extend_from_slice(b"ARM\x64");
    out.extend_from_slice(&0u32.to_le_bytes()); // res5
    assert_eq!(out.len(), HEADER);
    for word in words {
        out.extend_from_slice(&word.to_le_bytes());
    }
    out
}

/// The program the hermetic test runs.
///
/// Two halves in one image, because the board loads one image and both
/// processors run out of it:
///
/// ```text
///   ; the boot processor, entered by the ROM stub
///   x9  = RELEASE + 8          ; processor 1's word of the release table
///   x10 = SECOND               ; where it should start
///   str x10, [x9]
///   x11 = ANSWER
/// 1: ldr x12, [x11]
///   cbz x12, 1b                ; wait for the other one to say something
///   PSCI_SYSTEM_OFF            ; which stops the machine, and is the result
///
///   ; the second processor, released into this by the parking loop
///   x11 = ANSWER
///   x12 = 1
///   str x12, [x11]
///   b .
/// ```
fn two_processor_program() -> Vec<u8> {
    let mut words: Vec<u32> = Vec::new();
    words.extend_from_slice(&asm::load64(9, RELEASE + 8));
    words.extend_from_slice(&asm::load64(10, SECOND));
    words.push(asm::str_base(10, 9));
    words.extend_from_slice(&asm::load64(11, ANSWER));
    let spin = words.len() as i32;
    words.push(asm::ldr_base(12, 11));
    words.push(asm::cbz(12, spin - words.len() as i32));
    words.push(asm::movz(0, 0x0008));
    words.push(asm::movk(0, 0x8400, 16));
    words.push(asm::smc(0));
    words.push(asm::b(0));

    // The second processor's half, at a fixed offset so the first one can name
    // it without the two having to agree about the length of anything.
    let at = (SECOND - KERNEL_ADDR) as usize / 4;
    assert!(words.len() <= at, "the first half ran into the second");
    words.resize(at, 0);
    words.extend_from_slice(&asm::load64(11, ANSWER));
    words.push(asm::movz(12, 1));
    words.push(asm::str_base(12, 11));
    words.push(asm::b(0));
    image(&words)
}

#[test]
fn the_second_processor_waits_for_the_first_to_release_it() {
    let mut b = board(&two_processor_program(), &[], "16M");
    // Generous: the whole exchange is a few hundred instructions, and the
    // second processor spends most of it in the parking loop.
    for _ in 0..2000 {
        if b.power.peek().is_some() {
            break;
        }
        b.machine.run_quantum().expect("the machine advances");
    }
    assert_eq!(
        b.power.peek(),
        Some(Request::Poweroff),
        "the boot processor never saw the other one answer: either the reset vector did not park \
         it, or writing its word of the release table did not start it"
    );
}

#[test]
fn a_real_kernel_brings_up_both_processors() {
    let Ok(path) = std::env::var("RSEMU_ARM64_KERNEL") else {
        eprintln!(
            "RSEMU_ARM64_KERNEL is not set, so this test does nothing.\n\
             scripts/fetch-testdata.sh arm64-linux arm64-initramfs"
        );
        return;
    };
    let kernel =
        std::fs::read(&path).unwrap_or_else(|e| panic!("RSEMU_ARM64_KERNEL names `{path}`: {e}"));
    let ramdisk = match std::env::var("RSEMU_ARM64_INITRD") {
        Ok(path) => std::fs::read(&path)
            .unwrap_or_else(|e| panic!("RSEMU_ARM64_INITRD names `{path}`: {e}")),
        Err(_) => Vec::new(),
    };
    let mut b = board(&kernel, &ramdisk, &env_or("RSEMU_ARM64_RAM", "1G"));

    let quanta: usize = env_or("RSEMU_ARM64_QUANTA", "1200000")
        .parse()
        .expect("RSEMU_ARM64_QUANTA is a number of scheduler quanta");
    let mut out = Vec::new();
    let start = Instant::now();
    let (mut last, mut idle) = (0usize, 0usize);
    let mut typed = ramdisk.is_empty();
    for _ in 0..quanta {
        if b.power.peek().is_some() {
            break;
        }
        b.machine.run_quantum().expect("the machine advances");
        b.console.drain_into(&mut out);
        if out.len() == last {
            idle += 1;
            // A boot that has reached a prompt is silent, and so is one that
            // has stopped moving; either way the log is the result.
            if idle > 120_000 {
                break;
            }
        } else {
            idle = 0;
            last = out.len();
            if !typed && String::from_utf8_lossy(&out).contains("BusyBox") {
                typed = true;
                // Three questions for the shell, and the third is the answer
                // to "did the second processor run a userspace task": a
                // `poweroff` that arrives is a `/sbin/poweroff` that was
                // scheduled, on whichever processor took it, and a PSCI call
                // that left the core. The first two are for the log —
                // `/proc/interrupts` has a column per processor and the
                // architected timer's row is per-processor delivery through
                // the distributor's banked registers.
                b.console
                    .feed(b"cat /proc/interrupts; head -3 /proc/stat; poweroff -f\n");
            }
        }
    }
    let text = String::from_utf8_lossy(&out).into_owned();
    eprintln!("{text}");
    eprintln!(
        "--- {} byte(s) of console in {:?} ---",
        out.len(),
        start.elapsed()
    );

    assert!(
        text.contains("Machine model: rsemu arm64-virt-smp"),
        "the kernel did not parse the generated device tree"
    );
    // The four lines that mean a second processor is *running*, rather than
    // merely described. `CPU1: Booted secondary processor` is printed by the
    // secondary itself, so it cannot be printed by a board that only claimed
    // to have one.
    for line in [
        "smp: Bringing up secondary CPUs",
        "CPU1: Booted secondary processor",
        "smp: Brought up 1 node, 2 CPUs",
        "SMP: Total of 2 processors activated.",
    ] {
        assert!(text.contains(line), "the kernel never printed `{line}`");
    }
    if ramdisk.is_empty() {
        return;
    }
    assert!(
        text.contains("rsemu initramfs on Linux"),
        "`/init` never reached its own banner"
    );
    assert!(text.contains("BusyBox"), "there was no shell to type at");
    // `/proc/interrupts` has one column per online processor, so the header
    // alone is the kernel agreeing that both of them are up — and the rows
    // under it are the per-processor interrupt counts the GIC's banked
    // registers produced.
    assert!(
        text.contains("CPU0") && text.contains("CPU1"),
        "`/proc/interrupts` did not come back with a column for each processor"
    );
    assert_eq!(
        b.power.peek(),
        Some(Request::Poweroff),
        "the shell ran, but `poweroff -f` did not reach the board's power controller"
    );
}
