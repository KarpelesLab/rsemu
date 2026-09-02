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
//!
//! **Everything this test prints as evidence is a byte the guest itself wrote
//! to its own serial port.** That is reading what a program printed, which is
//! the most ordinary black-box observation there is (`ROADMAP.md` §1) — the
//! image is run, never read, and never vendored.
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

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::host::chardev::CharPort;
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

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
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::X86_64)?);
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

/// Where a run stopped, and what the guest had said by then.
struct Run {
    /// Everything the guest wrote to its serial port.
    text: String,
    /// Virtual time reached.
    at: GlobalTime,
    /// Whether `CS:RIP` had stopped moving.
    stuck: bool,
    /// Whether the processor ever reached long mode.
    long: bool,
    /// Whether it ever reached 32-bit protected mode.
    protected: bool,
}

/// Run the board until the processor stops making progress or time runs out.
///
/// The console is drained every slice. It has to be: a 16550 whose host will
/// not take a byte holds it in the transmit register with `THRE` clear, which
/// is real back pressure and would stop the guest dead.
fn run(m: &mut Machine, cpu: &X86, console: &CharPort, limit: GlobalTime) -> Run {
    /// One slice of virtual time between progress checks.
    const SLICE_NS: u64 = 1_000_000;

    // `RSEMU_KERNEL_TRACE` prints where the processor was at the end of every
    // slice. A kernel that stops saying anything is otherwise a black box, and
    // one sample per virtual millisecond is enough to see whether it is
    // looping, and where.
    let trace = std::env::var("RSEMU_KERNEL_TRACE").is_ok();

    let mut text: Vec<u8> = Vec::new();
    let mut last = (0u16, 0u64, 0u64, 0u64, 0u64, 0u64);
    let mut idle = 0u32;
    let mut long = false;
    let mut protected = false;
    let mut printed = 0usize;
    let mut shown = 0u64;
    while m.now() < limit {
        // A **span**, not a deadline: `run_for` takes how long to run, and
        // handing it an absolute time made every slice as long as the run so
        // far — the machine doubled its virtual time on each call and one
        // slice grew to hours of work.
        if let Err(e) = m.run_for(GlobalTime::from_nanos(SLICE_NS)) {
            text.extend_from_slice(format!("\n[rsemu: the machine stopped: {e}]\n").as_bytes());
            break;
        }
        console.drain_into(&mut text);
        // Print as it goes: a run that takes minutes should not be silent, and
        // a hang is much easier to place when the last line before it is
        // visible.
        while let Some(nl) = text[printed..].iter().position(|b| *b == b'\n') {
            let line = String::from_utf8_lossy(&text[printed..printed + nl]).into_owned();
            println!("  | {}", line.trim_end_matches('\r'));
            printed += nl + 1;
        }
        let regs = cpu.regs();
        // Not `RIP` alone. A `REP MOVSQ` of a megabyte holds one instruction
        // for thousands of slices — the decompressor relocating itself does
        // exactly that — and a detector watching only the instruction pointer
        // calls that a hang. The counters a string operation steps are part of
        // the state that has to stop moving before a machine is stopped.
        let here = (regs.cs, regs.rip, regs.rcx, regs.rsi, regs.rdi, regs.rsp);
        // Sampled as the run goes rather than read at the end: a kernel that
        // faults its way back to real mode would otherwise look as though it
        // had never left it.
        let sys = cpu.sys();
        long |= sys.long_mode();
        protected |= sys.protected();
        // Only when the sample has *moved*, by more than the span of a tight
        // loop. A guest spinning in a twelve-byte loop for two virtual minutes
        // would otherwise print two million identical lines and bury the
        // transition that led into it, which is the only interesting part.
        if trace && regs.rip.abs_diff(shown) > 64 {
            shown = regs.rip;
            let what = cpu
                .disassemble(regs.cs, regs.rip, 1)
                .first()
                .map_or_else(|| "??".to_string(), |d| format!("{d}"));
            println!(
                "  . {:>7}ms {:04x}:{:016x} ax={:x} dx={:x} di={:x} si={:x} sp={:x} fl={:x} {what}",
                m.now().as_nanos() / 1_000_000,
                regs.cs,
                regs.rip,
                regs.rax,
                regs.rdx,
                regs.rdi,
                regs.rsi,
                regs.rsp,
                regs.eflags
            );
        }
        // A core that owes clocks is not stopped, it is paying: a `REP MOVSQ`
        // of twelve megabytes charges more than a slice's worth in one
        // instruction, and the scheduler holds the core off until virtual time
        // catches up. Nothing moves for a hundred slices while that happens.
        if here == last && cpu.cycle_debt() == 0 {
            idle += 1;
            // A tenth of a virtual second with none of it moving is a halt or
            // a tight loop with no interrupt coming, either of which ends the
            // run.
            if idle >= 100 {
                return Run {
                    text: String::from_utf8_lossy(&text).into_owned(),
                    at: m.now(),
                    stuck: true,
                    long,
                    protected,
                };
            }
        } else {
            idle = 0;
            last = here;
        }
    }
    Run {
        text: String::from_utf8_lossy(&text).into_owned(),
        at: m.now(),
        stuck: false,
        long,
        protected,
    }
}

/// Print whatever the guest wrote after the last newline.
///
/// It matters more than it sounds: the decompressor's last word before several
/// hundred million instructions of work is `Decompressing Linux... `, with no
/// newline, and a reader who never sees it cannot tell a kernel that is
/// working from one that has stopped.
fn print_tail(text: &str) {
    if let Some(tail) = text.rsplit('\n').next()
        && !tail.is_empty()
    {
        println!("  | {tail}");
    }
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

    println!("pc64: what the guest wrote to its serial port at 0x3f8:");
    let run = run(
        &mut m,
        &cpu,
        &console,
        GlobalTime::from_nanos(ms * 1_000_000),
    );
    print_tail(&run.text);
    let regs = cpu.regs();
    println!(
        "pc64: stopped after {} ms of virtual time, stuck={}, protected={}, long={}",
        run.at.as_nanos() / 1_000_000,
        run.stuck,
        run.protected,
        run.long
    );
    let sys = cpu.sys();
    println!(
        "pc64: cs:rip={:04x}:{:016x} cr0={:08x} cr2={:#x} cr3={:#x} cr4={:#x} efer={:#x} \
         fl={:#x}",
        regs.cs, regs.rip, sys.cr0, sys.cr2, sys.cr3, sys.cr4, sys.efer, regs.eflags
    );
    println!(
        "pc64: rax={:016x} rbx={:016x} rcx={:016x} rdx={:016x} rsi={:016x} rdi={:016x} \
         rbp={:016x} rsp={:016x}",
        regs.rax, regs.rbx, regs.rcx, regs.rdx, regs.rsi, regs.rdi, regs.rbp, regs.rsp
    );
    for line in cpu.disassemble(regs.cs, regs.rip, 4) {
        println!("pc64:   {line}");
    }
    // The bytes themselves, through the page tables the guest built. A
    // disassembler that says `ud` is saying *this core does not decode this*,
    // and the only way to find out which instruction that is is to look.
    if let Some(pa) = cpu.translate_debug(regs.rip).phys(regs.rip) {
        let hex: Vec<String> = (0..48)
            .map(|i| {
                m.space("mem")
                    .expect("the memory space")
                    .read(pa - 16 + i, rsemu::core::value::Width::U8, MemAttrs::DEBUG)
                    .map_or_else(|_| "??".to_string(), |b| format!("{b:02x}"))
            })
            .collect();
        println!("pc64:   [{:#x}] {}", pa - 16, hex.join(" "));
    }

    // And the stack, which is where a kernel that stopped keeps its reason. An
    // exception frame is recognisable by eye — a code selector and a flags
    // word between two addresses — and it is the only way to find out what a
    // guest faulted on when it stopped before it had a console.
    let peek = |at: u64| -> Option<u64> {
        let pa = cpu.translate_debug(at).phys(at)?;
        m.space("mem")
            .expect("the memory space")
            .read(pa, rsemu::core::value::Width::U64, MemAttrs::DEBUG)
            .ok()
    };
    println!("pc64: the stack at rsp:");
    for row in 0..8u64 {
        let at = regs.rsp + row * 32;
        let cells: Vec<String> = (0..4)
            .map(|i| {
                peek(at + i * 8).map_or_else(|| "----------------".into(), |v| format!("{v:016x}"))
            })
            .collect();
        println!("pc64:   {at:016x}  {}", cells.join(" "));
    }

    // And, if the guest stopped with a page-fault address in `CR2`, the walk
    // for it — read out of the guest's own tables, entry by entry. A kernel
    // that faults where it did not expect to and one whose tables this core
    // walks wrongly look identical from outside; this is what tells them
    // apart.
    if sys.cr2 != 0 && sys.cr0 & 0x8000_0000 != 0 {
        let phys = |at: u64| -> u64 {
            m.space("mem")
                .expect("the memory space")
                .read(at, rsemu::core::value::Width::U64, MemAttrs::DEBUG)
                .unwrap_or(0)
        };
        let mut table = sys.cr3 & 0x000f_ffff_ffff_f000;
        println!("pc64: the walk for cr2 = {:#x}:", sys.cr2);
        for level in (0..4).rev() {
            let index = (sys.cr2 >> (12 + 9 * level)) & 0x1ff;
            let entry = phys(table + index * 8);
            println!(
                "pc64:   level {level}: table {table:#x} index {index:#5x} -> {entry:#018x}{}",
                if entry & 1 == 0 {
                    "  (not present)"
                } else if level > 0 && entry & (1 << 7) != 0 {
                    "  (a large page)"
                } else {
                    ""
                }
            );
            if entry & 1 == 0 || (level > 0 && entry & (1 << 7) != 0) {
                break;
            }
            table = entry & 0x000f_ffff_ffff_f000;
        }
        println!(
            "pc64:   and this core's own debug walk says {:?}",
            cpu.translate_debug(sys.cr2)
        );
    }

    // The claim, and it is the whole point of the board: a kernel that says
    // anything at all has already been entered in protected mode, checked the
    // processor, built page tables, entered long mode and decompressed itself.
    assert!(
        !run.text.is_empty(),
        "the kernel printed nothing; it never reached its own console"
    );
    assert!(
        run.text.contains("Linux version"),
        "the kernel never announced itself"
    );
}
