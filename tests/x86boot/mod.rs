//! Running a real Linux kernel on an x86 board, and saying where it stopped.
//!
//! Shared by [`tests/pc64_linux.rs`](../pc64_linux.rs) and
//! [`tests/q35_linux.rs`](../q35_linux.rs), which are the same measurement on
//! two boards: `pc64` is the smallest machine a kernel can be entered on at
//! all, and `q35-linux` is that entry on a chipset with a PCI bus under it. The
//! run loop, the input script and the post-mortem are identical, so they live
//! here rather than twice.
//!
//! Nothing in this file knows which board it is driving. It takes a built
//! machine, the processor in it and the host end of the console, runs until the
//! guest stops making progress, and prints what the guest said and where it
//! was when it stopped.
//!
//! **Everything printed as evidence is a byte the guest itself wrote to its own
//! serial port.** That is reading what a program printed, which is the most
//! ordinary black-box observation there is (`ROADMAP.md` §1) — the image is
//! run, never read, and never vendored.

// Three test binaries include this module — `pc64_linux`, `q35_linux` and
// `q35_uefi` — and each compiles its own copy of it. They use different
// subsets: a firmware is not held to `assert_booted`'s "Linux version" and
// spells its environment variables `RSEMU_OVMF_*` rather than reaching
// `from_env`. Without this, the half a binary does not call is dead code in
// that binary and `-D warnings` makes it a build failure.
#![allow(dead_code)]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::cpu::x86::X86;
use rsemu::host::chardev::CharPort;
use rsemu::machine::Machine;

/// Where a run stopped, and what the guest had said by then.
pub(crate) struct Run {
    /// Everything the guest wrote to its serial port.
    pub(crate) text: String,
    /// Virtual time reached.
    pub(crate) at: GlobalTime,
    /// Whether `CS:RIP` had stopped moving.
    pub(crate) stuck: bool,
    /// Whether the processor ever reached long mode.
    pub(crate) long: bool,
    /// Whether it ever reached 32-bit protected mode.
    pub(crate) protected: bool,
    /// How many `RSEMU_KERNEL_INPUT` steps were typed.
    pub(crate) typed: usize,
    /// Whether the run ended because the guest printed `RSEMU_KERNEL_STOP_AT`.
    pub(crate) reached: bool,
}

/// What to type at the guest, and what ends the run.
pub(crate) struct Script {
    /// One `(marker, text)` step per `RSEMU_KERNEL_INPUT` line.
    pub(crate) steps: Vec<(String, String)>,
    /// `RSEMU_KERNEL_STOP_AT`, or empty for "run until the clock".
    pub(crate) stop_at: String,
}

impl Script {
    /// Read both variables. Neither set is a run that types nothing, which is
    /// what every run before this existed did.
    pub(crate) fn from_env() -> Script {
        Script::from_vars("RSEMU_KERNEL_INPUT", "RSEMU_KERNEL_STOP_AT")
    }

    /// The same, from a pair of variables named by the caller.
    ///
    /// `tests/q35_uefi.rs` drives a firmware rather than a kernel and spells
    /// its variables `RSEMU_OVMF_*`; the parsing is the same, so it is here
    /// rather than copied.
    pub(crate) fn from_vars(input: &str, stop_at: &str) -> Script {
        let steps = std::env::var(input)
            .unwrap_or_default()
            .split('\n')
            .filter(|s| !s.trim().is_empty())
            .map(|step| {
                let (marker, text) = step
                    .split_once("=>")
                    .unwrap_or_else(|| panic!("`{step}` is not `marker=>text`"));
                (String::from(marker), unescape(text))
            })
            .collect();
        Script {
            steps,
            stop_at: std::env::var(stop_at).unwrap_or_default(),
        }
    }

    /// Enough tail to hold the longest marker that could arrive split across
    /// two slices.
    fn window(&self) -> usize {
        self.steps
            .iter()
            .map(|(marker, _)| marker.len())
            .chain(core::iter::once(self.stop_at.len()))
            .max()
            .unwrap_or(0)
            .max(1)
    }
}

/// `\n`, `\r`, `\t` and `\\` in a `RSEMU_KERNEL_INPUT` step's text.
///
/// Anything else after a backslash is two literal characters, because a shell
/// command is allowed to contain a backslash and this is not a shell.
fn unescape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut chars = text.chars();
    while let Some(c) = chars.next() {
        if c != '\\' {
            out.push(c);
            continue;
        }
        match chars.next() {
            Some('n') => out.push('\n'),
            Some('r') => out.push('\r'),
            Some('t') => out.push('\t'),
            Some('\\') => out.push('\\'),
            Some(other) => {
                out.push('\\');
                out.push(other);
            }
            None => out.push('\\'),
        }
    }
    out
}

/// Run the board until the processor stops making progress or time runs out.
///
/// The console is drained every slice. It has to be: a 16550 whose host will
/// not take a byte holds it in the transmit register with `THRE` clear, which
/// is real back pressure and would stop the guest dead.
pub(crate) fn run(
    m: &mut Machine,
    cpu: &X86,
    console: &CharPort,
    limit: GlobalTime,
    script: &Script,
) -> Run {
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
    // The tail of what the guest has said since the last keystroke, which is
    // what a marker is matched against. Trimmed to a few markers' worth, so a
    // run that prints megabytes does not search megabytes.
    let mut seen = String::new();
    let mut consumed = 0usize;
    let window = script.window();
    let mut step = 0usize;
    let mut reached = false;
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
        // The script, driven off what the guest *said*. Everything the loop
        // does here is decided by guest output, so two runs of the same board
        // type at the same instruction.
        if text.len() > consumed {
            seen.push_str(&String::from_utf8_lossy(&text[consumed..]));
            consumed = text.len();
        }
        if let Some((marker, what)) = script.steps.get(step)
            && seen.contains(marker.as_str())
        {
            println!("  > typing {what:?}");
            let fed = console.feed(what.as_bytes());
            assert_eq!(
                fed,
                what.len(),
                "the console took {fed} of {} byte(s)",
                what.len()
            );
            step += 1;
            // So the next marker is matched against what the guest says *after*
            // this keystroke rather than against the prompt that triggered it.
            seen.clear();
        }
        // Only once the script has run: the marker that ends a run is usually
        // the reply to the last thing typed.
        if !script.stop_at.is_empty()
            && step >= script.steps.len()
            && seen.contains(&script.stop_at)
        {
            println!("  > the guest printed {:?}; stopping", script.stop_at);
            reached = true;
            break;
        }
        if seen.len() > 4 * window {
            // Back off to a character boundary before cutting: a kernel that
            // prints one non-ASCII byte would otherwise panic the harness on a
            // `String::drain` in the middle of a code point, and the boundary
            // is at most three bytes away.
            let mut cut = seen.len() - 2 * window;
            while cut > 0 && !seen.is_char_boundary(cut) {
                cut -= 1;
            }
            seen.drain(..cut);
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
            // run — *unless* the script still has something to type. A shell
            // waiting at its prompt is halted between timer interrupts and
            // samples identically at every slice boundary, so a run that is
            // about to type would otherwise end one keystroke short of the
            // only thing it was there to prove.
            if idle >= 100 && step >= script.steps.len() {
                return Run {
                    text: String::from_utf8_lossy(&text).into_owned(),
                    at: m.now(),
                    stuck: true,
                    long,
                    protected,
                    typed: step,
                    reached,
                };
            }
        } else {
            idle = 0;
            last = here;
        }
    }
    console.drain_into(&mut text);
    Run {
        text: String::from_utf8_lossy(&text).into_owned(),
        at: m.now(),
        stuck: false,
        long,
        protected,
        typed: step,
        reached,
    }
}

/// Everything known about where the guest stopped, printed under `tag`.
///
/// The whole post-mortem, and every part of it has earned its place by having
/// been the only thing that told two indistinguishable failures apart: the
/// disassembly at `RIP` (a `ud` there is *this core does not decode this*), the
/// raw bytes under it through the guest's own page tables, the stack (an
/// exception frame is recognisable by eye), and the page-table walk for `CR2`.
pub(crate) fn report(tag: &str, m: &Machine, cpu: &X86, run: &Run, script: &Script) {
    print_tail(&run.text);
    let regs = cpu.regs();
    println!(
        "{tag}: stopped after {} ms of virtual time, stuck={}, protected={}, long={}",
        run.at.as_nanos() / 1_000_000,
        run.stuck,
        run.protected,
        run.long
    );
    if !script.steps.is_empty() || !script.stop_at.is_empty() {
        println!(
            "{tag}: typed {} of {} scripted step(s); the stop marker was {}",
            run.typed,
            script.steps.len(),
            if script.stop_at.is_empty() {
                String::from("not set")
            } else if run.reached {
                format!("{:?}, and the guest printed it", script.stop_at)
            } else {
                format!("{:?}, and the guest never printed it", script.stop_at)
            }
        );
    }
    let sys = cpu.sys();
    println!(
        "{tag}: cs:rip={:04x}:{:016x} cr0={:08x} cr2={:#x} cr3={:#x} cr4={:#x} efer={:#x} \
         fl={:#x}",
        regs.cs, regs.rip, sys.cr0, sys.cr2, sys.cr3, sys.cr4, sys.efer, regs.eflags
    );
    println!(
        "{tag}: rax={:016x} rbx={:016x} rcx={:016x} rdx={:016x} rsi={:016x} rdi={:016x} \
         rbp={:016x} rsp={:016x}",
        regs.rax, regs.rbx, regs.rcx, regs.rdx, regs.rsi, regs.rdi, regs.rbp, regs.rsp
    );
    for line in cpu.disassemble(regs.cs, regs.rip, 4) {
        println!("{tag}:   {line}");
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
        println!("{tag}:   [{:#x}] {}", pa - 16, hex.join(" "));
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
    println!("{tag}: the stack at rsp:");
    for row in 0..8u64 {
        let at = regs.rsp + row * 32;
        let cells: Vec<String> = (0..4)
            .map(|i| {
                peek(at + i * 8).map_or_else(|| "----------------".into(), |v| format!("{v:016x}"))
            })
            .collect();
        println!("{tag}:   {at:016x}  {}", cells.join(" "));
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
        println!("{tag}: the walk for cr2 = {:#x}:", sys.cr2);
        for level in (0..4).rev() {
            let index = (sys.cr2 >> (12 + 9 * level)) & 0x1ff;
            let entry = phys(table + index * 8);
            println!(
                "{tag}:   level {level}: table {table:#x} index {index:#5x} -> {entry:#018x}{}",
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
            "{tag}:   and this core's own debug walk says {:?}",
            cpu.translate_debug(sys.cr2)
        );
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

/// The three assertions every run on either board makes.
///
/// A kernel that says anything at all has already been entered in protected
/// mode, checked the processor with `CPUID`, built page tables, entered long
/// mode and decompressed itself; and a keystroke that was never typed and a
/// marker that never arrived both look exactly like a successful run from
/// outside, so a scripted run asserts what it was asked to do.
pub(crate) fn assert_booted(run: &Run, script: &Script) {
    assert!(
        !run.text.is_empty(),
        "the kernel printed nothing; it never reached its own console"
    );
    assert!(
        run.text.contains("Linux version"),
        "the kernel never announced itself"
    );
    assert_eq!(
        run.typed,
        script.steps.len(),
        "the guest never printed the marker for step {} of RSEMU_KERNEL_INPUT",
        run.typed + 1
    );
    assert!(
        script.stop_at.is_empty() || run.reached,
        "the guest never printed RSEMU_KERNEL_STOP_AT ({:?})",
        script.stop_at
    );
}
