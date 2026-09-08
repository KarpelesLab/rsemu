//! Running the interpreter and a translated engine side by side for a long
//! time, and naming the **first** quantum they stop agreeing on.
//!
//! # Why this exists
//!
//! `CLAUDE.md`, *CPU cores*: "the interpreter is the oracle". `ROADMAP.md` §0
//! asks for a bit-identical state hash across the interpreter and the JIT for
//! the same guest. `tests/a64_engines.rs`, `tests/riscv_virt_engines.rs` and
//! `tests/x86_engines.rs` all assert that — over forty quanta of a
//! six-instruction loop, which is what a test that runs on every commit can
//! afford.
//!
//! Two real defects in the A64 translating engine were found in September 2026
//! (`docs/platforms/arm64-virt.md`, "Twenty seconds was not far enough"). One
//! first appeared at **15.04 s** of guest time and the other at **23.46 s**;
//! both moved *where a quantum ends* rather than what an instruction computes,
//! so `State::debt` was the column that parted. Neither is reachable in forty
//! quanta of anything, and both were found by hand, by bisecting a 120-second
//! Linux boot. Nothing automated it. This module is the automation.
//!
//! # The shape, and why it is not a final-hash comparison
//!
//! Two machines are built from one description, differing only in `engine`,
//! and advanced **one quantum at a time in lockstep**. After every quantum:
//!
//! * their virtual clocks must read the same instant — a scheduling divergence
//!   is a different diagnosis from a state one and is reported as itself;
//! * every device whose snapshot chunk is small enough to save every quantum
//!   (the CPU, the interrupt controller, the UART, the virtio transports —
//!   everything except RAM and a framebuffer) must serialise to identical
//!   bytes;
//! * every `hash_every` quanta, the machines' full [`Machine::state_hash`]
//!   must agree, which is what covers RAM.
//!
//! Comparing at the end only would have caught the second defect and missed
//! the first, which self-corrects at the next quantum: the run would have ended
//! on one hash and the window would have closed unseen. And a final-hash
//! failure says "something differs somewhere in 120 seconds", which is the
//! report the last round had and spent a day bisecting by hand. This one says
//! *quantum 3 418 227, at 15.043 s, `cpu`: `debt` 5 against 11* — a bisect
//! already done.
//!
//! # Cost
//!
//! The per-quantum fingerprint is one `save` of each cheap device on each side.
//! On `arm64-virt` that is about 900 bytes a side, against a quantum of
//! emulation that costs orders of magnitude more: 120 guest seconds of an
//! arm64 Linux boot, interpreter against `jit`, is 179 s of wall time, and the
//! interpreter alone is most of it. The full state hash walks all of RAM, so
//! `hash_every` is coarse by default and the caller sizes it —
//! `docs/testing/long-run.md` has the measured table.

// Two test binaries include this file and neither uses all of it.
#![allow(dead_code)]

use std::fmt;
use std::time::{Duration, Instant};

use rsemu::core::clock::GlobalTime;
use rsemu::core::state::{MachineShape, Migrations, Source, StateReader, StateWriter};
use rsemu::machine::Machine;

/// The largest chunk this harness will re-serialise every quantum.
///
/// A probe saves each device once at the start and keeps the ones under this;
/// RAM and framebuffers fall out by their size rather than by a hard-coded list
/// of class names, so a board this file has never seen gets the right answer.
const CHEAP_CHUNK: usize = 64 * 1024;

/// One device the per-quantum fingerprint covers.
#[derive(Debug, Clone)]
struct Cheap {
    path: String,
    class: &'static str,
    version: u32,
}

/// How far to run and how often to take the expensive check.
#[derive(Debug, Clone)]
pub(crate) struct Options {
    /// Stop when the oracle's virtual clock reaches this.
    pub(crate) deadline: GlobalTime,
    /// Take a full [`Machine::state_hash`] every this many quanta. Zero never
    /// does, which is right only when RAM is covered some other way.
    pub(crate) hash_every: u64,
    /// Give up after this many quanta whatever the clock says — a machine that
    /// has wedged should fail the run rather than spin it out.
    pub(crate) max_quanta: u64,
    /// Print a progress line every this many quanta. Zero is silent.
    pub(crate) progress_every: u64,
}

impl Options {
    /// Run to `seconds` of guest time with sensible defaults for the rest.
    pub(crate) fn to_guest_seconds(seconds: u64) -> Options {
        Options {
            deadline: GlobalTime::from_nanos(seconds.saturating_mul(1_000_000_000)),
            // Coarse: a full hash walks RAM, and the per-quantum fingerprint is
            // what actually finds a divergence first.
            hash_every: 100_000,
            max_quanta: u64::MAX,
            progress_every: 0,
        }
    }

    /// How often the full hash is taken.
    pub(crate) fn hashing_every(mut self, quanta: u64) -> Options {
        self.hash_every = quanta;
        self
    }

    /// Print progress every `quanta` quanta.
    pub(crate) fn reporting_every(mut self, quanta: u64) -> Options {
        self.progress_every = quanta;
        self
    }

    /// Cap the number of quanta.
    pub(crate) fn at_most(mut self, quanta: u64) -> Options {
        self.max_quanta = quanta;
        self
    }
}

/// What a completed run did, for the log.
#[derive(Debug, Clone)]
pub(crate) struct Summary {
    /// Quanta advanced on each side.
    pub(crate) quanta: u64,
    /// Where the oracle's clock finished.
    pub(crate) guest: GlobalTime,
    /// Wall time the whole lockstep took, both machines and the comparison.
    pub(crate) wall: Duration,
    /// How many full state hashes were compared.
    pub(crate) hashes: u64,
    /// The devices the per-quantum fingerprint covered.
    pub(crate) watched: Vec<String>,
    /// The devices it did not, because their chunks are too large.
    pub(crate) unwatched: Vec<String>,
}

impl fmt::Display for Summary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} quanta, {} of guest time in {:?}, {} full state hashes; \
             watched every quantum: {}",
            self.quanta,
            seconds(self.guest),
            self.wall,
            self.hashes,
            self.watched.join(", "),
        )?;
        if !self.unwatched.is_empty() {
            write!(
                f,
                "; too large to watch per quantum (covered by the full hash \
                 only): {}",
                self.unwatched.join(", ")
            )?;
        }
        Ok(())
    }
}

/// What parted.
#[derive(Debug, Clone)]
pub(crate) enum What {
    /// The two schedulers are no longer on the same instant.
    Clock { oracle: u64, engine: u64 },
    /// One device's chunk differs.
    Device {
        path: String,
        class: String,
        detail: String,
    },
    /// The full state hashes differ but every watched device agreed, so what
    /// differs is RAM or a device too large to watch.
    Hash { oracle: u64, engine: u64 },
}

/// The first quantum on which the two engines stopped being the same machine.
#[derive(Debug, Clone)]
pub(crate) struct Divergence {
    /// The board this ran on.
    pub(crate) label: String,
    /// The engine under test, against the interpreter.
    pub(crate) engine: String,
    /// Which quantum, counting from one.
    pub(crate) quantum: u64,
    /// The oracle's virtual clock at the end of it.
    pub(crate) guest: GlobalTime,
    /// What parted.
    pub(crate) what: What,
}

impl fmt::Display for Divergence {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "\n{}: engine={} left the interpreter at quantum {}, {} of guest \
             time.",
            self.label,
            self.engine,
            self.quantum,
            seconds(self.guest)
        )?;
        match &self.what {
            What::Clock { oracle, engine } => writeln!(
                f,
                "    The two schedulers are on different instants: {oracle} ns \
                 interpreted, {engine} ns translated. That is a scheduling \
                 divergence, not a state one — the engines disagree about how \
                 much time a quantum was worth."
            ),
            What::Device {
                path,
                class,
                detail,
            } => writeln!(f, "    Device `{path}` ({class}):\n{detail}"),
            What::Hash { oracle, engine } => writeln!(
                f,
                "    The full state hashes differ — {oracle:#018x} interpreted, \
                 {engine:#018x} translated — while every device watched every \
                 quantum agreed. What parted is RAM, or a device whose chunk is \
                 too large to watch per quantum."
            ),
        }?;
        write!(
            f,
            "    ROADMAP.md §0 requires a bit-identical state hash across the \
             interpreter and the JIT for the same guest, and CLAUDE.md makes \
             the interpreter the oracle: the column on the left is the one to \
             believe."
        )
    }
}

/// A virtual instant, as seconds and microseconds.
///
/// Integer arithmetic on purpose (`CLAUDE.md`, *Determinism*): this is a
/// report, but a float here would be the only float in the file and the habit
/// is worth more than the convenience.
pub(crate) fn seconds(t: GlobalTime) -> String {
    let ns = t.as_nanos();
    format!(
        "{}.{:06} s",
        ns / 1_000_000_000,
        (ns % 1_000_000_000) / 1_000
    )
}

/// Advance `oracle` and `under_test` together, one quantum at a time, until
/// `opts.deadline` — or until they disagree.
///
/// # Errors
///
/// The first quantum on which anything watched differs. Boxed because a
/// `Divergence` carries a rendered diff and a `Result` that large in the happy
/// path is what `clippy::result_large_err` is about.
pub(crate) fn lockstep(
    label: &str,
    oracle: &mut Machine,
    engine: &str,
    under_test: &mut Machine,
    opts: &Options,
) -> Result<Summary, Box<Divergence>> {
    lockstep_pumping(label, oracle, engine, under_test, opts, &mut || {})
}

/// [`lockstep`], with a hook the caller runs after every quantum.
///
/// The hook is what a **host** does between quanta, and a synthetic guest has
/// no need of one: it is here for the console. A 16550 whose host never takes
/// a byte fills its port and starts refusing bytes
/// ([`CharPort::write`](rsemu::host::chardev::CharPort) is short when the
/// queue is full), so a kernel that prints its way through a boot would spend
/// the run waiting on a transmitter that never drains. That is the same on
/// both sides, so nothing *diverges* — and two machines wedged the same way
/// agree at every checkpoint, which is precisely the vacuous green
/// `assert_the_workload_ran` exists to catch on the synthetic legs.
///
/// It runs on both machines at once rather than once per machine, because the
/// two are meant to be indistinguishable and a hook that touched one of them
/// would be the harness introducing the asymmetry it is looking for.
///
/// # Errors
///
/// As [`lockstep`].
pub(crate) fn lockstep_pumping(
    label: &str,
    oracle: &mut Machine,
    engine: &str,
    under_test: &mut Machine,
    opts: &Options,
    pump: &mut dyn FnMut(),
) -> Result<Summary, Box<Divergence>> {
    let (shape, cheap, unwatched) = cheap_devices(oracle);
    let started = Instant::now();
    let mut quantum = 0u64;
    let mut hashes = 0u64;

    while oracle.now() < opts.deadline && quantum < opts.max_quanta {
        oracle.run_quantum().expect("the oracle runs");
        under_test
            .run_quantum()
            .expect("the machine under test runs");
        quantum += 1;
        pump();

        // The clock first: two machines on different instants are not
        // comparable at all, and every later report would be noise.
        if oracle.now() != under_test.now() {
            return Err(Box::new(Divergence {
                label: label.to_string(),
                engine: engine.to_string(),
                quantum,
                guest: oracle.now(),
                what: What::Clock {
                    oracle: oracle.now().as_nanos(),
                    engine: under_test.now().as_nanos(),
                },
            }));
        }

        let a = fingerprint(oracle, &shape, &cheap);
        let b = fingerprint(under_test, &shape, &cheap);
        if a != b {
            return Err(Box::new(Divergence {
                label: label.to_string(),
                engine: engine.to_string(),
                quantum,
                guest: oracle.now(),
                what: locate(&a, &b, &cheap),
            }));
        }

        if opts.hash_every != 0 && quantum.is_multiple_of(opts.hash_every) {
            hashes += 1;
            let ha = oracle.state_hash().expect("the oracle hashes");
            let hb = under_test.state_hash().expect("the machine hashes");
            if ha != hb {
                return Err(Box::new(Divergence {
                    label: label.to_string(),
                    engine: engine.to_string(),
                    quantum,
                    guest: oracle.now(),
                    what: What::Hash {
                        oracle: ha,
                        engine: hb,
                    },
                }));
            }
        }

        if opts.progress_every != 0 && quantum.is_multiple_of(opts.progress_every) {
            eprintln!(
                "    {label} engine={engine}: quantum {quantum}, {} of guest \
                 time, {:?} of wall time",
                seconds(oracle.now()),
                started.elapsed()
            );
        }
    }

    // One at the end, always — otherwise a run shorter than `hash_every`
    // silently loses the tier that covers RAM, and "0 full state hashes" in the
    // summary is a line nobody reads. It is also the check most like the one
    // the other engine tests make, so a run that agreed all the way through
    // finishes by agreeing the way they do.
    if opts.hash_every != 0 && quantum > 0 {
        hashes += 1;
        let ha = oracle.state_hash().expect("the oracle hashes");
        let hb = under_test.state_hash().expect("the machine hashes");
        if ha != hb {
            return Err(Box::new(Divergence {
                label: label.to_string(),
                engine: engine.to_string(),
                quantum,
                guest: oracle.now(),
                what: What::Hash {
                    oracle: ha,
                    engine: hb,
                },
            }));
        }
    }

    Ok(Summary {
        quanta: quantum,
        guest: oracle.now(),
        wall: started.elapsed(),
        hashes,
        watched: cheap.iter().map(|c| c.path.clone()).collect(),
        unwatched,
    })
}

// ---------------------------------------------------------------------------
// the fingerprint
// ---------------------------------------------------------------------------

/// Probe every device once and keep the ones cheap enough to watch.
fn cheap_devices(m: &Machine) -> (MachineShape, Vec<Cheap>, Vec<String>) {
    let mut shape = MachineShape::new();
    let mut kept = Vec::new();
    let mut dropped = Vec::new();
    for entry in m.devices() {
        let class = entry.class().name;
        let version = entry.class().version;
        let path = entry.path().to_string();
        match one_chunk(m, &path, class, version) {
            Some(bytes) if bytes.len() <= CHEAP_CHUNK => {
                shape
                    .add_device(&path, class)
                    .expect("a machine's own instance paths are unique");
                kept.push(Cheap {
                    path,
                    class,
                    version,
                });
            }
            // Either too large to save every quantum, or a device that declines
            // to save at all. Both are covered by the full state hash and
            // neither should stop the run.
            _ => dropped.push(path),
        }
    }
    (shape, kept, dropped)
}

/// One device's snapshot, as a whole container.
fn one_chunk(m: &Machine, path: &str, class: &str, version: u32) -> Option<Vec<u8>> {
    let entry = m.device(path)?;
    let mut shape = MachineShape::new();
    shape.add_device(path, class).ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk(path, class, version).ok()?;
        entry.device().save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

/// Every cheap device's state, in one container, in canonical path order.
fn fingerprint(m: &Machine, shape: &MachineShape, cheap: &[Cheap]) -> Vec<u8> {
    let mut w = StateWriter::new(shape.clone());
    for c in cheap {
        let entry = m
            .device(&c.path)
            .expect("the device is still in the machine");
        let mut chunk = w
            .chunk(&c.path, c.class, c.version)
            .expect("one chunk per path");
        entry
            .device()
            .save(&mut chunk)
            .expect("a device that saved during the probe saves now");
    }
    w.to_vec().expect("the writer serialises what it was given")
}

/// Which device in the fingerprint differs, and how.
fn locate(a: &[u8], b: &[u8], cheap: &[Cheap]) -> What {
    let (ra, rb) = match (StateReader::new(a), StateReader::new(b)) {
        (Ok(ra), Ok(rb)) => (ra, rb),
        _ => {
            return What::Device {
                path: "?".to_string(),
                class: "?".to_string(),
                detail: "        the fingerprint does not parse back, which is a \
                         harness bug rather than a divergence"
                    .to_string(),
            };
        }
    };
    let migrations = Migrations::new();
    for c in cheap {
        let la = ra.load(&c.path, c.class, c.version, &migrations);
        let lb = rb.load(&c.path, c.class, c.version, &migrations);
        let (la, lb) = match (la, lb) {
            (Ok(la), Ok(lb)) => (la, lb),
            _ => continue,
        };
        if la.data() == lb.data() {
            continue;
        }
        return What::Device {
            path: c.path.clone(),
            class: c.class.to_string(),
            detail: describe(c.class, la.data(), lb.data()),
        };
    }
    What::Device {
        path: "?".to_string(),
        class: "?".to_string(),
        detail: "        the fingerprints differ but no single device's chunk \
                 does, which is a harness bug"
            .to_string(),
    }
}

/// The field-level diff of one device's chunk.
///
/// A named decoder where there is one, and a byte-offset report where there is
/// not: the point is that the failure message says *which column* moved, so
/// the next person does not repeat the bisect this file exists to end.
fn describe(class: &str, a: &[u8], b: &[u8]) -> String {
    if let (Some(fa), Some(fb)) = (decode(class, a), decode(class, b)) {
        let mut out = String::new();
        for ((name, x), (_, y)) in fa.iter().zip(&fb) {
            if x != y {
                out.push_str(&format!(
                    "        {name:<14} {x:#018x} interpreted   {y:#018x} translated\n"
                ));
            }
        }
        if !out.is_empty() {
            return out.trim_end().to_string();
        }
    }
    bytewise(a, b)
}

/// The fallback: where the bytes first part, and what the eight bytes there
/// read as.
fn bytewise(a: &[u8], b: &[u8]) -> String {
    let at = a.iter().zip(b).position(|(x, y)| x != y).unwrap_or(0);
    let word = |s: &[u8], at: usize| -> String {
        if at + 8 <= s.len() {
            let mut v = [0u8; 8];
            v.copy_from_slice(&s[at..at + 8]);
            format!("{:#018x}", u64::from_be_bytes(v))
        } else {
            "(short)".to_string()
        }
    };
    format!(
        "        chunk is {} bytes interpreted and {} translated; first \
         difference at byte {at}\n        \
         as a big-endian word there: {} interpreted, {} translated",
        a.len(),
        b.len(),
        word(a, at),
        word(b, at)
    )
}

/// A class-specific field decoder, where this file has one.
fn decode(class: &str, data: &[u8]) -> Option<Vec<(String, u64)>> {
    match class {
        "cpu.arm.a64" => decode_a64(data),
        "cpu.riscv" => decode_riscv(data),
        "cpu.x86" => decode_x86(data),
        _ => None,
    }
}

/// `cpu.arm.a64`'s chunk, field by field.
///
/// The order is `Cpu::save`'s: X0-X30, the 32 SIMD&FP registers as two words
/// each, `PC`, three counters, two flags, `PSTATE`, then the thirty system
/// registers `Cpu::sysreg_words` writes, then the interrupt lines and the power
/// state. Read with a `ChunkReader` rather than at fixed offsets, because the
/// exclusive monitor is an `Option` and moves everything after it.
///
/// Returns `None` on anything unexpected, and the caller falls back to a byte
/// diff — a decoder that has drifted from `save` must not turn a real
/// divergence into a confident lie.
fn decode_a64(data: &[u8]) -> Option<Vec<(String, u64)>> {
    use rsemu::core::state::ChunkReader;

    const SYSREGS: [&str; 30] = [
        "sp_el0",
        "sp_el1",
        "sctlr",
        "actlr",
        "cpacr",
        "ttbr0",
        "ttbr1",
        "tcr",
        "mair",
        "amair",
        "contextidr",
        "spsr_el1",
        "elr_el1",
        "esr_el1",
        "far_el1",
        "vbar_el1",
        "afsr0",
        "afsr1",
        "tpidr_el1",
        "tpidr_el0",
        "tpidrro_el0",
        "mdscr",
        "fpcr",
        "fpsr",
        "cntfrq",
        "cntkctl",
        "cntp_ctl",
        "cntp_cval",
        "cntv_ctl",
        "cntv_cval",
    ];

    let mut r = ChunkReader::new(data);
    let mut out: Vec<(String, u64)> = Vec::new();
    for n in 0..31 {
        out.push((format!("x{n}"), r.read_u64().ok()?));
    }
    for n in 0..32 {
        out.push((format!("v{n}.lo"), r.read_u64().ok()?));
        out.push((format!("v{n}.hi"), r.read_u64().ok()?));
    }
    out.push(("pc".to_string(), r.read_u64().ok()?));
    out.push(("cycles".to_string(), r.read_u64().ok()?));
    out.push(("debt".to_string(), r.read_u64().ok()?));
    out.push(("faults".to_string(), r.read_u64().ok()?));
    out.push(("wfi".to_string(), u64::from(r.read_bool().ok()?)));
    let held = r.read_bool().ok()?;
    out.push(("exclusive?".to_string(), u64::from(held)));
    if held {
        out.push(("exclusive".to_string(), r.read_u64().ok()?));
    }
    out.push(("nzcv".to_string(), u64::from(r.read_u32().ok()?)));
    out.push(("daif".to_string(), r.read_u64().ok()?));
    out.push(("el".to_string(), u64::from(r.read_u8().ok()?)));
    out.push(("spsel".to_string(), u64::from(r.read_bool().ok()?)));
    for name in SYSREGS {
        out.push((name.to_string(), r.read_u64().ok()?));
    }
    out.push(("irq lines".to_string(), r.read_u64().ok()?));
    out.push(("powered".to_string(), u64::from(r.read_bool().ok()?)));
    out.push(("power pending".to_string(), u64::from(r.read_bool().ok()?)));
    out.push(("power entry".to_string(), r.read_u64().ok()?));
    out.push(("power context".to_string(), r.read_u64().ok()?));
    // A decoder that stopped early has drifted from `save`, and a partial field
    // list would name the wrong column. Say so by declining.
    r.end().ok()?;
    Some(out)
}

/// `cpu.x86`'s chunk, field by field.
///
/// The order is `X86::save`'s, and that chunk has grown five times, so the
/// layout is a prefix followed by four appended blocks rather than anything
/// tidy: the gdb i386 core block (eight general registers as doublewords,
/// `EIP`, `EFLAGS`, the six selectors), the cycle counter, the six segment
/// descriptor caches with `LDTR` and the task register, the two table
/// registers, the control, debug and test registers, the run-state flags, the
/// open bus, the fault counters, the **variable-length** prefetch queue, the
/// debt, the interrupt pins and the A20 gate — then the long-mode block, the
/// floating-point block, the multiprocessor block, `IA32_MISC_ENABLE` and the
/// memory-type range registers.
///
/// Read with a `ChunkReader` rather than at fixed offsets, because the
/// prefetch queue is length-prefixed and moves everything after it — the same
/// reason `decode_a64` cannot index either.
///
/// The 32-bit views and the 64-bit ones are **both** named, because `save`
/// writes both: a divergence in `RAX` is reported as `eax` and `rax` together,
/// and one that lives only in the upper half shows as `rax` alone. That is
/// worth keeping rather than deduplicating — a stale upper half is exactly
/// what `cpu::x86::engine::narrow_state_is_clean` exists for.
///
/// Returns `None` on anything unexpected, and the caller falls back to a byte
/// diff — a decoder that has drifted from `save` must not turn a real
/// divergence into a confident lie.
fn decode_x86(data: &[u8]) -> Option<Vec<(String, u64)>> {
    use rsemu::core::state::ChunkReader;

    /// `Reg::ALL`, which is gdb's i386 core ordering and the order the chunk
    /// opens with.
    const CORE32: [&str; 16] = [
        "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi", "eip", "eflags", "cs", "ss", "ds",
        "es", "fs", "gs",
    ];
    /// `isa::seg`'s index order — **not** the selector order above.
    const SEGS: [&str; 6] = ["es", "cs", "ss", "ds", "fs", "gs"];
    /// `Reg::WIDE`, in ModRM number order with `RIP` last.
    const WIDE: [&str; 17] = [
        "rax", "rcx", "rdx", "rbx", "rsp", "rbp", "rsi", "rdi", "r8", "r9", "r10", "r11", "r12",
        "r13", "r14", "r15", "rip",
    ];
    /// The long-mode block, after the seventeen wide registers.
    const LONG: [&str; 9] = [
        "cr4",
        "efer",
        "fs_base",
        "gs_base",
        "kernel_gs_base",
        "star",
        "lstar",
        "cstar",
        "sfmask",
    ];

    let mut r = ChunkReader::new(data);
    let mut out: Vec<(String, u64)> = Vec::new();

    for name in CORE32 {
        out.push((name.to_string(), u64::from(r.read_u32().ok()?)));
    }
    out.push(("cycles".to_string(), r.read_u64().ok()?));
    for name in SEGS {
        out.push((format!("{name}.sel"), u64::from(r.read_u16().ok()?)));
        out.push((format!("{name}.base"), r.read_u64().ok()?));
        out.push((format!("{name}.limit"), u64::from(r.read_u32().ok()?)));
        out.push((format!("{name}.ar"), u64::from(r.read_u32().ok()?)));
    }
    for name in ["ldtr", "tr"] {
        out.push((format!("{name}.sel"), u64::from(r.read_u16().ok()?)));
        out.push((format!("{name}.base"), r.read_u64().ok()?));
        out.push((format!("{name}.limit"), u64::from(r.read_u32().ok()?)));
        out.push((format!("{name}.ar"), u64::from(r.read_u32().ok()?)));
    }
    for name in ["gdtr", "idtr"] {
        out.push((format!("{name}.base"), r.read_u64().ok()?));
        out.push((format!("{name}.limit"), u64::from(r.read_u32().ok()?)));
    }
    out.push(("cr0".to_string(), u64::from(r.read_u32().ok()?)));
    out.push(("cr2".to_string(), r.read_u64().ok()?));
    out.push(("cr3".to_string(), r.read_u64().ok()?));
    for n in 0..8 {
        out.push((format!("dr{n}"), r.read_u64().ok()?));
    }
    for n in 0..8 {
        out.push((format!("test{n}"), u64::from(r.read_u32().ok()?)));
    }
    for name in ["halted", "shutdown", "reset pending", "int shadow"] {
        out.push((name.to_string(), u64::from(r.read_bool().ok()?)));
    }
    out.push(("open bus".to_string(), u64::from(r.read_u8().ok()?)));
    out.push(("faults".to_string(), r.read_u64().ok()?));
    out.push(("last fault".to_string(), r.read_u64().ok()?));
    // The prefetch queue is length-prefixed, which is why nothing after it can
    // be read at a fixed offset. Its bytes are folded into one value rather
    // than named one at a time: what a report needs to say is *the queue
    // differs*, and by how much is the next question rather than this one.
    let queued = r.read_u8().ok()?;
    out.push(("queue len".to_string(), u64::from(queued)));
    let mut queue = 0u64;
    for _ in 0..queued {
        queue = queue.rotate_left(8) ^ u64::from(r.read_u8().ok()?);
    }
    out.push(("queue".to_string(), queue));
    out.push(("debt".to_string(), r.read_u64().ok()?));
    out.push(("intr".to_string(), u64::from(r.read_bool().ok()?)));
    out.push(("nmi level".to_string(), u64::from(r.read_bool().ok()?)));
    out.push(("nmi latch".to_string(), u64::from(r.read_bool().ok()?)));
    out.push(("intr vector".to_string(), u64::from(r.read_u8().ok()?)));
    out.push(("a20".to_string(), u64::from(r.read_bool().ok()?)));
    for name in WIDE {
        out.push((name.to_string(), r.read_u64().ok()?));
    }
    for name in LONG {
        out.push((name.to_string(), r.read_u64().ok()?));
    }
    for n in 0..8 {
        out.push((format!("st{n}.sig"), r.read_u64().ok()?));
        out.push((format!("st{n}.exp"), u64::from(r.read_u16().ok()?)));
    }
    for name in ["x87 control", "x87 status", "x87 tag", "x87 op"] {
        out.push((name.to_string(), u64::from(r.read_u16().ok()?)));
    }
    out.push(("x87 ip".to_string(), r.read_u64().ok()?));
    out.push(("x87 dp".to_string(), r.read_u64().ok()?));
    out.push(("x87 cs".to_string(), u64::from(r.read_u16().ok()?)));
    out.push(("x87 ds".to_string(), u64::from(r.read_u16().ok()?)));
    for n in 0..16 {
        out.push((format!("xmm{n}.lo"), r.read_u64().ok()?));
        out.push((format!("xmm{n}.hi"), r.read_u64().ok()?));
    }
    out.push(("mxcsr".to_string(), u64::from(r.read_u32().ok()?)));
    for name in [
        "wait for sipi",
        "init pin",
        "init peer",
        "init latched",
        "startup?",
    ] {
        out.push((name.to_string(), u64::from(r.read_bool().ok()?)));
    }
    out.push(("startup page".to_string(), u64::from(r.read_u8().ok()?)));
    out.push(("misc_enable".to_string(), r.read_u64().ok()?));
    out.push(("mtrr_def_type".to_string(), r.read_u64().ok()?));
    for n in 0..16 {
        out.push((format!("mtrr_var{n}"), r.read_u64().ok()?));
    }
    for n in 0..11 {
        out.push((format!("mtrr_fix{n}"), r.read_u64().ok()?));
    }
    // A decoder that stopped early has drifted from `save`, and a partial
    // field list would name the wrong column. Say so by declining.
    r.end().ok()?;
    Some(out)
}

/// `cpu.riscv`'s chunk, field by field.
///
/// The order is `Hart::save`'s: the thirty-two integer registers, the
/// thirty-two floating-point ones, `PC`, three counters, the `WFI` flag, the
/// **optional** reservation, the privilege mode, the twenty-four control and
/// status registers it writes as a list, the physical-memory-protection
/// configuration, and the interrupt lines.
///
/// Read with a `ChunkReader` rather than at fixed offsets, because the
/// reservation is an `Option` and moves everything after it — the same reason
/// `decode_a64` cannot index either.
///
/// The integer registers are named the way the guest's own assembly names
/// them. `x18` and `s2` are the same register and only one of the two ever
/// appears in a listing, so a report that says `x18` sends the reader back to
/// the manual; `s2 (x18)` does not. *The RISC-V Instruction Set Manual, Volume
/// I*, chapter 25 is the table.
///
/// This is the third decoder and it was written because the RISC-V leg's
/// calibration run reported *"first difference at byte 512, as a big-endian
/// word 0x8411008000000000 against 0x8811008000000000"*. That is the program
/// counter, one instruction apart, in the wrong byte order — correct, useless,
/// and exactly the weakness `describe`'s fallback is documented to have.
///
/// Returns `None` on anything unexpected, and the caller falls back to a byte
/// diff — a decoder that has drifted from `save` must not turn a real
/// divergence into a confident lie.
fn decode_riscv(data: &[u8]) -> Option<Vec<(String, u64)>> {
    use rsemu::core::state::ChunkReader;

    /// The ABI names, in register-number order.
    const ABI: [&str; 32] = [
        "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3", "a4",
        "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4",
        "t5", "t6",
    ];
    /// The control and status registers `save` writes as one list.
    const CSRS: [&str; 24] = [
        "mstatus",
        "medeleg",
        "mideleg",
        "mie",
        "mtvec",
        "mcounteren",
        "mcountinhibit",
        "mscratch",
        "mepc",
        "mcause",
        "mtval",
        "menvcfg",
        "stvec",
        "scounteren",
        "sscratch",
        "sepc",
        "scause",
        "stval",
        "satp",
        "senvcfg",
        "fcsr",
        "minstret",
        "mcycle",
        "mtime",
    ];
    /// `csr::PMP_ENTRIES`.
    const PMP: usize = 16;

    let mut r = ChunkReader::new(data);
    let mut out: Vec<(String, u64)> = Vec::new();
    for (n, name) in ABI.iter().enumerate() {
        out.push((format!("{name} (x{n})"), r.read_u64().ok()?));
    }
    for n in 0..32 {
        out.push((format!("f{n}"), r.read_u64().ok()?));
    }
    out.push(("pc".to_string(), r.read_u64().ok()?));
    out.push(("cycles".to_string(), r.read_u64().ok()?));
    out.push(("debt".to_string(), r.read_u64().ok()?));
    out.push(("faults".to_string(), r.read_u64().ok()?));
    out.push(("wfi".to_string(), u64::from(r.read_bool().ok()?)));
    let held = r.read_bool().ok()?;
    out.push(("reserved?".to_string(), u64::from(held)));
    if held {
        out.push(("reservation".to_string(), r.read_u64().ok()?));
    }
    out.push(("priv".to_string(), u64::from(r.read_u8().ok()?)));
    for name in CSRS {
        out.push((name.to_string(), r.read_u64().ok()?));
    }
    out.push(("pmp count".to_string(), r.read_u64().ok()?));
    for n in 0..PMP {
        out.push((format!("pmpcfg{n}"), u64::from(r.read_u8().ok()?)));
    }
    for n in 0..PMP {
        out.push((format!("pmpaddr{n}"), r.read_u64().ok()?));
    }
    out.push(("irq lines".to_string(), r.read_u64().ok()?));
    // A decoder that stopped early has drifted from `save`, and a partial field
    // list would name the wrong column. Say so by declining.
    r.end().ok()?;
    Some(out)
}
