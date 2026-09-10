//! Collecting a run's counters into a [`Table`] somebody can read.
//!
//! [`crate::core::trace`] is the counter core: channels, a static array, and a
//! two-column rendering. This module is the other half — the part that knows
//! about a *machine*, walks it when the run ends, and turns what it finds into
//! rows.
//!
//! # Why the interesting numbers need no hook at all
//!
//! The tree already counts almost everything anybody has wanted. `jit::
//! DispatchStats` has counted block entries, chained entries, lookups and
//! translations since the block cache landed; `cpu::x86::JitStats` and
//! `cpu::arm::a64::JitStats` add retired-inside-a-block against interpreted.
//! Those numbers are why `docs/platforms/pc64.md` can say "364 481 972 blocks
//! executed from 132 053 translations, 311 713 835 of them (86%) reached by
//! following a patched exit".
//!
//! What did not exist was any way to **get them out of a process**. They are
//! reachable from a `&X86`, and a built machine hands back `Arc<dyn Device>`;
//! every figure in those documents was therefore read by hand out of a private
//! patch that was reverted afterwards. So the first thing this module does is
//! not to add counters but to open a door to the ones already there — which is
//! also why the `cpu` channel costs the block-entry path *nothing*: it reads a
//! `u64` once, after the guest has stopped.
//!
//! # How it gets hold of a processor
//!
//! The same seam [`crate::host::display::pc::capture`] uses, for the same
//! reason and with the same note attached: `Device` has no `Any` in its
//! supertrait chain, so there is no route from a `dyn Device` to a concrete
//! core. [`install`] therefore takes its handle at the one moment the concrete
//! type exists — construction — by replacing each CPU class's constructor with
//! one that keeps a clone before handing the device on.
//!
//! That means the constructor call is written **twice**: once in `cpu::x86::
//! bind` and once here. A drift between them would build a subtly different
//! machine, so it is not left to care: `tests/cli_trace.rs` asserts that the
//! same workload reaches the same state hash with tracing installed and
//! without, which is exactly the assertion a wrong default variant fails. When
//! `Device` grows a statistics hook the way it will grow a scanout one, this
//! and the display seam delete together.
//!
//! # What is not here
//!
//! A reason for each quantum boundary, which needs a "why did this round end"
//! type `core::sched` has not grown. `docs/testing/tracing.md` specifies it
//! precisely enough to apply.
//!
//! Per-region MMIO counts *are* here, and they are the exception to everything
//! above: the only channel whose numbers are pushed from a hook rather than
//! read off the machine, because an MMIO access leaves no trace on any object
//! this module can reach afterwards. `core::space::flat` interns each aperture
//! when it flattens and counts into `core::trace`'s per-region array from the
//! three dispatch arms that end in a `MemOps` call. What that costs — nothing
//! at all on the RAM path, in instructions or in bytes of `FlatEntry` — is
//! measured in `docs/testing/tracing.md`.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::trace::{Channel, Table};
use crate::machine::Machine;

#[allow(unused_imports)]
use crate::core::hosts::{Captured, HostKind, HostObjects};

/// The `HostKind` slot the captured processors live in.
///
/// The same kind the display seam uses, and safely so: an entry is keyed on
/// `(kind, name)` and the names here are CPU class names, which no scanout
/// capture claims. Nothing is ever `take`n out of it either — [`collect`] reads
/// through `Captured::all` and leaves the handles where they are, so a caller
/// may collect twice.
#[allow(dead_code)]
const KIND: HostKind = HostKind::CAPTURE;

/// Turn on the counting for `channels`.
///
/// Separate from [`install`] because they happen at different moments and one
/// of them may be skipped: the interception has to be in place *before* the
/// machine is built, while enabling can wait until the run is about to start —
/// and a caller that only wants `sched` never needs the interception at all.
pub fn enable(channels: &[Channel]) {
    for ch in channels {
        crate::core::trace::enable(*ch);
    }
}

/// Whether this build can actually count anything.
///
/// False without the `trace` feature. The caller that has to *say* so rather
/// than counting nothing quietly is the command line.
#[must_use]
pub fn available() -> bool {
    cfg!(feature = "trace")
}

/// Intercept this build's processor constructors, so the `cpu` channel has
/// something to read when the run ends.
///
/// Call it on the [`BuildOptions`](crate::machine::BuildOptions) *before*
/// [`crate::machine::build`]. Harmless when the build has no processor classes
/// in it at all: nothing is replaced and [`collect`] finds nothing.
///
/// # Errors
///
/// [`Error::Config`](crate::Error::Config) if something else has already
/// claimed this build's capture table under one of these class names.
#[allow(unused_variables, clippy::needless_pass_by_ref_mut)]
pub fn install(options: &mut crate::machine::BuildOptions) -> Result<()> {
    // One arm per core family that keeps translation statistics, exactly like
    // the registration lists in `machine::catalog` and the scanout arms in
    // `src/bin/rsemu.rs`: a family that is not named here is not in the trace,
    // and that is visible by reading the code rather than by running it.
    #[cfg(all(feature = "cpu-x86", feature = "cpu-x86-lift", feature = "jit"))]
    {
        use crate::cpu::x86::{Variant, X86};
        let seen: alloc::sync::Arc<Captured<X86>> =
            options
                .realize
                .hosts
                .open(KIND, crate::cpu::x86::CLASS.name, Captured::new)?;
        // `Variant::I80486` is what `cpu::x86::bind` defaults to; the state-hash
        // test is what keeps the two in step.
        options
            .bindings
            .replace(crate::cpu::x86::CLASS.name, move |props| {
                let cpu =
                    alloc::sync::Arc::new(X86::from_props_defaulting(props, Variant::I80486)?);
                seen.push(&cpu);
                Ok(cpu)
            });
    }
    #[cfg(all(feature = "cpu-arm-a64", feature = "cpu-arm-a64-lift", feature = "jit"))]
    {
        use crate::cpu::arm::a64::Cpu;
        let seen: alloc::sync::Arc<Captured<Cpu>> =
            options
                .realize
                .hosts
                .open(KIND, crate::cpu::arm::a64::CLASS.name, Captured::new)?;
        options
            .bindings
            .replace(crate::cpu::arm::a64::CLASS.name, move |props| {
                let cpu = alloc::sync::Arc::new(Cpu::from_props(props)?);
                seen.push(&cpu);
                Ok(cpu)
            });
    }
    #[cfg(all(feature = "cpu-riscv", feature = "cpu-riscv-lift", feature = "jit"))]
    {
        use crate::cpu::riscv::Hart;
        let seen: alloc::sync::Arc<Captured<Hart>> =
            options
                .realize
                .hosts
                .open(KIND, crate::cpu::riscv::CLASS.name, Captured::new)?;
        options
            .bindings
            .replace(crate::cpu::riscv::CLASS.name, move |props| {
                let hart = alloc::sync::Arc::new(Hart::from_props(props)?);
                seen.push(&hart);
                Ok(hart)
            });
    }
    Ok(())
}

/// Everything the run has to say, as a table ready to render.
///
/// `hosts` is the same [`HostObjects`] [`install`] was given; without it the
/// `cpu` channel has nothing to read and reports nothing, which is what a
/// caller that never installed should get.
#[must_use]
pub fn collect(machine: &Machine, hosts: &HostObjects, channels: &[Channel]) -> Table {
    let mut table = Table::new();
    table.note("machine", machine.name());
    table.note("guest-ns", &machine.now().as_nanos().to_string());
    table.note("threading", &machine.threading_mode().to_string());
    table.note(
        "channels",
        &channels
            .iter()
            .map(|c| c.name())
            .collect::<Vec<_>>()
            .join(","),
    );
    // The state hash goes in the header because a trace is only comparable
    // against another trace of *the same run*, and this is what says so. A
    // parallel or accelerated run has none, and says that instead of a number.
    match machine.state_hash() {
        Ok(hash) if machine.threading_mode().is_deterministic() => {
            table.note("state-hash", &format!("{hash:#018x}"));
        }
        _ => table.note("state-hash", "not reproducible under this threading mode"),
    }

    for ch in channels {
        match *ch {
            Channel::SCHED => table.collect(Channel::SCHED),
            Channel::CLOCK => clocks(machine, &mut table),
            Channel::CPU => cpus(machine, hosts, &mut table),
            Channel::MMIO => mmio(&mut table),
            _ => {}
        }
    }
    table
}

/// Per-region MMIO reads and writes, out of the counters
/// `core::space::flat`'s dispatch arms fed while the run was going.
///
/// The only channel here whose numbers were *pushed* rather than read off the
/// machine, so it is also the only one that needs no handle on anything: the
/// identity is interned process-wide, and the names come back with it.
///
/// A region with no accesses at all still gets its two rows. The set is small
/// and bounded by the board's apertures, and "the guest never touched the
/// RTC" is exactly the kind of answer this channel exists to give — unlike a
/// per-core zero, which cannot be told apart from a core that does not count.
fn mmio(table: &mut Table) {
    let (names, dropped) = crate::core::trace::mmio_regions();
    if names.is_empty() {
        table.note("mmio", "no MMIO aperture was flattened in this process");
        return;
    }
    let (mut reads, mut writes) = (0u64, 0u64);
    for (id, name) in names.iter().enumerate() {
        let Ok(id) = u16::try_from(id) else { break };
        let r = crate::core::trace::mmio_get(id, false);
        let w = crate::core::trace::mmio_get(id, true);
        // A row is two whitespace-separated fields and there is no quoting, so
        // a region whose name has a space in it would break the format for
        // every reader. Nothing in the tree names one that way; this is what
        // keeps that from being a promise the next device has to remember.
        let name: String = name
            .chars()
            .map(|c| if c.is_whitespace() { '_' } else { c })
            .collect();
        table.set(&format!("mmio.{name}.read"), r);
        table.set(&format!("mmio.{name}.write"), w);
        reads = reads.saturating_add(r);
        writes = writes.saturating_add(w);
    }
    table.set("mmio.read", reads);
    table.set("mmio.write", writes);
    if dropped != 0 {
        // Named rather than counted into some other region: a dense index has
        // to say when it ran out, or its totals quietly stop adding up.
        table.note(
            "mmio",
            &format!("{dropped} apertures past the counter array are not counted"),
        );
    }
}

/// Per-clock-domain tick totals.
///
/// The same numbers `rsemu run` already prints in its summary, in a form a
/// script can read — and in the same file as everything else, which is the
/// point: "the CPU ran 24 818 rounds and its domain turned over a billion
/// ticks" is one question, not two.
fn clocks(machine: &Machine, table: &mut Table) {
    for device in machine.devices() {
        let Some(domain) = device.domain() else {
            continue;
        };
        if let Ok(ticks) = machine.clocks().ticks(domain) {
            table.set(&format!("clock.{}.ticks", device.path()), ticks);
        }
    }
}

/// Per-processor execution statistics, out of each core's own `jit_stats`.
///
/// Rows are named by the *instance path* from the machine file — `cpu.cpu0.
/// blocks`, not `cpu.0.blocks` — because that is the name the machine file, the
/// summary and gdb all use, and a trace that invented a third naming would be a
/// trace nobody could correlate. The captured handles carry no path (they are
/// taken at construction, before the instance exists), so the two are matched
/// by construction order, which is declaration order; a machine where the
/// counts disagree falls back to an index rather than mislabelling a row.
#[allow(unused_variables)]
fn cpus(machine: &Machine, hosts: &HostObjects, table: &mut Table) {
    #[cfg(all(feature = "cpu-x86", feature = "cpu-x86-lift", feature = "jit"))]
    {
        let paths = paths_of(machine, crate::cpu::x86::CLASS.name);
        for (index, cpu) in captured::<crate::cpu::x86::X86>(hosts, crate::cpu::x86::CLASS.name)
            .iter()
            .enumerate()
        {
            let name = label(&paths, index);
            table.note(
                &name,
                &format!("engine={}", kebab(&format!("{:?}", cpu.engine()))),
            );
            let Some(stats) = cpu.jit_stats() else {
                continue;
            };
            let rows = [
                ("blocks", stats.blocks),
                ("compiled", stats.compiled),
                ("chained", stats.chained),
                ("translated", stats.translated),
                ("invalidated", stats.invalidated),
                ("retired", stats.retired),
                ("interpreted", stats.interpreted),
            ];
            emit(table, &name, &rows);
        }
    }
    #[cfg(all(feature = "cpu-arm-a64", feature = "cpu-arm-a64-lift", feature = "jit"))]
    {
        let paths = paths_of(machine, crate::cpu::arm::a64::CLASS.name);
        for (index, cpu) in
            captured::<crate::cpu::arm::a64::Cpu>(hosts, crate::cpu::arm::a64::CLASS.name)
                .iter()
                .enumerate()
        {
            let name = label(&paths, index);
            table.note(
                &name,
                &format!("engine={}", kebab(&format!("{:?}", cpu.engine()))),
            );
            let Some(stats) = cpu.jit_stats() else {
                continue;
            };
            let rows = [
                ("blocks", stats.blocks),
                ("compiled", stats.compiled),
                ("chained", stats.chained),
                ("translated", stats.translated),
                // A64 counts the two invalidation mechanisms apart, because a
                // single total lets one of them stop working while the other
                // holds the number up — a mutation pass demonstrated exactly
                // that. The sum is reported too, so the row means the same
                // thing it does on x86.
                ("invalidated", stats.smc + stats.smc_interpreted),
                ("invalidated.in-block", stats.smc),
                ("invalidated.interpreted", stats.smc_interpreted),
                ("retired", stats.retired),
                ("interpreted", stats.interpreted),
                ("fast-loads", stats.fast_loads),
                ("fast-stores", stats.fast_stores),
            ];
            emit(table, &name, &rows);
        }
    }
    #[cfg(all(feature = "cpu-riscv", feature = "cpu-riscv-lift", feature = "jit"))]
    {
        let paths = paths_of(machine, crate::cpu::riscv::CLASS.name);
        for (index, hart) in
            captured::<crate::cpu::riscv::Hart>(hosts, crate::cpu::riscv::CLASS.name)
                .iter()
                .enumerate()
        {
            let name = label(&paths, index);
            table.note(
                &name,
                &format!("engine={}", kebab(&format!("{:?}", hart.engine()))),
            );
            // `retired-total` is the architectural `minstret`, which counts
            // interpreted instructions too and is therefore *not* the
            // `retired` row below: that one is what retired inside a block,
            // and the two together are what says how much of a run the
            // translated engine actually carried.
            table.set(&format!("cpu.{name}.retired-total"), hart.instret());
            table.set(&format!("cpu.{name}.cycles"), hart.cycles());
            let Some(stats) = hart.jit_stats() else {
                continue;
            };
            let rows = [
                ("blocks", stats.blocks),
                ("compiled", stats.compiled),
                ("chained", stats.chained),
                ("translated", stats.translated),
                // Counted apart for the reason A64 counts them apart: a single
                // total lets one of the two mechanisms stop working while the
                // other holds the number up. The sum is reported too, so the
                // row means the same thing it does on x86.
                ("invalidated", stats.smc + stats.smc_interpreted),
                ("invalidated.in-block", stats.smc),
                ("invalidated.interpreted", stats.smc_interpreted),
                ("retired", stats.retired),
                ("interpreted", stats.interpreted),
                ("fast-loads", stats.fast_loads),
                ("fast-stores", stats.fast_stores),
            ];
            emit(table, &name, &rows);
        }
    }

    // A board whose processors keep none of these counters — a 6502, an
    // accelerated core, a build with no translation runtime — gets a sentence
    // rather than a column of zeroes. "Zero blocks executed" and "nothing here
    // counts blocks" are different facts, and a table that spelled them the
    // same way would be a table that lies about the second one.
    if !table.rows().any(|(name, _)| name.starts_with("cpu.")) {
        table.note(
            "cpu",
            "no processor in this machine keeps translation statistics",
        );
        return;
    }

    // Machine-wide totals. On a one-processor board they repeat the per-core
    // rows, which is the price of a script being able to ask one question of
    // any board; on an SMP one they are the number anybody actually wanted.
    //
    // A total is written only where at least one core contributed a row. A core
    // running the interpreter has a cycle count and no block count, and
    // `cpu.blocks 0` beside it would read as "the JIT ran and did nothing"
    // rather than "this core is not translating".
    for row in ["blocks", "compiled", "chained", "translated", "invalidated"] {
        if let Some(total) = sum_over(table, row) {
            table.set(&format!("cpu.{row}"), total);
        }
    }
    let retired = sum_over(table, "retired");
    let interpreted = sum_over(table, "interpreted");
    if let Some(retired) = retired {
        table.set("cpu.retired", retired);
    }
    if let Some(interpreted) = interpreted {
        table.set("cpu.interpreted", interpreted);
    }
    let (retired, interpreted) = (retired.unwrap_or(0), interpreted.unwrap_or(0));
    // Integer per-mille rather than a percentage with a decimal point: the
    // determinism rule forbids a float in anything a run produces, and the
    // documents' "99.3%" is this ÷ 10. The raw counts are both above it, so a
    // reader who wants more digits has them. A machine that retired no
    // instructions at all gets no row rather than a nought.
    if let Some(permille) = retired
        .saturating_mul(1_000)
        .checked_div(retired + interpreted)
    {
        table.set("cpu.retired.permille", permille);
    }
}

/// Write one core's rows under `cpu.<name>.<row>`.
#[allow(dead_code)]
fn emit(table: &mut Table, name: &str, rows: &[(&str, u64)]) {
    for (row, value) in rows {
        table.set(&format!("cpu.{name}.{row}"), *value);
    }
}

/// Add up every per-core row with this suffix.
///
/// Reads the table back rather than accumulating alongside it, so a core family
/// added later is included in the totals without touching this function.
#[allow(dead_code)]
fn sum_over(table: &Table, row: &str) -> Option<u64> {
    let suffix = format!(".{row}");
    let aggregate = format!("cpu.{row}");
    let mut total = None;
    for (_, value) in table.rows().filter(|(name, _)| {
        // The aggregate row itself is excluded by name rather than by counting
        // dots: an instance path can be nested (`soc.cpu0`), and a dot count
        // would then drop a real row.
        name.starts_with("cpu.") && name.ends_with(&suffix) && *name != aggregate
    }) {
        total = Some(total.unwrap_or(0) + value);
    }
    total
}

/// A core's `Engine` in the spelling the machine file uses.
///
/// The three engine enumerations have no `Display` and no `as_str`, and their
/// `Debug` spelling is `JitHost` where `engine = "jit-host"` is what a person
/// wrote in the board. Converting is six lines here against an accessor added
/// to three files somebody else owns; when those grow an `as_str`, this
/// deletes.
#[allow(dead_code)]
fn kebab(debug: &str) -> String {
    let mut out = String::new();
    for (index, ch) in debug.char_indices() {
        if ch.is_ascii_uppercase() {
            if index != 0 {
                out.push('-');
            }
            out.push(ch.to_ascii_lowercase());
        } else {
            out.push(ch);
        }
    }
    out
}

/// The instance paths of every device of `class`, in declaration order.
#[allow(dead_code)]
fn paths_of(machine: &Machine, class: &str) -> Vec<String> {
    machine
        .devices()
        .iter()
        .filter(|d| d.class().name == class)
        .map(|d| d.path().to_string())
        .collect()
}

/// The name row for the `index`th captured core of a class.
#[allow(dead_code)]
fn label(paths: &[String], index: usize) -> String {
    paths
        .get(index)
        .cloned()
        .unwrap_or_else(|| format!("cpu{index}"))
}

/// Every core of `class` this build constructed, oldest first.
///
/// Public because the handles are worth more than the table to a caller that
/// has a use for the core itself — a differential harness comparing two engines
/// on one board, an embedder reading a register — and because it is where the
/// key convention is written down: [`install`] files each core under
/// `(HostKind::CAPTURE, <class name>)`, and this is the matching read.
///
/// Empty unless [`install`] was called on the options this machine was built
/// from, which is the honest answer for a build that never asked to trace.
#[must_use]
pub fn captured<T: core::any::Any + Send + Sync>(
    hosts: &HostObjects,
    class: &'static str,
) -> Vec<alloc::sync::Arc<T>> {
    hosts
        .get::<Captured<T>>(KIND, class)
        .ok()
        .flatten()
        .map(|seen| seen.all())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests;
