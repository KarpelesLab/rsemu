//! What the collector promises, on a real machine.
//!
//! The determinism claim — that a traced run reaches the same state hash as an
//! untraced one — is asserted end to end against the shipped binary in
//! `tests/cli_trace.rs`, because that is where the whole path is: the
//! interception, the counting, and the file. What is asserted here is the
//! collector's own contract.

#[cfg(feature = "machine-apple1")]
use super::collect;
use super::kebab;
#[cfg(feature = "machine-apple1")]
use crate::core::trace::Channel;

#[test]
fn availability_is_the_feature_and_nothing_else() {
    assert_eq!(super::available(), cfg!(feature = "trace"));
}

#[test]
fn an_engine_name_comes_out_the_way_a_machine_file_spells_it() {
    assert_eq!(kebab("Interp"), "interp");
    assert_eq!(kebab("Jit"), "jit");
    assert_eq!(kebab("JitHost"), "jit-host");
    assert_eq!(kebab(""), "");
}

/// A machine with a clock in it, built without touching the catalog's own
/// bindings, so this test needs no CPU feature.
#[cfg(feature = "machine-apple1")]
fn apple1() -> (crate::machine::Machine, crate::machine::BuildOptions) {
    let entry = crate::machine::catalog::machine("apple1").expect("this build ships apple1");
    let mut options = crate::machine::catalog::build_options().expect("the catalog agrees");
    options
        .realize
        .media
        .insert("rom", &crate::dev::apple1::RSMON[..]);
    let registry = crate::machine::catalog::registry().expect("a registry");
    let machine = crate::machine::build(entry.name, entry.source, &registry, &options)
        .expect("the shipped apple1 builds");
    (machine, options)
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_header_says_which_run_this_is_even_with_no_channels() {
    let (machine, options) = apple1();
    let table = collect(&machine, &options.realize.hosts, &[]);
    let text = table.render();
    assert!(table.is_empty(), "no channel asked for, so no rows: {text}");
    assert!(text.contains("# machine         apple1\n"), "{text}");
    assert!(text.contains("# guest-ns        0\n"), "{text}");
    assert!(text.contains("# state-hash      0x"), "{text}");
    // The header is what makes a trace comparable against another trace of the
    // same run, so it is present whether or not anything was counted.
    assert!(text.contains("# threading       deterministic\n"), "{text}");
}

#[cfg(feature = "machine-apple1")]
#[test]
fn the_clock_channel_reports_what_the_forest_reports() {
    let mut machine = apple1().0;
    let options = crate::machine::catalog::build_options().expect("the catalog agrees");
    machine
        .run_for(crate::core::clock::GlobalTime::from_nanos(1_000_000))
        .expect("a millisecond of apple1");
    let table = collect(&machine, &options.realize.hosts, &[Channel::CLOCK]);
    let mut checked = 0;
    for device in machine.devices() {
        let Some(domain) = device.domain() else {
            continue;
        };
        let ticks = machine.clocks().ticks(domain).expect("a clocked device");
        assert_eq!(
            table.get(&alloc::format!("clock.{}.ticks", device.path())),
            Some(ticks),
            "the trace and the forest disagree about {}",
            device.path()
        );
        checked += 1;
    }
    assert!(checked > 0, "apple1 has clocked devices");
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_board_whose_processors_count_nothing_says_so_rather_than_reporting_zero() {
    // The 6502 keeps no translation statistics — there is no lifter for it — so
    // "zero blocks" would be a false answer to a question this board cannot be
    // asked at all.
    let (machine, options) = apple1();
    let table = collect(&machine, &options.realize.hosts, &[Channel::CPU]);
    let text = table.render();
    assert!(
        table.get("cpu.blocks").is_none(),
        "a column of zeroes is not an answer: {text}"
    );
    assert!(
        text.contains("no processor in this machine keeps translation statistics"),
        "{text}"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn the_scheduler_channel_reports_its_rows_whether_or_not_it_counted() {
    // *What* it counted is asserted in `tests/cli_trace.rs`, which owns its
    // process: enabling `sched` here would count every other test's machine as
    // well as this one, because the counters are process-global on purpose
    // (`core::trace`, "The counters are process-global"). What this asserts is
    // that the channel produces its rows at all — a named counter is written
    // even at zero, so "no declined boundaries" is an answer rather than a
    // missing line.
    let (machine, options) = apple1();
    let table = collect(&machine, &options.realize.hosts, &[Channel::SCHED]);
    for row in ["sched.quanta", "sched.quanta.idle", "sched.span-ns"] {
        assert!(
            table.get(row).is_some(),
            "{row} is missing:\n{}",
            table.render()
        );
    }
}
