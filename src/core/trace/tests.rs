//! What the counter core promises.
//!
//! # Why nothing in here enables `sched`
//!
//! The counters are process-global by design (see the module documentation),
//! `cargo test` runs a target's tests on several threads of one process, and
//! dozens of tests in this crate run a machine. So a test that switched the
//! `sched` channel on would count *their* scheduler rounds as well as its own,
//! and would fail or pass depending on what else happened to be running — which
//! is exactly the flake it looks like.
//!
//! The rule that falls out is short: **a test that enables a channel a hook
//! writes to must own its process.** `tests/cli_trace.rs` does, because it runs
//! the shipped binary, and that is where every exact assertion about `sched`
//! lives. Everything here is either channel-free ([`raw`](super::raw) writes
//! whatever it is given) or uses a channel no hook feeds, so it is exact and it
//! is isolated.
//!
//! [`SERIAL`] is still held, because these tests would otherwise collide with
//! *each other* over the same slots.

use super::{BUCKETS, Channel, Counter, SLOTS, Table};
use crate::core::sync::Global;

/// Held for the length of any test that touches the global counters.
///
/// [`Global`] rather than [`Mutex`](crate::core::sync::Mutex) because this is a
/// `static`: the `single` backend's mutex reports a second lock as a recursive
/// one, and `core::sync`'s own test enforces the rule.
static SERIAL: Global<()> = Global::new(());

/// A counter slot with no name and no hook: writing it disturbs nothing and
/// nothing disturbs it.
const SCRATCH: Counter = Counter(7);

#[test]
fn a_channel_round_trips_through_its_name() {
    for ch in Channel::ALL {
        assert_eq!(
            Channel::from_name(ch.name()),
            Some(*ch),
            "{} does not parse back to itself",
            ch.name()
        );
        assert!(!ch.summary().is_empty(), "{} has no summary", ch.name());
    }
    assert_eq!(
        Channel::from_name("mmio"),
        None,
        "not a channel this build has"
    );
    assert_eq!(Channel::from_name(""), None);
}

#[test]
fn every_named_counter_is_inside_the_array_and_clear_of_the_histogram() {
    assert_eq!(Counter::NAMES.len(), SLOTS);
    for (index, name) in Counter::NAMES.iter().enumerate() {
        if name.is_empty() {
            continue;
        }
        assert!(
            index < Counter::SPAN_LOG2.0 as usize,
            "counter {index} ({name}) is inside the histogram's range"
        );
    }
    assert!(
        Counter::SPAN_LOG2.0 as usize + BUCKETS as usize <= SLOTS,
        "the histogram runs off the end of the counter array"
    );
    assert!(
        Counter::NAMES[SCRATCH.0 as usize].is_empty(),
        "the slot these tests scribble on must stay unnamed and unhooked"
    );
}

#[test]
fn every_end_reason_has_a_slot_and_the_two_names_agree() {
    use crate::core::sched::Ended;

    // The window is eight wide and five are used, so a sixth reason costs no
    // slot arithmetic anywhere. `quantum_report` drops a reason past the eighth
    // rather than letting it land on the histogram.
    const { assert!(Ended::COUNT <= Counter::SPAN_LOG2.0 - Counter::ENDED.0) };
    for n in 0..Ended::COUNT {
        let slot = (Counter::ENDED.0 + n) as usize;
        let name = Counter::NAMES[slot];
        assert!(!name.is_empty(), "reason {n} has no name in slot {slot}");
        // `sched.ended.event` is `Ended::EVENT` and nothing else: the rendering
        // side reads `NAMES`, the counting side reads `Ended`, and a trace is
        // only readable if the two say the same thing.
        assert_eq!(
            name,
            alloc::format!("sched.ended.{}", Ended(n).name()),
            "slot {slot} and Ended({n}) disagree"
        );
    }
    assert!(
        Counter::NAMES[(Counter::ENDED.0 + Ended::COUNT) as usize].is_empty(),
        "an unused reason slot must stay unnamed, or it reaches a file as a zero row"
    );
}

#[test]
fn a_disabled_channel_counts_nothing() {
    let _serial = SERIAL.lock();
    super::reset();
    // `sched` is off, so the hook every `Machine::advance_to` calls does
    // nothing — which is also the state every build that did not ask to trace
    // runs in.
    super::quantum(1_000_000, 1, 500, 0);
    super::add(Channel::CLOCK, SCRATCH, 9);
    assert_eq!(super::get(Counter::QUANTA), 0, "nothing was enabled");
    assert_eq!(super::get(SCRATCH), 0);
    assert!(!super::any());
}

#[test]
fn an_enabled_channel_counts_and_reset_clears_it() {
    let _serial = SERIAL.lock();
    super::reset();
    // `clock` has no live hook — it is collected off the machine — so enabling
    // it turns nothing else in the process into a writer.
    super::enable(Channel::CLOCK);
    assert_eq!(
        super::on(Channel::CLOCK),
        cfg!(feature = "trace"),
        "a build without the feature cannot be asked to count, and does not pretend to"
    );
    assert!(!super::on(Channel::SCHED), "one channel is not all of them");
    assert_eq!(super::any(), cfg!(feature = "trace"));

    super::add(Channel::CLOCK, SCRATCH, 7);
    super::add(Channel::CLOCK, SCRATCH, 5);
    super::add(Channel::SCHED, SCRATCH, 1_000_000);

    #[cfg(feature = "trace")]
    assert_eq!(
        super::get(SCRATCH),
        12,
        "the channel that was on added, and the one that was off did not"
    );
    #[cfg(not(feature = "trace"))]
    assert_eq!(
        super::get(SCRATCH),
        0,
        "a build without the feature counts nothing, and says so by counting nothing"
    );

    super::reset();
    assert_eq!(super::get(SCRATCH), 0);
    assert!(!super::any());
}

#[test]
fn raw_writes_whatever_it_is_given_and_an_index_off_the_end_is_dropped() {
    let _serial = SERIAL.lock();
    super::reset();
    // The hot-path form: the caller has already decided, so there is no second
    // test of the mask.
    super::raw(SCRATCH, 3);
    super::raw(Counter(SLOTS as u16 + 100), 1);
    #[cfg(feature = "trace")]
    assert_eq!(super::get(SCRATCH), 3);
    assert_eq!(
        super::get(Counter(SLOTS as u16 + 100)),
        0,
        "a counter past the end of the array reads as zero rather than trapping"
    );
    super::reset();
}

#[test]
fn the_histogram_buckets_by_bit_width_and_saturates_rather_than_dropping() {
    let _serial = SERIAL.lock();
    super::reset();
    super::enable(Channel::CLOCK);
    for value in [0, 1, 1023, 1024, u64::MAX] {
        super::hist(Channel::CLOCK, Counter::SPAN_LOG2, value);
    }

    #[cfg(feature = "trace")]
    {
        assert_eq!(super::get(Counter(Counter::SPAN_LOG2.0)), 1, "zero");
        assert_eq!(super::get(Counter(Counter::SPAN_LOG2.0 + 1)), 1, "one bit");
        assert_eq!(super::get(Counter(Counter::SPAN_LOG2.0 + 10)), 1, "1023");
        assert_eq!(super::get(Counter(Counter::SPAN_LOG2.0 + 11)), 1, "1024");
        assert_eq!(
            super::get(Counter(Counter::SPAN_LOG2.0 + BUCKETS - 1)),
            1,
            "u64::MAX lands in the last bucket rather than off the end"
        );
    }
    super::reset();
}

#[test]
fn a_table_renders_sorted_two_column_text_with_a_hash_header() {
    let mut table = Table::new();
    table.note("machine", "apple1");
    table.set("zebra", 1);
    table.set("alpha", 2);
    table.add("alpha", 3);
    let text = table.render();

    assert!(text.starts_with("# rsemu-trace 1\n"), "{text}");
    assert!(text.contains("# machine         apple1\n"), "{text}");
    let rows: alloc::vec::Vec<&str> = text.lines().filter(|l| !l.starts_with('#')).collect();
    assert_eq!(rows.len(), 2);
    assert!(rows[0].starts_with("alpha "), "sorted: {rows:?}");
    assert!(rows[1].starts_with("zebra "), "sorted: {rows:?}");
    // The shape a script reads: two whitespace-separated fields.
    let fields: alloc::vec::Vec<&str> = rows[0].split_whitespace().collect();
    assert_eq!(fields, ["alpha", "5"], "add() accumulated onto set()");
    assert_eq!(table.get("alpha"), Some(5));
    assert_eq!(table.get("nothing"), None);
    assert_eq!(table.len(), 2);
    assert!(!table.is_empty());
    assert!(Table::new().is_empty());
    assert_eq!(table.rows().count(), 2);
}

#[test]
fn a_rendered_table_has_no_wall_clock_in_it() {
    // The determinism rule, asserted rather than promised: rendering the same
    // table twice is byte-identical, and there is nothing in the output that
    // could differ between two runs of one workload.
    let mut table = Table::new();
    table.note("machine", "apple1");
    table.set("sched.quanta", 1_000);
    assert_eq!(table.render(), table.render());
}

#[test]
fn histogram_rows_sort_into_numeric_order() {
    let _serial = SERIAL.lock();
    super::reset();
    super::enable(Channel::CLOCK);
    // Two buckets either side of ten, which is where an unpadded label sorts
    // wrongly: "10" is less than "9" as text.
    super::hist(Channel::CLOCK, Counter::SPAN_LOG2, 0b1_0000_0000);
    super::hist(
        Channel::CLOCK,
        Counter::SPAN_LOG2,
        0b100_0000_0000_0000_0000_0000,
    );
    let mut table = Table::new();
    table.collect(Channel::SCHED);
    super::reset();

    #[cfg(feature = "trace")]
    {
        let text = table.render();
        let buckets: alloc::vec::Vec<&str> = text
            .lines()
            .filter(|l| l.starts_with("sched.span-ns.log2."))
            .collect();
        assert_eq!(buckets.len(), 2, "{text}");
        assert!(buckets[0].starts_with("sched.span-ns.log2.09"), "{text}");
        assert!(buckets[1].starts_with("sched.span-ns.log2.23"), "{text}");
    }
    #[cfg(not(feature = "trace"))]
    let _ = table;
}

#[test]
fn collect_writes_a_named_counter_even_at_zero() {
    let _serial = SERIAL.lock();
    super::reset();
    let mut table = Table::new();
    table.collect(Channel::SCHED);
    assert_eq!(
        table.get("sched.quanta.idle"),
        Some(0),
        "`no declined boundaries` is an answer; a missing row is not"
    );
    // A channel with no live counters folds nothing in, rather than erroring.
    let mut other = Table::new();
    other.collect(Channel::CPU);
    assert!(other.is_empty());
}

#[test]
fn itoa_agrees_with_the_formatter_it_replaces() {
    for value in [0u64, 1, 9, 10, 99, 100, 4_294_967_296, u64::MAX] {
        assert_eq!(super::itoa(value), alloc::format!("{value}"));
    }
}
