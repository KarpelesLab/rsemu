//! The command engine, driven the way a terminal drives it.
//!
//! [`Monitor::execute`] takes a line and returns text, so a session is a `for`
//! loop over `&str` and there is no TTY anywhere in this file — which is the
//! whole reason the engine and the front end are separate. `tests/cli_monitor.rs`
//! is the other half: the same commands through the shipped binary, with the
//! script arriving on a pipe.

use super::*;

/// Build and drive an Apple 1, which is the smallest machine in the catalog
/// with a processor, a clocked device, an MMIO device with side effects and a
/// character port — everything a monitor command touches.
#[cfg(feature = "machine-apple1")]
fn apple1() -> (Machine, std::sync::Arc<HostObjects>) {
    // The `rom` slot takes the board's default built-in image — `rsmon`, ours
    // and MIT — exactly as `rsemu run apple1` binds it when nobody names a
    // file. A machine with an empty ROM would boot into open bus and every
    // assertion about what the guest does would be about nothing.
    let builtin = crate::machine::catalog::builtins("apple1")
        .first()
        .expect("the apple1 ships a monitor ROM");
    crate::machine::catalog::build_catalog_with_hosts("apple1", &[(builtin.slot, builtin.bytes)])
        .expect("this build has `machine-apple1`, because the test is gated on it")
}

/// An Apple 1 whose ROM sets the PIA up and then spins.
///
/// Seventeen bytes of 6502 — the same prologue the shipped `rsmon` opens with,
/// which is where the register values come from (`src/dev/apple1/monitor.rs`)
/// — and then `JMP` to itself. The point is what it does *not* do: the shipped
/// monitor polls `$D011` in a loop and reads `$D010` the instant a key latches,
/// so a test that wanted to find a keystroke still waiting would be racing a
/// guest that always wins. A processor in an infinite loop takes nothing.
#[cfg(feature = "machine-apple1")]
fn apple1_spinning() -> (Machine, std::sync::Arc<HostObjects>) {
    // The ROM region is $FF00..$FFFF, so the reset vector at $FFFC points at
    // the first byte of the image.
    let mut rom = [0u8; 256];
    rom[..0x14].copy_from_slice(&[
        0xd8, // CLD
        0xa2, 0xff, // LDX #$FF
        0x9a, // TXS
        0xa9, 0x7f, // LDA #$7F
        0x8d, 0x12, 0xd0, // STA $D012   — DDRB: PB0-PB6 are the display
        0xa9, 0xa7, // LDA #$A7
        0x8d, 0x11, 0xd0, // STA $D011   — CRA: DDR access on, so $D010 is the key
        0x8d, 0x13, 0xd0, // STA $D013   — CRB, the same
        0x4c, 0x11, 0xff, // JMP $FF11   — itself
    ]);
    rom[0xfc] = 0x00;
    rom[0xfd] = 0xff;
    crate::machine::catalog::build_catalog_with_hosts("apple1", &[("rom", &rom)])
        .expect("this build has `machine-apple1`, because the test is gated on it")
}

/// A one-line convenience: run a script and hand back everything it printed.
#[cfg(feature = "machine-apple1")]
fn script(machine: &mut Machine, hosts: &HostObjects, lines: &[&str]) -> String {
    let mut monitor = Monitor::new();
    let mut target = MachineTarget::new(machine);
    let mut env = Env {
        hosts: Some(hosts),
        // A deadline, so that a bare `cont` — which is interactive and stops
        // when a person presses Ctrl-C — is bounded here instead. Nothing else
        // reads it: `run <span>` and `cont <span>` compute their own end from
        // the span, and this one only ever caps the unbounded spelling.
        // `every_word_help_lists_is_a_command_that_answers` types every word in
        // the help, `cont` among them, and without this it would type that one
        // and never come back.
        deadline: Some(GlobalTime::from_nanos(1_000_000)),
        timeline: None,
    };
    let mut out = String::new();
    for line in lines {
        let response = monitor.execute(&mut target, &mut env, line);
        out.push_str(&response.text);
        if let Flow::Advance(span) = response.flow {
            out.push_str(&monitor.advance(&mut target, &mut env, span, |_| true));
        }
    }
    out
}

/// The byte a one-byte `x` dump reported.
///
/// Both callers need a machine to dump memory from, so this follows them
/// behind the board they use: `--features monitor` alone builds neither.
#[cfg(feature = "machine-apple1")]
fn dumped_byte(text: &str) -> u8 {
    let field = text
        .split_whitespace()
        .nth(1)
        .unwrap_or_else(|| panic!("no byte in the dump:\n{text}"));
    u8::from_str_radix(field, 16).unwrap_or_else(|_| panic!("`{field}` is not a byte, in:\n{text}"))
}

// ---------------------------------------------------------------------------
// parsing, which needs no machine
// ---------------------------------------------------------------------------

#[test]
fn an_address_is_hex_with_or_without_the_prefix() {
    assert_eq!(addr_of("d010"), Ok(0xd010));
    assert_eq!(addr_of("0xD010"), Ok(0xd010));
    assert_eq!(addr_of("0X10"), Ok(0x10));
    // Decimal-looking input is hex too, because that is what the gdbstub's own
    // monitor commands have always done and two conventions on one prompt is
    // how somebody patches the wrong byte.
    assert_eq!(addr_of("20"), Ok(0x20));
    assert!(addr_of("nowhere").is_err());
}

#[test]
fn a_span_carries_its_unit_or_it_is_refused() {
    assert_eq!(
        span_of(Some("1s")),
        Ok(GlobalTime::from_nanos(1_000_000_000))
    );
    assert_eq!(
        span_of(Some("500ms")),
        Ok(GlobalTime::from_nanos(500_000_000))
    );
    // A bare number would mean whichever unit this module happened to pick,
    // which is how a one-second step becomes a one-millisecond one.
    assert!(span_of(Some("1")).is_err());
    assert!(span_of(None).is_err());
}

#[test]
fn write_bytes_may_be_spaced_or_run_together() {
    assert_eq!(
        bytes_of(&["de", "ad", "be", "ef"]),
        Ok(vec![0xde, 0xad, 0xbe, 0xef])
    );
    assert_eq!(bytes_of(&["deadbeef"]), Ok(vec![0xde, 0xad, 0xbe, 0xef]));
    assert_eq!(
        bytes_of(&["dead", "beef"]),
        Ok(vec![0xde, 0xad, 0xbe, 0xef])
    );
}

#[test]
fn an_odd_number_of_hex_digits_is_refused_rather_than_padded() {
    // Padding it would have to guess which end the missing nibble belongs to,
    // and both answers write a byte the caller did not type.
    let e = bytes_of(&["abc"]).expect_err("three digits is not whole bytes");
    assert!(e.contains("pairs"), "the refusal does not say why: {e}");
    assert!(bytes_of(&[]).is_err());
    assert!(bytes_of(&["zz"]).is_err());
}

#[test]
fn a_write_longer_than_the_bound_names_the_way_to_load_an_image() {
    let long = "00".repeat(WRITE_MAX + 1);
    let e = bytes_of(&[&long]).expect_err("over the bound");
    assert!(
        e.contains("--media"),
        "a refused bulk write should say where bulk bytes go: {e}"
    );
}

#[test]
fn a_register_reads_little_endian_because_the_chunk_encoding_is() {
    // Printed in order this would read 0x0dd0, which is the same digits and a
    // different number — the sort of bug that survives a review.
    assert_eq!(hex_of(&[0xd0, 0x0d]), "0x0dd0");
    assert_eq!(hex_of(&[0x2a]), "0x2a");
}

#[test]
fn a_rate_is_printed_exactly_and_never_as_a_float() {
    use crate::core::clock::Rational;
    // 39375000/22 Hz is an NTSC 6502 and 1789772.7 Hz is a lie about it —
    // `CLAUDE.md`'s no-floats-in-the-time-path rule, applied to the place
    // people actually read the number off.
    // Written against `Rational`'s own numerator and denominator rather than
    // against the digits typed here, because `Rational::new` reduces the
    // fraction — 39375000/22 is stored as 19687500/11. The property under test
    // is that what comes out is the ratio the machine holds, not a rounding of
    // it.
    let odd = Rational::new(39_375_000, 22).expect("a representable rate");
    assert_eq!(hz_of(odd), format!("{}/{} Hz", odd.num(), odd.den()));
    assert!(!hz_of(odd).contains('.'), "a rate came out as a decimal");
    assert_eq!(hz_of(Rational::integer(60)), "60 Hz");
}

#[test]
fn help_with_no_topic_is_the_whole_command_set() {
    let text = help(None);
    for command in ["run", "step", "regs", "clocks", "wires", "rewind", "trace"] {
        assert!(
            text.contains(command),
            "`help` does not mention `{command}`"
        );
    }
    assert!(help(Some("nonsense")).contains("no help for"));
}

// ---------------------------------------------------------------------------
// the command set, against a real machine
// ---------------------------------------------------------------------------

#[cfg(feature = "machine-apple1")]
#[test]
fn a_blank_line_and_a_comment_are_nothing() {
    let (mut machine, hosts) = apple1();
    let out = script(
        &mut machine,
        &hosts,
        &["", "   ", "# a script may explain itself"],
    );
    assert!(out.is_empty(), "something answered a blank line: {out:?}");
}

#[cfg(feature = "machine-apple1")]
#[test]
fn an_unknown_command_says_so_and_points_at_help() {
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["chocolate"]);
    assert!(
        out.contains("unknown command `chocolate`") && out.contains("help"),
        "the refusal is unhelpful: {out}"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn every_word_help_lists_is_a_command_that_answers() {
    // The one test that keeps `help` honest. Each line of the command sections
    // starts with two spaces and a word; that word must dispatch to something.
    let (mut machine, hosts) = apple1();
    let mut names = Vec::new();
    for line in HELP.lines() {
        let Some(body) = line.strip_prefix("  ") else {
            continue;
        };
        let Some(word) = body.split_whitespace().next() else {
            continue;
        };
        if word.starts_with(char::is_alphabetic) && !body.starts_with("addresses") {
            names.push(word.to_string());
        }
    }
    assert!(
        names.len() >= 20,
        "help lists only {} commands",
        names.len()
    );
    for name in &names {
        // Argumentless: a command that needs one answers with its own
        // complaint, which is still an answer rather than a refusal to
        // recognise the word. `quit` is the exception — it ends the session
        // rather than printing — and it is covered by its own test.
        if name == "quit" {
            continue;
        }
        let out = script(&mut machine, &hosts, &[name]);
        assert!(
            !out.contains("unknown command"),
            "`{name}` is in `help` and is not a command:\n{out}"
        );
    }
}

#[cfg(feature = "machine-apple1")]
#[test]
fn every_command_the_gdbstub_already_answers_is_reachable_here() {
    // The eight `qRcmd` commands are forwarded rather than reimplemented (see
    // the module docs). This is the door that keeps them reachable: if the
    // forwarding arm is ever narrowed to a list, this fails.
    let (mut machine, hosts) = apple1();
    for name in ["devices", "spaces", "map", "time", "hash"] {
        let out = script(&mut machine, &hosts, &[name]);
        assert!(
            !out.is_empty() && !out.contains("unknown command"),
            "`{name}` is no longer forwarded to the debug target:\n{out}"
        );
    }
    for line in ["x d010 4", "xp ff00 4", "translate 20"] {
        let out = script(&mut machine, &hosts, &[line]);
        assert!(
            !out.contains("unknown command"),
            "`{line}` is no longer forwarded:\n{out}"
        );
    }
}

#[cfg(feature = "machine-apple1")]
#[test]
fn quit_and_its_spellings_end_the_session() {
    let (mut machine, hosts) = apple1();
    let mut monitor = Monitor::new();
    let mut target = MachineTarget::new(&mut machine);
    let mut env = Env {
        hosts: Some(&hosts),
        timeline: None,
        deadline: None,
    };
    for spelling in ["quit", "q", "exit"] {
        let response = monitor.execute(&mut target, &mut env, spelling);
        assert_eq!(response.flow, Flow::Quit, "`{spelling}` did not end it");
        assert!(response.text.is_empty(), "`{spelling}` printed something");
    }
}

#[cfg(feature = "machine-apple1")]
#[test]
fn run_advances_exactly_the_span_it_was_given() {
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["run 250ms"]);
    assert!(
        out.contains("stopped at"),
        "`run` said nothing about where it stopped: {out}"
    );
    // `run_until` stops on the machine's own scheduling boundaries, so the
    // instant reached is at or just under the deadline, never past it.
    let now = machine.now().as_nanos();
    assert!(
        (249_000_000..=250_000_000).contains(&now),
        "250ms of virtual time landed at {now} ns"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_span_taken_in_pieces_reaches_the_same_state_as_one_taken_whole() {
    // The property the whole design rests on: `Machine::run_until` is additive
    // (ROADMAP.md §11.6), so slicing an advance into ten-millisecond pieces —
    // which is what `Monitor::advance` does — cannot move the answer. Without
    // it, a monitor session and a headless run would disagree and neither
    // would be wrong.
    let (mut whole, hosts_a) = apple1();
    script(&mut whole, &hosts_a, &["run 1s"]);

    let (mut pieces, hosts_b) = apple1();
    script(
        &mut pieces,
        &hosts_b,
        &["run 250ms", "run 250ms", "run 250ms", "run 250ms"],
    );

    assert_eq!(
        whole.now(),
        pieces.now(),
        "the two runs ended at different instants"
    );
    assert_eq!(
        whole.state_hash().expect("deterministic threading"),
        pieces.state_hash().expect("deterministic threading"),
        "one span and four quarters of it reached different states"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn an_inspection_leaves_a_waiting_keystroke_exactly_where_it_was() {
    // `MemAttrs::debug` exists for this and nothing else. The Apple 1's PIA
    // clears the key-waiting flag in `$D011` when the guest reads the key out
    // of `$D010`; a monitor that read the same byte the same way would consume
    // the keystroke and the guest would never see it.
    use crate::core::space::MemAttrs;
    use crate::core::value::Width;
    use crate::host::chardev::ports;

    let (mut machine, hosts) = apple1_spinning();
    // Let the ROM configure the PIA — until `CRA` has the DDR-access bit, a
    // read of `$D010` is a read of the data-direction register and clears
    // nothing, so the test would pass without proving anything.
    machine
        .run_until(GlobalTime::from_nanos(1_000_000))
        .expect("the apple1 runs");
    let port = ports::get(&hosts, "console")
        .expect("the console port")
        .expect("the apple1 opens one");
    port.feed(b"A");
    // Long enough for the PIA's own 60 Hz tick to latch the byte. The guest is
    // in an infinite loop and will not take it: that is the whole reason this
    // test does not run the shipped monitor ROM, which polls `$D011` and would
    // have consumed the keystroke within microseconds of it arriving.
    machine
        .run_until(GlobalTime::from_nanos(60_000_000))
        .expect("the apple1 runs");

    let flag = |text: &str| dumped_byte(text) & 0x80;

    let before = script(&mut machine, &hosts, &["x d011 1"]);
    assert_ne!(
        flag(&before),
        0,
        "the PIA is not holding a key, so this test proves nothing:\n{before}"
    );

    // Four debug reads of the register a guest read would empty.
    let key = script(
        &mut machine,
        &hosts,
        &["x d010 1", "x d010 1", "x d010 1", "x d010 1"],
    );
    assert_eq!(
        dumped_byte(&key),
        0xc1,
        "the key is not the 'A' that was typed:\n{key}"
    );

    let after = script(&mut machine, &hosts, &["x d011 1"]);
    assert_ne!(
        flag(&after),
        0,
        "looking at $D010 cleared the key-waiting flag — MemAttrs::debug is not \
         reaching the device:\n{after}"
    );

    // And the other half of the proof: the flag really would have moved. A
    // read with ordinary attributes — the one a guest makes — clears it.
    let space = machine.space("cpubus").expect("the apple1's bus");
    space
        .read(0xd010, Width::U8, MemAttrs::DEFAULT)
        .expect("the PIA answers");
    let cleared = script(&mut machine, &hosts, &["x d011 1"]);
    assert_eq!(
        flag(&cleared),
        0,
        "a guest read did not clear the flag either, so the debug read above was \
         never the interesting case:\n{cleared}"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_write_reaches_guest_memory_and_reads_back() {
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["write 0x10 de ad be ef", "x 10 4"]);
    assert!(
        out.contains("wrote 4 bytes"),
        "the write was not acknowledged:\n{out}"
    );
    assert!(
        out.contains("de ad be ef"),
        "the bytes did not read back:\n{out}"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_write_the_bus_refuses_says_so_rather_than_succeeding_quietly() {
    // The PIA refuses a debug write outright rather than guessing at one: a
    // debug write to `$D012` would put a character on the screen and one to
    // `$D011` would change what the next read means, and neither is something
    // the core can make harmless (`ROADMAP.md` §15, invariant 5). The console
    // has to report the refusal rather than a cheerful "wrote 1 byte".
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["write d011 00"]);
    assert!(
        out.contains("cannot write"),
        "a write the device refused was reported as a success:\n{out}"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn selecting_a_processor_that_is_not_there_is_refused_with_the_count() {
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["cpu 7"]);
    assert!(
        out.contains("no processor 7") && out.contains('1'),
        "the refusal does not say how many there are:\n{out}"
    );
    let ok = script(&mut machine, &hosts, &["cpu 0"]);
    assert!(ok.contains("cpu.mos6502"), "selecting cpu 0 said:\n{ok}");
}

#[cfg(feature = "machine-apple1")]
#[test]
fn regs_answers_one_register_by_name_and_refuses_one_that_is_not_there() {
    let (mut machine, hosts) = apple1();
    let one = script(&mut machine, &hosts, &["regs pc"]);
    assert!(one.starts_with("pc"), "`regs pc` answered:\n{one}");
    assert_eq!(
        one.lines().count(),
        1,
        "`regs pc` printed more than pc:\n{one}"
    );
    let none = script(&mut machine, &hosts, &["regs rax"]);
    assert!(
        none.contains("no register `rax`"),
        "a 6502 was asked for rax and said:\n{none}"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn step_moves_the_program_counter_and_status_admits_it() {
    let (mut machine, hosts) = apple1();
    let mut monitor = Monitor::new();
    let mut target = MachineTarget::new(&mut machine);
    let mut env = Env {
        hosts: Some(&hosts),
        timeline: None,
        deadline: None,
    };
    let quiet = monitor.execute(&mut target, &mut env, "status");
    assert!(
        !quiet.text.contains("has used `step`"),
        "a session that has not stepped should not be warning about it"
    );
    assert!(!monitor.has_stepped());

    let stepped = monitor.execute(&mut target, &mut env, "step 2");
    assert!(
        stepped.text.contains("stepped 2 instructions"),
        "step said:\n{}",
        stepped.text
    );
    assert!(monitor.has_stepped());
    let noisy = monitor.execute(&mut target, &mut env, "status");
    assert!(
        noisy.text.contains("no longer comparable"),
        "a stepped session still offers its hash as comparable:\n{}",
        noisy.text
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_step_count_that_is_not_a_count_is_refused() {
    let (mut machine, hosts) = apple1();
    for bad in ["step 0", "step lots"] {
        let out = script(&mut machine, &hosts, &[bad]);
        assert!(out.contains("not a step count"), "`{bad}` said:\n{out}");
    }
}

#[cfg(feature = "machine-apple1")]
#[test]
fn clocks_names_every_domain_with_its_rate_and_its_position() {
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["run 100ms", "clocks"]);
    for domain in ["master", "cpu", "pia", "video"] {
        assert!(out.contains(domain), "`clocks` left out `{domain}`:\n{out}");
    }
    // The CPU domain is 11250000/11 Hz on an Apple 1 and the table must say so
    // rather than rounding it.
    assert!(out.contains('/'), "no exact ratio anywhere in:\n{out}");
    assert!(
        !out.contains('.'),
        "a rate was printed as a decimal:\n{out}"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_device_shows_its_class_its_properties_and_its_current_state() {
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["device pia"]);
    assert!(out.contains("apple1.pia"), "no class in:\n{out}");
    assert!(out.contains("properties"), "no property list in:\n{out}");
    assert!(
        out.contains("port"),
        "the `port` property is not listed:\n{out}"
    );
    assert!(out.contains("state"), "no state in:\n{out}");
    assert!(
        out.contains("key_ready"),
        "the PIA's own state is missing:\n{out}"
    );

    let missing = script(&mut machine, &hosts, &["device nowhere"]);
    assert!(missing.contains("no device at `nowhere`"), "{missing}");
    let unnamed = script(&mut machine, &hosts, &["device"]);
    assert!(unnamed.contains("device path is needed"), "{unnamed}");
}

#[cfg(feature = "machine-apple1")]
#[test]
fn the_scheduler_reports_its_mode_its_quantum_and_its_queue() {
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["sched"]);
    assert!(
        out.contains("deterministic"),
        "no threading mode in:\n{out}"
    );
    assert!(out.contains("quantum"), "no quantum in:\n{out}");
    assert!(out.contains("event(s)"), "no event queue in:\n{out}");
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_snapshot_round_trips_through_a_file() {
    let (mut machine, hosts) = apple1();
    let path = std::env::temp_dir().join(format!("rsemu-monitor-{}.state", std::process::id()));
    let name = path.display().to_string();

    script(&mut machine, &hosts, &["run 200ms"]);
    let saved = machine.state_hash().expect("deterministic");
    let out = script(&mut machine, &hosts, &[&format!("save {name}")]);
    assert!(out.contains("wrote"), "`save` said:\n{out}");

    script(&mut machine, &hosts, &["run 200ms"]);
    assert_ne!(
        machine.state_hash().expect("deterministic"),
        saved,
        "200ms more changed nothing, so the reload below proves nothing"
    );

    let out = script(&mut machine, &hosts, &[&format!("load {name}")]);
    assert!(out.contains("loaded"), "`load` said:\n{out}");
    assert_eq!(
        machine.state_hash().expect("deterministic"),
        saved,
        "the machine did not come back to where the snapshot was taken"
    );
    let _ = std::fs::remove_file(&path);
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_snapshot_file_that_is_not_there_is_refused_by_name() {
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["load /nonexistent/rsemu/state"]);
    assert!(
        out.contains("cannot read") && out.contains("/nonexistent/rsemu/state"),
        "the refusal does not name the file:\n{out}"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn rewind_without_a_timeline_says_which_sessions_have_one() {
    // A session under parallel or accel threading gets no timeline, because
    // `Machine::set_recorder` refuses one and a keyframe with no replay behind
    // it would restore to a machine that then diverged.
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["rewind 1s", "timeline"]);
    assert_eq!(
        out.matches("no timeline").count(),
        2,
        "one of the two did not explain itself:\n{out}"
    );
    assert!(out.contains("deterministic"), "no reason given in:\n{out}");
}

#[cfg(feature = "machine-apple1")]
#[test]
fn rewind_puts_the_machine_back_where_it_was() {
    use crate::core::record::Recorder;
    use crate::machine::Timeline;

    let (mut machine, hosts) = apple1();
    let recorder = std::sync::Arc::new(Recorder::recording());
    machine
        .set_recorder(std::sync::Arc::clone(&recorder))
        .expect("deterministic threading takes a recorder");
    let mut timeline = Timeline::with_default_cadence(recorder);

    let mut monitor = Monitor::new();
    let mut target = MachineTarget::new(&mut machine);
    let mut env = Env {
        hosts: Some(&hosts),
        timeline: Some(&mut timeline),
        deadline: None,
    };

    // Two virtual seconds, so the default one-second cadence has taken
    // keyframes to reach back through.
    let response = monitor.execute(&mut target, &mut env, "run 2s");
    assert_eq!(
        response.flow,
        Flow::Advance(Some(GlobalTime::from_nanos(2_000_000_000)))
    );
    monitor.advance(
        &mut target,
        &mut env,
        Some(GlobalTime::from_nanos(2_000_000_000)),
        |_| true,
    );
    let at_two = target.machine().state_hash().expect("deterministic");

    let held = monitor.execute(&mut target, &mut env, "timeline");
    assert!(
        held.text.contains("keyframes"),
        "no keyframes in:\n{}",
        held.text
    );

    let back = monitor.execute(&mut target, &mut env, "rewind 1s");
    assert!(
        back.text.contains("rewound to"),
        "`rewind` said:\n{}",
        back.text
    );
    let landed = target.machine().now();
    assert!(
        landed < GlobalTime::from_nanos(2_000_000_000),
        "the machine did not move back: still at {} ns",
        landed.as_nanos()
    );
    assert_ne!(
        target.machine().state_hash().expect("deterministic"),
        at_two,
        "the machine is at an earlier instant with the state it had later"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn a_deadline_bounds_a_cont_that_named_no_span() {
    let (mut machine, hosts) = apple1();
    let mut monitor = Monitor::new();
    let mut target = MachineTarget::new(&mut machine);
    let mut env = Env {
        hosts: Some(&hosts),
        timeline: None,
        deadline: Some(GlobalTime::from_nanos(120_000_000)),
    };
    let response = monitor.execute(&mut target, &mut env, "cont");
    assert_eq!(response.flow, Flow::Advance(None));
    monitor.advance(&mut target, &mut env, None, |_| true);
    let now = target.machine().now().as_nanos();
    assert!(
        (110_000_000..=120_000_000).contains(&now),
        "an unbounded cont with a 120ms deadline ran to {now} ns"
    );
}

#[cfg(feature = "machine-apple1")]
#[test]
fn an_advance_stops_when_the_session_says_to() {
    // What a Ctrl-C does: the pump returns false and the advance comes back at
    // the slice boundary rather than at its deadline.
    let (mut machine, hosts) = apple1();
    let mut monitor = Monitor::new();
    let mut target = MachineTarget::new(&mut machine);
    let mut env = Env {
        hosts: Some(&hosts),
        timeline: None,
        deadline: None,
    };
    let mut slices = 0;
    monitor.advance(
        &mut target,
        &mut env,
        Some(GlobalTime::from_nanos(10_000_000_000)),
        |_| {
            slices += 1;
            slices <= 3
        },
    );
    let now = target.machine().now().as_nanos();
    assert!(
        now < 100_000_000,
        "the advance ran past the point it was told to stop: {now} ns"
    );
}

#[cfg(all(feature = "machine-apple1", feature = "trace"))]
#[test]
fn trace_renders_the_same_table_the_flag_writes() {
    use crate::core::trace::Channel;

    crate::host::trace::enable(&[Channel::SCHED]);
    let (mut machine, hosts) = apple1();
    let out = script(&mut machine, &hosts, &["run 50ms", "trace sched"]);
    assert!(
        out.contains("# rsemu-trace 1"),
        "no trace header in:\n{out}"
    );
    assert!(out.contains("sched.quanta"), "no scheduler rows in:\n{out}");
    // Deliberately no wall clock and no timestamp, so two traces of one
    // workload diff cleanly.
    assert!(
        !out.contains("elapsed"),
        "a host time got into the table:\n{out}"
    );

    let wrong = script(&mut machine, &hosts, &["trace weather"]);
    assert!(
        wrong.contains("not a channel"),
        "a misspelled channel was accepted:\n{wrong}"
    );
}
