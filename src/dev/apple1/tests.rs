//! The whole Apple 1, driven from a script and checked against its output.
//!
//! Not a device test — [`pia`](super::pia) has those. This is the machine: a
//! `.machine` file, a real 6502 fetching a reset vector out of a monitor ROM,
//! the memory map, the scheduler, and a character port with a test on the far
//! end of it typing and reading back. If any part of that is wrong, the bytes
//! coming out are wrong and these fail.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::clock::GlobalTime;
use crate::host::chardev::{CharPort, ports};
use crate::machine::{Machine, build, catalog};

use super::monitor::RSMON;

/// Build the shipped `apple1` machine on its own private character port.
///
/// `paced` picks the real display rate or the flat-out one — the property that
/// exists so a test does not have to spend sixty virtual seconds watching a
/// memory dump crawl past.
fn boot(port_name: &str, paced: bool) -> (Machine, Arc<CharPort>) {
    let port = ports::open(port_name);
    // A name can be reused across runs of the same test binary.
    port.clear();

    let mut options = catalog::build_options().expect("this build has the classes");
    options.realize.media.insert("rom", &RSMON[..]);
    let options = options
        .with_param("console", port_name)
        .with_param("pace", if paced { "true" } else { "false" });

    let registry = catalog::registry().expect("this build has the classes");
    let machine = match build(
        "apple1.machine",
        catalog::APPLE1.source,
        &registry,
        &options,
    ) {
        Ok(m) => m,
        Err(e) => panic!("apple1.machine: {e}"),
    };
    (machine, port)
}

/// Type `input`, run for `millis` of virtual time, and return what came back.
fn exchange(machine: &mut Machine, port: &CharPort, input: &str, millis: u64) -> Vec<u8> {
    port.feed(input.as_bytes());
    machine
        .run_for(GlobalTime::from_nanos(millis * 1_000_000))
        .expect("the machine runs");
    port.drain()
}

/// Output as a human reads it, with the monitor's bare CRs made visible.
fn shown(bytes: &[u8]) -> String {
    let mut out = String::new();
    for &b in bytes {
        match b {
            b'\r' => out.push('\n'),
            0x20..=0x7e => out.push(b as char),
            other => out.push_str(&alloc::format!("\\x{other:02x}")),
        }
    }
    out
}

/// The machine boots, prints its banner and a prompt, and answers a memory
/// dump — the whole point of the exercise, in one test.
#[test]
fn it_boots_prints_a_prompt_and_dumps_memory_on_request() {
    let (mut machine, port) = boot("apple1.test.dump", false);

    // Power-on: the reset vector is fetched from the monitor ROM at $FF00 and
    // the banner comes out over the PIA.
    let banner = exchange(&mut machine, &port, "", 20);
    assert_eq!(banner, b"RSMON\r>", "got {:?}", shown(&banner));

    // Now type at it. Every keystroke is echoed by the monitor, which is what
    // the leading `FF00` is; then the dump, then a fresh prompt.
    let dump = exchange(&mut machine, &port, "FF00\r", 20);
    assert_eq!(
        dump,
        b"FF00\rFF00: D8 A2 FF 9A A9 7F 8D 12\r>",
        "got {:?}",
        shown(&dump)
    );

    // Those eight bytes are the first eight of the ROM, read back through the
    // bus by the guest itself.
    assert_eq!(
        &RSMON[..8],
        &[0xd8, 0xa2, 0xff, 0x9a, 0xa9, 0x7f, 0x8d, 0x12]
    );

    // Return on its own walks forward: the monitor keeps the address.
    let next = exchange(&mut machine, &port, "\r", 20);
    assert_eq!(
        next,
        b"\rFF08: D0 A9 A7 8D 11 D0 8D 13\r>",
        "got {:?}",
        shown(&next)
    );
}

/// A byte typed into RAM is there afterwards, which is the other half of a
/// monitor.
#[test]
fn it_deposits_a_byte_and_reads_it_back() {
    let (mut machine, port) = boot("apple1.test.deposit", false);
    let _banner = exchange(&mut machine, &port, "", 20);

    let deposit = exchange(&mut machine, &port, "0300:AA\r", 20);
    assert_eq!(deposit, b"0300:AA\r>", "got {:?}", shown(&deposit));

    let readback = exchange(&mut machine, &port, "0300\r", 20);
    assert_eq!(
        readback,
        b"0300\r0300: AA 00 00 00 00 00 00 00\r>",
        "got {:?}",
        shown(&readback)
    );

    // And the same byte, seen from outside through the address space — so the
    // deposit really reached the RAM object and not somewhere that only looks
    // like it from the guest's side.
    let space = machine.space("cpubus").expect("cpubus");
    let byte = space
        .read(
            0x0300,
            crate::core::value::Width::U8,
            crate::core::space::MemAttrs::DEBUG,
        )
        .expect("ram answers");
    assert_eq!(byte, 0xaa);
}

/// The display runs at the rate the hardware ran at.
#[test]
fn a_paced_display_takes_about_sixty_characters_a_second() {
    let (mut machine, port) = boot("apple1.test.paced", true);

    // A tenth of a second is six character times, so the seven-byte banner and
    // prompt cannot all have made it out yet.
    let early = exchange(&mut machine, &port, "", 100);
    assert!(
        early.len() <= 7,
        "{} bytes in 100 ms is faster than the hardware: {:?}",
        early.len(),
        shown(&early)
    );

    // Given a second, it finishes.
    let mut total = early;
    total.extend(exchange(&mut machine, &port, "", 1_000));
    assert_eq!(total, b"RSMON\r>", "got {:?}", shown(&total));

    // And a dump of eight bytes — 30 characters — is half a second of work,
    // exactly as it was on the real machine.
    let dump = exchange(&mut machine, &port, "FF00\r", 200);
    assert!(
        dump.len() < 30,
        "{} characters in 200 ms: {:?}",
        dump.len(),
        shown(&dump)
    );
    let mut whole = dump;
    whole.extend(exchange(&mut machine, &port, "", 1_000));
    assert_eq!(
        whole,
        b"FF00\rFF00: D8 A2 FF 9A A9 7F 8D 12\r>",
        "got {:?}",
        shown(&whole)
    );
}

/// The same run twice is the same run — the property every regression test in
/// the project is built on (`ROADMAP.md` §0).
#[test]
fn a_scripted_session_is_deterministic_and_snapshots() {
    let script = "FF00\r0300:5A\r0300\r";

    let (mut first, port_a) = boot("apple1.test.determinism.a", false);
    let a = exchange(&mut first, &port_a, script, 50);

    let (mut second, port_b) = boot("apple1.test.determinism.b", false);
    let b = exchange(&mut second, &port_b, script, 50);

    assert_eq!(a, b, "the same script produced different bytes");
    assert_eq!(
        first.state_hash().expect("hashes"),
        second.state_hash().expect("hashes"),
        "identical output but divergent state"
    );

    // And it round-trips: a save state restored into a fresh machine keeps
    // running identically. The port's queues are host state and are
    // deliberately not in the snapshot, so both sides start from an empty one.
    let saved = first.save().expect("saves");
    let (mut restored, port_c) = boot("apple1.test.determinism.c", false);
    restored.load(&saved).expect("loads");
    assert_eq!(
        restored.state_hash().expect("hashes"),
        first.state_hash().expect("hashes")
    );
    let _ = port_c.drain();

    let after_a = exchange(&mut first, &port_a, "0300\r", 20);
    let after_c = exchange(&mut restored, &port_c, "0300\r", 20);
    assert_eq!(after_a, after_c);
    assert_eq!(
        after_a,
        b"0300\r0300: 5A 00 00 00 00 00 00 00\r>",
        "got {:?}",
        shown(&after_a)
    );
}

/// The clock the machine file derives is the one the Apple 1 ran at.
#[test]
fn the_cpu_runs_at_the_apple_1s_clock() {
    let (mut machine, _port) = boot("apple1.test.clock", false);
    machine
        .run_for(GlobalTime::from_nanos(1_000_000_000))
        .expect("runs");

    let domain = machine
        .device("cpu")
        .and_then(crate::machine::machine::DeviceEntry::domain)
        .expect("the cpu has a clock domain");
    let ticks = machine.clocks().ticks(domain).expect("a tick count");

    // 315/22 MHz / 14 = 45/44 MHz = 1022727.27… Hz. A second of virtual time
    // is that many cycles, to within the one tick the residual carries.
    assert!(
        ticks.abs_diff(1_022_727) <= 1,
        "one virtual second is {ticks} cycles, not 1022727"
    );
}

/// The Woz Monitor, if the caller has fetched one.
///
/// Gated on an environment variable like every other corpus (`CLAUDE.md`):
/// Wozmon's copyright status is unclear, so rsemu never ships it.
/// `scripts/fetch-testdata.sh` will put one in `testdata/apple1/` for you.
/// Without the variable this passes trivially, so `cargo test` offline stays
/// green.
#[cfg(feature = "std")]
#[test]
fn the_woz_monitor_boots_and_answers_a_memory_dump() {
    let Ok(path) = std::env::var("RSEMU_APPLE1_ROM") else {
        std::println!("SKIP: set RSEMU_APPLE1_ROM to a 256-byte Apple 1 monitor image");
        return;
    };
    let image = std::fs::read(&path).expect("RSEMU_APPLE1_ROM is readable");

    let port = ports::open("apple1.test.wozmon");
    port.clear();
    let mut options = catalog::build_options().expect("classes");
    options.realize.media.insert("rom", image.as_slice());
    let options = options
        .with_param("console", "apple1.test.wozmon")
        .with_param("pace", "false");
    let registry = catalog::registry().expect("classes");
    let mut machine = match build(
        "apple1.machine",
        catalog::APPLE1.source,
        &registry,
        &options,
    ) {
        Ok(m) => m,
        Err(e) => panic!("{path}: {e}"),
    };

    // Wozmon greets with a backslash and a carriage return, then waits.
    let banner = exchange(&mut machine, &port, "", 50);
    std::println!("apple1 + {path}: banner {:?}", shown(&banner));
    assert!(
        banner.contains(&b'\\'),
        "no prompt from {path}: {:?}",
        shown(&banner)
    );

    // `FF00.FF07` is Wozmon's range examine.
    let dump = exchange(&mut machine, &port, "FF00.FF07\r", 50);
    let text = shown(&dump);
    std::println!("apple1 + {path}: dump {text:?}");
    assert!(text.contains("FF00:"), "no dump from {path}: {text:?}");
    for byte in &image[..8] {
        assert!(
            text.contains(&alloc::format!("{byte:02X}")),
            "byte {byte:02X} missing from {text:?}"
        );
    }
}

/// Print a whole session, for a human reading `--nocapture`.
///
/// Asserts nothing the tests above do not; it exists so that "does this
/// actually work?" has an answer you can read rather than infer.
#[cfg(feature = "std")]
#[test]
fn a_session_transcript() {
    let (mut machine, port) = boot("apple1.test.transcript", false);
    let mut transcript = exchange(&mut machine, &port, "", 20);
    for line in ["FF00\r", "\r", "0300:C0\r", "0300\r"] {
        transcript.extend(exchange(&mut machine, &port, line, 20));
    }
    std::println!("--- rsemu run apple1 ---\n{}", shown(&transcript));
    assert!(transcript.starts_with(b"RSMON"));
}
