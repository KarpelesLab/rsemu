//! The whole board, driven from a script and checked against its output.
//!
//! Not a device test — [`acia`](super::acia) and [`via`](super::via) have
//! those. This is the machine: a `.machine` file, a real 6502 fetching a reset
//! vector out of a 32 KiB EEPROM, the NAND gate's memory map, two oscillators,
//! the scheduler, and a character port with a test on the far end of it typing
//! and reading back. If any part of that is wrong, the bytes coming out are
//! wrong and these fail.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::clock::GlobalTime;
use crate::host::chardev::{CharPort, ports};
use crate::machine::{Machine, build, catalog};

use super::monitor::RSMON_IMAGE;
use super::wozmon::WOZMON_IMAGE;

/// Build the shipped `beneater-6502` machine on its own private port.
///
/// `paced` picks the real 19200-baud rate or the flat-out one — the property
/// that exists so a test does not have to spend virtual seconds watching a
/// memory dump arrive one character at a time.
fn boot(port_name: &str, paced: bool, image: &[u8]) -> (Machine, Arc<CharPort>) {
    let port = ports::open(port_name);
    // A name can be reused across runs of the same test binary.
    port.clear();

    let mut options = catalog::build_options().expect("this build has the classes");
    options.realize.media.insert("rom", image);
    let options = options
        .with_param("console", port_name)
        .with_param("pace", if paced { "true" } else { "false" });

    let registry = catalog::registry().expect("this build has the classes");
    let machine = match build(
        "beneater-6502.machine",
        catalog::BENEATER_6502.source,
        &registry,
        &options,
    ) {
        Ok(m) => m,
        Err(e) => panic!("beneater-6502.machine: {e}"),
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

/// Output as a human reads it, with bare CRs made visible.
fn shown(bytes: &[u8]) -> String {
    let mut out = String::new();
    for &b in bytes {
        match b {
            b'\r' => out.push('\n'),
            b'\n' => {}
            0x20..=0x7e => out.push(b as char),
            other => out.push_str(&alloc::format!("\\x{other:02x}")),
        }
    }
    out
}

/// One CPU-visible byte, read the way a debugger would.
fn peek(machine: &Machine, addr: u64) -> u8 {
    machine
        .space("cpubus")
        .expect("cpubus")
        .read(
            addr,
            crate::core::value::Width::U8,
            crate::core::space::MemAttrs::DEBUG,
        )
        .expect("open bus answers everything") as u8
}

// ---------------------------------------------------------------------------
// The map the NAND gate makes
// ---------------------------------------------------------------------------

/// Each chip answers everywhere its chip select is asserted, and nowhere else.
///
/// This is the test that fails if the decoding in the machine file drifts from
/// the schematic, and it checks the *mirroring* rather than one address per
/// chip — because the mirroring is the part that is easy to get wrong and
/// invisible until a program uses `$5004` or `$6010`.
#[test]
fn the_address_decoding_is_the_boards() {
    let (mut machine, _port) = boot("beneater.test.map", false, RSMON_IMAGE);
    // Long enough for the monitor's reset path to program the ACIA, which is
    // what makes its registers say something recognisable below.
    machine
        .run_for(GlobalTime::from_nanos(1_000_000))
        .expect("runs");
    let space = machine.space("cpubus").expect("cpubus");

    // RAM: 16 KiB, because A14 is the SRAM's output enable.
    space
        .write(
            0x3fff,
            crate::core::value::Width::U8,
            0x5a,
            crate::core::space::MemAttrs::DEFAULT,
        )
        .expect("the top of RAM is writable");
    assert_eq!(peek(&machine, 0x3fff), 0x5a);

    // The ACIA's four registers, over and over from $4000 to $5FFF. Its control
    // register is $1F because RSMON put it there, and $5003 and $4003 and
    // $5FFF are all the same register.
    assert_eq!(peek(&machine, 0x5003), 0x1f, "the control register");
    assert_eq!(
        peek(&machine, 0x4003),
        0x1f,
        "and at the bottom of the window"
    );
    assert_eq!(peek(&machine, 0x5fff), 0x1f, "and at the top of it");
    assert_eq!(peek(&machine, 0x5002), 0x0b, "the command register");

    // The VIA's sixteen, over and over from $6000 to $7FFF. Reading IER gives
    // bit 7 set and nothing else out of reset, which is a value no other chip
    // here would answer with.
    assert_eq!(peek(&machine, 0x600e), 0x80, "the VIA's IER");
    assert_eq!(peek(&machine, 0x601e), 0x80, "sixteen bytes up");
    assert_eq!(peek(&machine, 0x7ffe), 0x80, "and at the top of the window");

    // The EEPROM, all 32 KiB of it, with the vectors at the very top.
    assert_eq!(peek(&machine, 0x8000), RSMON_IMAGE[0]);
    assert_eq!(peek(&machine, 0xfe00), RSMON_IMAGE[0x7e00]);
    assert_eq!(peek(&machine, 0xfffc), 0x00);
    assert_eq!(peek(&machine, 0xfffd), 0x80, "RESET points at $8000");
}

/// The clocks the machine file derives are the ones on the breadboard.
#[test]
fn the_cpu_runs_at_one_megahertz_exactly() {
    let (mut machine, _port) = boot("beneater.test.clock", false, RSMON_IMAGE);
    machine
        .run_for(GlobalTime::from_nanos(1_000_000_000))
        .expect("runs");

    let cpu = machine
        .device("cpu")
        .and_then(crate::machine::machine::DeviceEntry::domain)
        .expect("the cpu has a clock domain");
    let ticks = machine.clocks().ticks(cpu).expect("a tick count");
    // A whole number, because the can really is 1.000 MHz — this is the board
    // where the rational frequency literal elsewhere is not needed, and saying
    // so is the point of the comment in the machine file.
    assert_eq!(ticks, 1_000_000, "one virtual second is not 10^6 cycles");

    // The VIA counts the same clock, so its timers are exact against the CPU.
    let via = machine
        .device("via")
        .and_then(crate::machine::machine::DeviceEntry::domain)
        .expect("the via has a clock domain");
    assert_eq!(machine.clocks().ticks(via).expect("ticks"), ticks);

    // The ACIA is on its own crystal and its own tree: 1920 characters a
    // second, which is 19200 baud 8-N-1.
    let acia = machine
        .device("acia")
        .and_then(crate::machine::machine::DeviceEntry::domain)
        .expect("the acia has a clock domain");
    let chars = machine.clocks().ticks(acia).expect("ticks");
    assert!(
        chars.abs_diff(1920) <= 1,
        "one virtual second is {chars} character times, not 1920"
    );
}

// ---------------------------------------------------------------------------
// RSMON/serial — the board's own monitor
// ---------------------------------------------------------------------------

/// The machine boots, prints its banner and a prompt, and answers a memory
/// dump — the whole point of the exercise, in one test.
#[test]
fn it_boots_prints_a_prompt_and_dumps_memory_on_request() {
    let (mut machine, port) = boot("beneater.test.dump", false, RSMON_IMAGE);

    // Power-on: the reset vector is fetched from $FFFC, which is the top of the
    // EEPROM, and the banner comes out over the ACIA.
    let banner = exchange(&mut machine, &port, "", 20);
    assert_eq!(banner, b"RSMON\r\n>", "got {:?}", shown(&banner));

    // Now type at it. Every keystroke is echoed, which is what the leading
    // `8000` is; then the dump, then a fresh prompt.
    let dump = exchange(&mut machine, &port, "8000\r", 20);
    assert_eq!(
        dump,
        b"8000\r\n8000: D8 A2 FF 9A A9 1F 8D 03\r\n>",
        "got {:?}",
        shown(&dump)
    );

    // Those eight bytes are the first eight of the ROM, read back through the
    // bus by the guest itself.
    assert_eq!(
        &RSMON_IMAGE[..8],
        &[0xd8, 0xa2, 0xff, 0x9a, 0xa9, 0x1f, 0x8d, 0x03]
    );

    // Return on its own walks forward: the monitor keeps the address.
    let next = exchange(&mut machine, &port, "\r", 20);
    assert_eq!(
        next,
        b"\r\n8008: 50 A9 0B 8D 02 50 A2 00\r\n>",
        "got {:?}",
        shown(&next)
    );
}

/// A byte typed into RAM is there afterwards, which is the other half of a
/// monitor.
#[test]
fn it_deposits_a_byte_and_reads_it_back() {
    let (mut machine, port) = boot("beneater.test.deposit", false, RSMON_IMAGE);
    let _banner = exchange(&mut machine, &port, "", 20);

    let deposit = exchange(&mut machine, &port, "0200:AA\r", 20);
    assert_eq!(deposit, b"0200:AA\r\n>", "got {:?}", shown(&deposit));

    let readback = exchange(&mut machine, &port, "0200\r", 20);
    assert_eq!(
        readback,
        b"0200\r\n0200: AA 00 00 00 00 00 00 00\r\n>",
        "got {:?}",
        shown(&readback)
    );

    // And the same byte seen from outside, so the deposit really reached the
    // SRAM and not somewhere that only looks like it from the guest's side.
    assert_eq!(peek(&machine, 0x0200), 0xaa);
}

/// The serial line runs at the rate the board runs at.
#[test]
fn a_paced_port_sends_about_nineteen_thousand_bits_a_second() {
    let (mut machine, port) = boot("beneater.test.paced", true, RSMON_IMAGE);

    // 1920 characters a second is a character every 521 µs, so the eight-byte
    // banner and prompt cannot all have made it out in 3 ms.
    let early = exchange(&mut machine, &port, "", 3);
    assert!(
        early.len() <= 8,
        "{} bytes in 3 ms is faster than 19200 baud: {:?}",
        early.len(),
        shown(&early)
    );

    // Given fifty milliseconds, it finishes.
    let mut total = early;
    total.extend(exchange(&mut machine, &port, "", 50));
    assert_eq!(total, b"RSMON\r\n>", "got {:?}", shown(&total));
}

/// The same run twice is the same run — the property every regression test in
/// the project is built on (`ROADMAP.md` §0).
#[test]
fn a_scripted_session_is_deterministic_and_snapshots() {
    let script = "8000\r0200:5A\r0200\r";

    let (mut first, port_a) = boot("beneater.test.determinism.a", false, RSMON_IMAGE);
    let a = exchange(&mut first, &port_a, script, 50);

    let (mut second, port_b) = boot("beneater.test.determinism.b", false, RSMON_IMAGE);
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
    let (mut restored, port_c) = boot("beneater.test.determinism.c", false, RSMON_IMAGE);
    restored.load(&saved).expect("loads");
    assert_eq!(
        restored.state_hash().expect("hashes"),
        first.state_hash().expect("hashes")
    );
    let _ = port_c.drain();

    let after_a = exchange(&mut first, &port_a, "0200\r", 20);
    let after_c = exchange(&mut restored, &port_c, "0200\r", 20);
    assert_eq!(after_a, after_c);
    assert_eq!(
        after_a,
        b"0200\r\n0200: 5A 00 00 00 00 00 00 00\r\n>",
        "got {:?}",
        shown(&after_a)
    );
}

/// A VIA timer loaded through the bus expires on the φ2 cycle it should.
///
/// Driven from outside rather than by a guest program, because what is under
/// test is the *machine*: the VIA's window at `$6000`, its clock domain, and
/// the sync-on-access catch-up that has to happen before an `LDA $600D` is
/// answered. A stale VIA reports the flag late; an unclocked one never
/// reports it at all.
#[test]
fn a_via_timer_expires_on_the_cycle_it_should() {
    use crate::core::space::MemAttrs;
    use crate::core::value::Width;

    let (mut machine, _port) = boot("beneater.test.timer", false, RSMON_IMAGE);
    let poke = |machine: &Machine, addr: u64, value: u64| {
        machine
            .space("cpubus")
            .expect("cpubus")
            .write(addr, Width::U8, value, MemAttrs::DEFAULT)
            .expect("the VIA answers");
    };

    // Let the machine settle, then load T1 with 999: it must time out 1000 φ2
    // cycles later, which at 1 MHz is exactly one millisecond.
    machine
        .run_for(GlobalTime::from_nanos(1_000_000))
        .expect("runs");
    poke(&machine, 0x6004, 0xe7); // T1 low latch
    poke(&machine, 0x6005, 0x03); // T1C-H: transfer and count down
    assert_eq!(peek(&machine, 0x600d) & 0x40, 0, "not yet");

    // Half a millisecond in, still counting.
    machine
        .run_for(GlobalTime::from_nanos(500_000))
        .expect("runs");
    assert_eq!(peek(&machine, 0x600d) & 0x40, 0, "still counting");

    // And past a millisecond, the flag is up. Reading `$6004` clears it, which
    // is the data sheet's own rule and the last link in the chain.
    machine
        .run_for(GlobalTime::from_nanos(600_000))
        .expect("runs");
    assert_eq!(peek(&machine, 0x600d) & 0x40, 0x40, "T1 timed out");
    machine
        .space("cpubus")
        .expect("cpubus")
        .read(0x6004, Width::U8, MemAttrs::DEFAULT)
        .expect("T1C-L");
    assert_eq!(peek(&machine, 0x600d) & 0x40, 0, "and the read cleared it");
}

/// Print a whole session, for a human reading `--nocapture`.
///
/// Asserts nothing the tests above do not; it exists so that "does this
/// actually work?" has an answer you can read rather than infer.
#[cfg(feature = "std")]
#[test]
fn a_session_transcript() {
    let (mut machine, port) = boot("beneater.test.transcript", false, RSMON_IMAGE);
    let mut transcript = exchange(&mut machine, &port, "", 20);
    for line in ["8000\r", "\r", "0200:C0\r", "0200\r"] {
        transcript.extend(exchange(&mut machine, &port, line, 20));
    }
    std::println!("--- rsemu run beneater-6502 ---\n{}", shown(&transcript));
    assert!(transcript.starts_with(b"RSMON"));
}

// ---------------------------------------------------------------------------
// The Woz Monitor of 1976
// ---------------------------------------------------------------------------

/// Woz's own monitor, on Ben Eater's board, in rsemu.
///
/// The image is the 1976 object code with three blocks re-plumbed for the ACIA
/// ([`wozmon`](super::wozmon)); everything this test asserts on is behaviour
/// nobody at rsemu designed. The prompt is a backslash, `AAAA.BBBB` examines a
/// range eight bytes to a line, and the line endings are bare carriage returns
/// because that is what a 1976 terminal wanted.
#[test]
fn the_woz_monitor_boots_and_examines_a_range() {
    let (mut machine, port) = boot("beneater.test.wozmon", false, WOZMON_IMAGE);

    // Wozmon greets with `\` and a CR and then waits. The `\` is `$DC` in the
    // listing — a backslash with bit 7 set, which the adaptation masks off on
    // the way out.
    let banner = exchange(&mut machine, &port, "", 20);
    assert_eq!(banner, b"\\\r", "got {:?}", shown(&banner));

    // A range examine of the monitor's own first page, which is the strongest
    // thing this can assert: the bytes it prints are the bytes the *Apple-1
    // Operation Manual* prints, fetched by Woz's code through our bus.
    let dump = exchange(&mut machine, &port, "FF00.FF0F\r", 20);
    assert_eq!(
        dump,
        // The echo of the line, then a CR, then one CR-prefixed row per eight
        // bytes. There is no prompt after it: Wozmon only prints `\` when it
        // takes an escape, and a completed line just leaves you at a new one.
        b"FF00.FF0F\r\rFF00: D8 58 A0 7F A9 1F 8D 03\rFF08: 50 A9 0B 8D 02 50 EA C9\r",
        "got {:?}",
        shown(&dump)
    );

    // Deposit and read back, which is the other half of it. Wozmon's syntax is
    // `AAAA: xx yy` and it takes several bytes on one line.
    //
    // `$0300`, not `$0200`: the monitor's own line buffer is `$0200-$027F`, as
    // the manual warns on the page before the listing. Depositing there works
    // and then reads back as whatever you just typed, which is a confusing way
    // to rediscover a documented fact.
    //
    // The `0300: 00` in the reply is not an echo: `0300` is parsed while the
    // monitor is still in examine mode, so it examines the byte *before* the
    // `:` switches to store mode and the bytes after it go in. Woz's own code,
    // and a nice illustration of how little of it there is.
    let deposit = exchange(&mut machine, &port, "0300: AA BB CC\r", 20);
    assert_eq!(
        deposit,
        b"0300: AA BB CC\r\r0300: 00\r",
        "got {:?}",
        shown(&deposit)
    );
    assert_eq!(peek(&machine, 0x0300), 0xaa);
    assert_eq!(peek(&machine, 0x0301), 0xbb);
    assert_eq!(peek(&machine, 0x0302), 0xcc);

    let readback = exchange(&mut machine, &port, "0300.0302\r", 20);
    assert_eq!(
        readback,
        b"0300.0302\r\r0300: AA BB CC\r",
        "got {:?}",
        shown(&readback)
    );
}

/// A Woz Monitor session, for a human reading `--nocapture`.
#[cfg(feature = "std")]
#[test]
fn a_woz_monitor_transcript() {
    let (mut machine, port) = boot("beneater.test.wozmon.transcript", false, WOZMON_IMAGE);
    let mut transcript = exchange(&mut machine, &port, "", 20);
    for line in ["FF00.FF0F\r", "0300: AA BB CC\r", "0300.0302\r"] {
        transcript.extend(exchange(&mut machine, &port, line, 20));
    }
    std::println!(
        "--- rsemu run beneater-6502 --monitor wozmon ---\n{}",
        shown(&transcript)
    );
    assert!(transcript.starts_with(b"\\"));
}

/// Ben Eater's own `wozmon.bin`, if the caller has fetched one.
///
/// **This is a fixture, not a vendored file.** His image is CC-BY (all the code
/// in his videos is; see <https://eater.net/6502>) and rsemu could ship it with
/// attribution, but there is no reason to: rsemu's own transcription of the
/// public-domain 1976 listing is in [`wozmon`](super::wozmon) and is what the
/// tests above run. This one exists to check rsemu against a binary that
/// *predates* rsemu and was assembled by somebody else, which is a different
/// and better kind of evidence.
///
/// It needs a **65C02**: at `$FFF5` his image has `DEC A` (`$3A`) inside a
/// `LDA #$FF / DEC A / BNE -3` transmit delay, an instruction the NMOS part
/// decodes as an undocumented `NOP` — so on the core rsemu has today that loop
/// never ends. Until `cpu.mos6502` grows a CMOS variant and the machine file
/// names it, this reports the hang rather than pretending it passed.
///
/// His image is at <https://eater.net/downloads/wozmon.bin>; put it under
/// `testdata/` — which is git-ignored, like every other corpus — and point
/// `RSEMU_BENEATER_ROM` at it. Without the variable this passes trivially, so
/// `cargo test` offline stays green.
#[cfg(feature = "std")]
#[test]
fn ben_eaters_own_image_boots() {
    let Ok(path) = std::env::var("RSEMU_BENEATER_ROM") else {
        std::println!("SKIP: set RSEMU_BENEATER_ROM to a 32 KiB image for this board");
        return;
    };
    let image = std::fs::read(&path).expect("RSEMU_BENEATER_ROM is readable");
    // `LDA #$FF / DEC A`, the transmit delay. On an NMOS core `$3A` is an
    // undocumented `NOP`, so `A` never changes, `Z` never sets, and the `BNE`
    // after it loops forever: the image sends exactly one character and stops.
    // Finding the sequence rather than one byte at one address keeps this a
    // statement about the program instead of about a build of it.
    let needs_cmos = image.windows(3).any(|w| w == [0xa9, 0xff, 0x3a]);
    let (mut machine, port) = boot("beneater.test.foreign", false, &image);

    let banner = exchange(&mut machine, &port, "", 50);
    std::println!("beneater-6502 + {path}: banner {:?}", shown(&banner));
    let dump = exchange(&mut machine, &port, "FF00.FF07\r", 50);
    let text = shown(&dump);
    std::println!("beneater-6502 + {path}: dump {text:?}");

    if dump.is_empty() && needs_cmos {
        std::println!(
            "SKIP: {path} has `LDA #$FF / DEC A` in it and needs a 65C02. \
             `cpu.mos6502` has no CMOS variant yet, so that delay loop cannot \
             terminate and the image stops after its first character \
             ({:?} came out). See machines/beneater-6502.machine.",
            shown(&banner)
        );
        return;
    }
    assert!(
        !banner.is_empty(),
        "no output at all from {path}: {:?}",
        shown(&banner)
    );
    assert!(text.contains("FF00:"), "no dump from {path}: {text:?}");
    for byte in &image[0x7f00..0x7f08] {
        assert!(
            text.contains(&alloc::format!("{byte:02X}")),
            "byte {byte:02X} missing from {text:?}"
        );
    }
}
