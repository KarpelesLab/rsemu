//! The `rsemu` command-line tool.
//!
//! The subcommand surface is fixed by `ROADMAP.md` §2 so that it does not drift
//! as components land. Commands whose machinery does not exist yet say exactly
//! that and exit non-zero, rather than pretending to work.
//!
//! This is the one place in the tree that opens a file. Everything below it is
//! `no_std`: a machine description arrives as text and a ROM image arrives as
//! bytes bound to a named media slot, so the emulation core never learns what a
//! path is (`CLAUDE.md`, and [`MediaTable`](rsemu::machine::MediaTable)).

use std::path::Path;
use std::process::ExitCode;
use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::clock::GlobalTime;
use rsemu::host::chardev::{CharDevice, CharPort, ports};
use rsemu::host::terminal::Terminal;
use rsemu::machine::{Machine, catalog};

const USAGE: &str = "\
rsemu — a multiplatform emulator built bottom-up on a generic framework

USAGE:
    rsemu <COMMAND> [OPTIONS]

COMMANDS:
    run <machine>       Run a machine description
    debug <machine>     Run it under a debugger, stopped, on :1234
    machines            List machines this build can emulate
    devices             List registered device classes
    describe <class>    Show a device class: properties, defaults, buses
    convert <machine>   Convert a machine file between its text and JSON forms

RUN OPTIONS:
    <machine>           A path to a .machine file, or a name from `rsemu machines`
    --cart <file>       Bind the `cart` media slot (a NES cartridge)
    --rom <file>        Bind the `rom` media slot
    --monitor <name>    Bind the `rom` slot to one of rsemu's own monitor
                        images instead of a file: `rsmon` (the default, ours,
                        MIT) or `wozmon` (the 1976 Woz Monitor, public domain)
    --disk <file>       Bind the `disk` media slot
    --media <n>=<file>  Bind any media slot by name
    -p <name>=<value>   Override a `param` declared in the machine file
    --for <duration>    How much virtual time to run, as `1s`, `500ms`, `2m`
                        (default 1s, or forever with a console attached)
    --console <name>    Attach this terminal to a named character port. A
                        machine that opens exactly one is picked up on its own,
                        so `rsemu run apple1` is interactive already
    --headless          Do not attach a terminal, whatever the machine opened
    --gdb <addr>        Listen for GDB on <addr> and hold the machine stopped
                        until it attaches. `1234`, `:1234` and `host:1234` all
                        work; a bare port binds the loopback interface only,
                        because the far end can read and write all of guest
                        memory. `rsemu debug` implies `--gdb :1234`
    -q, --quiet         Only print the summary

OPTIONS:
    -h, --help          Print this help
    -V, --version       Print version and build configuration
";

fn main() -> ExitCode {
    // Hand-rolled argument parsing: the dependency policy has no room for a
    // CLI crate, and the surface is small enough that it does not need one.
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(first) = args.first().map(String::as_str) else {
        print!("{USAGE}");
        return ExitCode::from(2);
    };

    match first {
        "-h" | "--help" | "help" => {
            print!("{USAGE}");
            ExitCode::SUCCESS
        }
        "-V" | "--version" | "version" => {
            println!("{}", rsemu::build_info());
            ExitCode::SUCCESS
        }
        "machines" => machines(),
        "devices" => devices(),
        "describe" => describe(args.get(1).map(String::as_str)),
        "run" => run(&args[1..]),
        #[cfg(feature = "gdb")]
        "debug" => debug(&args[1..]),
        "convert" => {
            eprintln!(
                "rsemu: {}",
                rsemu::Error::Unimplemented("the JSON projection (ROADMAP.md §5)")
            );
            ExitCode::from(2)
        }
        other => {
            eprintln!("rsemu: unknown command `{other}`\n");
            eprint!("{USAGE}");
            ExitCode::from(2)
        }
    }
}

// ---------------------------------------------------------------------------
// introspection
// ---------------------------------------------------------------------------

/// `rsemu machines` — the shipped catalog, filtered to what this build has.
fn machines() -> ExitCode {
    let machines = catalog::machines();
    if machines.is_empty() {
        // A machine is a feature set, so an empty catalog is a correct answer
        // about this build rather than a failure.
        println!("no machines in this build; rebuild with a `machine-*` feature");
        return ExitCode::SUCCESS;
    }
    for entry in machines {
        println!("{:<12} {}", entry.name, entry.summary);
        if !entry.media.is_empty() {
            let slots: Vec<String> = entry
                .media
                .iter()
                .map(|s| format!("--{s} <file>"))
                .collect();
            // "media", not "needs": a slot with no default is an error naming
            // itself when the machine is built, and `apple1` binds its own
            // monitor ROM when nothing else does.
            println!("{:<12} media {}", "", slots.join(", "));
        }
    }
    ExitCode::SUCCESS
}

/// `rsemu devices` — every class the registry can construct.
fn devices() -> ExitCode {
    let registry = match catalog::registry() {
        Ok(r) => r,
        Err(e) => return fail(&e),
    };
    if registry.is_empty() {
        println!("no device classes in this build");
        return ExitCode::SUCCESS;
    }
    for class in registry.classes() {
        println!("{:<16} {}", class.name, class.summary);
    }
    ExitCode::SUCCESS
}

/// `rsemu describe <class>` — one class, its version and its properties.
fn describe(class: Option<&str>) -> ExitCode {
    let Some(name) = class else {
        eprintln!("rsemu: describe needs a class name; `rsemu devices` lists them");
        return ExitCode::from(2);
    };
    let registry = match catalog::registry() {
        Ok(r) => r,
        Err(e) => return fail(&e),
    };
    let Some(class) = registry.get(name) else {
        // The registry composes the "is its feature enabled?" message and the
        // near-miss suggestion; reproducing them here would be a second copy.
        let e = registry
            .create(name, &rsemu::core::props::Props::new())
            .expect_err("a class that is not there cannot construct");
        return fail(&e);
    };
    println!("{} (v{})", class.name, class.version);
    println!("  {}", class.summary);
    if class.properties.is_empty() {
        println!("  no properties");
        return ExitCode::SUCCESS;
    }
    println!("  properties:");
    for p in class.properties {
        println!(
            "    {:<10} {:<10} {:<9} {}",
            p.name,
            p.kind.as_str(),
            if p.required { "required" } else { "optional" },
            p.summary
        );
    }
    ExitCode::SUCCESS
}

// ---------------------------------------------------------------------------
// run
// ---------------------------------------------------------------------------

/// Everything `rsemu run` was told.
struct RunArgs {
    machine: String,
    /// Media slot to file path, in the order they were given.
    media: Vec<(String, String)>,
    /// A built-in monitor image for the `rom` slot, if the user named one.
    monitor: Option<String>,
    params: Vec<(String, String)>,
    span: GlobalTime,
    /// Whether `--for` was given. Without it an interactive machine runs until
    /// the user stops it, and a headless one runs for a second.
    span_given: bool,
    /// The character port to attach this terminal to, if the user named one.
    console: Option<String>,
    /// Whether the user asked for no terminal at all.
    headless: bool,
    quiet: bool,
    /// Where to listen for a debugger, if `--gdb` was given.
    #[cfg(feature = "gdb")]
    gdb: Option<String>,
}

fn run(args: &[String]) -> ExitCode {
    let parsed = match parse_run(args) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("rsemu: {e}");
            return ExitCode::from(2);
        }
    };

    // Read every image before building anything, so a typo'd path fails before
    // a machine is half assembled.
    let mut images: Vec<(String, Vec<u8>)> = Vec::new();
    for (slot, path) in &parsed.media {
        match std::fs::read(path) {
            Ok(bytes) => images.push((slot.clone(), bytes)),
            Err(e) => {
                eprintln!("rsemu: cannot read {path}: {e}");
                return ExitCode::FAILURE;
            }
        }
    }

    // A machine that wants a `rom` and was given no file gets a built-in one,
    // so `rsemu run apple1` and `rsemu run beneater-6502` work with no
    // arguments and no image of unclear provenance. Media nothing asked for is
    // ignored, so a machine with no `rom` slot is unaffected.
    if !images.iter().any(|(slot, _)| slot == "rom") {
        match builtin_rom(parsed.monitor.as_deref(), &parsed.machine) {
            Ok(Some(image)) => images.push((String::from("rom"), image)),
            Ok(None) => {}
            Err(e) => {
                eprintln!("rsemu: {e}");
                return ExitCode::from(2);
            }
        }
    }

    // Same for a `firmware` slot. `spi-panel` is a demonstration board rather
    // than a product, so `rsemu run spi-panel` with no arguments draws its own
    // test picture instead of executing an empty ROM.
    if !images.iter().any(|(slot, _)| slot == "firmware")
        && let Some(image) = builtin_firmware(&parsed.machine)
    {
        images.push((String::from("firmware"), image));
    }

    // A path wins over a catalog name, so a user editing a copy of a shipped
    // file gets their copy.
    let (name, source) = match load_description(&parsed.machine) {
        Ok(pair) => pair,
        Err(e) => {
            eprintln!("rsemu: {e}");
            return ExitCode::from(2);
        }
    };

    let mut options = match catalog::build_options() {
        Ok(o) => o,
        Err(e) => return fail(&e),
    };
    for (slot, bytes) in &images {
        options
            .realize
            .media
            .insert(slot.as_str(), bytes.as_slice());
    }
    for (key, value) in &parsed.params {
        options = options.with_param(key.clone(), value.clone());
    }

    let registry = match catalog::registry() {
        Ok(r) => r,
        Err(e) => return fail(&e),
    };
    let mut machine = match rsemu::machine::build(&name, &source, &registry, &options) {
        Ok(m) => m,
        // Front-end failures already carry `file:line:col` and a caret; realize
        // failures name the instance. Either way `{e}` is the whole report.
        Err(e) => return fail(&e),
    };

    if !parsed.quiet {
        describe_machine(&machine);
    }

    // A debugger, if one was asked for, owns when the machine advances — so it
    // is checked before the console loop, which would otherwise own that.
    #[cfg(feature = "gdb")]
    if let Some(addr) = parsed.gdb.clone() {
        let port = match console_port(&parsed) {
            Ok(port) => port,
            Err(e) => {
                eprintln!("rsemu: {e}");
                return ExitCode::from(2);
            }
        };
        return debug_session(&mut machine, &addr, port.as_ref(), &parsed);
    }

    // A machine that opened a character port has a console; attach this
    // terminal to it and hand the keyboard over.
    match console_port(&parsed) {
        Err(e) => {
            eprintln!("rsemu: {e}");
            return ExitCode::from(2);
        }
        Ok(Some(port)) => return interact(&mut machine, &port, &parsed),
        Ok(None) => {}
    }

    if let Err(e) = machine.run_for(parsed.span) {
        eprintln!("rsemu: {e}");
        summarise(&machine);
        return ExitCode::FAILURE;
    }
    summarise(&machine);
    ExitCode::SUCCESS
}

/// `rsemu debug <machine>`: `run` with a debugger attached (`ROADMAP.md` §2).
///
/// The only difference from `run --gdb` is the default: `debug` with no
/// `--gdb` listens on `:1234`, which is the port every GDB user already has in
/// their fingers.
#[cfg(feature = "gdb")]
fn debug(args: &[String]) -> ExitCode {
    let mut args = args.to_vec();
    if !args.iter().any(|a| a == "--gdb") {
        args.push(String::from("--gdb"));
        args.push(String::from(":1234"));
    }
    run(&args)
}

/// Run a machine with the gdbstub in charge of when it advances.
///
/// The machine is held stopped until a debugger attaches, so `target remote`
/// lands on the reset vector rather than wherever a free-running guest had got
/// to. A console, if the machine opened one, is pumped between session turns —
/// so a guest can be typed at and stepped through at the same time.
#[cfg(feature = "gdb")]
fn debug_session(
    machine: &mut Machine,
    addr: &str,
    port: Option<&Arc<CharPort>>,
    args: &RunArgs,
) -> ExitCode {
    use rsemu::host::gdb::{ExitReason, GdbServer};

    let mut server = match GdbServer::bind(addr) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("rsemu: cannot listen on `{addr}`: {e}");
            return ExitCode::FAILURE;
        }
    };
    if !args.quiet {
        match server.local_addr() {
            Ok(bound) => eprintln!("  gdbstub listening on {bound} — the machine is stopped"),
            Err(_) => eprintln!("  gdbstub listening — the machine is stopped"),
        }
        eprintln!("  attach with: gdb -ex 'target remote {addr}'");
        // Which devices became GDB threads, and — the hour-saving part — which
        // of them upstream GDB has no architecture for. A target description
        // tells GDB what the registers *are*; it still needs a gdbarch to know
        // what the machine *is*, and for a 6502 it has none. It says
        // "Architecture rejected target-supplied description", falls back to its
        // own register layout, and `target remote` then fails outright. That is
        // a property of GDB, not of this stub, and reading it here beats
        // discovering it from that message.
        let mut thread = 0u32;
        for entry in machine.devices() {
            let Some(arch) = rsemu::host::gdb::arch::for_class(entry.class().name) else {
                continue;
            };
            thread += 1;
            match arch.architecture {
                Some(name) => eprintln!(
                    "  thread {thread}: {} ({}), gdb architecture `{name}`",
                    entry.path(),
                    entry.class().name
                ),
                None => eprintln!(
                    "  thread {thread}: {} ({}) — upstream gdb has no architecture for\n    \
                     this core, so it rejects the target description and `target remote`\n    \
                     fails. The protocol is served in full to any client that reads the\n    \
                     description rather than insisting on a gdbarch.",
                    entry.path(),
                    entry.class().name
                ),
            }
        }
        eprintln!();
    }

    let terminal = port.map(|_| Terminal::open());
    let status = match rsemu::host::gdb::serve(machine, &mut server, |_| {
        if let (Some(term), Some(port)) = (terminal.as_ref(), port) {
            term.pump(port);
            if term.interrupted() {
                return false;
            }
        }
        true
    }) {
        Ok(ExitReason::Killed | ExitReason::Stopped) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("rsemu: {e}");
            ExitCode::FAILURE
        }
    };
    if let Some(term) = terminal {
        term.flush();
    }
    if !args.quiet {
        println!();
        summarise(machine);
    }
    status
}

/// Which character port this terminal should attach to, if any.
///
/// The machine has already been built, so every port its devices asked for is
/// open. One is unambiguous; several need `--console` to choose between them,
/// because guessing would put the keyboard on the wrong machine.
fn console_port(args: &RunArgs) -> Result<Option<Arc<CharPort>>, String> {
    if args.headless {
        return Ok(None);
    }
    if let Some(name) = &args.console {
        return ports::get(name).map(Some).ok_or_else(|| {
            format!(
                "no character port named `{name}`; this machine opened {}",
                list(&ports::names())
            )
        });
    }
    let names = ports::names();
    match names.len() {
        0 => Ok(None),
        1 => Ok(ports::get(&names[0])),
        _ => Err(format!(
            "this machine has {} character ports ({}); pick one with --console, or --headless",
            names.len(),
            list(&names)
        )),
    }
}

/// `a`, `b` and `c`, or "none".
fn list(names: &[String]) -> String {
    if names.is_empty() {
        return String::from("none");
    }
    names
        .iter()
        .map(|n| format!("`{n}`"))
        .collect::<Vec<String>>()
        .join(", ")
}

/// How much virtual time to advance between two visits to the terminal.
///
/// Ten milliseconds: short enough that a keystroke is picked up before the eye
/// notices, long enough that the scheduler is not the bottleneck.
const SLICE: GlobalTime = GlobalTime::from_nanos(10_000_000);

/// How many quiet slices after end-of-input before a piped session gives up.
///
/// Two virtual seconds: long enough for a paced 60-character-a-second display
/// to finish a screenful it was still emitting when the script ran out.
const IDLE_SLICES: u32 = 200;

/// Run the machine with the terminal attached, until the user stops it.
///
/// Virtual time is held to real time by sleeping off whatever the slice did not
/// use. Nothing below `host/` reads a clock (`CLAUDE.md`); this is `host/`'s
/// job, and it is why an Apple 1 here feels like an Apple 1 rather than
/// finishing a screenful of output before the terminal has drawn a line.
fn interact(machine: &mut Machine, port: &CharPort, args: &RunArgs) -> ExitCode {
    let term = Terminal::open();
    if !args.quiet {
        if term.is_raw() {
            eprintln!("  console attached — Ctrl-C to stop\n");
        } else {
            eprintln!(
                "  console attached, cooked mode — stdin could not be put in raw mode.\n  \
                 On a terminal that means input arrives a line at a time and is\n  \
                 echoed twice, once by the host and once by the guest.\n"
            );
        }
    }

    let deadline = args
        .span_given
        .then(|| machine.now().saturating_add(args.span));
    let started = Instant::now();
    let mut elapsed = GlobalTime::ZERO;
    let mut idle = 0u32;

    let status = loop {
        if term.interrupted() {
            break ExitCode::SUCCESS;
        }
        if deadline.is_some_and(|d| machine.now() >= d) {
            break ExitCode::SUCCESS;
        }
        let mut moved = term.pump(port);
        if let Err(e) = machine.run_until(machine.now().saturating_add(SLICE)) {
            eprintln!("\r\nrsemu: {e}");
            break ExitCode::FAILURE;
        }
        moved += term.pump(port);

        // A script on stdin has an end; a person does not. Once the input is
        // exhausted *and* the machine has gone quiet, there is nobody left to
        // wait for, so `printf … | rsemu run apple1` finishes rather than
        // hanging on a machine that will never be typed at again.
        if term.at_eof() && moved == 0 {
            idle += 1;
            if idle >= IDLE_SLICES {
                break ExitCode::SUCCESS;
            }
        } else {
            idle = 0;
        }

        // Hold virtual time to real time. A slice that took longer than its
        // own span to simulate simply does not sleep, and the machine runs
        // slow rather than jumping.
        elapsed = elapsed.saturating_add(SLICE);
        let target = Duration::from_nanos(elapsed.as_nanos());
        if let Some(wait) = target.checked_sub(started.elapsed()) {
            std::thread::sleep(wait);
        }
    };

    // Restore the terminal before anything else is printed on it.
    term.flush();
    drop(term);
    if !args.quiet {
        println!();
        summarise(machine);
    }
    status
}

/// A machine description by path, or by catalog name.
fn load_description(what: &str) -> Result<(String, String), String> {
    let path = Path::new(what);
    if path.is_file() {
        let text = std::fs::read_to_string(path).map_err(|e| format!("cannot read {what}: {e}"))?;
        return Ok((what.to_string(), text));
    }
    if let Some(entry) = catalog::machine(what) {
        return Ok((entry.name.to_string(), entry.source.to_string()));
    }
    // A name that looks like a path deserves the filesystem's complaint rather
    // than "not in the catalog", which would send the reader the wrong way.
    if what.contains('/') || what.ends_with(".machine") {
        return Err(format!("no such machine file: {what}"));
    }
    Err(format!(
        "no machine named `{what}`; `rsemu machines` lists this build's catalog"
    ))
}

/// The monitor image to bind to an unfilled `rom` slot.
///
/// `--monitor` names one explicitly; without it each board gets its own
/// default, which is always rsemu's own (`ROADMAP.md` §1 — the machine has to
/// demonstrate itself with nothing whose licence anyone has to think about).
/// `Ok(None)` means this build has no image to offer, which is not an error:
/// the machine may not want a `rom` slot at all.
/// The image a machine's `firmware` slot falls back to, if it has one.
///
/// Unlike [`builtin_rom`] there is no monitor to choose between: this is one
/// board's own demonstration program.
fn builtin_firmware(machine: &str) -> Option<Vec<u8>> {
    let stem = machine
        .rsplit('/')
        .next()
        .unwrap_or(machine)
        .strip_suffix(".machine")
        .unwrap_or_else(|| machine.rsplit('/').next().unwrap_or(machine));
    match stem {
        #[cfg(feature = "machine-spi-panel")]
        "spi-panel" => Some(rsemu::dev::lcd::demo::PANEL_DEMO.to_vec()),
        _ => {
            let _ = stem;
            None
        }
    }
}

fn builtin_rom(monitor: Option<&str>, machine: &str) -> Result<Option<Vec<u8>>, String> {
    // The Woz Monitor is only ported to one of these boards, so `wozmon` is
    // answered from the module that has it rather than per machine.
    #[cfg(feature = "dev-wdc")]
    if monitor == Some("wozmon") {
        return Ok(Some(rsemu::dev::wdc::WOZMON_IMAGE.to_vec()));
    }
    if let Some(name) = monitor
        && name != "rsmon"
    {
        return Err(format!(
            "--monitor {name}: this build has `rsmon`{}",
            if cfg!(feature = "dev-wdc") {
                " and `wozmon`"
            } else {
                ""
            }
        ));
    }
    let stem = machine
        .rsplit('/')
        .next()
        .unwrap_or(machine)
        .strip_suffix(".machine")
        .unwrap_or_else(|| machine.rsplit('/').next().unwrap_or(machine));
    match stem {
        #[cfg(feature = "dev-wdc")]
        "beneater-6502" => Ok(Some(rsemu::dev::wdc::RSMON_IMAGE.to_vec())),
        #[cfg(feature = "dev-apple1")]
        "apple1" => Ok(Some(rsemu::dev::apple1::RSMON.to_vec())),
        _ => Ok(None),
    }
}

fn parse_run(args: &[String]) -> Result<RunArgs, String> {
    let mut out = RunArgs {
        machine: String::new(),
        media: Vec::new(),
        monitor: None,
        params: Vec::new(),
        // One second of virtual time: long enough to prove a machine runs,
        // short enough that a broken one does not hang a terminal. There is no
        // "until the user quits" yet — that needs the host window (phase 3).
        span: GlobalTime::from_nanos(1_000_000_000),
        span_given: false,
        console: None,
        headless: false,
        quiet: false,
        #[cfg(feature = "gdb")]
        gdb: None,
    };
    let mut i = 0;
    while i < args.len() {
        let arg = args[i].as_str();
        let mut value = |name: &str| -> Result<String, String> {
            i += 1;
            args.get(i)
                .cloned()
                .ok_or_else(|| format!("{name} needs a value"))
        };
        match arg {
            "--cart" | "--rom" | "--disk" => {
                let slot = arg.trim_start_matches('-').to_string();
                let path = value(arg)?;
                out.media.push((slot, path));
            }
            "--media" => {
                let spec = value(arg)?;
                let (slot, path) = spec
                    .split_once('=')
                    .ok_or_else(|| format!("--media wants <name>=<file>, got `{spec}`"))?;
                out.media.push((slot.to_string(), path.to_string()));
            }
            "-p" | "--param" => {
                let spec = value(arg)?;
                let (key, val) = spec
                    .split_once('=')
                    .ok_or_else(|| format!("-p wants <name>=<value>, got `{spec}`"))?;
                out.params.push((key.to_string(), val.to_string()));
            }
            "--for" => {
                let text = value(arg)?;
                let d = rsemu::core::props::parse_duration(&text)
                    .map_err(|e| format!("--for {text}: {e}"))?;
                // The timeline is in 2^-64-second units and durations are in
                // picoseconds; nanoseconds is the unit both agree on.
                out.span = GlobalTime::from_nanos(d.as_picos() / 1_000);
                out.span_given = true;
            }
            "--monitor" => out.monitor = Some(value(arg)?),
            #[cfg(feature = "gdb")]
            "--gdb" => out.gdb = Some(value(arg)?),
            "--console" => out.console = Some(value(arg)?),
            "--headless" => out.headless = true,
            "-q" | "--quiet" => out.quiet = true,
            other if other.starts_with('-') => {
                return Err(format!("unknown option `{other}`"));
            }
            other => {
                if !out.machine.is_empty() {
                    return Err(format!("`{other}`: only one machine at a time"));
                }
                out.machine = other.to_string();
            }
        }
        i += 1;
    }
    if out.machine.is_empty() {
        return Err(String::from(
            "run needs a machine; `rsemu machines` lists this build's catalog",
        ));
    }
    Ok(out)
}

/// What was assembled, before it starts running.
fn describe_machine(machine: &Machine) {
    println!("machine \"{}\"", machine.name());
    for space in machine.spaces() {
        println!("  space  {:<8} {} bits", space.name(), space.space().bits());
    }
    for device in machine.devices() {
        let clock = match device.domain() {
            Some(_) => "clocked",
            None => "",
        };
        println!(
            "  object {:<8} {:<16} {clock}",
            device.path(),
            device.class().name
        );
    }
}

/// Where the machine got to.
fn summarise(machine: &Machine) {
    println!("ran to {} ns of virtual time", machine.now().as_nanos());
    for device in machine.devices() {
        let Some(domain) = device.domain() else {
            continue;
        };
        if let Ok(ticks) = machine.clocks().ticks(domain) {
            println!("  {:<8} {ticks} ticks", device.path());
        }
    }
    match machine.state_hash() {
        // The regression method of §0 in one number: run deterministically for
        // N virtual units and compare this.
        Ok(hash) => println!("state hash {hash:#018x}"),
        Err(e) => eprintln!("rsemu: cannot hash state: {e}"),
    }
}

/// Print an error the way §5 promises and exit non-zero.
fn fail(e: &rsemu::Error) -> ExitCode {
    eprintln!("rsemu: {e}");
    ExitCode::FAILURE
}
