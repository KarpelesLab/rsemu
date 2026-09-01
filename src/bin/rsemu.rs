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

use std::num::NonZeroUsize;
use std::path::Path;
use std::process::ExitCode;
use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::HostObjects;
use rsemu::core::clock::GlobalTime;
use rsemu::core::sched::ThreadingMode;
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
    --bios <file>       Bind the `bios` media slot: a PC's system firmware.
                        rsemu ships none — point this at your own copy, the
                        way you would point qemu at one. Running a firmware
                        binary as a guest is ordinary use whatever its licence;
                        redistributing it is not, which is why there is a flag
                        here and no file in the repository
    --vgabios <file>    Bind the `vgabios` media slot: a video option ROM
    --floppy <file>     Bind the `floppy` media slot: a raw diskette image
    --hd0 <file>        Bind the `hd0` media slot: a raw hard disk image for
                        the first IDE bay. Unbound is an empty bay.
    --hd1 <file>        Bind the `hd1` media slot: the second IDE bay
    --flash0 <file>     Bind the `flash0` media slot: a NOR flash bank's
                        contents. `riscv-virt` boots UEFI out of it.
    --flash1 <file>     Bind the `flash1` media slot: the second NOR bank,
                        which is where UEFI keeps its variables.
    --initrd <file>     Bind the `initrd` media slot: a ramdisk staged in
                        guest RAM, which the generated device tree then points
                        the kernel at
    --media <n>=<file>  Bind any media slot by name
    -p <name>=<value>   Override a `param` declared in the machine file
    --threading <mode>  How guest execution is spread over host threads
                        (ROADMAP.md 4.2). `deterministic` (the default) is one
                        thread, round-robin, and bit-reproducible.
                        `parallel[:N]` is a thread per CPU with a rendezvous
                        barrier per quantum -- faster on a machine with more
                        than one CPU, and NOT reproducible, so a state hash is
                        refused in it. N is the worker count; without it, one
                        per runnable, capped at what the host has
    --for <duration>    How much virtual time to run, as `1s`, `500ms`, `2m`
                        (default 1s, or forever with a console attached)
    --console <name>    Attach this terminal to a named character port. A
                        machine that opens exactly one is picked up on its own,
                        so `rsemu run apple1` is interactive already
    --headless          Do not attach a terminal, whatever the machine opened
    --screenshot <file> Write the machine's display to a PNG when the run ends.
                        Needs a build with `display-png` and a machine with a
                        display; a machine with neither says so rather than
                        writing nothing
    --record-audio <f>  Write the machine's sound to a RIFF/WAVE file when the
                        run ends. Needs a machine with an audio device. The
                        device's ring has to hold the whole run, so a recording
                        is capped at about 18 seconds; longer runs say what they
                        lost.
    --audio-rate <hz>   Sample rate for --record-audio (default 44100)

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
    /// Where to write a PNG of the display when the run ends, if `--screenshot`
    /// was given.
    screenshot: Option<String>,
    /// Where to write a WAV of the sound when the run ends, if `--record-audio`
    /// was given.
    record_audio: Option<String>,
    /// The rate `--record-audio` writes at.
    audio_rate: u32,
    quiet: bool,
    /// How guest execution is spread over host threads (§4.2), and how many
    /// pool workers to ask for.
    ///
    /// `None` workers means *one per runnable*, which is what "a thread per
    /// CPU" means and which the count is only known after the machine is
    /// built.
    threading: (ThreadingMode, Option<usize>),
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

    // And for the NOR flash banks a `riscv-virt` has: nothing bound means a
    // board with blank parts on it, which is what a factory ships and what a
    // UEFI build will format for itself. Naming the slots explicitly on every
    // run to say "empty" would be ceremony, and an unbound slot is an error by
    // design (`machine::realize`). The same goes for a PC's two IDE bays: no
    // bytes bound is an empty bay, which is what most PCs of the period had in
    // at least one of them.
    for slot in ["flash0", "flash1", "initrd", "disk", "hd0", "hd1"] {
        if !images.iter().any(|(bound, _)| bound == slot) {
            images.push((String::from(slot), Vec::new()));
        }
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
    options.realize.scheduler.mode = parsed.threading.0;

    // A host gets a typed handle on a display device at the one moment the
    // concrete type exists: construction. Installed unconditionally rather than
    // only under `--screenshot`, so that asking for a screenshot after the fact
    // is never the thing that changes how the machine was built.
    if let Err(e) = install_capture(&mut options, &parsed) {
        return fail(&e);
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

    // The pool goes in *after* the build, because `--threading parallel` with
    // no count means one worker per runnable and there is no way to count them
    // before the machine exists. Realizing twice to find out is not an option:
    // a `RealizeOptions`'s host objects are deliberately shared between builds,
    // so a second realize would open the same character port twice.
    if parsed.threading.0 == ThreadingMode::Parallel {
        let workers = parsed.threading.1.unwrap_or_else(|| {
            let runnables = machine
                .devices()
                .iter()
                .filter(|d| d.runnable().is_some())
                .count();
            // Never more than the host has: a worker per runnable on a
            // four-core box running an eight-CPU guest is eight threads
            // fighting over four cores, which is slower than four.
            let hosted = std::thread::available_parallelism().map_or(1, NonZeroUsize::get);
            runnables.min(hosted)
        });
        machine
            .scheduler_mut()
            .set_pool(Arc::new(rsemu::core::sync::Pool::new(workers)));
    }

    if !parsed.quiet {
        describe_machine(&machine);
    }

    // A debugger, if one was asked for, owns when the machine advances — so it
    // is checked before the console loop, which would otherwise own that.
    #[cfg(feature = "gdb")]
    if let Some(addr) = parsed.gdb.clone() {
        let port = match console_port(&parsed, &options.realize.hosts) {
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
    match console_port(&parsed, &options.realize.hosts) {
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
        write_screenshot(&parsed, &options.realize.hosts);
        write_recording(&parsed, &options.realize.hosts);
        return ExitCode::FAILURE;
    }
    summarise(&machine);
    // Both, always: a `--screenshot` that failed must not be the reason a
    // recording is silently skipped.
    let drew = write_screenshot(&parsed, &options.realize.hosts);
    let played = write_recording(&parsed, &options.realize.hosts);
    if !drew || !played {
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

/// Wrap the machine's bindings so a display or audio device hands the host a
/// handle.
///
/// One arm per device family that publishes a scanout or a sample stream,
/// exactly like the registration lists in `machine::catalog`: a family that is
/// not named here is not in the build, and that is visible by reading the code.
#[allow(unused_variables, unused_mut)]
fn install_capture(
    options: &mut rsemu::machine::BuildOptions,
    args: &RunArgs,
) -> rsemu::Result<()> {
    #[cfg(feature = "dev-nes-ppu")]
    rsemu::host::display::nes::capture::install(options)?;
    #[cfg(feature = "dev-pc-video")]
    rsemu::host::display::pc::capture::install(options)?;
    #[cfg(feature = "dev-nes-apu")]
    rsemu::host::audio::nes::capture::install(options, ring_for(args))?;
    Ok(())
}

/// How deep an audio device's output ring has to be for this run.
///
/// **The recording must not change how the machine is driven.** A headless run
/// is one `run_for(span)` with nothing between, so there is no cadence at which
/// the host could drain — which leaves exactly one honest option: make the ring
/// big enough for the whole run. That is why a recording is capped at what the
/// device will allocate (`dev::apu::MAX_SAMPLE_BUFFER`, about 18 seconds) and
/// why `write_recording` reports what a longer run lost rather than quietly
/// producing a short file.
///
/// Slicing the run is no longer *wrong* — `Machine::run_for` is additive, so a
/// run cut into pieces reaches the same state as the same run taken whole
/// (§11.6) — but it is still not free: each slice ends on a scheduling boundary
/// and the host would have to be driven between them. The ring stays the
/// simple answer.
///
/// Zero when nothing is being recorded, which leaves whatever the machine
/// description asked for untouched.
///
/// Only the audio interception calls it, so a build with no sound chip in it
/// has no use for the function — and the compiler is right to say so rather
/// than being told to be quiet, exactly as with `take_scanout`.
#[cfg(feature = "dev-nes-apu")]
fn ring_for(args: &RunArgs) -> u64 {
    if args.record_audio.is_none() {
        return 0;
    }
    // One sample per APU cycle at a little under 1 MHz, so a nanosecond of
    // virtual time is under a thousandth of a sample. Dividing by 1000 is
    // therefore an over-estimate by about 12%, which is the direction to err.
    args.span.as_nanos() / 1_000
}

/// The display of the machine just built, if it has one this build can see.
///
/// `hosts` is the table the machine was built against, which is where its
/// constructors left their handles — one table per build, so this can only
/// return this machine's screen.
///
/// Only the PNG path calls it, so a build without an encoder has no use for it
/// — and the compiler is right to say so rather than being told to be quiet
/// about a function that might one day be called.
#[cfg(feature = "display-png")]
#[allow(unused_variables)]
fn take_scanout(hosts: &HostObjects) -> Option<Box<dyn rsemu::host::display::Scanout>> {
    #[cfg(feature = "dev-pc-video")]
    if let Some(s) = rsemu::host::display::pc::capture::take(hosts) {
        return Some(Box::new(s));
    }
    #[cfg(feature = "dev-nes-ppu")]
    if let Some(s) = rsemu::host::display::nes::capture::take(hosts) {
        return Some(Box::new(s));
    }
    None
}

/// Write `--screenshot`'s PNG, reporting whether the run should still count as
/// a success.
///
/// Returns true when nothing was asked for. A `--screenshot` that could not be
/// honoured is an error rather than a silence: the user asked for a file and
/// there has to be one, or a reason.
fn write_screenshot(args: &RunArgs, hosts: &HostObjects) -> bool {
    let Some(path) = args.screenshot.as_deref() else {
        return true;
    };
    #[cfg(not(feature = "display-png"))]
    {
        let _ = (path, hosts);
        eprintln!("rsemu: --screenshot needs a build with the `display-png` feature");
        false
    }
    #[cfg(feature = "display-png")]
    {
        use rsemu::host::display::{Surface, png};
        let Some(scanout) = take_scanout(hosts) else {
            eprintln!("rsemu: --screenshot: this machine has no display");
            return false;
        };
        let mut surface = Surface::for_scanout(scanout.as_ref());
        scanout.capture(&mut surface);
        let bytes = match png::encode(&surface) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("rsemu: --screenshot: {e}");
                return false;
            }
        };
        match std::fs::write(path, &bytes) {
            Ok(()) => {
                if !args.quiet {
                    println!(
                        "screenshot  {path} ({}x{}, {} bytes)",
                        surface.width(),
                        surface.height(),
                        bytes.len()
                    );
                }
                true
            }
            Err(e) => {
                eprintln!("rsemu: cannot write {path}: {e}");
                false
            }
        }
    }
}

/// The sound of the machine just built, if it has any this build can hear.
#[allow(unused_variables)]
fn take_audio(hosts: &HostObjects) -> Option<Box<dyn rsemu::host::audio::AudioSource>> {
    #[cfg(feature = "dev-nes-apu")]
    if let Some(s) = rsemu::host::audio::nes::capture::take(hosts) {
        return Some(Box::new(s));
    }
    None
}

/// Write `--record-audio`'s WAV, reporting whether the run should still count
/// as a success.
///
/// Returns true when nothing was asked for. As with `--screenshot`, a
/// recording that could not be made is an error rather than a silence.
fn write_recording(args: &RunArgs, hosts: &HostObjects) -> bool {
    let Some(path) = args.record_audio.as_deref() else {
        return true;
    };
    use rsemu::host::audio::{AudioStream, SampleFormat, wav};

    let Some(source) = take_audio(hosts) else {
        eprintln!("rsemu: --record-audio: this machine has no audio device");
        return false;
    };
    let mut stream = AudioStream::new(source, args.audio_rate, SampleFormat::S16);
    // The whole run is in the device's ring, so nothing may be trimmed on the
    // way out: the default queue limit is two seconds and this is not that.
    stream.set_limit_frames(u64::MAX);
    stream.pull();

    let bytes = wav::encode(stream.info(), stream.buffer());
    match std::fs::write(path, &bytes) {
        Ok(()) => {
            let frames = stream.buffer().frames();
            if !args.quiet {
                let ms = frames.saturating_mul(1000) / u64::from(args.audio_rate.max(1));
                println!(
                    "audio       {path} ({} Hz, {frames} frames, {}.{:03} s, {} bytes)",
                    args.audio_rate,
                    ms / 1000,
                    ms % 1000,
                    bytes.len()
                );
            }
            let lost = stream.dropped();
            if lost > 0 {
                eprintln!(
                    "rsemu: --record-audio: {lost} samples were lost. The device's ring holds \
                     about 18 seconds of audio and this run was longer, so the file is its \
                     *tail* — a ring keeps the newest. Record a shorter --for."
                );
            }
            true
        }
        Err(e) => {
            eprintln!("rsemu: cannot write {path}: {e}");
            false
        }
    }
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
/// open in `hosts` — the table that build used, and nobody else's. One is
/// unambiguous; several need `--console` to choose between them, because
/// guessing would put the keyboard on the wrong device.
fn console_port(args: &RunArgs, hosts: &HostObjects) -> Result<Option<Arc<CharPort>>, String> {
    if args.headless {
        return Ok(None);
    }
    let opened = |name: &str| ports::get(hosts, name).ok().flatten();
    if let Some(name) = &args.console {
        return opened(name).map(Some).ok_or_else(|| {
            format!(
                "no character port named `{name}`; this machine opened {}",
                list(&ports::names(hosts))
            )
        });
    }
    let names = ports::names(hosts);
    match names.len() {
        0 => Ok(None),
        1 => Ok(opened(&names[0])),
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
        screenshot: None,
        // 44 100 rather than 48 000: it is what a `.wav` is expected to be, and
        // every player on earth opens one without resampling it again.
        record_audio: None,
        audio_rate: 44_100,
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
        // Deterministic unless asked otherwise: reproducibility is the default
        // a person gets, and giving it up has to be a thing they typed.
        threading: (ThreadingMode::Deterministic, None),
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
            // One arm, because every one of these is `--<slot> <file>` and a
            // list is easier to extend than a match with a line each. They
            // exist at all because `--media bios=…` is correct and nobody
            // types it.
            "--cart" | "--rom" | "--disk" | "--bios" | "--vgabios" | "--floppy" | "--flash0"
            | "--flash1" | "--initrd" | "--hd0" | "--hd1" => {
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
            "--screenshot" => out.screenshot = Some(value(arg)?),
            "--record-audio" => out.record_audio = Some(value(arg)?),
            "--audio-rate" => {
                let text = value(arg)?;
                let hz: u32 = text
                    .parse()
                    .map_err(|_| format!("--audio-rate {text}: not a number of hertz"))?;
                if !(8_000..=384_000).contains(&hz) {
                    return Err(format!("--audio-rate {hz}: outside 8000..=384000 Hz"));
                }
                out.audio_rate = hz;
            }
            "--threading" => {
                let text = value(arg)?;
                out.threading = parse_threading(&text)?;
            }
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

/// `deterministic`, `parallel`, or `parallel:<workers>`.
///
/// `accel` is deliberately absent: it needs the hardware backends of
/// `ROADMAP.md` 10 and would only be a way to type an error message.
fn parse_threading(text: &str) -> Result<(ThreadingMode, Option<usize>), String> {
    let (name, workers) = match text.split_once(':') {
        Some((name, count)) => {
            let n: usize = count
                .parse()
                .map_err(|_| format!("--threading {text}: `{count}` is not a worker count"))?;
            (name, Some(n))
        }
        None => (text, None),
    };
    match name {
        "deterministic" => {
            if workers.is_some() {
                return Err(String::from(
                    "--threading deterministic takes no worker count: it is one thread by \
                     definition",
                ));
            }
            Ok((ThreadingMode::Deterministic, None))
        }
        "parallel" => Ok((ThreadingMode::Parallel, workers)),
        other => Err(format!(
            "--threading {other}: expected `deterministic` or `parallel[:<workers>]`"
        )),
    }
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
    if !machine.threading_mode().is_deterministic() {
        // Not an error, and not a number either: §4.2 says a parallel run is
        // non-deterministic, so printing a hash here would invite somebody to
        // paste it into a test. Say what happened instead.
        println!(
            "state hash: not reproducible under `{}` threading",
            machine.threading_mode()
        );
        return;
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
