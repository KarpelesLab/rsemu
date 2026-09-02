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
                        Unbound, `pc-at` gets rsemu's own minimal legacy BIOS,
                        assembled from this repository rather than shipped as
                        a blob. Point this at somebody else's image and it
                        wins. Running a firmware binary as a guest is ordinary
                        use whatever its licence; redistributing it is not,
                        which is why the only one in the repository is ours
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
    --drive <n>=<file>[,<opt>…]
                        Back a media slot with the image *file* rather than a
                        copy of its bytes: the guest's writes go to the file,
                        the capacity comes from the image, and a structured
                        format is understood -- raw (sparse), qcow2, DMG,
                        DiskCopy 4.2 and LUKS, all through `fstool`. So
                        `--hd0 disk.img` runs a 64 MiB disk out of 64 MiB of
                        RAM and discards what the guest wrote, and
                        `--drive hd0=disk.qcow2` runs a sparse one off the
                        disk and keeps it. Options, comma separated after the
                        path: `ro` opens it read-only and write protects the
                        drive; `new=<size>` creates the image instead of
                        opening it (a `.qcow2` extension makes a qcow2, any
                        other a sparse raw file); `cluster=<size>` sets a new
                        qcow2's allocation granularity (default 64K, as
                        qemu-img); `password=<word>` unlocks an
                        encrypted container; `snapshot=capture|reference|refuse`
                        chooses what a machine snapshot does about the bytes
                        (default `reference` -- the image stays outside the
                        snapshot, which is the only honest answer for a big
                        one). Needs a build with `dev-blk`
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
                        run ends. Needs a machine with an audio device. A
                        headless run visits the host only once, so the device's
                        ring has to hold the whole thing and a recording is
                        capped at about 18 seconds; longer runs say what they
                        lost. Under --vnc the ring is drained every frame and
                        there is no cap.
    --audio-rate <hz>   Sample rate for --record-audio (default 44100)

    --gdb <addr>        Listen for GDB on <addr> and hold the machine stopped
                        until it attaches. `1234`, `:1234` and `host:1234` all
                        work; a bare port binds the loopback interface only,
                        because the far end can read and write all of guest
                        memory. `rsemu debug` implies `--gdb :1234`
    --vnc <addr>        Serve the machine's display over VNC (RFB, RFC 6143) and
                        take keyboard and pointer input from whoever connects.
                        `5900`, `:5900` and `host:5900` all work; a bare port
                        binds the loopback interface only, because there is no
                        authentication. The machine runs at wall-clock speed
    --record-input <f>  With --vnc, write every input event and the virtual
                        instant it was delivered at to <f>
    --replay-input <f>  With --vnc, take input from <f> instead of from the
                        network, at the instants it records. The run is then
                        the recorded one, bit for bit
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
    /// `--drive <slot>=<file>[,ro][,new=<size>]`: media slots backed by the
    /// host file itself rather than by a copy of its bytes in RAM.
    #[cfg(feature = "dev-blk")]
    drives: Vec<Drive>,
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
    /// Where to listen for VNC clients, if `--vnc` was given.
    #[cfg(feature = "vnc")]
    vnc: Option<String>,
    /// Where to write the input log, if `--record-input` was given.
    #[cfg(feature = "vnc")]
    record_input: Option<String>,
    /// Where to read one from, if `--replay-input` was given.
    #[cfg(feature = "vnc")]
    replay_input: Option<String>,
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
    // `floppy` and `vgabios` are the same claim about a PC: an empty drive and
    // an empty option-ROM socket are both ordinary configurations, and a board
    // that refused to assemble without a diskette would be describing no
    // machine anyone ever owned.
    for slot in [
        "flash0", "flash1", "initrd", "disk", "hd0", "hd1", "floppy", "vgabios",
    ] {
        if !images.iter().any(|(bound, _)| bound == slot) {
            images.push((String::from(slot), Vec::new()));
        }
    }

    // A PC's system firmware, if this build has one and the user named none.
    // `rsemu` ships exactly one piece of firmware and this is where it is
    // offered (`fw::pcbios`); `--bios` still wins, because the slot is the
    // user's and always will be (`docs/platforms/pc-at.md`).
    if !images.iter().any(|(slot, _)| slot == "bios")
        && let Some(image) = builtin_bios(&parsed.machine)
    {
        images.push((String::from("bios"), image));
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

    // `--drive` images open here: before the build, so a drive finds its medium
    // waiting under the media slot the machine file names, and so a path that
    // does not exist fails before anything is realized.
    #[cfg(feature = "dev-blk")]
    if let Err(e) = install_drives(&options, &parsed) {
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

    // A `--drive` nothing picked up is almost always a typo in the slot name,
    // and it fails *silently*: the machine builds, the bay is empty, and the
    // guest reports no disk. Say so rather than letting the user wonder why
    // their image did not boot.
    #[cfg(feature = "dev-blk")]
    for slot in rsemu::dev::medium::names(&options.realize.hosts) {
        if rsemu::dev::medium::get(&options.realize.hosts, &slot)
            .is_ok_and(|found| found.is_some_and(|s| s.is_occupied()))
        {
            eprintln!(
                "rsemu: warning: --drive {slot}=… was never picked up; no device in this \
                 machine names the `{slot}` media slot"
            );
        }
    }

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
        let status = debug_session(&mut machine, &addr, port.as_ref(), &parsed);
        return finish(&machine, status);
    }

    // A remote frontend owns when the machine advances, for the same reason a
    // debugger does — so it is checked before the console loop.
    #[cfg(feature = "vnc")]
    if parsed.vnc.is_some() {
        let status = vnc_session(&mut machine, &parsed, &options.realize.hosts);
        return finish(&machine, status);
    }

    // A machine that opened a character port has a console; attach this
    // terminal to it and hand the keyboard over.
    match console_port(&parsed, &options.realize.hosts) {
        Err(e) => {
            eprintln!("rsemu: {e}");
            return ExitCode::from(2);
        }
        Ok(Some(port)) => {
            let status = interact(&mut machine, &port, &parsed);
            return finish(&machine, status);
        }
        Ok(None) => {}
    }

    if let Err(e) = machine.run_for(parsed.span) {
        eprintln!("rsemu: {e}");
        summarise(&machine);
        write_screenshot(&parsed, &options.realize.hosts, &machine);
        write_recording(&parsed, &options.realize.hosts);
        return finish(&machine, ExitCode::FAILURE);
    }
    summarise(&machine);
    // Both, always: a `--screenshot` that failed must not be the reason a
    // recording is silently skipped.
    let drew = write_screenshot(&parsed, &options.realize.hosts, &machine);
    let played = write_recording(&parsed, &options.realize.hosts);
    if !drew || !played {
        return finish(&machine, ExitCode::FAILURE);
    }
    finish(&machine, ExitCode::SUCCESS)
}

/// End a run: push what the guest wrote out to the host, and report.
///
/// **Every way out of a run goes through here**, including the failing ones. A
/// guest that was never asked to flush has promised nothing, and a drive is
/// entitled to hold a write in its cache — but that entitlement lasts as long
/// as the machine does, and when the process exits there is nobody left to
/// write those bytes. On a qcow2 it is worse than staleness: the data cluster
/// and the L2 entry that finds it are both in the image, and losing one of them
/// leaves a hole where a sector used to be.
///
/// A machine that crashed still flushes. What the guest managed to write before
/// it died is not made better by throwing it away, and `--drive` is how a user
/// asked for a disk that outlives the run.
///
/// A flush failure turns a successful run into a failing exit, because the run
/// did not do what the user asked: the bytes are not on disk.
fn finish(machine: &Machine, status: ExitCode) -> ExitCode {
    match machine.flush() {
        Ok(()) => status,
        Err(e) => {
            eprintln!("rsemu: flushing at exit: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Wrap the machine's bindings so a display or audio device hands the host a
/// handle.
///
/// One arm per device family that publishes a scanout or a sample stream,
/// exactly like the registration lists in `machine::catalog`: a family that is
/// not named here is not in the build, and that is visible by reading the code.
#[allow(unused_variables, unused_mut)]
/// One `--drive <slot>=<file>[,ro][,new=<size>][,snapshot=<policy>]`.
///
/// The file-backed counterpart of `--hd0`, and deliberately a separate flag
/// rather than a change to it: `--hd0 disk.img` copies the file's bytes into a
/// buffer and throws the guest's writes away when the run ends, and `--drive
/// hd0=disk.img` writes *through* to the file. Those are different contracts,
/// and quietly upgrading one to the other would make an existing command line
/// modify a file it never used to touch.
#[cfg(feature = "dev-blk")]
#[derive(Debug)]
struct Drive {
    slot: String,
    path: String,
    options: rsemu::dev::blk::ImageOptions,
}

#[cfg(feature = "dev-blk")]
impl Drive {
    fn parse(spec: &str) -> Result<Drive, String> {
        let (head, rest) = spec.split_once('=').ok_or_else(|| {
            format!("--drive wants <slot>=<file>[,ro][,new=<size>], got `{spec}`")
        })?;
        // The path comes first and options follow it, so a comma inside a
        // filename is only ambiguous for a name that also ends in `,ro` — say
        // so rather than guessing.
        let mut parts = rest.split(',');
        let path = parts.next().unwrap_or("").to_string();
        if path.is_empty() {
            return Err(format!("--drive {head}= needs a file"));
        }
        let mut options = rsemu::dev::blk::ImageOptions::new();
        for opt in parts {
            match opt.split_once('=') {
                None if opt == "ro" || opt == "readonly" => options = options.read_only(true),
                Some(("new", size)) => {
                    let bytes = rsemu::core::props::parse_size(size)
                        .map_err(|e| format!("--drive {head}: new={size}: {e}"))?;
                    options = options.create(bytes);
                }
                Some(("snapshot", policy)) => {
                    let chosen =
                        rsemu::dev::medium::Snapshot::from_name(policy).ok_or_else(|| {
                            format!(
                                "--drive {head}: snapshot={policy}: one of `capture`, `reference` \
                             or `refuse`"
                            )
                        })?;
                    options = options.snapshot(chosen);
                }
                Some(("password", word)) => options = options.password(word),
                Some(("cluster", size)) => {
                    let bytes = rsemu::core::props::parse_size(size)
                        .map_err(|e| format!("--drive {head}: cluster={size}: {e}"))?;
                    let bytes = u32::try_from(bytes)
                        .map_err(|_| format!("--drive {head}: cluster={size} is implausible"))?;
                    options = options.cluster(bytes);
                }
                _ => return Err(format!("--drive {head}: `{opt}` is not a drive option")),
            }
        }
        Ok(Drive {
            slot: head.to_string(),
            path,
            options,
        })
    }
}

/// Open every `--drive` image and install it under the media slot it names.
///
/// Before the machine is built, so the drive finds its medium waiting when it
/// is constructed — and so a bad path fails before anything is realized.
#[cfg(feature = "dev-blk")]
fn install_drives(options: &rsemu::machine::BuildOptions, args: &RunArgs) -> rsemu::Result<()> {
    use std::sync::Arc;
    for drive in &args.drives {
        let image =
            rsemu::dev::blk::Image::open(std::path::Path::new(&drive.path), &drive.options)?;
        rsemu::dev::blk::install(&options.realize.hosts, &drive.slot, Arc::new(image))?;
    }
    Ok(())
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
    #[cfg(feature = "dev-gb")]
    rsemu::host::display::gb::capture::install(options)?;
    #[cfg(feature = "dev-sms")]
    rsemu::host::display::sms::capture::install(options)?;
    #[cfg(feature = "dev-lcdc")]
    rsemu::host::display::lcd::capture::install(options)?;
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
/// `machine` is here for the one adapter that cannot answer without it. A
/// generic `lcd.scanout` engine does not know its own frame rate: the rate is a
/// property of the clock forest rather than of the device, and a device cannot
/// reach the forest from `&self`. So the realized machine is handed in and
/// `lcd::capture::take` resolves the domain's exact rational frequency out of it
/// (`host::display::lcd`). Every other adapter ignores it, which is why it is
/// `unused_variables` on a build with none of them.
///
/// Only the PNG path calls it, so a build without an encoder has no use for it
/// — and the compiler is right to say so rather than being told to be quiet
/// about a function that might one day be called.
#[cfg(any(feature = "display-png", feature = "vnc"))]
#[allow(unused_variables)]
fn take_scanout(
    hosts: &HostObjects,
    machine: &Machine,
) -> Option<Box<dyn rsemu::host::display::Scanout>> {
    #[cfg(feature = "dev-pc-video")]
    if let Some(s) = rsemu::host::display::pc::capture::take(hosts) {
        return Some(Box::new(s));
    }
    #[cfg(feature = "dev-nes-ppu")]
    if let Some(s) = rsemu::host::display::nes::capture::take(hosts) {
        return Some(Box::new(s));
    }
    #[cfg(feature = "dev-gb")]
    if let Some(s) = rsemu::host::display::gb::capture::take(hosts) {
        return Some(Box::new(s));
    }
    #[cfg(feature = "dev-sms")]
    if let Some(s) = rsemu::host::display::sms::capture::take_vdp(hosts) {
        return Some(Box::new(s));
    }
    // Last, because it is the generic one: a board with a console's own video
    // chip *and* an `lcd.scanout` engine is showing the console, and the engine
    // is whatever else it happens to have.
    #[cfg(feature = "dev-lcdc")]
    if let Some(s) = rsemu::host::display::lcd::capture::take(hosts, machine) {
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
fn write_screenshot(args: &RunArgs, hosts: &HostObjects, machine: &Machine) -> bool {
    let Some(path) = args.screenshot.as_deref() else {
        return true;
    };
    #[cfg(not(feature = "display-png"))]
    {
        let _ = (path, hosts, machine);
        eprintln!("rsemu: --screenshot needs a build with the `display-png` feature");
        false
    }
    #[cfg(feature = "display-png")]
    {
        use rsemu::host::display::{Surface, png};
        let Some(scanout) = take_scanout(hosts, machine) else {
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
        eprintln!("  once attached, `monitor help` lists rsemu's own commands");
        // Which devices became GDB threads, and — the hour-saving part — which
        // of them upstream GDB has no architecture for. A target description
        // tells GDB what the registers *are*; it still needs a gdbarch to know
        // what the machine *is*, and for a 6502 it has none.
        //
        // What it does then is worth being exact about, because it was
        // overstated here until somebody measured it (GDB 16.3):
        // `target remote` **succeeds**. GDB warns "Architecture rejected
        // target-supplied description", falls back to *its own host*
        // architecture's register layout, and then reads the `g` packet
        // through that layout — so a 6502 session connects and reports
        // "Truncated register 1 in remote 'g' packet. The program has no
        // registers now." A session that looks connected and answers nonsense
        // is worse than one that refuses, which is why this is printed before
        // anybody attaches.
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
                     this core, so it rejects the target description, falls back to its\n    \
                     own host layout, and reads the registers wrong. The protocol is\n    \
                     served in full to any client that reads the description rather than\n    \
                     insisting on a gdbarch.",
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

/// Serve the machine over VNC until the user stops it.
///
/// The frontend loop lives in `host::vnc::session`, not here: the binary's job
/// is to find the machine's screen, decide which input sinks it has, and hand
/// the loop a stop condition. Ctrl-C on the emulator's own terminal ends it.
///
/// **Which sinks a machine gets is decided by what it opened, not by a flag.**
/// A character port literally named `keyboard` is what `pc.kbc` opens, and a
/// pad port is what `nes.ports` opens; a machine with a serial console does not
/// get scan codes typed into it, because a serial console is not a keyboard.
#[cfg(feature = "vnc")]
fn vnc_session(machine: &mut Machine, args: &RunArgs, hosts: &HostObjects) -> ExitCode {
    use rsemu::host::vnc::{VncServer, VncSession};

    let addr = args.vnc.as_deref().unwrap_or(":5900");
    let Some(scanout) = take_scanout(hosts, machine) else {
        eprintln!("rsemu: --vnc: this machine has no display to serve");
        return ExitCode::from(2);
    };
    let server = match VncServer::bind(addr) {
        Ok(s) => s.named(machine.name()),
        Err(e) => {
            eprintln!("rsemu: --vnc {addr}: {e}");
            return ExitCode::from(2);
        }
    };
    let where_it_landed = match server.local_addr() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("rsemu: --vnc: {e}");
            return ExitCode::FAILURE;
        }
    };

    let mut session = VncSession::new(server, scanout);

    // The keyboard, if this machine has one.
    if let Ok(Some(port)) = rsemu::host::chardev::ports::get(hosts, "keyboard") {
        session = session.with_sink(Arc::new(rsemu::host::input::KeyboardSink::new(port)));
    }
    // The controllers, if it has those instead. Whichever console's they are:
    // all three families file their port under the same `pad` host kind, so the
    // loop this replaced — list the kind, ask for a NES-typed pad — got a Game
    // Boy's and a Master System's *names* and then failed the downcast in
    // silence, which is a screen with no buttons. `host::input::Pads` asks each
    // family for its own type.
    #[cfg(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms"))]
    if let Some(pad) = rsemu::host::input::PadSink::open(hosts, 0) {
        session = session.with_sink(Arc::new(pad));
    }

    // Recording and replaying are `core::record`'s, not this frontend's: what
    // the flags do is attach a recorder to the machine and register the
    // session's channel with it. The instant each event lands at is then the
    // machine's own round boundary rather than anything decided out here.
    let recorder = match (&args.record_input, &args.replay_input) {
        (Some(_), Some(_)) => {
            eprintln!("rsemu: --record-input and --replay-input are mutually exclusive");
            return ExitCode::from(2);
        }
        (Some(_), None) => Some(Arc::new(rsemu::core::record::Recorder::recording())),
        (None, Some(path)) => match std::fs::read(path) {
            Ok(bytes) => match rsemu::core::record::InputLog::decode(&bytes) {
                Ok(log) => {
                    if !args.quiet {
                        eprintln!("  replaying {} input events from {path}", log.len());
                    }
                    Some(Arc::new(rsemu::core::record::Recorder::replaying(log)))
                }
                Err(e) => {
                    eprintln!("rsemu: --replay-input {path}: {e}");
                    return ExitCode::from(2);
                }
            },
            Err(e) => {
                eprintln!("rsemu: --replay-input: cannot read {path}: {e}");
                return ExitCode::FAILURE;
            }
        },
        (None, None) => None,
    };
    if let Some(recorder) = &recorder {
        if let Err(e) = session.attach(recorder) {
            eprintln!("rsemu: {e}");
            return ExitCode::FAILURE;
        }
        // Refused outright under parallel threading, because a recording of one
        // could not be replayed (ROADMAP.md §4.2).
        if let Err(e) = machine.set_recorder(Arc::clone(recorder)) {
            eprintln!("rsemu: {e}");
            return ExitCode::from(2);
        }
    }

    // Sound, if the user asked for it. **This is where the headless
    // "make the ring big enough for the whole run" argument stops applying**:
    // the session drains the device every slice, so the ring holds one frame's
    // worth rather than the whole run and a recording is no longer capped at
    // about eighteen seconds. The queue limit is lifted for the same reason
    // `write_recording` lifts it — nothing may be trimmed on the way to a file.
    if args.record_audio.is_some() {
        let Some(source) = take_audio(hosts) else {
            eprintln!("rsemu: --record-audio: this machine has no audio device");
            return ExitCode::from(2);
        };
        let mut stream = rsemu::host::audio::AudioStream::new(
            source,
            args.audio_rate,
            rsemu::host::audio::SampleFormat::S16,
        );
        stream.set_limit_frames(u64::MAX);
        session = session.with_audio(stream);
    }

    if !args.quiet {
        eprintln!("  vnc://{where_it_landed} — Ctrl-C to stop\n");
    }

    let term = Terminal::open();
    let deadline = args
        .span_given
        .then(|| machine.now().saturating_add(args.span));
    let status = session.run(machine, |m| {
        !term.interrupted() && !deadline.is_some_and(|d| m.now() >= d)
    });
    drop(term);

    if let Err(e) = status {
        eprintln!("rsemu: {e}");
        return ExitCode::FAILURE;
    }
    if let (Some(path), Some(recorder)) = (&args.record_input, &recorder) {
        let bytes = match recorder.log().encode() {
            Ok(bytes) => bytes,
            Err(e) => {
                eprintln!("rsemu: --record-input {path}: {e}");
                return ExitCode::FAILURE;
            }
        };
        if let Err(e) = std::fs::write(path, bytes) {
            eprintln!("rsemu: --record-input {path}: {e}");
            return ExitCode::FAILURE;
        }
    }
    if let (Some(path), Some(stream)) = (&args.record_audio, session.audio()) {
        let bytes = rsemu::host::audio::wav::encode(stream.info(), stream.buffer());
        if let Err(e) = std::fs::write(path, &bytes) {
            eprintln!("rsemu: --record-audio {path}: {e}");
            return ExitCode::FAILURE;
        }
        if !args.quiet {
            let frames = stream.buffer().frames();
            println!(
                "audio       {path} ({} Hz, {frames} frames, {} bytes)",
                args.audio_rate,
                bytes.len()
            );
        }
    }
    if !args.quiet {
        summarise(machine);
    }
    ExitCode::SUCCESS
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

/// The system firmware for a machine whose `bios` slot the user left unbound.
///
/// Only `pc-at` has one, and only because `ROADMAP.md` phase 6a said it must:
/// every legacy PC BIOS anyone could otherwise reach for is GPL, and running
/// one is fine while shipping one is not. `--bios` overrides this, so pointing
/// the board at a real image is still one flag.
fn builtin_bios(machine: &str) -> Option<Vec<u8>> {
    let stem = machine
        .rsplit('/')
        .next()
        .unwrap_or(machine)
        .strip_suffix(".machine")
        .unwrap_or_else(|| machine.rsplit('/').next().unwrap_or(machine));
    match stem {
        #[cfg(all(feature = "fw-pcbios", feature = "machine-pc-at"))]
        "pc-at" => Some(rsemu::fw::pcbios::image()),
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
        #[cfg(feature = "dev-blk")]
        drives: Vec::new(),
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
        #[cfg(feature = "vnc")]
        vnc: None,
        #[cfg(feature = "vnc")]
        record_input: None,
        #[cfg(feature = "vnc")]
        replay_input: None,
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
            #[cfg(feature = "dev-blk")]
            "--drive" => {
                let spec = value(arg)?;
                out.drives.push(Drive::parse(&spec)?);
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
            #[cfg(feature = "vnc")]
            "--vnc" => out.vnc = Some(value(arg)?),
            #[cfg(feature = "vnc")]
            "--record-input" => out.record_input = Some(value(arg)?),
            #[cfg(feature = "vnc")]
            "--replay-input" => out.replay_input = Some(value(arg)?),
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
