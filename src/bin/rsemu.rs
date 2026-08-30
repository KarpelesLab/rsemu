//! The `rsemu` command-line tool.
//!
//! The subcommand surface is fixed by `ROADMAP.md` §2 so that it does not drift
//! as components land. Commands whose machinery does not exist yet say exactly
//! that and exit non-zero, rather than pretending to work.

use std::process::ExitCode;

const USAGE: &str = "\
rsemu — a multiplatform emulator built bottom-up on a generic framework

USAGE:
    rsemu <COMMAND> [OPTIONS]

COMMANDS:
    run <machine>       Run a machine description
    machines            List machines this build can emulate
    devices             List registered device classes
    describe <class>    Show a device class: properties, defaults, buses
    convert <machine>   Convert a machine file between its text and JSON forms

OPTIONS:
    -h, --help          Print this help
    -V, --version       Print version and build configuration

Nothing is emulated yet — see ROADMAP.md for the phase plan. Commands that
need machinery which does not exist report that and exit 2.
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
        "machines" => {
            // A machine is a feature set, so an empty list is the correct
            // answer for this build rather than a failure.
            println!("no machines in this build");
            ExitCode::SUCCESS
        }
        "devices" => {
            println!("no device classes in this build");
            ExitCode::SUCCESS
        }
        "run" | "describe" | "convert" => {
            let what: &'static str = match first {
                "run" => "running a machine (ROADMAP.md phases 1-3)",
                "describe" => "the device registry (ROADMAP.md §4.4)",
                _ => "the machine description language (ROADMAP.md §5)",
            };
            eprintln!("rsemu: {}", rsemu::Error::Unimplemented(what));
            ExitCode::from(2)
        }
        other => {
            eprintln!("rsemu: unknown command `{other}`\n");
            eprint!("{USAGE}");
            ExitCode::from(2)
        }
    }
}
