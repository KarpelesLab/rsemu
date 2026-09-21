//! The monitor console: a stopped machine you can ask questions of, and steer
//! (`ROADMAP.md` §8, "Console/monitor", and phase 9).
//!
//! A gdbstub answers questions a *debugger* asks — registers, memory,
//! breakpoints — because that is the vocabulary the protocol has. A monitor
//! answers the questions this emulator can answer and GDB has no packet for:
//! what devices are in this machine, what is mapped where, what every clock
//! domain's rate and position are, what the wire graph settles at, what the
//! scheduler has queued, what the counters say, and how to go back.
//!
//! # The command engine is not the front end
//!
//! [`Monitor::execute`] takes a line and returns text plus a [`Flow`]. It never
//! reads, never writes, never sleeps and never touches a terminal, so the whole
//! command set is testable by calling it — which is what
//! `src/host/monitor/tests.rs` does, and why there is no fake TTY anywhere in
//! this module. `src/bin/rsemu.rs` supplies the loop: read a line, execute it,
//! print the text, and act on the flow.
//!
//! # The scheduler still owns time
//!
//! Nothing here advances a machine except [`Monitor::advance`], and that calls
//! [`Machine::run_until`](crate::machine::Machine::run_until) in ten-millisecond
//! slices — the same call, and the same slice, as `rsemu run --headless`.
//! `run_until` is additive (`ROADMAP.md` §11.6), so a span taken in pieces and
//! the same span taken whole reach the same state: a session that types
//! `run 1s` and a headless `--for 1s` land on one state hash, which
//! `tests/cli_monitor.rs` asserts.
//!
//! `step` is the one exception and it is labelled as one: it is the debugger's
//! stepper ([`DebugTarget::step`]), which splits a scheduling round and is
//! therefore *not* additive. That is the price of stepping an instruction at a
//! time, it is the gdbstub's price too, and a session that steps is no longer
//! comparable with a headless run — which the `status` line says out loud.
//!
//! # Looking never changes anything
//!
//! Every memory read goes through [`MachineTarget`], whose single attribute
//! constructor starts from [`MemAttrs::DEBUG`](crate::core::space::MemAttrs).
//! That is the whole reason `MemAttrs::debug` exists: an inspection must not pop
//! a FIFO, clear a status bit or advance a pointer. `tests/cli_monitor.rs`
//! checks it against a device that would otherwise do exactly that.
//!
//! # Why some commands are answered somewhere else
//!
//! Eight of the commands below — `devices`, `spaces`, `map`, `x`, `xp`,
//! `translate`, `time`, `hash` — were already implemented for GDB's `qRcmd`
//! packet, against the same [`MachineTarget`]. This console **forwards** any
//! line it does not own to [`DebugTarget::monitor`] rather than reimplementing
//! them, so `monitor map` typed into gdb and `map` typed at this prompt are one
//! piece of code with one answer. A second copy would be a second thing to keep
//! honest, and the drift would be invisible until somebody compared the two.
//! `every_command_the_gdbstub_already_answers_is_reachable_here` is the test
//! that holds that door open.

#[cfg(test)]
mod tests;

use core::fmt::Write as _;

use crate::core::clock::GlobalTime;
use crate::core::hosts::HostObjects;
use crate::core::props::parse_duration;
use crate::core::sched::EventTarget;
use crate::core::trace::Channel;
use crate::machine::{Machine, Timeline};

use super::gdb::{DebugTarget, MachineTarget};

/// How much virtual time one slice of an advance covers.
///
/// Ten milliseconds, which is what `run_headless`, the console loop and the
/// gdbstub's free-running slice all use. The number matters only for how often
/// the host gets a turn — `Machine::run_until` is additive, so slicing changes
/// nothing about where the machine ends up.
const SLICE: GlobalTime = GlobalTime::from_nanos(10_000_000);

/// The most bytes a `write` command will take in one line.
///
/// A monitor write is for patching a word or a vector, not for loading an
/// image; `rsemu run --media` is how bytes get into a machine in bulk.
const WRITE_MAX: usize = 1024;

/// The most queued events `sched` lists before it stops and says how many are
/// left.
const EVENTS_SHOWN: usize = 32;

/// The most characters of a device's own `Debug` rendering `device` prints.
///
/// A device that holds a framebuffer or a disk image renders megabytes, and a
/// prompt that scrolls for a minute has answered nothing. The cut is announced
/// rather than silent.
const STATE_MAX: usize = 8192;

/// What the session should do after a command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Flow {
    /// Print the text and come back to the prompt.
    Stay,
    /// Leave the session. End of input means the same thing.
    Quit,
    /// Advance the machine: `Some(span)` of virtual time, or — for `cont` —
    /// `None`, meaning until the session is interrupted or reaches the deadline
    /// `--for` gave it.
    Advance(Option<GlobalTime>),
}

/// What one command produced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Response {
    /// What to print. Ends in a newline when it is not empty.
    pub text: String,
    /// What to do next.
    pub flow: Flow,
}

impl Response {
    /// A command that printed something and wants the prompt back.
    fn stay(text: impl Into<String>) -> Response {
        Response {
            text: text.into(),
            flow: Flow::Stay,
        }
    }
}

/// Everything a command may need that is not the machine itself.
///
/// A struct rather than four arguments because three of the four are optional
/// and a call site with `None, None, Some(x), None` in it reads as nothing at
/// all. Each field is `None` in a session that has no such thing, and the
/// command that wanted it says which flag would have provided it.
#[derive(Debug, Default)]
pub struct Env<'a> {
    /// The build's host-object table, which is where the trace collector finds
    /// the per-CPU counters.
    pub hosts: Option<&'a HostObjects>,
    /// The rewind history, when the session is keeping one.
    pub timeline: Option<&'a mut Timeline>,
    /// Where `--for` says the session ends, if it said.
    ///
    /// `cont` runs to here; without one it runs until the session is
    /// interrupted.
    pub deadline: Option<GlobalTime>,
}

/// The console's own state: which processor commands act on, and whether the
/// session has stepped.
#[derive(Debug, Default)]
pub struct Monitor {
    cpu: usize,
    /// Whether [`DebugTarget::step`] has been used in this session.
    ///
    /// Kept because it is the one thing that makes a session's final state
    /// incomparable with a headless run, and a `status` that did not say so
    /// would be inviting somebody to paste a hash into a test.
    stepped: bool,
}

impl Monitor {
    /// A fresh console, looking at processor 0.
    #[must_use]
    pub fn new() -> Monitor {
        Monitor::default()
    }

    /// Which processor `regs`, `step`, `x` and `translate` act on.
    #[must_use]
    pub fn cpu(&self) -> usize {
        self.cpu
    }

    /// Whether this session has used the debugger's stepper.
    ///
    /// A stepped session has split a scheduling round, so its state hash is no
    /// longer comparable with a headless run of the same span. `status` says so
    /// once this is true.
    #[must_use]
    pub fn has_stepped(&self) -> bool {
        self.stepped
    }

    /// What to print before reading a line.
    #[must_use]
    pub fn prompt(&self) -> &'static str {
        "(rsemu) "
    }

    /// The two lines a session opens with.
    #[must_use]
    pub fn banner(&self, target: &MachineTarget<'_>) -> String {
        let machine = target.machine();
        format!(
            "monitor attached to \"{}\" — the machine is stopped at {} ns\n\
             {} processor{}; `help` lists what this console can answer\n",
            machine.name(),
            machine.now().as_nanos(),
            target.cpu_count(),
            if target.cpu_count() == 1 { "" } else { "s" },
        )
    }

    /// Run one command line.
    ///
    /// A blank line and a `#` comment are both nothing, so a script piped into
    /// the session can be annotated. Anything this console does not recognise is
    /// offered to the debug target before it is refused — see the module docs.
    pub fn execute(
        &mut self,
        target: &mut MachineTarget<'_>,
        env: &mut Env<'_>,
        line: &str,
    ) -> Response {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            return Response::stay(String::new());
        }
        let mut words = line.split_whitespace();
        let command = words.next().unwrap_or("");
        let args: Vec<&str> = words.collect();

        match command {
            "help" | "?" => Response::stay(help(args.first().copied())),
            "quit" | "q" | "exit" => Response {
                text: String::new(),
                flow: Flow::Quit,
            },
            "status" => Response::stay(self.status(target)),

            "run" => match span_of(args.first().copied()) {
                Ok(span) => Response {
                    text: String::new(),
                    flow: Flow::Advance(Some(span)),
                },
                Err(e) => Response::stay(e),
            },
            "cont" | "c" => match args.first().copied() {
                // Unbounded, and it says so. A `cont` with no span is the
                // interactive spelling and ends when a person presses Ctrl-C;
                // a session driven from a pipe has nobody to press it, so the
                // line that would otherwise be an unexplained silence names the
                // two things that do end it.
                None if env.deadline.is_none() => Response {
                    text: String::from(
                        "running until interrupted — Ctrl-C stops it, and `--for` bounds it\n",
                    ),
                    flow: Flow::Advance(None),
                },
                None => Response {
                    text: String::new(),
                    flow: Flow::Advance(None),
                },
                Some(text) => match span_of(Some(text)) {
                    Ok(span) => Response {
                        text: String::new(),
                        flow: Flow::Advance(Some(span)),
                    },
                    Err(e) => Response::stay(e),
                },
            },
            "step" => Response::stay(self.step(target, args.first().copied())),
            "cpu" => Response::stay(self.select(target, args.first().copied())),
            "cpus" => Response::stay(self.cpus(target)),
            "regs" => Response::stay(self.regs(target, args.first().copied())),
            "sched" => Response::stay(sched(target.machine())),

            "device" => Response::stay(device(target.machine(), args.first().copied())),
            "media" => Response::stay(media(target.machine())),
            "insert" => Response::stay(insert(target.machine(), &args)),
            "eject" => Response::stay(eject(target.machine(), args.first().copied())),
            "clocks" => Response::stay(clocks(target.machine())),
            "wires" => Response::stay(wires(target.machine())),

            "write" => Response::stay(self.write(target, &args, false)),
            "writep" => Response::stay(self.write(target, &args, true)),

            "save" => Response::stay(save(target.machine(), args.first().copied())),
            "load" => Response::stay(load(target.machine_mut(), args.first().copied())),
            "timeline" => Response::stay(timeline(env)),
            "rewind" => Response::stay(rewind(target, env, args.first().copied())),

            "trace" => Response::stay(trace(target.machine(), env, &args)),

            // Not ours. The gdbstub's `qRcmd` set is answered by the same
            // target, so `devices`, `spaces`, `map`, `x`, `xp`, `translate`,
            // `time` and `hash` land here and are answered there.
            _ => match target.monitor(self.cpu, line) {
                Some(text) => Response::stay(text),
                None => {
                    Response::stay(format!("unknown command `{command}` — `help` lists them\n"))
                }
            },
        }
    }

    /// Advance the machine, in slices, letting the caller do its own work
    /// between them.
    ///
    /// `keep_going` is called once a slice and is where the session pumps a
    /// character port, drains a capture and checks whether a signal has arrived;
    /// returning false stops the advance where it is. `span` of `None` is
    /// `cont`: run to [`Env::deadline`] if there is one, and otherwise until
    /// `keep_going` says stop.
    pub fn advance(
        &mut self,
        target: &mut MachineTarget<'_>,
        env: &mut Env<'_>,
        span: Option<GlobalTime>,
        mut keep_going: impl FnMut(&mut Machine) -> bool,
    ) -> String {
        let start = target.machine().now();
        let end = match span {
            Some(span) => Some(start.saturating_add(span)),
            None => env.deadline,
        };
        // A `cont` in a session that already passed its `--for` deadline is not
        // an error and not a no-op worth reporting twice; it just has nothing
        // left to do.
        let mut out = String::new();
        loop {
            let now = target.machine().now();
            if end.is_some_and(|end| now >= end) {
                break;
            }
            if !keep_going(target.machine_mut()) {
                break;
            }
            let next = match end {
                Some(end) => end.min(now.saturating_add(SLICE)),
                None => now.saturating_add(SLICE),
            };
            // The timeline, when the session keeps one, drives the same
            // `run_until` underneath and takes a keyframe on the round
            // boundaries its cadence falls on. Going through it rather than
            // around it is what makes `rewind` able to reach this span later.
            let stepped = match env.timeline.as_mut() {
                Some(timeline) => timeline.run_until(target.machine_mut(), next),
                None => target.machine_mut().run_until(next),
            };
            if let Err(e) = stepped {
                let _ = writeln!(out, "rsemu: {e}");
                break;
            }
        }
        let now = target.machine().now();
        let _ = writeln!(
            out,
            "stopped at {} ns (+{} ns)",
            now.as_nanos(),
            now.as_nanos().saturating_sub(start.as_nanos())
        );
        out
    }

    /// `status` — where the machine is and what can still be said about it.
    fn status(&self, target: &MachineTarget<'_>) -> String {
        let machine = target.machine();
        let mut out = String::new();
        let _ = writeln!(out, "machine    \"{}\"", machine.name());
        let _ = writeln!(out, "stopped at {} ns", machine.now().as_nanos());
        let _ = writeln!(out, "threading  {}", machine.threading_mode());
        let _ = writeln!(
            out,
            "processors {} (selected: {})",
            target.cpu_count(),
            self.cpu
        );
        let _ = writeln!(out, "devices    {}", machine.devices().len());
        match machine.state_hash() {
            Ok(hash) => {
                let _ = writeln!(out, "state hash {hash:#018x}");
            }
            Err(e) => {
                let _ = writeln!(out, "state hash unavailable: {e}");
            }
        }
        if self.stepped {
            // Said once the session has actually done it, because until then
            // the hash above *is* comparable and a standing warning would train
            // people to ignore it.
            let _ = writeln!(
                out,
                "note       this session has used `step`, which splits a scheduling round;\n\
                 \x20          the hash above is no longer comparable with a headless run"
            );
        }
        out
    }

    /// `step [n]` — the debugger's stepper, on the selected processor.
    fn step(&mut self, target: &mut MachineTarget<'_>, count: Option<&str>) -> String {
        let count: u64 = match count {
            None => 1,
            Some(text) => match text.parse() {
                Ok(0) | Err(_) => return format!("`{text}` is not a step count\n"),
                Ok(n) => n,
            },
        };
        if target.cpu_count() == 0 {
            return String::from("this machine has no processor a debugger can step\n");
        }
        let mut out = String::new();
        for _ in 0..count {
            match target.step(self.cpu) {
                Ok(stop) => {
                    self.stepped = true;
                    let _ = stop;
                }
                Err(e) => {
                    let _ = writeln!(out, "cannot step: {e}");
                    return out;
                }
            }
        }
        let _ = writeln!(
            out,
            "stepped {count} instruction{} on cpu {}",
            if count == 1 { "" } else { "s" },
            self.cpu
        );
        out.push_str(&self.where_is(target, self.cpu));
        out
    }

    /// `cpu [n]` — show, or change, which processor the per-CPU commands use.
    fn select(&mut self, target: &MachineTarget<'_>, which: Option<&str>) -> String {
        let Some(text) = which else {
            return self.where_is(target, self.cpu);
        };
        let Ok(index) = text.parse::<usize>() else {
            return format!("`{text}` is not a processor number\n");
        };
        if index >= target.cpu_count() {
            return format!(
                "there is no processor {index}; this machine has {}\n",
                target.cpu_count()
            );
        }
        self.cpu = index;
        self.where_is(target, index)
    }

    /// `cpus` — one line per processor.
    fn cpus(&self, target: &MachineTarget<'_>) -> String {
        if target.cpu_count() == 0 {
            return String::from("no processor in this machine has a register map in this build\n");
        }
        let mut out = String::new();
        for index in 0..target.cpu_count() {
            let _ = write!(out, "{} ", if index == self.cpu { '*' } else { ' ' });
            out.push_str(&self.where_is(target, index));
        }
        out
    }

    /// One processor's line: index, path, class, and where it is.
    fn where_is(&self, target: &MachineTarget<'_>, index: usize) -> String {
        let Ok(path) = target.cpu_path(index) else {
            return format!("there is no processor {index}\n");
        };
        let path = path.to_string();
        let Ok(arch) = target.arch(index) else {
            return format!("{index}  {path}\n");
        };
        let pc = match target.read_register(index, arch.pc) {
            Ok(bytes) => hex_of(&bytes),
            Err(e) => format!("(pc unreadable: {e})"),
        };
        let class = arch.class.name;
        format!("{index}  {path:<12} {class:<16} pc {pc}\n")
    }

    /// `regs [name]` — the selected processor's registers.
    fn regs(&self, target: &MachineTarget<'_>, only: Option<&str>) -> String {
        let arch = match target.arch(self.cpu) {
            Ok(arch) => arch,
            Err(e) => return format!("cpu {}: {e}\n", self.cpu),
        };
        let mut out = String::new();
        let mut column = 0;
        let mut found = false;
        for (index, reg) in arch.regs.iter().enumerate() {
            if only.is_some_and(|name| !name.eq_ignore_ascii_case(reg.name)) {
                continue;
            }
            found = true;
            let value = match target.read_register(self.cpu, index) {
                Ok(bytes) => hex_of(&bytes),
                Err(e) => format!("({e})"),
            };
            let name = reg.name;
            let _ = write!(out, "{name:<8} {value:<20}");
            column += 1;
            if column % 3 == 0 {
                out.push('\n');
            }
        }
        if column % 3 != 0 {
            out.push('\n');
        }
        if !found {
            let name = only.unwrap_or("");
            return format!(
                "cpu {} has no register `{name}`; `regs` lists them\n",
                self.cpu
            );
        }
        out
    }

    /// `write`/`writep` — patch guest memory, virtual or physical.
    fn write(&self, target: &mut MachineTarget<'_>, args: &[&str], physical: bool) -> String {
        let Some(addr) = args.first().copied() else {
            return String::from("an address and at least one byte are needed\n");
        };
        let addr = match addr_of(addr) {
            Ok(addr) => addr,
            Err(e) => return e,
        };
        let bytes = match bytes_of(&args[1..]) {
            Ok(bytes) => bytes,
            Err(e) => return e,
        };
        let wrote = if physical {
            target.write_physical(self.cpu, addr, &bytes)
        } else {
            target.write_memory(self.cpu, addr, &bytes)
        };
        match wrote {
            Ok(()) => format!("wrote {} bytes at {addr:#x}\n", bytes.len()),
            Err(e) => format!("cannot write at {addr:#x}: {e}\n"),
        }
    }
}

// ---------------------------------------------------------------------------
// commands that need no per-session state
// ---------------------------------------------------------------------------

/// `sched` — the scheduler's own view.
fn sched(machine: &Machine) -> String {
    let scheduler = machine.scheduler();
    let config = scheduler.config();
    let queue = scheduler.queue();
    let mut out = String::new();
    let _ = writeln!(out, "now       {} ns", machine.now().as_nanos());
    let _ = writeln!(out, "threading {}", config.mode);
    let _ = writeln!(out, "quantum   {} ns", config.quantum.as_nanos());
    let _ = writeln!(out, "queued    {} event(s)", queue.len());
    let events = queue.events();
    for event in events.iter().take(EVENTS_SHOWN) {
        let _ = writeln!(
            out,
            "  {:>16} ns  {}  token {}",
            event.time.as_nanos(),
            target_path(machine, event.target),
            event.token
        );
    }
    if events.len() > EVENTS_SHOWN {
        let _ = writeln!(out, "  … and {} more", events.len() - EVENTS_SHOWN);
    }
    out
}

/// The instance path an event is bound for.
///
/// [`EventTarget`] is an index into the device list, which is meaningless to
/// read and trivial to resolve.
fn target_path(machine: &Machine, target: EventTarget) -> String {
    match machine.devices().get(target.0 as usize) {
        Some(entry) => entry.path().to_string(),
        None => format!("device #{}", target.0),
    }
}

/// `device <path>` — one device in full.
fn device(machine: &Machine, path: Option<&str>) -> String {
    let Some(path) = path else {
        return String::from("a device path is needed; `devices` lists them\n");
    };
    let Some(entry) = machine.device(path) else {
        return format!("no device at `{path}`; `devices` lists them\n");
    };
    let class = entry.class();
    let mut out = String::new();
    let _ = writeln!(out, "path       {}", entry.path());
    let _ = writeln!(out, "class      {} v{}", class.name, class.version);
    let _ = writeln!(out, "           {}", class.summary);
    match entry.domain() {
        Some(domain) => {
            let rate = machine
                .clocks()
                .domain_frequency(domain)
                .map_or_else(|_| String::from("?"), hz_of);
            let ticks = machine.clocks().ticks(domain).unwrap_or(0);
            let _ = writeln!(out, "clock      {rate}, {ticks} ticks");
        }
        None => {
            let _ = writeln!(out, "clock      none (not clocked)");
        }
    }
    match entry.space_index().and_then(|i| machine.spaces().get(i)) {
        Some(space) => {
            let _ = writeln!(
                out,
                "space      {} ({} bits)",
                space.name(),
                space.space().bits()
            );
        }
        None => {
            let _ = writeln!(out, "space      none");
        }
    }
    let _ = writeln!(out, "requester  {}", entry.requester().0);
    if entry.runnable().is_some() {
        let _ = writeln!(out, "runnable   yes");
    }
    if entry.lazy().is_some() {
        let _ = writeln!(out, "lazy       yes");
    }
    if class.properties.is_empty() {
        let _ = writeln!(out, "properties none declared");
    } else {
        let _ = writeln!(out, "properties");
        for spec in class.properties {
            let _ = writeln!(
                out,
                "  {:<14} {:<8} {}{}",
                spec.name,
                spec.kind.as_str(),
                spec.summary,
                if spec.required { " (required)" } else { "" }
            );
        }
    }
    // The device's own `Debug`, which every device in the tree derives and
    // which is therefore the one "current state" view that works on every
    // machine in the catalog. `CLAUDE.md` requires the derive; this is what
    // makes it earn its keep.
    let state = format!("{:#?}", entry.device());
    let _ = writeln!(out, "state");
    if state.len() > STATE_MAX {
        for line in state[..STATE_MAX].lines() {
            let _ = writeln!(out, "  {line}");
        }
        let _ = writeln!(
            out,
            "  … {} more characters; a snapshot (`save`) is the whole of it",
            state.len() - STATE_MAX
        );
    } else {
        for line in state.lines() {
            let _ = writeln!(out, "  {line}");
        }
    }
    out
}

// ---------------------------------------------------------------------------
// media: what is in the drives, and changing it
// ---------------------------------------------------------------------------

/// How a bay is named at this prompt.
///
/// `df0` when the device at that path has exactly one bay, which is every
/// drive in the tree today; `df0:disk` always. Two spellings rather than one
/// because "the disk in df0" is what a person means and `df0:disk` is what a
/// machine with a two-bay drive would need, and neither is worth losing.
///
/// The device path is the *front* of it rather than the back on purpose: it is
/// what `devices` already prints, so a session reads one listing and types out
/// of it.
#[cfg(feature = "dev-medium")]
fn find_bay(
    machine: &Machine,
    spec: &str,
) -> Result<(crate::dev::medium::MediaPort, String), String> {
    use crate::dev::medium;

    let (path, wanted) = match spec.split_once(':') {
        Some((path, bay)) => (path, Some(bay)),
        None => (spec, None),
    };
    let port = medium::attached_at(machine, path).map_err(|e| format!("{e}\n"))?;
    let bays = port.bays();
    match wanted {
        Some(name) => {
            if bays.iter().any(|b| b.name == name) {
                Ok((port, String::from(name)))
            } else {
                Err(format!(
                    "`{path}` has no bay called `{name}`; it has {}\n",
                    names_of(&bays)
                ))
            }
        }
        None if bays.len() == 1 => Ok((port, bays[0].name.clone())),
        None => Err(format!(
            "`{path}` has {} bays ({}); name one as `{path}:<bay>`\n",
            bays.len(),
            names_of(&bays)
        )),
    }
}

/// The bay names of a device, for a message that has to list them.
#[cfg(feature = "dev-medium")]
fn names_of(bays: &[crate::dev::medium::MediaBay]) -> String {
    bays.iter()
        .map(|b| format!("`{}`", b.name))
        .collect::<Vec<_>>()
        .join(", ")
}

/// `media` — every removable bay in the machine and what is in it.
#[cfg(feature = "dev-medium")]
fn media(machine: &Machine) -> String {
    let attached = crate::dev::medium::attached(machine);
    if attached.is_empty() {
        return String::from(
            "this machine has no removable media — no device in it publishes a bay\n",
        );
    }
    let mut out = String::new();
    for device in &attached {
        let bays = device.port.bays();
        let one = bays.len() == 1;
        for bay in bays {
            let name = if one {
                device.path.clone()
            } else {
                format!("{}:{}", device.path, bay.name)
            };
            match bay.medium {
                None => {
                    let _ = writeln!(out, "{name:<12} empty      {}", bay.summary);
                }
                Some(held) => {
                    let _ = writeln!(
                        out,
                        "{name:<12} {:<10} {} ({} bytes)",
                        if held.write_protected {
                            "protected"
                        } else {
                            "writable"
                        },
                        held.describe,
                        held.capacity
                    );
                }
            }
        }
    }
    out
}

/// `insert <bay> <file> [ro]` — put a medium in, and let the guest know.
#[cfg(feature = "dev-medium")]
fn insert(machine: &Machine, args: &[&str]) -> String {
    let (Some(spec), Some(file)) = (args.first().copied(), args.get(1).copied()) else {
        return String::from("`insert <bay> <file> [ro]`; `media` lists the bays\n");
    };
    let protect = match args.get(2).copied() {
        None => false,
        Some("ro") => true,
        Some(other) => return format!("`{other}` is not an insert option; the only one is `ro`\n"),
    };
    let (port, bay) = match find_bay(machine, spec) {
        Ok(found) => found,
        Err(e) => return e,
    };
    // The same reader `--media` uses, so a scheme works here too: `insert df0
    // adf:/path/to/disks.iso,disk=Workbench` is the whole point of it being
    // one function rather than a `std::fs::read`.
    let loaded = match crate::host::media::read(file) {
        Ok(loaded) => loaded,
        Err(e) => return format!("{e}\n"),
    };
    let described = loaded.note.clone().unwrap_or_else(|| String::from(file));
    let bytes = crate::dev::medium::from_bytes(&loaded.bytes);
    match port.insert(&bay, bytes, protect) {
        Ok(()) => format!(
            "{spec}: {described}\n\
             the drive has raised its disk-change signal; the guest sees it at its own pace\n"
        ),
        Err(e) => format!("{e}\n"),
    }
}

/// `eject <bay>` — take the medium out, and let the guest know.
#[cfg(feature = "dev-medium")]
fn eject(machine: &Machine, spec: Option<&str>) -> String {
    let Some(spec) = spec else {
        return String::from("`eject <bay>`; `media` lists the bays\n");
    };
    let (port, bay) = match find_bay(machine, spec) {
        Ok(found) => found,
        Err(e) => return e,
    };
    match port.eject(&bay) {
        Ok(()) => format!(
            "{spec}: empty\n\
             the drive has raised its disk-change signal; the guest sees it at its own pace\n"
        ),
        Err(e) => format!("{e}\n"),
    }
}

/// What the three commands say in a build with no storage seam at all.
///
/// A NES build has no `dev-medium` and therefore no `Medium` to put anywhere,
/// so the commands still answer rather than falling through to "unknown
/// command" — which would send somebody looking for a typo.
#[cfg(not(feature = "dev-medium"))]
fn media(_machine: &Machine) -> String {
    String::from(NO_MEDIUM)
}

#[cfg(not(feature = "dev-medium"))]
fn insert(_machine: &Machine, _args: &[&str]) -> String {
    String::from(NO_MEDIUM)
}

#[cfg(not(feature = "dev-medium"))]
fn eject(_machine: &Machine, _spec: Option<&str>) -> String {
    String::from(NO_MEDIUM)
}

#[cfg(not(feature = "dev-medium"))]
const NO_MEDIUM: &str =
    "this build has no storage seam at all; rebuild with the `dev-medium` feature\n";

/// `clocks` — the forest, one line per domain.
fn clocks(machine: &Machine) -> String {
    let forest = machine.clocks();
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:<16} {:<18} {:>16} {:>10}  gated",
        "domain", "rate", "ticks", "lead"
    );
    for id in forest.domains() {
        let Ok(domain) = forest.domain(id) else {
            continue;
        };
        let rate = forest
            .domain_frequency(id)
            .map_or_else(|_| String::from("?"), hz_of);
        let ticks = forest.ticks(id).unwrap_or(0);
        let lead = forest.lead(id).unwrap_or(0);
        let gated = forest.is_gated(id).unwrap_or(false);
        let _ = writeln!(
            out,
            "{:<16} {rate:<18} {ticks:>16} {lead:>10}  {}",
            domain.name(),
            if gated { "yes" } else { "no" }
        );
    }
    out
}

/// `wires` — the nets, their drivers and what each settles at.
fn wires(machine: &Machine) -> String {
    if machine.nets().is_empty() {
        return String::from("this machine has no wires\n");
    }
    let mut out = String::new();
    let _ = writeln!(out, "{:<6} {:<8} {:>6}  driven by", "net", "level", "sinks");
    for (index, net) in machine.nets().iter().enumerate() {
        let wire = net.wire();
        let drivers: Vec<String> = net
            .sources()
            .iter()
            .map(|pin| match machine.devices().get(pin.device) {
                Some(entry) => format!("{}.{}", entry.path(), pin.port),
                None => format!("#{}.{}", pin.device, pin.port),
            })
            .collect();
        let _ = writeln!(
            out,
            "{index:<6} {:<8} {:>6}  {}",
            if wire.resolve_net().is_high() {
                "high"
            } else {
                "low"
            },
            wire.sink_count(),
            if drivers.is_empty() {
                String::from("(nothing)")
            } else {
                drivers.join(", ")
            }
        );
    }
    out
}

/// `save <file>` — a machine snapshot on the host's disk.
fn save(machine: &Machine, path: Option<&str>) -> String {
    let Some(path) = path else {
        return String::from("a file to write the snapshot to is needed\n");
    };
    let bytes = match machine.save() {
        Ok(bytes) => bytes,
        Err(e) => return format!("cannot take a snapshot: {e}\n"),
    };
    match std::fs::write(path, &bytes) {
        Ok(()) => format!("wrote {} bytes to {path}\n", bytes.len()),
        Err(e) => format!("cannot write {path}: {e}\n"),
    }
}

/// `load <file>` — restore one.
fn load(machine: &mut Machine, path: Option<&str>) -> String {
    let Some(path) = path else {
        return String::from("a snapshot file to load is needed\n");
    };
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(e) => return format!("cannot read {path}: {e}\n"),
    };
    match machine.load(&bytes) {
        // A snapshot of a differently shaped machine fails with a diff rather
        // than a crash (§4.5), and the diff is the whole value of the message —
        // so it is passed through verbatim.
        Err(e) => format!("cannot load {path}: {e}\n"),
        Ok(()) => format!(
            "loaded {} bytes; the machine is at {} ns\n",
            bytes.len(),
            machine.now().as_nanos()
        ),
    }
}

/// `timeline` — what the rewind history holds.
fn timeline(env: &Env<'_>) -> String {
    let Some(timeline) = env.timeline.as_ref() else {
        return String::from(
            "this session keeps no timeline; rewind needs deterministic threading\n",
        );
    };
    let mut out = String::new();
    let _ = writeln!(out, "cadence   {} ns", timeline.cadence().as_nanos());
    let _ = writeln!(
        out,
        "keyframes {} ({} bytes held)",
        timeline.keyframes(),
        timeline.bytes_held()
    );
    for instant in timeline.instants() {
        let _ = writeln!(out, "  {} ns", instant.as_nanos());
    }
    out
}

/// `rewind <span>` — go back, by restoring the nearest keyframe and replaying.
fn rewind(target: &mut MachineTarget<'_>, env: &mut Env<'_>, span: Option<&str>) -> String {
    let span = match span_of(span) {
        Ok(span) => span,
        Err(e) => return e,
    };
    let Some(timeline) = env.timeline.as_mut() else {
        return String::from(
            "this session keeps no timeline; rewind needs deterministic threading\n",
        );
    };
    let now = target.machine().now();
    let at = now.saturating_sub(span);
    match timeline.rewind_to(target.machine_mut(), at) {
        // The landing instant and the instant asked for are both printed even
        // when they match: a rewind lands on the *nearest keyframe at or
        // before* what was asked, and somebody reading one number would have no
        // way to tell whether it had.
        Ok(landed) => format!(
            "rewound to {} ns (asked for {} ns)\n",
            landed.as_nanos(),
            at.as_nanos()
        ),
        Err(e) => format!("cannot rewind to {} ns: {e}\n", at.as_nanos()),
    }
}

/// `trace [channel…]` — the counters, now, in the same two columns `--trace`
/// writes at the end of a run.
fn trace(machine: &Machine, env: &Env<'_>, args: &[&str]) -> String {
    if !super::trace::available() {
        return String::from("this build has no counters; rebuild with the `trace` feature\n");
    }
    let Some(hosts) = env.hosts else {
        return String::from("this session has no host-object table to collect from\n");
    };
    let channels: Vec<Channel> = if args.is_empty() {
        Channel::ALL.to_vec()
    } else {
        let mut chosen = Vec::new();
        for name in args {
            match Channel::from_name(name) {
                Some(channel) => chosen.push(channel),
                None => {
                    return format!(
                        "`{name}` is not a channel; they are {}\n",
                        Channel::ALL
                            .iter()
                            .map(|c| c.name())
                            .collect::<Vec<_>>()
                            .join(", ")
                    );
                }
            }
        }
        chosen
    };
    super::trace::collect(machine, hosts, &channels).render()
}

// ---------------------------------------------------------------------------
// parsing and rendering
// ---------------------------------------------------------------------------

/// A span, as every command that takes one writes it.
///
/// The unit is mandatory, exactly as it is for `--for`: a bare number would
/// mean whichever unit this module happened to pick, which is how a one-second
/// step becomes a one-millisecond one.
fn span_of(text: Option<&str>) -> Result<GlobalTime, String> {
    let Some(text) = text else {
        return Err(String::from(
            "a span is needed, with its unit: `1s`, `500ms`, `20us`\n",
        ));
    };
    match parse_duration(text) {
        Ok(span) => Ok(GlobalTime::from_nanos(span.as_nanos())),
        Err(e) => Err(format!("{e}\n")),
    }
}

/// An address: hex, with or without `0x`, the same spelling the gdbstub's own
/// monitor commands take.
fn addr_of(text: &str) -> Result<u64, String> {
    let body = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X"));
    u64::from_str_radix(body.unwrap_or(text), 16)
        .map_err(|_| format!("`{text}` is not a hex address\n"))
}

/// The bytes of a `write`: hex pairs, either spaced or run together.
///
/// Both spellings because both are what a person has in front of them — a
/// vector copied out of a datasheet is `ff 1f`, and one copied out of a hex
/// dump is `ff1f`.
fn bytes_of(words: &[&str]) -> Result<Vec<u8>, String> {
    let joined: String = words.concat();
    if joined.is_empty() {
        return Err(String::from("at least one byte is needed\n"));
    }
    if !joined.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(format!("`{joined}` is not hex bytes\n"));
    }
    if !joined.len().is_multiple_of(2) {
        return Err(format!(
            "`{joined}` is {} hex digits; bytes come in pairs\n",
            joined.len()
        ));
    }
    if joined.len() / 2 > WRITE_MAX {
        return Err(format!(
            "at most {WRITE_MAX} bytes in one line; `rsemu run --media` is how an image gets in\n"
        ));
    }
    let mut bytes = Vec::with_capacity(joined.len() / 2);
    for pair in joined.as_bytes().chunks(2) {
        let text = core::str::from_utf8(pair).unwrap_or("");
        bytes.push(u8::from_str_radix(text, 16).map_err(|_| format!("`{text}` is not a byte\n"))?);
    }
    Ok(bytes)
}

/// A register's bytes as a number.
///
/// The chunk encoding is flat little-endian (`core::state`), so the most
/// significant byte is the last one — printing them in order would read every
/// register backwards.
fn hex_of(bytes: &[u8]) -> String {
    let mut out = String::from("0x");
    for byte in bytes.iter().rev() {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

/// A rate, as a person reads it: whole hertz where it is whole, and the exact
/// ratio where it is not.
///
/// Never a float. A 6502 in an NTSC NES runs at 39375000/22 Hz and rounding
/// that to 1789772.7 is exactly the lie `CLAUDE.md`'s no-floats-in-the-time-path
/// rule exists to prevent — the monitor is a place people read numbers off, so
/// it prints the number the machine actually has.
fn hz_of(rate: crate::core::clock::Rational) -> String {
    if rate.den() == 1 {
        format!("{} Hz", rate.num())
    } else {
        format!("{}/{} Hz", rate.num(), rate.den())
    }
}

// ---------------------------------------------------------------------------
// help
// ---------------------------------------------------------------------------

/// The whole command set, in the order somebody reaches for it.
const HELP: &str = "\
rsemu monitor — the machine, while it is stopped
  addresses are hex (`0x` optional); spans carry their unit (`1s`, `500ms`, `20us`)

session
  help [command]        this, or one command in detail
  status                where the machine is, and whether its hash still means anything
  quit                  leave the session; end of input does the same

execution
  run <span>            advance virtual time by <span>, then stop
  cont [span]           the same; with no span, until interrupted or --for's deadline
  step [n]              one instruction (or n) on the selected processor
  cpu [n]               show, or select, the processor the per-CPU commands use
  cpus                  every processor: path, class and where it is
  regs [name]           the selected processor's registers, or one of them
  sched                 the scheduler: mode, quantum, and what is queued

the machine
  devices               the device tree, with class and instance path
  device <path>         one device: class, clock, space, properties, current state
  spaces                the machine's address spaces
  map [space]           what is mapped where
  clocks                every clock domain: rate, ticks, lead, gating
  wires                 every net, what drives it and what it settles at

media
  media                 every removable bay, and what is in it
  insert <bay> <file>   put a disk, disc or card in; a trailing `ro` protects it
  eject <bay>           take it out. Both raise the drive's disk-change signal

memory
  x <addr> [len]        read at a VIRTUAL address, through the selected processor
  xp <addr> [len]       read at a PHYSICAL address, no translation
  write <addr> <hex>    write bytes at a virtual address
  writep <addr> <hex>   write bytes at a physical address
  translate <addr>      where the selected processor's MMU maps an address

state
  time                  the current virtual instant
  hash                  the machine state hash (ROADMAP.md \u{a7}0)
  save <file>           write a snapshot
  load <file>           restore one
  timeline              the rewind history: cadence, keyframes, bytes held
  rewind <span>         go back <span>: restore the nearest keyframe and replay

counters
  trace [channel...]    what --trace reports, now, for sched/cpu/clock/mmio

Every read here sets MemAttrs::debug, so looking at a device never changes it.
";

/// One command in detail, for the few where the one-liner is not the whole
/// story.
fn help(topic: Option<&str>) -> String {
    let Some(topic) = topic else {
        return String::from(HELP);
    };
    let text = match topic {
        "run" | "cont" => {
            "\
run <span> advances virtual time by exactly <span> and stops; cont does the same
with no bound. Both go through Machine::run_until in ten-millisecond slices —
the same call rsemu run --headless makes — and run_until is additive, so
`run 500ms` twice and `run 1s` once reach the same state.
"
        }
        "step" => {
            "\
step [n] runs n instructions on the selected processor using the debugger's
stepper. That one splits a scheduling round, which run_until refuses to do, so
it is NOT additive: a session that has stepped is no longer comparable with a
headless run of the same span, and `status` says so once you have used it.
"
        }
        "rewind" => {
            "\
rewind <span> goes back <span> of virtual time. It is not a separate mechanism:
the session restores the newest keyframe at or before the instant asked for and
replays the recorded input forward, so it lands on the nearest keyframe rather
than exactly where you asked. `timeline` shows where those are. Output the guest
already emitted has left and will be emitted again; a writable disk image is
outside the snapshot and does not move. Needs deterministic threading.
"
        }
        "hash" | "status" => {
            "\
The state hash is FNV-1a over a full snapshot, which is the regression method
ROADMAP.md \u{a7}0 describes: run deterministically for N virtual units and compare
this number. It is refused outright under parallel or accel threading, because
neither is reproducible and a number there would invite somebody to paste it
into a test.
"
        }
        "write" | "writep" => {
            "\
write <addr> <hex> patches guest memory. The bytes may be spaced (`ff 1f`) or run
together (`ff1f`). write goes through the selected processor's MMU; writep names
a bus address and skips it. A write that lands on code invalidates any
translation standing behind it.
"
        }
        "trace" => {
            "\
trace renders the counters as `rsemu run --trace` would, but now rather than at
the end of a run: two whitespace-separated columns sorted by name, under a #
header, with no wall clock in it. Name channels to narrow it: sched, cpu, clock,
mmio.
"
        }
        "media" | "insert" | "eject" => {
            "\
media lists every bay a device in this machine publishes as removable, and
insert and eject change what is in one. A bay is named by the device path
`devices` prints -- `df0`, `fdc`, `cdrom` -- or as `<path>:<bay>` when a device
has more than one.

insert takes the same file specifications `rsemu run --media` does, schemes
included: `insert df0 adf:disks.iso,disk=Workbench` reaches into an Amiga
Forever disc image. A trailing `ro` write protects what goes in.

**The guest is told.** Each drive raises the signal its own hardware raises --
a PC's DSKCHG in the digital input register, an Amiga's CHNG* on the drive
connector, an ATAPI unit attention with sense 28h/00h -- because a swap the
guest does not see corrupts the filesystem it has mounted. What it does about
it is its own business and may take a while.

Both commands run with the machine stopped, between scheduling rounds, which is
what keeps a session reproducible: a script that runs, ejects and runs again
lands on one state hash every time.
"
        }
        "x" | "xp" | "map" | "devices" | "spaces" | "translate" | "time" => {
            "\
Answered by the same code the gdbstub's `monitor` command uses, so what this
prompt says and what `monitor <command>` says inside gdb are one thing.
Addresses are hex; a length is decimal and bounded.
"
        }
        _ => {
            return format!("no help for `{topic}`; `help` lists the commands\n");
        }
    };
    String::from(text)
}
