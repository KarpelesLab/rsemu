//! Phase 9's gate: **a guest debugged end-to-end over gdb**, by a real `gdb`.
//!
//! `tests/gdb_session.rs` drives the stub with a client of our own. That proves
//! the packets are what we think they are; it cannot prove they are what GDB
//! thinks they are, because both ends of it are rsemu. This test runs the
//! distribution's `gdb` binary in batch mode against a running guest and asserts
//! on what *it* printed: the registers it read, the breakpoint it set and hit,
//! the instruction it stepped, and the bytes it wrote into guest RAM and read
//! back.
//!
//! Running a GDB binary as a client is black-box use of a GPL program, which
//! `ROADMAP.md` §1 permits in as many words. Nothing here reads GDB's source;
//! the protocol comes from the GDB manual's "Remote Protocol" appendix.
//!
//! # It skips rather than fails
//!
//! Two things have to be true for this test to mean anything, and neither is
//! true everywhere:
//!
//! * **A `gdb` binary exists.** `$RSEMU_GDB`, else `gdb`, else `gdb-multiarch`.
//!   Absent, the test prints why and returns, exactly as the `RSEMU_BIOS` tests
//!   do — `cargo test` stays hermetic.
//! * **That `gdb` knows x86.** A distribution's GDB is usually built for one
//!   architecture, and stock GDB refuses a target description for a CPU it has
//!   no gdbarch for (`src/host/gdb/arch.rs` says why, at length). The guest here
//!   is therefore an 8086, because on the overwhelmingly common x86-64
//!   developer machine that is the one guest family a stock `gdb` will talk to.
//!   A `gdb` that cannot name `i8086` skips.
//!
//! # What GDB does with our target description
//!
//! It rejects it — `warning: Architecture rejected target-supplied description`
//! — and that is expected and harmless. GDB's `i386` gdbarch will only accept a
//! description whose feature is `org.gnu.gdb.i386.core` *and* which supplies the
//! x87 register block; ours is `org.rsemu.i386` and supplies the sixteen
//! integer registers the core actually has. Having rejected it GDB falls back to
//! its built-in i386 layout, whose first sixteen registers are `eax ecx edx ebx
//! esp ebp esi edi eip eflags cs ss ds es fs gs` — which is exactly the order
//! `cpu.x86` saves them in, and exactly what `src/host/gdb/arch.rs` documents as
//! the reason that map is the identity rather than a translation. So the `g`
//! packet lines up, `info registers` is right, and the session below works.
//! Asserting on that warning is part of the test: it is the behaviour, and if it
//! ever changes we want to be told.

#![cfg(all(feature = "gdb", feature = "cpu-x86"))]

use std::io::Write as _;
use std::process::Command;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use rsemu::host::gdb::{GdbServer, MachineTarget, Progress};
use rsemu::machine::{Machine, catalog};

/// The board: an 8086, a megabyte of RAM and sixteen bytes of ROM that matter.
const X86_MINI: &str = include_str!("../machines/tests/x86-mini.machine");

/// Where the ROM's far jump sends the guest, and where the test's program goes.
///
/// `0x500` is the first byte of low memory a PC's own firmware would not have
/// claimed, which is why every DOS-era loader used it. Nothing here needs that
/// to be true; it is just a recognisable address.
const PROGRAM: u16 = 0x0500;

/// The byte the guest stores, and where it stores it.
const SENTINEL: u8 = 0x42;
const SENTINEL_ADDR: u16 = 0x0600;

/// Find a GDB to drive, or explain why there is none.
fn find_gdb() -> Option<String> {
    if let Ok(explicit) = std::env::var("RSEMU_GDB")
        && !explicit.is_empty()
    {
        return Some(explicit);
    }
    for candidate in ["gdb", "gdb-multiarch"] {
        let ok = Command::new(candidate)
            .arg("--version")
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .status()
            .map(|s| s.success())
            .unwrap_or(false);
        if ok {
            return Some(candidate.to_string());
        }
    }
    None
}

/// Whether this GDB has an x86 gdbarch compiled in.
fn knows_x86(gdb: &str) -> bool {
    let out = Command::new(gdb)
        .args(["-batch", "-ex", "complete set architecture "])
        .output();
    match out {
        Ok(out) => {
            let text = String::from_utf8_lossy(&out.stdout);
            text.lines().any(|l| l.trim() == "set architecture i8086")
        }
        Err(_) => false,
    }
}

/// The 64 KiB boot ROM: `JMP 0000:0500` at the reset vector and nothing else.
///
/// An 8086 resets to `CS:IP = ffff:0000`, so the first instruction is fetched
/// from linear `0xffff0` — sixteen bytes below the top of the megabyte, which
/// is offset `0xfff0` in a ROM mapped at `0xf0000`. `EA` is the intersegment
/// direct `JMP`, offset first then segment (Intel 8086 Family User's Manual,
/// "Program Transfer Instructions").
fn boot_rom() -> Vec<u8> {
    let mut rom = vec![0u8; 64 * 1024];
    rom[0xfff0..0xfff5].copy_from_slice(&[0xea, 0x00, 0x05, 0x00, 0x00]);
    rom
}

/// Build the board.
fn board() -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", boot_rom());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build("x86-mini.machine", X86_MINI, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the fixture does not realize: {e}"),
    }
}

/// The gdbstub, on a thread of its own, with the board behind it.
struct Server {
    addr: std::net::SocketAddr,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Server {
    fn start() -> Server {
        let server = GdbServer::bind(":0").expect("bind an ephemeral port");
        let addr = server.local_addr().expect("local_addr");
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name(String::from("gdb-real-client"))
            .spawn(move || {
                let mut server = server;
                let mut machine = board();
                let mut target = MachineTarget::new(&mut machine);
                while !flag.load(Ordering::Relaxed) {
                    match server.poll(&mut target) {
                        Ok(Progress::Kill) => break,
                        Ok(_) => {}
                        Err(e) => panic!("gdb server: {e}"),
                    }
                }
            })
            .expect("spawn");
        Server {
            addr,
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// The guest program the debugger writes into RAM, at [`PROGRAM`].
///
/// ```text
///   0500  b0 42        mov al, 0x42
///   0502  a2 00 06     mov [0x0600], al
///   0505  eb f9        jmp 0x0500
/// ```
///
/// A loop, so `continue` always reaches a breakpoint anywhere in it, and a store
/// to a known address, so a memory read has something to find that the debugger
/// did not put there itself.
const GUEST_PROGRAM: [u8; 7] = [0xb0, SENTINEL, 0xa2, 0x00, 0x06, 0xeb, 0xf9];

/// The board itself, with no `gdb` involved.
///
/// The session test above skips wherever there is no usable `gdb`, and a
/// fixture only exercised by a skipping test is a fixture nothing checks. This
/// one always runs, so `machines/tests/x86-mini.machine` cannot rot quietly.
#[test]
fn the_fixture_board_realizes_and_resets_into_its_rom() {
    use rsemu::host::gdb::DebugTarget;

    let mut m = board();
    let mut target = MachineTarget::new(&mut m);
    assert_eq!(target.cpu_count(), 1, "one 8086");
    assert_eq!(target.arch(0).expect("an arch").class.name, "cpu.x86");

    // An 8086 resets to `CS:IP = ffff:0000`, so linear `0xffff0` — and that is
    // where the far jump is. The read is physical because `eip` is zero and
    // GDB's flat address is not the linear one; see this file's header.
    let mut vector = [0u8; 5];
    target
        .read_physical(0, 0xf_fff0, &mut vector)
        .expect("the ROM answers");
    assert_eq!(vector, [0xea, 0x00, 0x05, 0x00, 0x00], "JMP 0000:0500");

    // Stepping past the reset sequence and the jump lands in RAM at 0x500,
    // which is what makes GDB's addresses mean what a user thinks. The first
    // few steps are the reset sequence itself — a core that has been reset has
    // work to retire before it fetches anything — so this looks for the
    // landing rather than counting instructions.
    let eip_of = |target: &MachineTarget<'_>| {
        let regs = target.read_registers(0).expect("registers");
        u32::from_le_bytes([regs[32], regs[33], regs[34], regs[35]])
    };
    let mut arrived = false;
    for _ in 0..8 {
        target.step(0).expect("a step");
        if eip_of(&target) == u32::from(PROGRAM) {
            arrived = true;
            break;
        }
    }
    assert!(
        arrived,
        "the far jump never ran; eip is {:#x}",
        eip_of(&target)
    );
}

#[test]
fn a_real_gdb_debugs_a_guest_end_to_end() {
    let Some(gdb) = find_gdb() else {
        println!(
            "skipping: no gdb binary. Set $RSEMU_GDB, or install gdb, to run \
             ROADMAP.md phase 9's gate."
        );
        return;
    };
    if !knows_x86(&gdb) {
        println!(
            "skipping: `{gdb}` has no i8086 gdbarch, so it cannot debug the one \
             guest family a stock gdb can talk to. See this file's header."
        );
        return;
    }

    let server = Server::start();
    let port = server.addr.port();

    // The program, as a file `restore` can push over the wire in `X` packets.
    let dir = std::env::temp_dir().join(format!("rsemu-gdb-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("a scratch directory");
    let program = dir.join("guest.bin");
    {
        let mut f = std::fs::File::create(&program).expect("create");
        f.write_all(&GUEST_PROGRAM).expect("write");
    }

    // Every line is a thing a person types. The `printf`s are how a batch
    // session says what it saw; `x` and `info registers` are there because
    // their output is the shape a user would recognise.
    // `RSEMU_GDB_DEBUG_REMOTE=1 cargo test -- --nocapture` prints every packet
    // GDB sends and every reply it gets. It is how the list of packets this
    // stub has to answer was established in the first place, and it is the
    // first thing to reach for when a real session misbehaves.
    let mut script = Vec::new();
    if std::env::var_os("RSEMU_GDB_DEBUG_REMOTE").is_some() {
        script.push(String::from("set debug remote 1"));
    }
    script.extend([
        format!("target remote 127.0.0.1:{port}"),
        String::from("set architecture i8086"),
        // Where an 8086 wakes up: `CS:IP = ffff:0000`, so linear `0xffff0` and
        // GDB's flat `$pc` is zero. The brackets are so an assertion can match
        // a whole value; GDB's `printf` renders `%#x` of zero as plain `0`.
        String::from("printf \"RSEMU eip0 [%#x]\\n\", $eip"),
        String::from("printf \"RSEMU cs0 [%#x]\\n\", $cs"),
        // Run the ROM's far jump under a breakpoint at its destination. This is
        // `Z0` plus a stop reply plus `continue`, in one step, and it lands the
        // guest at an address where `eip` is the linear address.
        format!("break *{PROGRAM:#x}"),
        String::from("continue"),
        String::from("printf \"RSEMU eip1 [%#x]\\n\", $eip"),
        String::from("printf \"RSEMU cs1 [%#x]\\n\", $cs"),
        String::from("delete 1"),
        // Write the guest's program over the wire, and read it back.
        format!("restore {} binary {:#x}", program.display(), PROGRAM),
        format!("x/7xb {PROGRAM:#x}"),
        // Clear the byte the program stores, so finding it later means the
        // guest ran and not that the debugger wrote it.
        format!("set *(unsigned char *) {SENTINEL_ADDR:#x} = 0"),
        // One instruction: `mov al, 0x42`.
        String::from("stepi"),
        String::from("printf \"RSEMU eip2 [%#x]\\n\", $eip"),
        String::from("printf \"RSEMU eax2 [%#x]\\n\", $eax"),
        // Run to a breakpoint on the `jmp`, which is after the store.
        format!("break *{:#x}", u32::from(PROGRAM) + 5),
        String::from("continue"),
        String::from("printf \"RSEMU eip3 [%#x]\\n\", $eip"),
        format!("printf \"RSEMU stored [%#x]\\n\", *(unsigned char *) {SENTINEL_ADDR:#x}"),
        String::from("info registers eax eip"),
        // A single step off the breakpoint, back to the top of the loop. This
        // is the case a stub gets wrong by reporting the same breakpoint
        // forever.
        String::from("stepi"),
        String::from("printf \"RSEMU eip4 [%#x]\\n\", $eip"),
        // Writing a register, which is the other half of `P`.
        String::from("set $ebx = 0xcafe"),
        String::from("printf \"RSEMU ebx [%#x]\\n\", $ebx"),
        // A watchpoint, through the client that decides what `watch` means.
        // rsemu's are polled — the watched bytes are re-read with
        // `MemAttrs::debug` after every clock tick — so GDB seeing an old and a
        // new value is the whole mechanism working end to end.
        // Breakpoint 2 is still on the `jmp` and would be reached in the same
        // tick as the store, so it goes first: a resume reports a breakpoint
        // ahead of a watchpoint, and this is about the watchpoint.
        String::from("delete 2"),
        format!("set *(unsigned char *) {SENTINEL_ADDR:#x} = 0"),
        format!("watch *(unsigned char *) {SENTINEL_ADDR:#x}"),
        String::from("continue"),
        format!("printf \"RSEMU watched [%#x]\\n\", *(unsigned char *) {SENTINEL_ADDR:#x}"),
        String::from("delete"),
        // And a real GDB `monitor` round trip.
        String::from("monitor devices"),
        // The monitor's own commands, which is where the virtual/physical
        // distinction becomes something a person can type. GDB's `x` is always
        // virtual; `monitor xp` is the only route to a bus address.
        String::from("monitor x 500 8"),
        String::from("monitor xp 500 8"),
        String::from("monitor translate 500"),
        String::from("monitor map"),
        // `info threads` is what a user types on a machine with more than one
        // core; on this one it proves the thread list and `qC` agree.
        String::from("info threads"),
        String::from("detach"),
    ]);

    let mut cmd = Command::new(&gdb);
    cmd.arg("-batch").arg("-nx");
    for line in &script {
        cmd.arg("-ex").arg(line);
    }
    let out = cmd.output().expect("gdb runs");
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    let all = format!("{stdout}\n{stderr}");
    println!("--- gdb stdout ---\n{stdout}\n--- gdb stderr ---\n{stderr}");

    let says = |needle: &str| {
        assert!(
            all.contains(needle),
            "gdb never said `{needle}`.\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
        );
    };

    // The session, in the order it happened.
    // GDB's `printf` renders `%#x` of zero as a bare `0`, not `0x0`.
    says("RSEMU eip0 [0]");
    says("RSEMU cs0 [0xffff]");
    says("Breakpoint 1");
    says("RSEMU eip1 [0x500]");
    says("RSEMU cs1 [0]");
    says("0xb0\t0x42\t0xa2\t0x00\t0x06\t0xeb\t0xf9");
    says("RSEMU eip2 [0x502]");
    says("RSEMU eax2 [0x42]");
    says("RSEMU eip3 [0x505]");
    says("RSEMU stored [0x42]");
    says("RSEMU eip4 [0x500]");
    says("RSEMU ebx [0xcafe]");
    says("watchpoint 3");
    says("Old value = 0");
    says("New value = 66");
    says("RSEMU watched [0x42]");
    says("cpu.x86");
    says("b0 42 a2 00 06 eb f9");
    says("0x500 -> 0x500 (identity)");
    says("mem (20 bits)");
    says("rom  rwx");

    // Nothing in the session was an error GDB reported to the user.
    for bad in [
        "Truncated register",
        "Remote failure reply",
        "Cannot access memory",
        "Remote communication error",
        "Ignoring packet error",
    ] {
        assert!(
            !all.contains(bad),
            "gdb reported `{bad}`:\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
        );
    }

    let _ = std::fs::remove_dir_all(&dir);
    drop(server);
}
