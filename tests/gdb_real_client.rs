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
//! # Three guests
//!
//! There is a **16-bit x86** session, an **x86-64** one and an **AArch64** one,
//! and they are not the same test three times. The two x86 guests are the ones
//! a stock `gdb` on an x86-64 developer machine will talk to, so they are the
//! sessions that actually run in most places, and they exercise `cpu.x86`'s
//! *two* register views: the same class is an 8088 on one board and an x86-64
//! part on another, and which view a board gets is decided by the width of the
//! address space it gave its core. The AArch64 guest is `arm64-virt`'s core.
//! Whichever `gdb` is to hand, at least one of them says something.
//!
//! Several tests need no `gdb` at all: the x86 fixture board resets into its
//! ROM, the x86-64 register map agrees with the core it describes, the two
//! views go to the right boards, `cpu.i8086` is debuggable, the AArch64
//! register map agrees with its core, and an AArch64 breakpoint stops where it
//! was put. Those run everywhere, which matters because the AArch64 session
//! skips on the common host.
//!
//! # It skips rather than fails
//!
//! Two things have to be true for a real-`gdb` session to mean anything, and
//! neither is true everywhere:
//!
//! * **A `gdb` binary exists.** `$RSEMU_GDB`, else `gdb`, else `gdb-multiarch`.
//!   Absent, the test prints why and returns, exactly as the `RSEMU_BIOS` tests
//!   do — `cargo test` stays hermetic.
//! * **That `gdb` knows the guest's architecture.** A distribution's GDB is
//!   usually built for one, and stock GDB refuses a target description for a
//!   CPU it has no gdbarch for (`src/host/gdb/arch.rs` says why, at length).
//!   [`knows`] asks GDB's own `complete set architecture` rather than guessing;
//!   a session whose architecture is missing prints why and returns.
//!
//! # What GDB does with our target description
//!
//! It **accepts** all three, and that is asserted in all three, because it used
//! not to. A feature named `org.gnu.gdb.<arch>.core` is a promise that it holds
//! exactly the registers GDB's gdbarch for that architecture expects, in its
//! order and at its widths; a description that breaks the promise is refused
//! with `warning: Architecture rejected target-supplied description`, after
//! which GDB reads the `g` packet through a built-in layout that may or may not
//! happen to agree.
//!
//! For x86 it did break the promise, and it happened to agree: the map was
//! sixteen integer registers under an `org.rsemu.i386` name, GDB fell back to
//! its own i386 layout, and that layout's first sixteen are `eax ecx edx ebx
//! esp ebp esi edi eip eflags cs ss ds es fs gs` — exactly the order `cpu.x86`
//! saves them in. Agreeing by luck is not the same as being right, and it is no
//! use at all in 64-bit mode, where GDB's fallback layout is not this chunk.
//! Both x86 maps now supply the x87 block GDB's `i386.core` requires, so the
//! description is accepted; the 64-bit one claims
//! `<architecture>i386:x86-64</architecture>` with it, which is what makes
//! `$r15` and the high half of `$rax` reachable at all.

#![cfg(all(feature = "gdb", any(feature = "cpu-x86", feature = "cpu-arm-a64")))]

#[cfg(feature = "cpu-x86")]
use std::io::Write as _;
use std::process::Command;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use rsemu::host::gdb::{GdbServer, MachineTarget, Progress};
use rsemu::machine::{Machine, catalog};

/// The board: an 8086, a megabyte of RAM and sixteen bytes of ROM that matter.
#[cfg(feature = "cpu-x86")]
const X86_MINI: &str = include_str!("../machines/tests/x86-mini.machine");

/// Where the ROM's far jump sends the guest, and where the test's program goes.
///
/// `0x500` is the first byte of low memory a PC's own firmware would not have
/// claimed, which is why every DOS-era loader used it. Nothing here needs that
/// to be true; it is just a recognisable address.
#[cfg(feature = "cpu-x86")]
const PROGRAM: u16 = 0x0500;

/// The byte the guest stores, and where it stores it.
#[cfg(feature = "cpu-x86")]
const SENTINEL: u8 = 0x42;
#[cfg(feature = "cpu-x86")]
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

/// Whether this GDB has a gdbarch for `arch` compiled in.
///
/// A distribution's GDB is usually built for one architecture, and one that
/// has never heard of the guest cannot debug it whatever the stub says — so
/// this is what turns a test into a skip rather than a failure. `complete set
/// architecture ` is GDB's own list, which is why it is asked rather than
/// guessed from the host triple.
fn knows(gdb: &str, arch: &str) -> bool {
    let out = Command::new(gdb)
        .args(["-batch", "-ex", "complete set architecture "])
        .output();
    let want = format!("set architecture {arch}");
    match out {
        Ok(out) => {
            let text = String::from_utf8_lossy(&out.stdout);
            text.lines().any(|l| l.trim() == want)
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
#[cfg(feature = "cpu-x86")]
fn boot_rom() -> Vec<u8> {
    let mut rom = vec![0u8; 64 * 1024];
    rom[0xfff0..0xfff5].copy_from_slice(&[0xea, 0x00, 0x05, 0x00, 0x00]);
    rom
}

/// Build the board.
#[cfg(feature = "cpu-x86")]
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
    /// Serve `build`'s board on an ephemeral loopback port.
    ///
    /// The board is built on the session thread rather than handed in, because
    /// a `Machine` is not `Send` and because the debugger and the machine
    /// share a thread on purpose — that sharing is what makes "attaching stops
    /// the world" true without a barrier (`src/host/gdb/mod.rs`).
    fn start<F: FnOnce() -> Machine + Send + 'static>(build: F) -> Server {
        let server = GdbServer::bind(":0").expect("bind an ephemeral port");
        let addr = server.local_addr().expect("local_addr");
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name(String::from("gdb-real-client"))
            .spawn(move || {
                let mut server = server;
                let mut machine = build();
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
#[cfg(feature = "cpu-x86")]
const GUEST_PROGRAM: [u8; 7] = [0xb0, SENTINEL, 0xa2, 0x00, 0x06, 0xeb, 0xf9];

/// The board itself, with no `gdb` involved.
///
/// The session test above skips wherever there is no usable `gdb`, and a
/// fixture only exercised by a skipping test is a fixture nothing checks. This
/// one always runs, so `machines/tests/x86-mini.machine` cannot rot quietly.
#[cfg(feature = "cpu-x86")]
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

#[cfg(feature = "cpu-x86")]
#[test]
fn a_real_gdb_debugs_a_guest_end_to_end() {
    let Some(gdb) = find_gdb() else {
        println!(
            "skipping: no gdb binary. Set $RSEMU_GDB, or install gdb, to run \
             ROADMAP.md phase 9's gate."
        );
        return;
    };
    if !knows(&gdb, "i8086") {
        println!(
            "skipping: `{gdb}` has no i8086 gdbarch, so it cannot debug the one \
             guest family a stock gdb can talk to. See this file's header."
        );
        return;
    }

    let server = Server::start(board);
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
        // What GDB settled on from a description that names no architecture,
        // before anybody tells it. Asserted below, because the answer is the
        // reason the 32-bit map leaves `<architecture>` out.
        String::from("show architecture"),
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

    // The description is accepted rather than refused, which is what supplying
    // the x87 block gdb's `org.gnu.gdb.i386.core` requires buys — and it holds
    // under `set architecture i8086`, which is the whole reason the 32-bit map
    // supplies no `<architecture>` of its own.
    assert!(
        !all.contains("Architecture rejected target-supplied description"),
        "gdb rejected the i386 target description:\n{all}"
    );
    // And with nothing said, gdb resolves the description to `i386` — not to
    // the x86-64 it is itself built for, which would read this `g` packet
    // through the wrong layout. That is the whole benefit of leaving
    // `<architecture>` out of a map that covers two of gdb's gdbarches.
    says("(currently \"i386\")");

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

// ---------------------------------------------------------------------------
// x86-64
// ---------------------------------------------------------------------------

/// A board whose core is an x86-64 part on a 64-bit bus.
///
/// The one difference from [`X86_MINI`] that the debugger cares about is
/// `width = 64`: `host::gdb::arch::for_cpu` reads the width of a CPU's address
/// space to decide which of `cpu.x86`'s two register views to serve, because
/// nothing reachable from a `Machine` says which variant the machine file asked
/// for. Every board in `machines/` that says `variant = "x86-64"` gives its
/// core a 64-bit space, and this fixture is written the same way for the same
/// reason.
#[cfg(feature = "cpu-x86")]
const X86_64_MINI: &str = r#"
machine "gdb-x86-64" {
  osc xtal = 100000000 Hz
  space mem  { width = 64, unassigned = read-as-ones }
  space port { width = 16, unassigned = read-as-ones }
  object cpu "cpu.x86" {
    clock   = xtal
    space   = mem
    iospace = "port"
    variant = "x86-64"
    engine  = "interp"
  }
  object dram "ram" { size = 1M }
  map mem 0x00000 size 1M = dram
}
"#;

/// The same board with a 32-bit bus, which is what `pc-at` and `pc-apic` have.
#[cfg(feature = "cpu-x86")]
const X86_32_MINI: &str = r#"
machine "gdb-x86-32" {
  osc xtal = 100000000 Hz
  space mem  { width = 32, unassigned = read-as-ones }
  space port { width = 16, unassigned = read-as-ones }
  object cpu "cpu.x86" {
    clock   = xtal
    space   = mem
    iospace = "port"
    variant = "80486"
    engine  = "interp"
  }
  object dram "ram" { size = 1M }
  map mem 0x00000 size 1M = dram
}
"#;

/// The 64-bit values the fixture seeds, so a debugger showing thirty-two bits
/// of them is visibly wrong rather than plausibly right.
#[cfg(feature = "cpu-x86")]
const X64_RAX: u64 = 0x1122_3344_5566_7788;
#[cfg(feature = "cpu-x86")]
const X64_RBX: u64 = 0xdead_beef_cafe_babe;
#[cfg(feature = "cpu-x86")]
const X64_R15: u64 = 0x8000_0000_0000_0001;
/// What the gdb session writes into `r14`, which the core is asked for
/// afterwards.
#[cfg(feature = "cpu-x86")]
const X64_R14: u64 = 0xfeed_face_1234_5678;

/// Build an x86 board from `src`, keeping the core.
#[cfg(feature = "cpu-x86")]
fn x86_board_with_core(name: &str, src: &str) -> (Machine, Arc<rsemu::cpu::x86::X86>) {
    use rsemu::core::Captured;
    use rsemu::cpu::x86::X86;

    let cores: Arc<Captured<X86>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut bindings = catalog::bindings().expect("this build's bindings");
    bindings.replace("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let options = rsemu::machine::BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(bindings);
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build(name, src, &registry, &options)
        .unwrap_or_else(|e| panic!("the fixture does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the core");
    (machine, cpu)
}

/// The x86-64 register map, checked register by register against the core.
///
/// This is the test the 64-bit view exists for. `cpu.x86` has written
/// `RAX`-`R15` and `RIP` at full width since chunk version 4, and a debugger
/// could not reach them: they sit behind the **prefetch queue**, which `save`
/// writes length-prefixed, so the long-mode block is at a different offset
/// depending on how many bytes the bus interface unit happens to be holding.
/// Every register below is therefore compared against the core's own accessor,
/// and then compared again with the queue full — which is the case a table of
/// constants gets wrong and says nothing about.
#[cfg(feature = "cpu-x86")]
#[test]
fn the_x86_64_register_map_agrees_with_the_core() {
    use rsemu::cpu::x86::Reg;
    use rsemu::host::gdb::DebugTarget;

    /// gdb's AMD64 core numbering, which is the DWARF one and not ModRM's.
    const GDB_ORDER: [Reg; 16] = [
        Reg::Rax,
        Reg::Rbx,
        Reg::Rcx,
        Reg::Rdx,
        Reg::Rsi,
        Reg::Rdi,
        Reg::Rbp,
        Reg::Rsp,
        Reg::R8,
        Reg::R9,
        Reg::R10,
        Reg::R11,
        Reg::R12,
        Reg::R13,
        Reg::R14,
        Reg::R15,
    ];

    let (mut m, cpu) = x86_board_with_core("gdb-x86-64.machine", X86_64_MINI);
    let mut target = MachineTarget::new(&mut m);
    assert_eq!(target.cpu_count(), 1, "one core");
    let arch = target.arch(0).expect("a register map");
    assert_eq!(arch.class.name, "cpu.x86");
    assert_eq!(arch.feature, "org.gnu.gdb.i386.core");
    assert_eq!(arch.architecture, Some("i386:x86-64"));
    // 16 * 8 + rip + eflags + six selectors + eight ten-byte x87 registers +
    // eight control words: gdb's own AMD64 core block, with no SSE feature.
    assert_eq!(arch.packet_len(), 276);
    assert_eq!(arch.regs[16].name, "rip");
    assert_eq!(arch.regs[arch.pc].name, "rip");

    // A distinct value in every register, so a map that is off by one entry
    // cannot pass — and every value with its high half set, so a map that is
    // thirty-two bits wide cannot either.
    for (i, reg) in GDB_ORDER.iter().enumerate() {
        cpu.set_reg(*reg, 0xa500_0000_0000_0000 | i as u64);
    }
    let mut x87 = cpu.x87();
    for (i, slot) in x87.regs.iter_mut().enumerate() {
        slot.sig = 0xc0c0_0000_0000_0000 | i as u64;
        slot.sign_exp = 0x4000 | i as u16;
    }
    x87.control = 0x037f;
    x87.status = 0x1234;
    x87.tag = 0x5555;
    x87.last_op = 0x01ff;
    x87.last_ip = 0x1111_2222_3333_4444;
    x87.last_dp = 0x5555_6666_7777_8888;
    x87.last_cs = 0x0008;
    x87.last_ds = 0x0010;
    cpu.set_x87(x87);

    let check = |target: &MachineTarget<'_>, why: &str| {
        let u64_of =
            |bytes: &[u8]| u64::from_le_bytes(<[u8; 8]>::try_from(bytes).expect("eight bytes"));
        let u32_of =
            |bytes: &[u8]| u32::from_le_bytes(<[u8; 4]>::try_from(bytes).expect("four bytes"));
        let g = target.read_registers(0).expect("the whole register file");
        assert_eq!(g.len(), 276, "{why}");
        for (i, reg) in GDB_ORDER.iter().enumerate() {
            assert_eq!(
                u64_of(&g[i * 8..i * 8 + 8]),
                cpu.reg(*reg),
                "{why}: {reg:?} is not where the map says it is"
            );
        }
        assert_eq!(u64_of(&g[128..136]), cpu.reg(Reg::Rip), "{why}: rip");
        assert_eq!(
            u32_of(&g[136..140]),
            cpu.reg(Reg::Eflags) as u32,
            "{why}: eflags"
        );
        for (i, reg) in [Reg::Cs, Reg::Ss, Reg::Ds, Reg::Es, Reg::Fs, Reg::Gs]
            .into_iter()
            .enumerate()
        {
            assert_eq!(
                u32_of(&g[140 + i * 4..144 + i * 4]),
                cpu.reg(reg) as u32,
                "{why}: {reg:?}"
            );
        }
        // The x87 file: ten bytes each, significand then sign-and-exponent,
        // which is the 80-bit format's own layout.
        let x87 = cpu.x87();
        for (i, slot) in x87.regs.iter().enumerate() {
            let at = 164 + i * 10;
            assert_eq!(
                u64::from_le_bytes(<[u8; 8]>::try_from(&g[at..at + 8]).expect("8")),
                slot.sig,
                "{why}: st{i} significand"
            );
            assert_eq!(
                u16::from_le_bytes(<[u8; 2]>::try_from(&g[at + 8..at + 10]).expect("2")),
                slot.sign_exp,
                "{why}: st{i} sign and exponent"
            );
        }
        for (i, (name, want)) in [
            ("fctrl", u32::from(x87.control)),
            ("fstat", u32::from(x87.status)),
            ("ftag", u32::from(x87.tag)),
            ("fiseg", u32::from(x87.last_cs)),
            ("fioff", x87.last_ip as u32),
            ("foseg", u32::from(x87.last_ds)),
            ("fooff", x87.last_dp as u32),
            ("fop", u32::from(x87.last_op)),
        ]
        .into_iter()
        .enumerate()
        {
            let at = 244 + i * 4;
            assert_eq!(u32_of(&g[at..at + 4]), want, "{why}: {name}");
        }
    };

    check(&target, "with an empty prefetch queue");

    // And now with the queue full. `save` writes one byte of length and then
    // that many bytes, so every register above has just moved.
    cpu.set_prefetch_queue(&[0x90, 0x90, 0x90, 0x90, 0x90, 0x90])
        .expect("the queue takes six bytes on an x86-64 part");
    check(&target, "with six bytes queued");

    // Writes land, on the far side of the same hole.
    target
        .write_register(0, 15, &0x1234_5678_9abc_def0u64.to_le_bytes())
        .expect("r15 is writable");
    assert_eq!(cpu.reg(Reg::R15), 0x1234_5678_9abc_def0);
    target
        .write_register(0, 16, &0x0000_7fff_0000_1000u64.to_le_bytes())
        .expect("rip is writable");
    assert_eq!(cpu.reg(Reg::Rip), 0x0000_7fff_0000_1000);
    // A control word gdb declares as thirty-two bits and the core keeps as
    // sixteen.
    target
        .write_register(0, 32, &0x0000_027fu32.to_le_bytes())
        .expect("fctrl is writable");
    assert_eq!(cpu.x87().control, 0x027f);

    // A whole-register-file write round-trips.
    let packet = target.read_registers(0).expect("the register file");
    target
        .write_registers(0, &packet)
        .expect("the whole file is writable");
    assert_eq!(
        target.read_registers(0).expect("read back"),
        packet,
        "the register file does not read back what was written to it"
    );
}

/// Which of `cpu.x86`'s two register views a board gets, and why.
///
/// The map is per class and the register file is per instance, so something has
/// to choose; the width of the address space the board gave its core is what
/// does. This pins both halves, because the failure mode is silent in both
/// directions — a 64-bit guest debugged through a 32-bit window shows no
/// `R8`-`R15`, and a real-mode guest presented as x86-64 gets disassembled as
/// x86-64.
#[cfg(feature = "cpu-x86")]
#[test]
fn the_register_view_follows_the_address_space_width() {
    use rsemu::host::gdb::DebugTarget;

    let (mut wide, _) = x86_board_with_core("gdb-x86-64.machine", X86_64_MINI);
    let target = MachineTarget::new(&mut wide);
    let arch = target.arch(0).expect("a map");
    assert_eq!(arch.architecture, Some("i386:x86-64"));
    assert_eq!(arch.regs[0].name, "rax");

    let (mut narrow, _) = x86_board_with_core("gdb-x86-32.machine", X86_32_MINI);
    let target = MachineTarget::new(&mut narrow);
    let arch = target.arch(0).expect("a map");
    assert_eq!(arch.architecture, None);
    assert_eq!(arch.regs[0].name, "eax");
    // The i386 view is `org.gnu.gdb.i386.core` too, and gdb only accepts that
    // name with the x87 block after the integer file.
    assert_eq!(arch.feature, "org.gnu.gdb.i386.core");
    assert_eq!(arch.regs.len(), 32);
    assert_eq!(arch.regs[16].name, "st0");
    assert_eq!(arch.regs[31].name, "fop");

    // And the 20-bit fixture the session test drives, which is the case that
    // would break loudly if the threshold ever moved.
    let mut mini = board();
    let target = MachineTarget::new(&mut mini);
    assert_eq!(target.arch(0).expect("a map").regs[0].name, "eax");
}

/// `cpu.i8086` is the same core under its older class name, and a machine file
/// that uses it must get threads rather than silence.
#[cfg(feature = "cpu-x86")]
#[test]
fn the_older_class_name_is_debuggable_too() {
    let x86 = rsemu::host::gdb::arch::for_class("cpu.x86").expect("cpu.x86 has a map");
    let i8086 = rsemu::host::gdb::arch::for_class("cpu.i8086").expect("cpu.i8086 has a map");
    assert_eq!(i8086.regs.len(), x86.regs.len());
    assert_eq!(i8086.feature, x86.feature);
    assert_eq!(i8086.pc, x86.pc);
    for (a, b) in i8086.regs.iter().zip(x86.regs) {
        assert_eq!(a, b, "the two views of one core disagree");
    }
    // The versions do *not* agree, and that is the core's defect rather than
    // the map's — see `src/host/gdb/arch.rs`'s `I8086`. The map is verified
    // against whatever each class claims, so both `check()`s pass; this asserts
    // the drift is still there so that fixing it fails here and gets the
    // comment removed.
    assert!(i8086.check() && x86.check());
    assert_ne!(
        i8086.class.version, x86.class.version,
        "`cpu.i8086` and `cpu.x86` now agree on a version — good; drop the \
         note in `arch.rs` and give this map `verified_version` {}",
        x86.class.version
    );
}

/// The 64-bit half of phase 9's gate: a real `gdb`, reading a 64-bit register
/// file off a 64-bit guest.
///
/// The claim being proved is narrow and was the largest gap in the debug
/// surface: gdb's AMD64 gdbarch accepts `org.gnu.gdb.i386.core` only when it
/// carries all forty registers it expects — the integer sixteen at full width,
/// `rip`, `eflags`, the six selectors and the x87 block — so a session that
/// prints `$r15` and `$rax` with their high halves intact is a session gdb
/// built out of *this* description rather than out of its own fallback layout.
/// The rejection warning is asserted absent for the same reason it is asserted
/// present in the AArch64 test.
#[cfg(feature = "cpu-x86")]
#[test]
fn a_real_gdb_sees_a_sixty_four_bit_register_file() {
    use rsemu::core::Captured;
    use rsemu::cpu::x86::{Reg, X86};

    let Some(gdb) = find_gdb() else {
        println!("skipping: no gdb binary. Set $RSEMU_GDB, or install gdb.");
        return;
    };
    if !knows(&gdb, "i386:x86-64") {
        println!(
            "skipping: `{gdb}` has no x86-64 gdbarch, so it cannot read a 64-bit \
             register file however the description is written."
        );
        return;
    }

    let cores: Arc<Captured<X86>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let server = Server::start(move || {
        let (machine, cpu) = x86_board_with_core("gdb-x86-64.machine", X86_64_MINI);
        // Seeded here rather than before the server starts, because the board
        // is built on the session's own thread — a `Machine` is not `Send`.
        cpu.set_reg(Reg::Rax, X64_RAX);
        cpu.set_reg(Reg::Rbx, X64_RBX);
        cpu.set_reg(Reg::R15, X64_R15);
        kept.push(&cpu);
        machine
    });
    let port = server.addr.port();

    let mut script = Vec::new();
    if std::env::var_os("RSEMU_GDB_DEBUG_REMOTE").is_some() {
        script.push(String::from("set debug remote 1"));
    }
    script.extend([
        format!("target remote 127.0.0.1:{port}"),
        // No `set architecture`: the description is supposed to be enough, and
        // saying which architecture it settled on is half the assertion.
        // `show architecture` rather than `$_gdb_setting_str("architecture")`,
        // which reports the *setting* — "auto" — and not what auto resolved to.
        String::from("show architecture"),
        // The registers a 32-bit window cannot show: two with their high half
        // set, and one that only exists in long mode.
        String::from("printf \"RSEMU rax [%#llx]\\n\", (unsigned long long) $rax"),
        String::from("printf \"RSEMU rbx [%#llx]\\n\", (unsigned long long) $rbx"),
        String::from("printf \"RSEMU r15 [%#llx]\\n\", (unsigned long long) $r15"),
        // And the other direction: gdb writes sixty-four bits, and the core is
        // asked afterwards whether it got them.
        format!("set $r14 = {X64_R14:#x}"),
        String::from("printf \"RSEMU r14 [%#llx]\\n\", (unsigned long long) $r14"),
        // `info registers` is the shape a user recognises, and it is also what
        // reads the whole `g` packet through the description's own layout.
        String::from("info registers rax rbx r14 r15 rip eflags"),
        String::from("info float"),
        String::from("monitor devices"),
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

    says("(currently \"i386:x86-64\")");
    says(&format!("RSEMU rax [{X64_RAX:#x}]"));
    says(&format!("RSEMU rbx [{X64_RBX:#x}]"));
    says(&format!("RSEMU r15 [{X64_R15:#x}]"));
    says(&format!("RSEMU r14 [{X64_R14:#x}]"));
    says("cpu.x86");

    // The description is accepted, which is what claiming `i386:x86-64` and
    // supplying gdb's own forty registers buys.
    assert!(
        !all.contains("Architecture rejected target-supplied description"),
        "gdb rejected the x86-64 target description:\n{all}"
    );
    for bad in [
        "Truncated register",
        "Remote failure reply",
        "Remote communication error",
        "Ignoring packet error",
    ] {
        assert!(
            !all.contains(bad),
            "gdb reported `{bad}`:\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
        );
    }

    // What gdb wrote is in the core, at full width.
    let cpu = cores.last().expect("the fixture captured its core");
    assert_eq!(
        cpu.reg(Reg::R14),
        X64_R14,
        "gdb's sixty-four bit write did not reach the core"
    );

    drop(server);
}
// ---------------------------------------------------------------------------
// AArch64
// ---------------------------------------------------------------------------

/// The AArch64 board: a clock, a megabyte of RAM and a core, and nothing else.
///
/// Built from a string rather than taken from `machines/`, for the reason
/// `tests/a64_engines.rs` gives: `arm64-virt` wants a kernel image and
/// `a64-mini` wants a firmware, and a debugger test should not be a boot test.
/// The reset vector is zero, which is also where the program is, so `$pc` is
/// `0` the instant GDB attaches.
#[cfg(feature = "cpu-arm-a64")]
const A64_MINI: &str = r#"
machine "gdb-a64" {
  osc sysclk = 100000000 Hz
  space mem { width = 64 }
  object cpu "cpu.arm.a64" {
    clock  = sysclk
    space  = mem
    cpu    = "cortex-a53"
    reset  = 0x00000000
  }
  object dram "ram" { size = 1M }
  map mem 0x00000000 size 1M = dram
}
"#;

/// Where the guest stores its byte, and what it stores.
#[cfg(feature = "cpu-arm-a64")]
const A64_SENTINEL_ADDR: u64 = 0x600;
#[cfg(feature = "cpu-arm-a64")]
const A64_SENTINEL: u64 = 0x42;

/// The guest program, at `0`.
///
/// ```text
///   0x00  movz x0, #0x42
///   0x04  movz x1, #0x600
///   0x08  strb w0, [x1]
///   0x0c  ldxr x2, [x1]      arms the exclusive monitor
///   0x10  b    0x00
/// ```
///
/// Encodings from *Arm Architecture Reference Manual for A-profile*
/// (DDI 0487), C6.2: `MOVZ` (32-bit immediate, `hw = 0`), `STRB` (unsigned
/// offset), `LDXR` and `B`.
///
/// The `LDXR` is not decoration. `cpu.arm.a64`'s snapshot writes the exclusive
/// monitor's address **only when the monitor is armed**, so every field after
/// it — `PSTATE` and both stack pointers among them — moves by eight bytes the
/// moment this instruction retires. `src/host/gdb/arch.rs`'s `Computed` hook
/// exists for exactly that, and this is what makes it a tested claim rather
/// than a comment.
#[cfg(feature = "cpu-arm-a64")]
const A64_PROGRAM: [u32; 5] = [
    0xd280_0840,
    0xd280_c001,
    0x3900_0020,
    0xc85f_7c22,
    0x17ff_fffc,
];

/// Build the board, and keep the core so a test can ask it what it thinks.
#[cfg(feature = "cpu-arm-a64")]
fn a64_board_with_core() -> (Machine, Arc<rsemu::cpu::arm::a64::Cpu>) {
    use rsemu::core::Captured;
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;
    use rsemu::cpu::arm::a64::Cpu;

    let cores: Arc<Captured<Cpu>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut bindings = catalog::bindings().expect("this build's bindings");
    bindings.replace("cpu.arm.a64", move |props| {
        let cpu = Arc::new(Cpu::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let options = rsemu::machine::BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(bindings);
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build("gdb-a64.machine", A64_MINI, &registry, &options)
        .unwrap_or_else(|e| panic!("the AArch64 fixture does not realize: {e}"));
    let cpu = cores.take().expect("the binding captured the core");

    // `build` realizes *and* resets, and a cold reset zeroes RAM, so the
    // program goes in afterwards.
    let space = cpu.space().expect("the core has its space");
    for (i, word) in A64_PROGRAM.iter().enumerate() {
        space
            .write(
                4 * i as u64,
                Width::U32,
                u64::from(*word),
                MemAttrs::DEFAULT,
            )
            .expect("the program fits in RAM");
    }
    (machine, cpu)
}

/// The same board, for [`Server::start`].
#[cfg(feature = "cpu-arm-a64")]
fn a64_board() -> Machine {
    a64_board_with_core().0
}

/// The AArch64 register map, checked against the core it describes.
///
/// The map is a table of byte offsets into a snapshot chunk, so the one thing
/// that can go wrong silently is an offset that names the wrong bytes: GDB
/// would show plausible numbers that are not the machine's. Every register
/// below is therefore compared against the core's *own* accessor, and the two
/// that no offset can name — `SP`, which is one of two banked registers, and
/// `cpsr`, which is four fields composed — are checked on both sides of the
/// `LDXR` that moves them.
#[cfg(feature = "cpu-arm-a64")]
#[test]
fn the_aarch64_register_map_agrees_with_the_core() {
    use rsemu::host::gdb::DebugTarget;

    /// Register numbers, as `org.gnu.gdb.aarch64.core` numbers them.
    const SP: usize = 31;
    const PC: usize = 32;
    const CPSR: usize = 33;

    let (mut m, cpu) = a64_board_with_core();
    let mut target = MachineTarget::new(&mut m);
    assert_eq!(target.cpu_count(), 1, "one AArch64 core");
    let arch = target.arch(0).expect("a register map");
    assert_eq!(arch.class.name, "cpu.arm.a64");
    assert_eq!(arch.feature, "org.gnu.gdb.aarch64.core");
    assert_eq!(arch.architecture, Some("aarch64"));
    // 31 * 8 + sp + pc + a four-byte cpsr: what GDB's own AArch64 layout is.
    assert_eq!(arch.packet_len(), 268);
    assert_eq!(arch.regs[SP].name, "sp");
    assert_eq!(arch.regs[PC].name, "pc");
    assert_eq!(arch.regs[CPSR].name, "cpsr");

    let u64_of =
        |bytes: &[u8]| u64::from_le_bytes(<[u8; 8]>::try_from(bytes).expect("eight bytes"));
    let u32_of = |bytes: &[u8]| u32::from_le_bytes(<[u8; 4]>::try_from(bytes).expect("four bytes"));

    // The general registers, with a distinct value in each so a table that is
    // off by one entry cannot pass.
    for i in 0..31u32 {
        cpu.set_x(i, 0xa500_0000_0000_0000 | u64::from(i));
    }
    let g = target.read_registers(0).expect("the whole register file");
    assert_eq!(g.len(), 268);
    for i in 0..31usize {
        assert_eq!(
            u64_of(&g[i * 8..i * 8 + 8]),
            0xa500_0000_0000_0000 | i as u64,
            "x{i} does not come out of the chunk where the map says it does"
        );
    }
    assert_eq!(u64_of(&g[248..256]), cpu.sp(), "sp");
    assert_eq!(u64_of(&g[256..264]), cpu.pc(), "pc");
    assert_eq!(u32_of(&g[264..268]), cpu.sysregs().spsr() as u32, "cpsr");

    // `SP` is banked, and which bank it names is `PSTATE`'s business. Give the
    // two different values and check the debugger follows the selection rather
    // than always reading `SP_EL0`.
    let mut sys = cpu.sysregs();
    sys.sp_el0 = 0x1111_0000;
    sys.sp_el1 = 0x2222_0000;
    sys.spsel = true;
    cpu.set_sysregs(sys);
    assert_eq!(
        u64_of(&target.read_register(0, SP).expect("sp")),
        0x2222_0000,
        "with SPSel set at EL1 the debugger must show SP_EL1"
    );
    let mut sys = cpu.sysregs();
    sys.spsel = false;
    cpu.set_sysregs(sys);
    assert_eq!(
        u64_of(&target.read_register(0, SP).expect("sp")),
        0x1111_0000,
        "with SPSel clear the debugger must show SP_EL0"
    );

    // And a write goes to the bank that is selected, leaving the other alone.
    target
        .write_register(0, SP, &0x3333_0000u64.to_le_bytes())
        .expect("sp is writable");
    assert_eq!(cpu.sp(), 0x3333_0000);
    assert_eq!(cpu.sysregs().sp_el0, 0x3333_0000);
    assert_eq!(cpu.sysregs().sp_el1, 0x2222_0000, "the other bank moved");

    // `cpsr` round-trips through `PSTATE`'s four fields.
    let el1h = 0xf000_0000u32 | 0x3c0 | 0b0101;
    target
        .write_register(0, CPSR, &el1h.to_le_bytes())
        .expect("cpsr is writable");
    assert_eq!(cpu.sysregs().spsr() as u32, el1h);
    assert_eq!(
        u32_of(&target.read_register(0, CPSR).expect("cpsr")),
        el1h,
        "cpsr does not read back what was written to it"
    );
    // ... and a `PSTATE` naming a level this core does not have is refused
    // rather than written, because a chunk carrying it would fail to load.
    let el3h = 0x0000_000du32; // M[3:0] = 0b1101: EL3h.
    assert!(
        target.write_register(0, CPSR, &el3h.to_le_bytes()).is_err(),
        "an exception level this core does not have must be refused"
    );
    assert_eq!(
        cpu.sysregs().spsr() as u32,
        el1h,
        "the refusal wrote anyway"
    );
}

/// A whole-register-file write, which is where the two computed registers can
/// fight.
///
/// GDB's `G` packet carries every register in `g`-packet order, and that order
/// puts `sp` at 31 and `cpsr` at 33 — so a stub that walks it once writes the
/// stack pointer into the bank the *old* `PSTATE` selected and then changes
/// the selection, and the value the user asked for is in the wrong bank. This
/// is the case that catches it: the packet says "EL1h, and SP is this", and
/// the core has to end up with that in `SP_EL1`.
#[cfg(feature = "cpu-arm-a64")]
#[test]
fn a_whole_register_file_write_lands_the_stack_pointer_in_the_selected_bank() {
    use rsemu::host::gdb::DebugTarget;

    let (mut m, cpu) = a64_board_with_core();
    let mut target = MachineTarget::new(&mut m);

    // Start at EL1t, so `SP` names `SP_EL0` and the packet below has to move
    // the selection before its stack pointer means anything.
    let mut sys = cpu.sysregs();
    sys.spsel = false;
    sys.sp_el0 = 0;
    sys.sp_el1 = 0;
    cpu.set_sysregs(sys);

    let mut packet = target.read_registers(0).expect("the register file");
    packet[248..256].copy_from_slice(&0x7fff_0000u64.to_le_bytes());
    // NZCV all set, DAIF all masked, `M[3:0] = 0b0101`: EL1h.
    packet[264..268].copy_from_slice(&(0xf000_0000u32 | 0x3c0 | 0b0101).to_le_bytes());
    target
        .write_registers(0, &packet)
        .expect("the whole file is writable");

    assert_eq!(cpu.sysregs().el.bits(), 1, "PSTATE.EL did not move to EL1");
    assert!(cpu.sysregs().spsel, "SPSel did not move");
    assert_eq!(
        cpu.sysregs().sp_el1,
        0x7fff_0000,
        "the stack pointer went into the bank PSTATE selected *before* the packet"
    );
    assert_eq!(cpu.sysregs().sp_el0, 0, "and not into the other one");
    assert_eq!(
        target.read_registers(0).expect("read back"),
        packet,
        "the register file does not read back what was written to it"
    );
}

/// The same map, on the far side of the variable-length field it has to see
/// through.
///
/// `cpu.arm.a64`'s snapshot writes the exclusive monitor's address only when
/// the monitor is armed, so `LDXR` moves `PSTATE` and both stack pointers
/// eight bytes further down the chunk. A map of constants would read the
/// wrong bytes from here on and say nothing about it.
#[cfg(feature = "cpu-arm-a64")]
#[test]
fn the_exclusive_monitor_does_not_move_the_register_map() {
    use rsemu::host::gdb::DebugTarget;

    const SP: usize = 31;
    const CPSR: usize = 33;

    let (mut m, cpu) = a64_board_with_core();
    let mut target = MachineTarget::new(&mut m);
    let mut sys = cpu.sysregs();
    sys.sp_el1 = 0x4444_0000;
    sys.spsel = true;
    cpu.set_sysregs(sys);

    let sp_before = target.read_register(0, SP).expect("sp");
    let cpsr_before = target.read_register(0, CPSR).expect("cpsr");
    assert_eq!(sp_before, 0x4444_0000u64.to_le_bytes());

    // Step past `movz`, `movz`, `strb` and into the `LDXR`, which arms the
    // monitor. Four steps rather than a loop with a condition, because that is
    // what the program is.
    for _ in 0..4 {
        target.step(0).expect("a step");
    }
    let mut stored = [0u8; 1];
    target
        .read_memory(0, A64_SENTINEL_ADDR, &mut stored)
        .expect("the store landed");
    assert_eq!(
        u64::from(stored[0]),
        A64_SENTINEL,
        "the guest never ran its store, so the monitor is probably not armed \
         either and this test is checking nothing"
    );

    // Everything after the monitor has moved eight bytes, and neither register
    // may notice.
    assert_eq!(
        target.read_register(0, SP).expect("sp"),
        sp_before,
        "the stack pointer moved when the exclusive monitor was armed"
    );
    assert_eq!(
        target.read_register(0, CPSR).expect("cpsr"),
        cpsr_before,
        "PSTATE moved when the exclusive monitor was armed"
    );
    assert_eq!(
        u64::from_le_bytes(
            <[u8; 8]>::try_from(&target.read_register(0, SP).expect("sp")[..]).expect("8")
        ),
        cpu.sp(),
        "and it still agrees with the core"
    );

    // A write on this side of the hole lands too.
    target
        .write_register(0, SP, &0x5555_0000u64.to_le_bytes())
        .expect("sp is writable");
    assert_eq!(cpu.sp(), 0x5555_0000);
}

/// A breakpoint and a step on the AArch64 board, with no `gdb` involved.
///
/// The register map is what a debugger reads; this is what it *does*. It runs
/// everywhere, so `arm64-virt`'s debug surface is covered on a host whose
/// `gdb` has never heard of AArch64 — which is most of them.
#[cfg(feature = "cpu-arm-a64")]
#[test]
fn an_aarch64_guest_stops_where_the_breakpoint_is() {
    use rsemu::host::gdb::{DebugTarget, StopKind};

    let (mut m, cpu) = a64_board_with_core();
    let mut target = MachineTarget::new(&mut m);
    assert_eq!(cpu.pc(), 0, "the core resets to its RVBAR");

    // `strb w0, [x1]`, the third instruction: reached only by going round the
    // loop, so a stub that never checks would run for ever.
    target.add_breakpoint(0x08, false).expect("Z0");
    target.begin_resume();
    let mut stop = None;
    for _ in 0..8 {
        if let Some(hit) = target.resume().expect("the machine advances") {
            stop = Some(hit);
            break;
        }
    }
    let stop = stop.expect("the breakpoint was never reached");
    assert_eq!(stop.kind, StopKind::Breakpoint { hardware: false });
    assert_eq!(stop.cpu, 0);
    assert_eq!(cpu.pc(), 0x08, "stopped somewhere else");
    assert_eq!(cpu.x(0), A64_SENTINEL, "the first movz did not run");

    // One instruction, and exactly one.
    target.step(0).expect("a step");
    assert_eq!(cpu.pc(), 0x0c);
}

/// The AArch64 half of phase 9's gate: a real `gdb`, driving `arm64-virt`'s
/// core.
///
/// Skips wherever the distribution's `gdb` has no AArch64 gdbarch, which on an
/// x86-64 developer machine is the usual case — `gdb-multiarch`, or a
/// cross `aarch64-linux-gnu-gdb` named in `$RSEMU_GDB`, is what runs it.
///
/// Unlike the x86 session below, GDB **accepts** this target description
/// rather than rejecting it and falling back: `org.gnu.gdb.aarch64.core` with
/// `x0`-`x30`, `sp`, `pc` and `cpsr` is exactly what its AArch64 gdbarch asks
/// for, so `set architecture` is not even needed. That is asserted, because it
/// is the difference the map was written to make.
#[cfg(feature = "cpu-arm-a64")]
#[test]
fn a_real_gdb_debugs_an_aarch64_guest_end_to_end() {
    let Some(gdb) = find_gdb() else {
        println!("skipping: no gdb binary. Set $RSEMU_GDB, or install gdb.");
        return;
    };
    if !knows(&gdb, "aarch64") {
        println!(
            "skipping: `{gdb}` has no aarch64 gdbarch. Install gdb-multiarch, or point \
             $RSEMU_GDB at a cross gdb, to run this."
        );
        return;
    }

    let server = Server::start(a64_board);
    let port = server.addr.port();

    let mut script = Vec::new();
    if std::env::var_os("RSEMU_GDB_DEBUG_REMOTE").is_some() {
        script.push(String::from("set debug remote 1"));
    }
    script.extend([
        format!("target remote 127.0.0.1:{port}"),
        // No `set architecture`: the description is supposed to be enough.
        // `show architecture` and not `$_gdb_setting_str("architecture")`: the
        // setting is "auto", and what it resolved to is the thing being tested.
        String::from("show architecture"),
        String::from("printf \"RSEMU pc0 [%#x]\\n\", $pc"),
        // Run to the store, which is three instructions in.
        String::from("break *0x8"),
        String::from("continue"),
        String::from("printf \"RSEMU pc1 [%#x]\\n\", $pc"),
        String::from("printf \"RSEMU x0 [%#x]\\n\", $x0"),
        String::from("printf \"RSEMU x1 [%#x]\\n\", $x1"),
        String::from("delete 1"),
        // One instruction: the store.
        String::from("stepi"),
        format!("printf \"RSEMU stored [%#x]\\n\", *(unsigned char *) {A64_SENTINEL_ADDR:#x}"),
        // The two registers no byte offset can name.
        String::from("set $sp = 0x8000"),
        String::from("printf \"RSEMU sp [%#x]\\n\", $sp"),
        String::from("printf \"RSEMU cpsr [%#x]\\n\", $cpsr"),
        // A watchpoint on the byte the loop rewrites every time round.
        format!("set *(unsigned char *) {A64_SENTINEL_ADDR:#x} = 0"),
        format!("watch *(unsigned char *) {A64_SENTINEL_ADDR:#x}"),
        String::from("continue"),
        format!("printf \"RSEMU watched [%#x]\\n\", *(unsigned char *) {A64_SENTINEL_ADDR:#x}"),
        String::from("delete"),
        String::from("monitor devices"),
        String::from("monitor translate 8"),
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

    says("(currently \"aarch64\")");
    says("RSEMU pc0 [0]");
    says("Breakpoint 1");
    says("RSEMU pc1 [0x8]");
    says("RSEMU x0 [0x42]");
    says("RSEMU x1 [0x600]");
    says("RSEMU stored [0x42]");
    says("RSEMU sp [0x8000]");
    says("Old value = 0");
    says("New value = 66");
    says("RSEMU watched [0x42]");
    says("cpu.arm.a64");
    says("0x8 -> 0x8 (identity)");

    // The description is accepted, which is the whole point of naming the
    // feature `org.gnu.gdb.aarch64.core` and claiming the architecture.
    assert!(
        !all.contains("Architecture rejected target-supplied description"),
        "gdb rejected the AArch64 target description:\n{all}"
    );
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

    drop(server);
}
