//! Does `q35-uefi` assemble, do its two flash banks behave like flash, and how
//! far does a real UEFI firmware get on it?
//!
//! Three questions, and only the last one needs anything downloaded. The first
//! two run on every `cargo test` and hold the board's own claims: that the code
//! bank ends at the reset vector, that the variable bank answers the probe EDK
//! II's `OvmfPkg` flash driver opens with, and that a program clears bits and
//! an erase is what puts them back.
//!
//! The third is [`a_uefi_firmware_from_the_environment_reaches_its_console`],
//! gated on `RSEMU_OVMF_CODE` exactly as `tests/q35_linux.rs` is gated on
//! `RSEMU_KERNEL` and for the same reasons: the image is several megabytes,
//! it is not ours, and `CLAUDE.md` forbids vendoring a fixture. Nothing here is
//! committed and nothing is required for `cargo test`.
//!
//! ```console
//! scripts/fetch-testdata.sh ovmf
//! RSEMU_OVMF_CODE=testdata/x86/OVMF_CODE.fd \
//! RSEMU_OVMF_VARS=testdata/x86/OVMF_VARS.fd \
//! RSEMU_OVMF_VARS_OUT=testdata/x86/OVMF_VARS.fd \
//! RSEMU_OVMF_STOP_AT='Shell>' \
//!     cargo test --release --features machine-q35-uefi --test q35_uefi -- --nocapture
//! ```
//!
//! | Variable | What it does |
//! | --- | --- |
//! | `RSEMU_OVMF_CODE` | the firmware bank's image. Unset, the boot test skips. |
//! | `RSEMU_OVMF_VARS` | the variable bank's. Unset, the store comes up erased. |
//! | `RSEMU_OVMF_VARS_OUT` | writes the variable bank back out when the run ends; pointing it at the file `RSEMU_OVMF_VARS` read makes the next run a reboot. |
//! | `RSEMU_OVMF_MS` | virtual milliseconds to run for (default 60000). |
//! | `RSEMU_OVMF_EXTMEM` | how much memory above 1 MiB the board has. |
//! | `RSEMU_OVMF_STOP_AT` | end the run at the first output containing this. |
//! | `RSEMU_OVMF_EXPECT` | a string the guest must have printed for the test to pass. |
//! | `RSEMU_OVMF_INPUT` | `marker=>text` steps, one per line, typed at the console. |
//! | `RSEMU_KERNEL_TRACE` | print where the processor is once per virtual millisecond. Shared with the kernel boots, because the run loop is. |
//!
//! **Everything printed as evidence is a byte the guest itself wrote to COM1.**
//! The firmware is run, never read (`ROADMAP.md` §1).

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-q35",
    feature = "dev-pc-apic",
    feature = "dev-pc-hpet",
    feature = "dev-flash-cfi",
    feature = "machine-q35-uefi"
))]

mod x86boot;

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::{AddressSpace, MemAttrs};
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::host::chardev::CharPort;
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

use x86boot::Script;

/// How long to let the board run, in virtual milliseconds.
///
/// A ceiling rather than a target: the run stops early when the processor stops
/// making progress or when the guest prints `RSEMU_OVMF_STOP_AT`.
const DEFAULT_MS: u64 = 60_000;

/// The top of the address space, which is where the flash ends.
const TOP: u64 = 0x1_0000_0000;

/// Everything the board needs, with a `cpu.x86` that pushes what it builds into
/// `cpus`.
///
/// The same shape `tests/q35_linux.rs` uses, and for the same reason: `Device`
/// keeps `Any` out of its supertrait chain, so construction is the one moment
/// the concrete type exists.
fn bindings(cpus: &Arc<Captured<X86>>) -> Bindings {
    let mut b = rsemu::machine::catalog::bindings().expect("this build's bindings");
    let kept = Arc::clone(cpus);
    b.replace("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::X86_64)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    b
}

/// Build the board from its own machine file, with `code` and `vars` in the two
/// banks.
fn board(
    code: Vec<u8>,
    vars: Vec<u8>,
    params: &[(&str, String)],
) -> Result<(Machine, Arc<X86>, Arc<CharPort>), String> {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings(&cpus));
    for (name, value) in params {
        options = options.with_param(*name, value.as_str());
    }
    options.realize.media.insert("flash0", code);
    options.realize.media.insert("flash1", vars);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = build(
        "q35-uefi.machine",
        rsemu::machine::catalog::Q35_UEFI.source,
        &registry,
        &options,
    )
    .map_err(|e| format!("{e}"))?;
    machine.reset(ResetKind::Cold);
    machine.sweep();
    let console = rsemu::host::chardev::ports::open(&options.realize.hosts, "console")
        .expect("the 16550 opened the board's console port");
    let cpu = cpus.take().expect("the constructor kept a handle");
    Ok((machine, cpu, console))
}

/// The board with both sockets stuffed and nothing programmed into them.
fn bare_board() -> Machine {
    match board(Vec::new(), Vec::new(), &[]) {
        Ok((machine, _cpu, _console)) => machine,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Read a byte out of a space the way a guest would.
fn read8(space: &AddressSpace, at: u64) -> u8 {
    space
        .read(at, Width::U8, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("read of {at:#x} faulted: {e:?}")) as u8
}

/// Write a byte into a space the way a guest would.
fn write8(space: &AddressSpace, at: u64, value: u8) {
    space
        .write(at, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("write of {at:#x} faulted: {e:?}"));
}

// ---------------------------------------------------------------------------
// the board, without a firmware
// ---------------------------------------------------------------------------

/// The two banks are one contiguous run of flash whose top is the reset vector.
///
/// This is the board's central claim and the one thing a UEFI machine cannot
/// get wrong: an x86 processor fetches from `0xfffffff0` (SDM Vol. 3A §9.1.4),
/// so the last sixteen bytes of the code bank *are* the reset vector, and the
/// variable bank has to sit immediately below rather than anywhere convenient
/// — a split OVMF build has the distance between the two compiled into it.
#[test]
fn the_two_banks_run_contiguously_up_to_the_reset_vector() {
    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");

    // The defaults: 2 MiB of flash of which 128 KiB is the variable store.
    const FLASH: u64 = 2 * 1024 * 1024;
    const VARS: u64 = 128 * 1024;

    // Erased, at both ends and at the seam. An unprogrammed NOR part reads all
    // ones; a hole in the map would read all ones too, so the assertion that
    // separates them is the *write* below.
    for at in [
        TOP - FLASH,        // the bottom of the variable store
        TOP - FLASH + VARS, // the bottom of the firmware bank
        TOP - 0x10,         // the reset vector
        TOP - 1,            // the last byte of the array
    ] {
        assert_eq!(read8(mem, at), 0xff, "{at:#x} is erased flash");
    }

    // And nothing decodes just below the pair, which is what makes the seam a
    // seam rather than the middle of one large window.
    assert_eq!(
        mem.read(TOP - FLASH - 4, Width::U32, MemAttrs::DEFAULT)
            .unwrap_or(0xffff_ffff),
        0xffff_ffff,
        "below the flash is open bus"
    );

    // The old firmware socket is gone: `q35` maps a `pc.rom` at 0xf0000 and
    // this board maps nothing there at all.
    assert_eq!(
        mem.read(0xf_0000, Width::U32, MemAttrs::DEFAULT)
            .unwrap_or(0xffff_ffff),
        0xffff_ffff,
        "0xf0000-0xfffff is the BIOS socket on `q35` and open bus here"
    );
}

/// The probe EDK II's `OvmfPkg` flash driver opens with, on the variable bank.
///
/// `QemuFlashDetected` tells flash from RAM and from ROM by writing a **single
/// byte** command and reading the same address back: a part that answers with
/// its status register is flash, one that answers with the byte just written is
/// RAM, and one that answers with what was there before is ROM. Every cycle is
/// a byte, which is why this board wires an x8 part rather than the RISC-V
/// board's pair of x16s — and the assertion below is that the device gets all
/// three answers right.
#[test]
fn the_variable_bank_answers_the_flash_detection_probe() {
    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    const VARS: u64 = TOP - 2 * 1024 * 1024;

    // `0x70` is Read Status Register. A flash answers with SR, which is 0x80
    // — ready, no errors — and is neither the command nor the array byte.
    write8(mem, VARS, 0x70);
    let status = read8(mem, VARS);
    assert_eq!(status, 0x80, "SR.7 alone: ready, and not RAM's 0x70");

    // `0xff` is Read Array, and puts it back.
    write8(mem, VARS, 0xff);
    assert_eq!(
        read8(mem, VARS),
        0xff,
        "erased array, not a status register"
    );

    // The code bank is `readonly`, and that is `WP#` tied low rather than a
    // ROM: an Intel part still *answers* every command with `WP#` low — the
    // pin gates the lock bits, not the command interface (StrataFlash P30
    // datasheet, block locking) — so the probe above finds flash here too. What
    // it cannot do is change the array, and the status register says why.
    const CODE: u64 = TOP - 2 * 1024 * 1024 + 128 * 1024;
    write8(mem, CODE, 0x10);
    write8(mem, CODE, 0x00);
    let status = read8(mem, CODE);
    assert_eq!(
        status & 0x02,
        0x02,
        "SR.1: the program was refused because the block is locked"
    );
    write8(mem, CODE, 0x50); // clear status
    write8(mem, CODE, 0xff); // read array
    assert_eq!(
        read8(mem, CODE),
        0xff,
        "and the firmware bank still reads what it held"
    );
}

/// A program clears bits, an erase is the only thing that sets them, and both
/// happen through the window the firmware executes from.
///
/// Not a restatement of `src/dev/flash/cfi.rs`'s own tests: those exercise the
/// device, and this exercises the *board* — the same address the guest's
/// firmware would use, through the address space, at the width the driver uses.
#[test]
fn a_variable_store_program_clears_bits_and_an_erase_puts_them_back() {
    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    const VARS: u64 = TOP - 2 * 1024 * 1024;

    // Word program, in the `0x10` encoding the OvmfPkg driver uses rather than
    // the `0x40` one. Setup, data, then read array.
    write8(mem, VARS + 4, 0x10);
    write8(mem, VARS + 4, 0x0f);
    write8(mem, VARS + 4, 0xff);
    assert_eq!(read8(mem, VARS + 4), 0x0f);

    // A second program can only clear further. This is the property UEFI's
    // append-only variable log is built on: `0xf0` over `0x0f` is `0x00`.
    write8(mem, VARS + 4, 0x10);
    write8(mem, VARS + 4, 0xf0);
    write8(mem, VARS + 4, 0xff);
    assert_eq!(read8(mem, VARS + 4), 0x00, "a program only clears bits");

    // Block erase: setup at any address in the block, then confirm. 4 KiB
    // blocks, so this touches nothing above.
    write8(mem, VARS, 0x20);
    write8(mem, VARS, 0xd0);
    write8(mem, VARS, 0xff);
    assert_eq!(read8(mem, VARS + 4), 0xff, "an erase is what sets bits");
}

// ---------------------------------------------------------------------------
// and the firmware
// ---------------------------------------------------------------------------

/// Boot whatever `RSEMU_OVMF_CODE` points at and report what it printed.
///
/// Skipped, cleanly, when the variable is unset — which is every ordinary
/// `cargo test` run. `scripts/fetch-testdata.sh ovmf` copies a split OVMF out
/// of the distribution's own firmware package and prints the command line.
#[test]
fn a_uefi_firmware_from_the_environment_reaches_its_console() {
    let Ok(path) = std::env::var("RSEMU_OVMF_CODE") else {
        println!(
            "q35-uefi: set RSEMU_OVMF_CODE to a split UEFI firmware image to run one on this \
             board; see the module docs"
        );
        return;
    };
    let code = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let vars_path = std::env::var("RSEMU_OVMF_VARS").ok();
    let vars = vars_path
        .as_ref()
        .map(|p| std::fs::read(p).unwrap_or_else(|e| panic!("{p}: {e}")))
        .unwrap_or_default();

    // The board's two sizes come out of the two images, so a 2 MiB build and a
    // 4 MiB one both work with nothing written down: the pair has to end at
    // 4 GiB, and that is the only constraint there is.
    let mut params: Vec<(&str, String)> = vec![
        ("flash", format!("{}", code.len() + vars.len())),
        ("vars", format!("{}", vars.len().max(0x1000))),
    ];
    if let Ok(extmem) = std::env::var("RSEMU_OVMF_EXTMEM") {
        params.push(("extmem", extmem));
    }
    println!(
        "q35-uefi: {} bytes of firmware and {} bytes of variable store, mapped at {:#x}",
        code.len(),
        vars.len(),
        TOP - (code.len() + vars.len()) as u64
    );

    let (mut m, cpu, console) = match board(code, vars, &params) {
        Ok(built) => built,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let ms: u64 = std::env::var("RSEMU_OVMF_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_MS);

    let script = Script::from_vars("RSEMU_OVMF_INPUT", "RSEMU_OVMF_STOP_AT");
    println!("q35-uefi: what the guest wrote to its serial port at 0x3f8:");
    let run = x86boot::run(
        &mut m,
        &cpu,
        &console,
        GlobalTime::from_nanos(ms * 1_000_000),
        &script,
    );
    x86boot::report("q35-uefi", &m, &cpu, &run, &script);
    report_chipset(&m);
    write_back_the_variable_store(&m);
    assert_reached_uefi(&run, &script);
}

/// What a run of a UEFI firmware has to have shown to count.
///
/// `x86boot::assert_booted` is the kernel's version of this and looks for
/// `Linux version`. A firmware cannot be held to that, and the reason is worth
/// stating rather than working around: **a `RELEASE` EDK II says nothing at all
/// until its console driver comes up**, and its `DEBUG()` output goes to I/O
/// port `0x402` behind a detect that this board fails, so a run that gets most
/// of the way through DXE and a run that faulted in SEC print the same nothing.
/// `docs/platforms/q35-uefi.md` has that ledger and what would change it.
///
/// So the standing assertion is the one that is falsifiable today: the reset
/// vector executed out of the flash and the processor reached **long mode**,
/// which is SEC's whole job and which nothing but a working code bank can
/// produce. `RSEMU_OVMF_EXPECT` adds a string the guest must have printed, for
/// the day one does.
fn assert_reached_uefi(run: &x86boot::Run, script: &Script) {
    assert!(
        run.protected,
        "the firmware never left real mode; the reset vector did not execute out of the flash"
    );
    assert!(
        run.long,
        "a UEFI firmware is 64-bit code and this one never reached long mode"
    );
    if let Ok(want) = std::env::var("RSEMU_OVMF_EXPECT") {
        assert!(
            run.text.contains(&want),
            "the guest never printed RSEMU_OVMF_EXPECT ({want:?})"
        );
    } else if run.text.is_empty() {
        println!(
            "q35-uefi: the firmware printed nothing on COM1, which a RELEASE EDK II does until \
             its console driver loads; docs/platforms/q35-uefi.md says how far it got and what \
             stopped it"
        );
    }
    assert_eq!(
        run.typed,
        script.steps.len(),
        "the guest never printed the marker for step {} of RSEMU_OVMF_INPUT",
        run.typed + 1
    );
    assert!(
        script.stop_at.is_empty() || run.reached,
        "the guest never printed RSEMU_OVMF_STOP_AT ({:?})",
        script.stop_at
    );
}

/// Write the variable bank back out, if `RSEMU_OVMF_VARS_OUT` says where.
///
/// Pointing it at the file `RSEMU_OVMF_VARS` was read from makes the next run a
/// reboot, and a variable written in one run is there in the next — which is
/// the whole reason the store is a flash device rather than memory.
///
/// The read is a **debug** read of the mapped window, which is exactly what
/// `MemAttrs::debug` is for: a `flash.cfi` left in status or identifier mode by
/// a firmware that never issued a final Read Array would otherwise hand back
/// its status register instead of its contents.
fn write_back_the_variable_store(m: &Machine) {
    let Ok(path) = std::env::var("RSEMU_OVMF_VARS_OUT") else {
        return;
    };
    let length = |var: &str| -> Option<u64> {
        std::env::var(var)
            .ok()
            .and_then(|p| std::fs::metadata(p).ok())
            .map(|meta| meta.len())
    };
    let (Some(vars_len), Some(code_len)) = (length("RSEMU_OVMF_VARS"), length("RSEMU_OVMF_CODE"))
    else {
        println!("q35-uefi: RSEMU_OVMF_VARS_OUT needs both images to place the bank");
        return;
    };
    // The variable bank is the bottom of the contiguous pair, and the pair ends
    // at 4 GiB — so its base is the top of the address space less both images,
    // which is the one place this arrangement is written down twice and the
    // reason the machine file takes sizes rather than addresses.
    let vars_base = TOP - code_len - vars_len;
    let len = vars_len;
    let mem = m.space("mem").expect("the board declares `mem`");
    let mut out = vec![0u8; usize::try_from(len).expect("a bank fits in host memory")];
    mem.read_bytes(vars_base, &mut out, MemAttrs::DEBUG)
        .unwrap_or_else(|e| panic!("reading the variable store back faulted: {e:?}"));
    match std::fs::write(&path, &out) {
        Ok(()) => println!("q35-uefi: wrote {} bytes back to {path}", out.len()),
        Err(e) => println!("q35-uefi: could not write {path}: {e}"),
    }
}

/// What the firmware left the chipset and the flash holding.
///
/// A firmware that prints nothing before its console driver loads — every
/// `RELEASE` build of EDK II — is otherwise a black box, and this is the
/// cheapest instrument there is: every register below is one a *specific* phase
/// of the boot writes, so which of them moved says how far it got. Read with
/// `MemAttrs::DEBUG` so that looking does not pop a FIFO or advance a pointer.
fn report_chipset(m: &Machine) {
    let mem = m.space("mem").expect("the board declares `mem`");
    let port = m.space("port").expect("the board declares `port`");
    let cfg = |dev: u64, func: u64, off: u64| -> u32 {
        let addr = 0x8000_0000 | dev << 11 | func << 8 | (off & 0xfc);
        // `MemAttrs::DEFAULT`, and deliberately: configuration mechanism #1 is
        // an address latch and a data window, so *reaching* a register means
        // writing one — which is exactly what a debug access may not do. The
        // run is over by the time this is called, so a guest-visible access
        // costs nothing; every read below that has a side-effect-free route
        // takes it instead.
        let _ = port.write(0xcf8, Width::U32, addr, MemAttrs::DEFAULT);
        port.read(0xcfc, Width::U32, MemAttrs::DEFAULT)
            .unwrap_or(!0) as u32
    };
    println!("q35-uefi: what the firmware left behind:");
    println!("q35-uefi:   00:00.0 id      = {:#010x}", cfg(0, 0, 0x00));
    println!(
        "q35-uefi:   PCIEXBAR        = {:#010x}_{:08x}",
        cfg(0, 0, 0x64),
        cfg(0, 0, 0x60)
    );
    println!(
        "q35-uefi:   PAM0-6          = {:#010x} {:#010x}",
        cfg(0, 0, 0x90),
        cfg(0, 0, 0x94)
    );
    println!("q35-uefi:   00:1f.0 id      = {:#010x}", cfg(31, 0, 0x00));
    println!("q35-uefi:   PMBASE/ACPI_CNTL= {:#010x}", cfg(31, 0, 0x40));
    println!("q35-uefi:   PIRQ[A-D]_ROUT  = {:#010x}", cfg(31, 0, 0x60));
    let byte = |space: &AddressSpace, at: u64| {
        space.read(at, Width::U8, MemAttrs::DEBUG).unwrap_or(!0) as u8
    };
    println!(
        "q35-uefi:   8259A masks     = {:#04x} {:#04x}, ELCR = {:#04x} {:#04x}",
        byte(port, 0x21),
        byte(port, 0xa1),
        byte(port, 0x4d0),
        byte(port, 0x4d1)
    );
    println!(
        "q35-uefi:   local APIC ID/SVR= {:#010x} {:#010x}",
        mem.read(0xfee0_0020, Width::U32, MemAttrs::DEBUG)
            .unwrap_or(!0),
        mem.read(0xfee0_00f0, Width::U32, MemAttrs::DEBUG)
            .unwrap_or(!0)
    );
    // And the flash: how many bytes of the variable store are no longer erased
    // is what says whether the variable driver ever wrote one.
    let length = |var: &str| -> u64 {
        std::env::var(var)
            .ok()
            .and_then(|p| std::fs::metadata(p).ok())
            .map_or(0, |meta| meta.len())
    };
    let (vars_len, code_len) = (length("RSEMU_OVMF_VARS"), length("RSEMU_OVMF_CODE"));
    if vars_len == 0 {
        return;
    }
    let mut store = vec![0u8; usize::try_from(vars_len).expect("a bank fits in host memory")];
    if mem
        .read_bytes(TOP - code_len - vars_len, &mut store, MemAttrs::DEBUG)
        .is_err()
    {
        return;
    }
    let programmed = store.iter().filter(|b| **b != 0xff).count();
    let last = store.iter().rposition(|b| *b != 0xff).map_or(0, |i| i + 1);
    // Against the image as shipped, because the interesting number is not how
    // much of the store is programmed but whether *this run* programmed any of
    // it: UEFI's variable store is an append-only log, so a firmware that
    // reached its variable driver leaves the end of the log further along.
    let shipped = std::env::var("RSEMU_OVMF_VARS")
        .ok()
        .and_then(|p| std::fs::read(p).ok())
        .map_or(0, |bytes| bytes.iter().filter(|b| **b != 0xff).count());
    println!(
        "q35-uefi:   variable store  = {programmed} byte(s) programmed ({shipped} as shipped), \
         log ends at {last:#08x}"
    );
}
