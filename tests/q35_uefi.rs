//! Does `q35-uefi` assemble, do its two flash banks behave like flash, and does
//! a real UEFI firmware boot on it and **keep what it writes**?
//!
//! Nine of the questions need nothing downloaded and run on every
//! `cargo test`: that the code bank ends at the reset vector, that the variable
//! bank answers the detection probe EDK II's `OvmfPkg` flash driver opens with
//! — byte by byte, exactly as `QemuFlashDetected` issues it — that a
//! program clears bits while only an erase puts them back, four about the
//! **disk**: that `00:04.0` is the class code `NvmExpressDxe` binds on, that
//! `CAP` survives the single 64-bit read that driver makes of it, and that its
//! window decodes where a base address register was told to put it — through
//! **both** routes to configuration space, because for a while it only did
//! through one — and two about what the board **tells the firmware**:
//! [`the_loader_script_the_board_hands_over_builds_a_madt`], which runs EDK II's
//! own loader procedure against this board's `fw_cfg` and follows the result to
//! a MADT that names the machine's own interrupt controllers, and
//! [`a_debug_read_of_the_data_port_does_not_advance_the_cursor`].
//!
//! [`the_disk_controllers_window_decodes_when_ecam_placed_it`] was committed
//! `#[ignore]`d as a reproduction of the one thing standing between this board
//! and an operating system: a BAR programmed through the ECAM window never
//! decoded, so a UEFI firmware enumerated the controller, bound a driver to it
//! and then read `0xffffffff` out of every register. It passes now, and its
//! own doc comment has the mechanism.
//!
//! The other three need a firmware and are gated on `RSEMU_OVMF_CODE`, exactly
//! as `tests/q35_linux.rs` is gated on `RSEMU_KERNEL` and for the same reasons:
//! the image is several megabytes, it is not ours, and `CLAUDE.md` forbids
//! vendoring a fixture. Nothing here is committed and nothing is required for
//! `cargo test`. They are
//! [`a_uefi_firmware_from_the_environment_reaches_its_console`], which boots one
//! and reports where it got to,
//! [`a_variable_written_at_the_shell_is_there_after_a_reboot`], which boots it
//! **twice** and carries nothing between the two but the variable bank, and
//! [`the_firmwares_debug_log_reaches_the_port_at_0x402`], which drains the
//! *other* character port — EDK II's `DEBUG()` log, which no committed test
//! had ever asserted a byte of.
//!
//! **What a fetchable firmware can and cannot show.** The 2 MiB `edk2-ovmf`
//! pair `scripts/fetch-testdata.sh ovmf` copies is a `RELEASE` build: `DEBUG()`
//! is compiled out of it, so it never probes `0x402` and never writes a byte
//! there, and the debug-log test says so and passes. A build with debug strings
//! in it — the 4 MiB pair in a distribution's own qemu firmware package,
//! `edk2-x86_64-code.fd` and `edk2-i386-vars.fd`, at `-p flash=4M -p vars=528K`
//! — narrates the boot from the reset vector, and against one of those the same
//! test asserts the log is there and starts where SEC starts. Which kind
//! arrived is decided by the image rather than by a flag, so the test is honest
//! either way; `scripts/fetch-testdata.sh ovmf-debug` copies the second pair.
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
//! | `RSEMU_OVMF_DEBUG` | writes the `DEBUG()` log the firmware put on `0x402` to this file. The log is captured whether or not it is set; this is how it leaves a `cargo test` run. |
//! | `RSEMU_OVMF_DISK` | a raw disk image for the NVMe namespace. `scripts/fetch-testdata.sh esp` builds one with an EFI application at `\EFI\BOOT\BOOTX64.EFI`. |
//! | `RSEMU_OVMF_MS` | virtual milliseconds to run for (default 420000, which is past the shell prompt at ~367000). |
//! | `RSEMU_OVMF_EXTMEM` | how much memory above 1 MiB the board has. |
//! | `RSEMU_OVMF_STOP_AT` | end the run at the first output containing this. |
//! | `RSEMU_OVMF_EXPECT` | a string the guest must have printed for the test to pass. |
//! | `RSEMU_OVMF_INPUT` | `marker=>text` steps, one per line, typed at the console. |
//! | `RSEMU_KERNEL_TRACE` | print where the processor is once per virtual millisecond. Shared with the kernel boots, because the run loop is. |
//! | `RSEMU_ENGINE` | `interp`, `jit` or `jit-host`, overriding the machine file. |
//! | `RSEMU_OVMF_DISASM` | a comma-separated list of guest addresses to disassemble after the run — for the address a firmware's own exception dump names. |
//! | `RSEMU_OVMF_ASCII` | print the NUL-terminated string at each address — for the assertion buffer a firmware formats and then does not print. |
//! | `RSEMU_OVMF_HEX` | hex dump `addr[:length]`, through the guest's page tables — for the structure a register was left pointing at. |
//! | `RSEMU_OVMF_WHOIS` | name the driver each address is in, out of the loaded image's own PE/COFF debug directory: `1` for the final `RIP` and the stack, or a comma-separated list of addresses. |
//! | `RSEMU_OVMF_PROBE` | replay the boot and report the first exception the firmware takes, with the frame the processor pushed. Costs a second boot. |
//! | `RSEMU_OVMF_PROBE_MS` | how far back that replay switches to one-instruction stepping (default 150). |
//!
//! **Everything printed as evidence is a byte the guest itself wrote**, to COM1
//! or — in the debug-log test — to the port at `0x402`, with one exception, and
//! it is one the board cannot be asked for any other
//! way: [`report_fwcfg`] prints the selectors the firmware wrote to `fw_cfg`,
//! read out of the device the bindings kept a handle to. A selector write
//! leaves nothing behind in any address space, so "which files did OVMF ask
//! for" is otherwise answerable only from the `DEBUG()` log — which this board
//! does have a port for now, and which the third test reads. The firmware is
//! still run, never read (`ROADMAP.md` §1).
//!
//! What it gets to is in [`docs/platforms/q35-uefi.md`](../docs/platforms/q35-uefi.md):
//! a UEFI Shell prompt that answers what is typed at it, on all three engines,
//! at the same virtual instant — and, with `RSEMU_OVMF_DISK` and a kernel on
//! the fixture, a Linux 6.6 userspace entered through the EFI stub off that
//! disk, in symmetric I/O mode off a MADT the firmware built from what this
//! board told it.

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
use rsemu::core::hosts::HostObjects;
use rsemu::core::space::{AddressSpace, MemAttrs, RamStore};
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::dev::medium::Medium;
use rsemu::dev::pc::debugcon::DebugConsole;
use rsemu::dev::q35::fwcfg::FwCfg;
use rsemu::host::chardev::CharPort;
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

use x86boot::{Drains, Script};

/// How long to let the board run, in virtual milliseconds.
///
/// A ceiling rather than a target: the run stops early when the processor stops
/// making progress or when the guest prints `RSEMU_OVMF_STOP_AT`.
///
/// Eight minutes of virtual time, because the UEFI Shell prompt arrives at
/// about **372** seconds of it and a ceiling below that is a run that ends in
/// BDS having printed nothing — which the assertions here read as a failure,
/// and rightly, since a firmware that never reaches a console is a firmware
/// nothing can be said about. A shorter ceiling is `RSEMU_OVMF_MS`.
///
/// It was 900 000, and the shell used to arrive at 816 000. The difference was
/// **`NvmExpressDxe` timing out**: the controller at `00:04.0` was enumerated
/// and bound, and then every register it read was `0xffffffff`, because a BAR
/// programmed through the ECAM window did not decode
/// ([`the_disk_controllers_window_decodes_when_ecam_placed_it`]). `CAP.TO` read
/// as ones with the rest, which the driver takes for 128 seconds per wait. So
/// 444 of those 816 seconds were a measurement of that defect, and they went
/// away with it. Booting an operating system off the disk needs far longer than
/// either number and names its own ceiling.
const DEFAULT_MS: u64 = 480_000;

/// The top of the address space, which is where the flash ends.
const TOP: u64 = 0x1_0000_0000;

/// Everything the board needs, with a `cpu.x86` that pushes what it builds into
/// `cpus`.
///
/// The same shape `tests/q35_linux.rs` uses, and for the same reason: `Device`
/// keeps `Any` out of its supertrait chain, so construction is the one moment
/// the concrete type exists.
fn bindings(
    cpus: &Arc<Captured<X86>>,
    fwcfg: &Arc<Captured<FwCfg>>,
    debugcon: &Arc<Captured<DebugConsole>>,
) -> Bindings {
    let mut b = rsemu::machine::catalog::bindings().expect("this build's bindings");
    // And to the debug console, for the third time and the third reason: its
    // counters are what separate "the firmware never asked whether a debug port
    // was fitted" from "it asked, and then said nothing" from "it said
    // something and the host lost some of it". None of those three is visible
    // in an address space, and they are the whole of what
    // [`the_firmwares_debug_log_reaches_the_port_at_0x402`] has to tell apart.
    let kept_debugcon = Arc::clone(debugcon);
    b.replace(rsemu::dev::pc::debugcon::CLASS_NAME, move |props| {
        let dev = Arc::new(DebugConsole::new(props)?);
        kept_debugcon.push(&dev);
        Ok(dev)
    });
    // The same capture, for the same reason, applied to the one device whose
    // *diagnostics* are not reachable through an address space: what a firmware
    // asked fw_cfg for is a list inside the device, and `Device` deliberately
    // keeps `Any` out of its supertrait chain.
    let kept_fwcfg = Arc::clone(fwcfg);
    b.replace(rsemu::dev::q35::fwcfg::CLASS_NAME, move |props| {
        let dev = Arc::new(FwCfg::new(props)?);
        kept_fwcfg.push(&dev);
        Ok(dev)
    });
    let kept = Arc::clone(cpus);
    b.replace("cpu.x86", move |props| {
        // `RSEMU_ENGINE` overrides the machine file's `engine = "interp"`, the
        // same way `tests/q35_linux.rs` does it and for the same reason: the
        // three engines are a speed knob and never a semantic one
        // (`ROADMAP.md` §0), and a whole UEFI boot is the widest thing there is
        // to say so on.
        let cpu = Arc::new(x86boot::with_engine_from_env(X86::from_props_defaulting(
            props,
            Variant::X86_64,
        )?));
        kept.push(&cpu);
        Ok(cpu)
    });
    b
}

/// What building this board hands back: the machine, the processor the
/// bindings captured, the two character ports the board opens for the firmware
/// to talk on, and the `fw_cfg` and `pc.debugcon` devices — the objects whose
/// *internals* a test needs and no address space can be asked for.
struct Board {
    machine: Machine,
    cpu: Arc<X86>,
    /// COM1's port: the UEFI console, and what every script here is typed at.
    ///
    /// The only one a caller needs a handle to, because it is the one a script
    /// is *typed* at. The `0x402` port — EDK II's whole `DEBUG()` log, on a
    /// stream of its own so that it does not interleave into the shell's line
    /// editor — reaches a test through `x86boot::Drains`, which finds it in
    /// `hosts` by name.
    console: Arc<CharPort>,
    fwcfg: Arc<FwCfg>,
    debugcon: Arc<DebugConsole>,
    /// Every host object this build opened, which is where `x86boot::Drains`
    /// finds the two ports a run is not reading. This board opens **three** —
    /// `console`, `debug` and the 8042's `keyboard` — and a loop that drains
    /// one of them leaves the others to fill.
    hosts: Arc<HostObjects>,
}

/// Build the board from its own machine file, with `code` and `vars` in the two
/// banks.
fn board(
    code: Vec<u8>,
    vars: Vec<u8>,
    disk: Vec<u8>,
    params: &[(&str, String)],
) -> Result<Board, String> {
    board_on_a_medium(code, vars, disk, None, params)
}

/// The same, with the variable bank bound to a **medium** rather than filled
/// from the media table — which is the difference between `--flash1 vars.fd`
/// and `--drive flash1=vars.fd`, and the only one of the two that a
/// [`Machine::flush`] can write back.
///
/// The caller keeps the store it passed in: it is what the bank was loaded
/// from and what its `flush` reaches, so reading it after the run is reading
/// the file a `--drive` run would have left behind.
fn board_on_a_medium(
    code: Vec<u8>,
    vars: Vec<u8>,
    disk: Vec<u8>,
    store: Option<Arc<RamStore>>,
    params: &[(&str, String)],
) -> Result<Board, String> {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let fwcfgs: Arc<Captured<FwCfg>> = Arc::new(Captured::new());
    let debugcons: Arc<Captured<DebugConsole>> = Arc::new(Captured::new());
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings(&cpus, &fwcfgs, &debugcons));
    for (name, value) in params {
        options = options.with_param(*name, value.as_str());
    }
    options.realize.media.insert("flash0", code);
    options.realize.media.insert("flash1", vars);
    // The namespace's contents, stamped into a blank one. Bound even when it is
    // empty, because the slot the machine file names has to exist: an unbound
    // one is refused at realize, and an empty one is a namespace of `size`
    // erased bytes — a disk the firmware enumerates and finds no file system
    // on, which is the ordinary case here and the one the three hermetic tests
    // build.
    options.realize.media.insert("nvme0", disk);
    if let Some(store) = &store {
        rsemu::dev::medium::install(
            &options.realize.hosts,
            "flash1",
            Arc::clone(store) as Arc<dyn Medium>,
        )
        .map_err(|e| format!("{e}"))?;
    }
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
    // Asked for and thrown away, which is an assertion rather than an
    // omission: `pc.debugcon` calls `ports::attach` in `new(props)`, so the
    // port exists from the build under that name, and that name is the whole of
    // how `x86boot::Drains` reaches it — a device that stopped opening it would
    // leave every run draining a port that is not there, silently. Nobody has
    // to remember to drain it any more; a caller that wants to *keep* what it
    // says asks with `Drains::keeping("debug")`.
    rsemu::host::chardev::ports::open(&options.realize.hosts, "debug")
        .expect("the debug console opened the board's `debug` port");
    let cpu = cpus.take().expect("the constructor kept a handle");
    let fwcfg = fwcfgs.take().expect("the board declares one fw_cfg");
    let debugcon = debugcons
        .take()
        .expect("the board declares one debug console");
    Ok(Board {
        machine,
        cpu,
        console,
        fwcfg,
        debugcon,
        hosts: options.realize.hosts,
    })
}

/// The board with both sockets stuffed and nothing programmed into them.
fn bare_board() -> Machine {
    match board(Vec::new(), Vec::new(), Vec::new(), &[]) {
        Ok(built) => built.machine,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Whatever `RSEMU_OVMF_DISK` names, as the NVMe namespace's contents.
///
/// Empty when it is unset, which is every run that is only asking about the
/// firmware: the controller is still on the bus, and a namespace of erased
/// bytes is a disk with no file system on it — which is a fact about the board
/// worth being able to observe, since it is what `map: No mapping found.`
/// looks like from the *other* side.
///
/// `scripts/fetch-testdata.sh esp` builds one: a FAT16 volume with an EFI
/// application at `\EFI\BOOT\BOOTX64.EFI`, which is the file name UEFI 2.10
/// §3.5.1.1 says an x64 boot manager looks for on a device it has no
/// `Boot####` for.
fn disk_from_env() -> Vec<u8> {
    let Ok(path) = std::env::var("RSEMU_OVMF_DISK") else {
        return Vec::new();
    };
    std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
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
/// `QemuFlashDetected` — `OvmfPkg/QemuFlashFvbServicesRuntimeDxe/QemuFlash.c`,
/// BSD-2-Clause-Patent and readable under `CLAUDE.md` — tells flash from RAM
/// and from ROM with **single-byte** cycles at one address, and everything the
/// variable store ever gets written depends on it answering yes. It is
/// replayed here exactly, because getting it *nearly* right is what a run that
/// boots to a shell and silently keeps its variables in RAM looks like.
///
/// The sequence, and what each answer means to the driver:
///
/// | it writes | it reads | and concludes |
/// | --- | --- | --- |
/// | `0x50` (clear status) | the byte back | RAM |
/// | `0x70` (read status) | the original byte | ROM |
/// | | `0x70` | RAM |
/// | | **`0x00`** | flash — carry on |
/// | `0x10`, the original byte, `0x70` | SR.4 set | flash, write protected |
/// | | SR.4 clear | **flash, writable** |
///
/// The load-bearing line is the status register reading **zero**. It is zero
/// because SR.7 is a latch the write state machine sets when it finishes an
/// operation, and the Clear Status Register command clears the whole register
/// (Intel StrataFlash P30 datasheet §14.1.1) — a part that answered `0x80`
/// there would fall off the end of every branch above, and
/// `docs/platforms/q35-uefi.md` records the boot where one did.
#[test]
fn the_variable_bank_answers_the_flash_detection_probe() {
    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    const VARS: u64 = TOP - 2 * 1024 * 1024;

    // The driver probes the first byte of block 0 that is neither of the two
    // status commands nor zero, so that it can tell its own writes apart from
    // what was already there. On an erased bank that is the very first byte.
    let original = read8(mem, VARS);
    assert_eq!(original, 0xff, "an erased bank's first byte is the probe");

    // Clear Status Register. A RAM would hand `0x50` straight back.
    write8(mem, VARS, 0x50);
    assert_ne!(
        read8(mem, VARS),
        0x50,
        "not RAM: the array, not the command"
    );

    // Read Status Register, and this is the answer the whole boot turns on:
    // the register was just cleared and nothing has been asked of the part
    // since, so it reads zero.
    write8(mem, VARS, 0x70);
    assert_eq!(
        read8(mem, VARS),
        0x00,
        "a status register the Clear Status Register command has just cleared"
    );

    // Write the original byte back over itself. On flash that is a legal
    // program — every bit it would set is already set — and SR.4 stays clear,
    // which is how the driver decides the part is writable rather than
    // write protected.
    write8(mem, VARS, 0x10);
    write8(mem, VARS, original);
    write8(mem, VARS, 0x70);
    let status = read8(mem, VARS);
    assert_eq!(status & 0x10, 0, "SR.4 clear: the program was accepted");
    assert_eq!(status & 0x80, 0x80, "SR.7: and the write state machine ran");
    write8(mem, VARS, 0xff);
    assert_eq!(read8(mem, VARS), original, "and the array is unchanged");

    // The code bank is `readonly`, and that is `WP#` tied low rather than a
    // ROM: an Intel part still *answers* every command with `WP#` low — the
    // pin gates the lock bits, not the command interface (StrataFlash P30
    // datasheet, block locking) — so the probe above finds flash here too. What
    // it cannot do is change the array, and the status register says why: SR.4
    // set is the "flash, write-protected" leg of the table above.
    const CODE: u64 = TOP - 2 * 1024 * 1024 + 128 * 1024;
    write8(mem, CODE, 0x10);
    write8(mem, CODE, 0x00);
    let status = read8(mem, CODE);
    assert_eq!(
        status & 0x12,
        0x12,
        "SR.4 and SR.1: the program was refused because the block is locked"
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

/// The disk is on the bus, at the address the machine file names, with the
/// class code EDK II's storage driver binds on.
///
/// `NvmExpressDxe` is a `MdeModulePkg` driver and its `Supported()` accepts a
/// function whose class, subclass and programming interface are 01/08/02 (NVM
/// Express 1.4 §2.1.5, "Class Code"); nothing about the platform enters into
/// it. So this is the whole of what the board has to get right for a UEFI
/// firmware to find a disk here, and it is asserted without a firmware — the
/// same division of labour as the flash probe above, where the byte-wide
/// detection sequence is checked here and the boot that depends on it is
/// gated on an image.
///
/// Both routes to configuration space, because `PciBusDxe` reaches a function
/// through whichever one the platform's `PciHostBridgeLib` published and this
/// board publishes ECAM.
#[test]
fn the_disk_controller_is_the_class_code_the_uefi_driver_binds_on() {
    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    let port = machine.space("port").expect("the board declares `port`");

    port.write(0xcf8, Width::U32, 0x8000_2000, MemAttrs::DEFAULT)
        .expect("CONFADD is a dword register");
    let id = port
        .read(0xcfc, Width::U32, MemAttrs::DEFAULT)
        .expect("CONFDATA") as u32;
    assert_ne!(id, 0xffff_ffff, "00:04.0 answers");

    port.write(0xcf8, Width::U32, 0x8000_2008, MemAttrs::DEFAULT)
        .expect("CONFADD");
    let class = port
        .read(0xcfc, Width::U32, MemAttrs::DEFAULT)
        .expect("CONFDATA") as u32;
    assert_eq!(
        class >> 8,
        0x0001_0802,
        "NVM Express is class 010802h, which is what NvmExpressDxe binds on"
    );

    // And through the window `PCIEXBAR` places, which is the route the
    // firmware's own PCI bus driver takes: bus 0 is 0, device 4 is 4 * 32 KiB
    // (Intel 3 Series datasheet §5.1.16).
    assert_eq!(
        mem.read(0xe000_0000 + 4 * 0x8000, Width::U32, MemAttrs::DEFAULT)
            .expect("the ECAM window decodes") as u32,
        id,
        "the same function through the window the (G)MCH placed"
    );
}

/// `CAP` read the way the UEFI driver reads it: **one 64-bit access**.
///
/// `CAP` is the only 64-bit register a controller has to answer before anything
/// else works (NVM Express 1.4 §3.1.1, "Offset 00h: CAP"), and the two drivers
/// this repository points at the same part read it differently. Linux's
/// `nvme_pci_enable` uses `lo_hi_readq`, which is **two 32-bit reads**;
/// EDK II's `NvmExpressDxe` uses `PciIo->Mem.Read (EfiPciIoWidthUint64, ...)`,
/// which is one. A part that answers the first correctly and the second with
/// the low half in both places is a part a kernel boots from and a firmware
/// asserts on — and it asserted on exactly the field the halves disagree
/// about:
///
/// ```text
/// ASSERT MdeModulePkg/Bus/Pci/NvmExpressDxe/NvmExpressHci.c(778):
///     (Private->Cap.Mpsmin + 12) <= 12
/// ```
///
/// `MPSMIN` is `CAP[51:48]`, in the *upper* half. The controller's real answer
/// there is 0 — 4 KiB, the only page size the driver supports — and the value
/// it asserted on is bit 48 of a duplicated lower half, which is
/// `CAP[19:16] = 1`.
///
/// So this reads the register both ways at the address the board's own PCI
/// configuration space puts it, and asserts they agree. It needs no firmware.
#[test]
fn the_disk_controllers_capabilities_survive_a_single_64_bit_read() {
    /// Somewhere in the hole below 4 GiB, which is where this board's PCI
    /// window is and where nothing else decodes.
    const BAR0: u64 = 0x8_0000_0000;

    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    let port = machine.space("port").expect("the board declares `port`");
    let cfg = |offset: u32, value: u32| {
        port.write(
            0xcf8,
            Width::U32,
            u64::from(0x8000_2000 | offset),
            MemAttrs::DEFAULT,
        )
        .expect("CONFADD is a dword register");
        port.write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("CONFDATA");
    };
    // The sequence `PciBusDxe` puts a function through, in its order: size the
    // register by writing all-ones to both halves, then place it, then enable
    // memory space. The sizing write is not decoration — it asks the function
    // to decode at the top of the address space for as long as it takes to
    // read the mask back, and a model that took it literally would leave a
    // region there.
    cfg(0x10, 0xffff_ffff);
    cfg(0x14, 0xffff_ffff);
    cfg(0x10, BAR0 as u32);
    cfg(0x14, (BAR0 >> 32) as u32);
    cfg(0x04, 0x0006);

    let read = |at: u64, width: Width| {
        mem.read(at, width, MemAttrs::DEFAULT)
            .unwrap_or_else(|e| panic!("a read of {at:#x} faulted: {e:?}"))
    };
    let (low, high) = (read(BAR0, Width::U32), read(BAR0 + 4, Width::U32));
    assert_ne!(
        low, 0xffff_ffff,
        "the register block decodes where it was put"
    );
    assert_eq!(
        read(BAR0, Width::U64),
        low | (high << 32),
        "one 64-bit read of CAP is the two 32-bit reads of it, in the order the \
         register is laid out"
    );
    assert_eq!(
        (high >> 16) & 0xf,
        0,
        "CAP.MPSMIN is 4 KiB: NvmExpressDxe asserts on anything else"
    );
}

/// The same window, placed the way a **UEFI** firmware places it: through ECAM.
///
/// **This test was committed `#[ignore]`d as a reproduction and now passes.**
/// It was the whole of what stood between this board and an operating system,
/// and it was not a UEFI problem at all: *any* guest that programmed a base
/// address register through the memory-mapped configuration window got a
/// function that answered its configuration space and decoded nothing.
///
/// The mechanism is written down in `src/bus/pci/bar.rs`'s own module docs,
/// under "Moving a mapping from inside a configuration write". A BAR write
/// arrives inside an address-space access, so the space's topology lock is
/// already held for reading and the blocking `AddressSpace::topology` would
/// invert `core::sync`'s ladder. `Bars::sync` therefore takes the order-exempt
/// `try_topology`, and when that fails it sets a `stale` flag. What that flag
/// used to wait for was **the next configuration access**, on an assumption the
/// file stated plainly:
///
/// > A configuration cycle **travels through the I/O space** […] the retry at
/// > the next configuration access fails for the same reason, for ever.
///
/// It said that of an *I/O* BAR, and refused to map one. But a q35 has a second
/// route to configuration space — ECAM, in the **memory** space — and through
/// it every BAR was in exactly that position: the write is a memory access, so
/// `try_topology` on the memory space cannot succeed, and neither can the
/// retry, or the retry after that. A firmware that never touches `0xcf8` never
/// healed it.
///
/// The retry now lands somewhere that is not an access at all. `PciBus`
/// remembers that a function came out of a configuration cycle owing a
/// retopology, and the host bridge — which already keeps a clock domain because
/// its own `PCIEXBAR` window moves from inside an ECAM write — drains the
/// fabric from `Device::advance_to`, which the run loop calls between rounds.
/// So this test **runs the machine** between programming the register and
/// looking at it, which is the honest statement of what the fix buys: a window
/// placed through ECAM decodes within one scheduler round, not within one
/// instruction.
///
/// [`a_window_placed_through_the_port_pair_decodes_inside_the_write`] is the
/// control, and it is a warning about instruments as much as a control:
/// [`report_nvme`] reaches configuration space that way, so while this was
/// broken every register it printed looked perfect *because looking at it fixed
/// it*.
#[test]
fn the_disk_controllers_window_decodes_when_ecam_placed_it() {
    const BAR0: u64 = 0x8_0000_0000;
    let mut machine = bare_board();
    {
        let mem = machine.space("mem").expect("the board declares `mem`");
        // Bus 0, device 4, function 0 is 4 * 32 KiB into the window `PCIEXBAR`
        // placed (Intel 3 Series datasheet §5.1.16).
        let ecam = |offset: u64, value: u32| {
            mem.write(
                0xe000_0000 + 4 * 0x8000 + offset,
                Width::U32,
                u64::from(value),
                MemAttrs::DEFAULT,
            )
            .expect("the ECAM window takes a dword");
        };
        ecam(0x10, BAR0 as u32);
        ecam(0x14, (BAR0 >> 32) as u32);
        ecam(0x04, 0x0006);
    }
    // One scheduler round. `Machine::run_quantum` is what the run loop does,
    // and `Scheduler::sync_lazy_devices` at the end of it is where the bridge
    // gets its moment with no access in flight. There is no firmware in this
    // board's flash, so the processor spends the round executing erased bytes —
    // which is fine, and is rather the point: nothing the *guest* does is what
    // settles the fabric.
    machine.run_quantum().expect("a quantum runs");
    let mem = machine.space("mem").expect("the board declares `mem`");
    let after_ecam = mem.read(BAR0, Width::U32, MemAttrs::DEFAULT).unwrap_or(!0) as u32;
    println!("after ECAM and one round: {after_ecam:#010x}");
    assert_ne!(
        after_ecam, 0xffff_ffff,
        "a BAR programmed through the ECAM window decodes where it was put"
    );
    assert_eq!(
        after_ecam, 0x0101_03ff,
        "and what decodes there is `CAP`, not something else that answers"
    );
}

/// The other route to the same register, which never had the defect.
///
/// A configuration cycle through `0xcf8`/`0xcfc` travels through the **I/O**
/// space, so the memory space's topology is free and the window is placed
/// inside the write itself — no round, no scheduler. Linux takes this route for
/// resource assignment, which is why `tests/q35_linux.rs` never saw the bug.
#[test]
fn a_window_placed_through_the_port_pair_decodes_inside_the_write() {
    const BAR0: u64 = 0x8_0000_0000;
    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    let port = machine.space("port").expect("the board declares `port`");
    let cfg = |offset: u32, value: u32| {
        port.write(
            0xcf8,
            Width::U32,
            u64::from(0x8000_2000 | offset),
            MemAttrs::DEFAULT,
        )
        .expect("CONFADD is a dword register");
        port.write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("CONFDATA");
    };
    cfg(0x10, BAR0 as u32);
    cfg(0x14, (BAR0 >> 32) as u32);
    cfg(0x04, 0x0006);
    assert_eq!(
        mem.read(BAR0, Width::U32, MemAttrs::DEFAULT).unwrap_or(!0) as u32,
        0x0101_03ff,
        "no round needed: the try-lock is against a space this access is not in"
    );
}

// ---------------------------------------------------------------------------
// what the board tells the firmware about itself
// ---------------------------------------------------------------------------

/// Select an item, the way `QemuFwCfgSelectItem` does: one 16-bit write.
fn fwcfg_select(port: &AddressSpace, key: u16) {
    port.write(0x510, Width::U16, u64::from(key), MemAttrs::DEFAULT)
        .expect("the selector takes a word");
}

/// Read `n` bytes of the selected item, the way `IoReadFifo8` does.
fn fwcfg_read(port: &AddressSpace, n: usize) -> Vec<u8> {
    (0..n).map(|_| read8(port, 0x511)).collect()
}

/// The file directory, as `QemuFwCfgFindFile` walks it: a big-endian count,
/// then a big-endian size, key and reserved word per file, then 56 bytes of
/// NUL-terminated name.
fn fwcfg_dir(port: &AddressSpace) -> Vec<(String, u16, u32)> {
    fwcfg_select(port, 0x0019);
    let count = u32::from_be_bytes(fwcfg_read(port, 4).try_into().expect("four bytes"));
    (0..count)
        .map(|_| {
            let size = u32::from_be_bytes(fwcfg_read(port, 4).try_into().expect("four bytes"));
            let key = u16::from_be_bytes(fwcfg_read(port, 2).try_into().expect("two bytes"));
            let _reserved = fwcfg_read(port, 2);
            let name = fwcfg_read(port, 56);
            let end = name.iter().position(|b| *b == 0).unwrap_or(name.len());
            (
                String::from_utf8_lossy(&name[..end]).into_owned(),
                key,
                size,
            )
        })
        .collect()
}

/// Look a file up by name and read the whole thing.
fn fwcfg_file(port: &AddressSpace, name: &str) -> Option<Vec<u8>> {
    let (_, key, size) = fwcfg_dir(port).into_iter().find(|(n, _, _)| n == name)?;
    fwcfg_select(port, key);
    Some(fwcfg_read(port, size as usize))
}

/// The board hands a firmware a table set, and following the firmware's own
/// procedure produces a MADT that describes this board's interrupt controllers.
///
/// **This is the milestone in hermetic form.** A UEFI-booted kernel finds its
/// APIC because OVMF installed a MADT, and OVMF installed a MADT because it
/// executed `etc/table-loader` against `etc/acpi/tables`. So what is asserted
/// here is not "the device answers" — it is that *running the loader script the
/// way EDK II runs it* yields tables that check out. The procedure is
/// `OvmfPkg/Library/AcpiPlatformLib/QemuFwCfgAcpi.c` (BSD-2-Clause-Patent):
///
/// 1. `Allocate`: download the named file into memory, at the alignment asked.
/// 2. `AddPointer`: the field at `PointerOffset` in one file holds an *offset*
///    into another; add that file's base to it.
/// 3. `AddChecksum`: store `0x100 - sum(range)` at `ResultOffset` — which only
///    brings the range to zero because the byte it lands on started at zero.
/// 4. Walk the `AddPointer` commands again: whatever each one now points at is
///    an ACPI table if it carries a plausible length and sums to zero.
///
/// Step 3 is the one that makes this a test rather than a paraphrase: a blob
/// whose checksums were already filled in would pass every "does the device
/// answer" assertion and produce tables the firmware silently declines to
/// install.
#[test]
fn the_loader_script_the_board_hands_over_builds_a_madt() {
    let machine = bare_board();
    let port = machine.space("port").expect("the board declares `port`");

    // The probe `QemuFwCfgInitialize` opens with. A firmware that reads
    // anything else here takes no tables at all and says nothing about it.
    fwcfg_select(port, 0x0000);
    assert_eq!(&fwcfg_read(port, 4), b"QEMU", "the interface signature");
    fwcfg_select(port, 0x0001);
    let revision = u32::from_le_bytes(fwcfg_read(port, 4).try_into().expect("four bytes"));
    assert!(revision >= 1, "revision {revision} is not the interface");
    assert_eq!(
        revision & 2,
        0,
        "FW_CFG_F_DMA is deliberately clear: this device serves the port path only"
    );
    // And the processor count, which is what keeps `MpInitLib` from waiting out
    // its timeout counting application processors that are not there.
    fwcfg_select(port, 0x0005);
    assert_eq!(
        u16::from_le_bytes(fwcfg_read(port, 2).try_into().expect("two bytes")),
        1,
        "one boot processor, little-endian as the pre-directory keys are"
    );

    let files = fwcfg_dir(port);
    let names: Vec<&str> = files.iter().map(|(n, _, _)| n.as_str()).collect();
    for want in ["etc/acpi/rsdp", "etc/acpi/tables", "etc/table-loader"] {
        assert!(
            names.contains(&want),
            "the directory has no {want:?}: {names:?}"
        );
    }

    // -- run the script -----------------------------------------------------
    //
    // Two bases picked out of the air, exactly as a firmware's allocator picks
    // them: what matters is that nothing in the blob knew them.
    let mut blobs: Vec<(String, u64, Vec<u8>)> = Vec::new();
    let base_for = |name: &str| -> u64 {
        match name {
            "etc/acpi/tables" => 0x7000_0040,
            _ => 0x000e_1000,
        }
    };
    let loader = fwcfg_file(port, "etc/table-loader").expect("the script is served");
    assert_eq!(loader.len() % 128, 0, "a loader is whole commands");
    let name_at = |cmd: &[u8], at: usize| -> String {
        let field = &cmd[at..at + 56];
        assert_eq!(field[55], 0, "a file name field is NUL-terminated");
        let end = field.iter().position(|b| *b == 0).unwrap_or(0);
        String::from_utf8_lossy(&field[..end]).into_owned()
    };
    for cmd in loader.chunks(128) {
        match u32::from_le_bytes(cmd[..4].try_into().expect("four bytes")) {
            // Allocate.
            1 => {
                let name = name_at(cmd, 4);
                let align = u64::from(u32::from_le_bytes(
                    cmd[60..64].try_into().expect("four bytes"),
                ));
                assert!(
                    align.is_power_of_two() && align <= 4096,
                    "EDK II refuses an alignment above a page: {align}"
                );
                let bytes = fwcfg_file(port, &name)
                    .unwrap_or_else(|| panic!("{name} is allocated but not served"));
                let base = base_for(&name);
                assert_eq!(base % align, 0, "the rig honours the alignment asked for");
                blobs.push((name, base, bytes));
            }
            // AddPointer.
            2 => {
                let (pointer, pointee) = (name_at(cmd, 4), name_at(cmd, 60));
                let offset =
                    u32::from_le_bytes(cmd[116..120].try_into().expect("four bytes")) as usize;
                let size = cmd[120] as usize;
                assert!(matches!(size, 1 | 2 | 4 | 8), "pointer size {size}");
                let target = blobs
                    .iter()
                    .find(|(n, _, _)| *n == pointee)
                    .map(|(_, base, bytes)| (*base, bytes.len()))
                    .unwrap_or_else(|| panic!("{pointee} is pointed at before it is allocated"));
                let blob = blobs
                    .iter_mut()
                    .find(|(n, _, _)| *n == pointer)
                    .unwrap_or_else(|| panic!("{pointer} is patched before it is allocated"));
                let mut value = [0u8; 8];
                value[..size].copy_from_slice(&blob.2[offset..offset + size]);
                let value = u64::from_le_bytes(value);
                assert!(
                    (value as usize) < target.1,
                    "a pointer's value is an offset into {pointee}, and {value:#x} is past its end"
                );
                let patched = (value + target.0).to_le_bytes();
                if size < 8 {
                    assert_eq!(
                        u64::from_le_bytes(patched) >> (size * 8),
                        0,
                        "a {size}-byte field cannot hold the address it was given"
                    );
                }
                blob.2[offset..offset + size].copy_from_slice(&patched[..size]);
            }
            // AddChecksum.
            3 => {
                let name = name_at(cmd, 4);
                let result =
                    u32::from_le_bytes(cmd[60..64].try_into().expect("four bytes")) as usize;
                let start =
                    u32::from_le_bytes(cmd[64..68].try_into().expect("four bytes")) as usize;
                let length =
                    u32::from_le_bytes(cmd[68..72].try_into().expect("four bytes")) as usize;
                let blob = blobs
                    .iter_mut()
                    .find(|(n, _, _)| *n == name)
                    .unwrap_or_else(|| panic!("{name} is checksummed before it is allocated"));
                assert_eq!(
                    blob.2[result], 0,
                    "the byte a checksum lands on has to start at zero, or the range sums to \
                     minus what was already there"
                );
                let sum = blob.2[start..start + length]
                    .iter()
                    .fold(0u8, |acc, b| acc.wrapping_add(*b));
                blob.2[result] = sum.wrapping_neg();
            }
            other => panic!("unknown loader command {other}"),
        }
    }

    // -- and look at what it built ------------------------------------------
    let (_, tables_base, tables) = blobs
        .iter()
        .find(|(n, _, _)| n == "etc/acpi/tables")
        .expect("the tables were allocated");
    let table_at = |address: u64| -> &[u8] {
        let at = (address - tables_base) as usize;
        let length =
            u32::from_le_bytes(tables[at + 4..at + 8].try_into().expect("four bytes")) as usize;
        &tables[at..at + length]
    };
    let sums_to_zero = |bytes: &[u8]| bytes.iter().fold(0u8, |acc, b| acc.wrapping_add(*b)) == 0;

    // The RSDP names the XSDT, and the XSDT names everything else — every one
    // of those addresses having been an offset until the script ran.
    let (_, _, rsdp) = blobs
        .iter()
        .find(|(n, _, _)| n == "etc/acpi/rsdp")
        .expect("the RSDP was allocated");
    assert_eq!(
        &rsdp[..8],
        b"RSD PTR ",
        "the RSDP's signature, blank and all"
    );
    assert!(sums_to_zero(&rsdp[..20]), "the RSDP's first checksum");
    assert!(sums_to_zero(&rsdp[..36]), "and its extended one");
    let xsdt_at = u64::from_le_bytes(rsdp[24..32].try_into().expect("eight bytes"));
    let xsdt = table_at(xsdt_at);
    assert_eq!(&xsdt[..4], b"XSDT");
    assert!(sums_to_zero(xsdt), "the XSDT checksums");

    let mut found: Vec<(String, Vec<u8>)> = Vec::new();
    for entry in xsdt[36..].chunks(8) {
        let table = table_at(u64::from_le_bytes(entry.try_into().expect("eight bytes")));
        assert!(
            sums_to_zero(table),
            "a table the XSDT lists does not checksum; the firmware would not install it"
        );
        found.push((
            String::from_utf8_lossy(&table[..4]).into_owned(),
            table.to_vec(),
        ));
    }
    let signatures: Vec<&str> = found.iter().map(|(s, _)| s.as_str()).collect();
    assert!(
        signatures.contains(&"APIC"),
        "no MADT: this is the whole point, and the kernel's fallback without one is virtual \
         wire mode. The XSDT listed {signatures:?}"
    );
    assert!(signatures.contains(&"FACP"), "no FADT: {signatures:?}");
    assert!(signatures.contains(&"MCFG"), "no MCFG: {signatures:?}");
    assert!(
        signatures.contains(&"HPET"),
        "no HPET table: {signatures:?}"
    );

    // The MADT describes *this* board: the local APIC page and the I/O APIC
    // page the machine file maps, read back out of the table rather than
    // restated. (ACPI §5.2.12, Tables 5.19, 5.22 and 5.24.)
    let madt = &found
        .iter()
        .find(|(s, _)| s == "APIC")
        .expect("checked above")
        .1;
    assert_eq!(
        u32::from_le_bytes(madt[36..40].try_into().expect("four bytes")),
        0xfee0_0000,
        "the local APIC address the board maps"
    );
    let mut processors = 0;
    let mut ioapic = None;
    let mut at = 44;
    while at + 2 <= madt.len() {
        let (kind, length) = (madt[at], madt[at + 1] as usize);
        assert!(length >= 2, "a MADT structure with no length");
        match kind {
            0 => processors += 1,
            1 => {
                ioapic = Some(u32::from_le_bytes(
                    madt[at + 4..at + 8].try_into().expect("four bytes"),
                ))
            }
            _ => {}
        }
        at += length;
    }
    assert_eq!(at, madt.len(), "the structures fill the table exactly");
    assert_eq!(processors, 1, "one enabled processor");
    assert_eq!(ioapic, Some(0xfec0_0000), "the I/O APIC the board maps");

    // The FADT's own two pointers, which are the reason the FACS and the DSDT
    // are in the blob at all: neither is listed in the XSDT (§5.2.8).
    let fadt = &found
        .iter()
        .find(|(s, _)| s == "FACP")
        .expect("checked above")
        .1;
    let facs_at = u64::from_le_bytes(fadt[132..140].try_into().expect("eight bytes"));
    let dsdt_at = u64::from_le_bytes(fadt[140..148].try_into().expect("eight bytes"));
    assert_eq!(
        &tables[(facs_at - tables_base) as usize..][..4],
        b"FACS",
        "X_FIRMWARE_CTRL points at the FACS"
    );
    assert_eq!(
        &tables[(dsdt_at - tables_base) as usize..][..4],
        b"DSDT",
        "X_DSDT points at the DSDT"
    );
    assert_eq!(
        u64::from(u32::from_le_bytes(
            fadt[36..40].try_into().expect("four bytes")
        )),
        facs_at,
        "and the 32-bit FIRMWARE_CTRL agrees with the 64-bit one"
    );
    assert_eq!(
        u64::from(u32::from_le_bytes(
            fadt[40..44].try_into().expect("four bytes")
        )),
        dsdt_at,
        "as do the two DSDT fields"
    );
    assert!(
        sums_to_zero(table_at(dsdt_at)),
        "the DSDT checksums, which is what makes the firmware install it"
    );
}

/// A debug read of the data port hands back the byte under the cursor and does
/// not move it.
///
/// `CLAUDE.md`: a debugger read must not pop a FIFO, clear a status bit or
/// advance a pointer — and a `fw_cfg` data port is precisely a pointer being
/// advanced. A monitor dumping the I/O space would otherwise leave a firmware
/// reading a table from the wrong offset, which is a corrupted ACPI set and no
/// evidence of why.
#[test]
fn a_debug_read_of_the_data_port_does_not_advance_the_cursor() {
    let machine = bare_board();
    let port = machine.space("port").expect("the board declares `port`");
    fwcfg_select(port, 0x0000);
    for _ in 0..4 {
        assert_eq!(
            port.read(0x511, Width::U8, MemAttrs::DEBUG)
                .expect("the data port answers a debug read") as u8,
            b'Q',
            "every debug read is the same byte: the cursor has not moved"
        );
    }
    assert_eq!(
        &fwcfg_read(port, 4),
        b"QEMU",
        "and an ordinary read still is"
    );
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
    let disk = disk_from_env();
    if !disk.is_empty() {
        params.push(("disk", format!("{}", disk.len())));
    }
    println!(
        "q35-uefi: {} bytes of firmware and {} bytes of variable store, mapped at {:#x}",
        code.len(),
        vars.len(),
        TOP - (code.len() + vars.len()) as u64
    );

    let Board {
        machine: mut m,
        cpu,
        console,
        fwcfg,
        debugcon,
        hosts,
        ..
    } = match board(code.clone(), vars.clone(), disk, &params) {
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
        Drains::open(&hosts, &console),
        GlobalTime::from_nanos(ms * 1_000_000),
        &script,
    );
    x86boot::report("q35-uefi", &m, &cpu, &run, &script);
    report_debugcon(&debugcon, &run);
    report_fwcfg(&fwcfg);
    report_chipset(&m);
    report_nvme(&m, &cpu);
    report_disassembly(&m, &cpu);
    report_modules(&m, &cpu);
    report_ascii(&m, &cpu);
    report_hex(&m, &cpu);
    probe_first_exception(&params, run.at);
    write_back_the_variable_store(&m);
    assert_reached_uefi(&run, &script);
}

/// What this run left on the `0x402` port, said in one line.
///
/// This loop is not *reading* that port — it is `x86boot::run`, and COM1 is
/// what a script is typed at — but it does drain it, every slice, along with
/// every other port the board opened. That is the difference between this run
/// and the one that used to be here: the firmware's log is still thrown away,
/// because nothing asked to keep it, but it is thrown away **by the host**
/// rather than by a port that filled and a device that then had nowhere to put
/// the overflow. `x86boot::Drains` has the argument for why that is the
/// harness's decision and not the device's.
///
/// So `dropped` should now be zero on any image: a `pc.debugcon` only drops
/// when its port is full, and a port drained every millisecond of virtual time
/// never is. The two counters that remain interesting are what the firmware
/// *asked* (`probes`) and what it *said* (`written`), neither of which is
/// visible in an address space.
///
/// To keep the log rather than count it: `Drains::keeping("debug")`, which is
/// what [`the_firmwares_debug_log_reaches_the_port_at_0x402`] does, or
/// `rsemu run … --capture debug=boot.log` from a command line.
fn report_debugcon(debugcon: &DebugConsole, run: &x86boot::Run) {
    println!(
        "q35-uefi: the firmware probed 0x402 {} time(s) and wrote {} byte(s) of DEBUG() log to \
         it; this run drained {} of them and threw them away, and the device dropped {} — see \
         `the_firmwares_debug_log_reaches_the_port_at_0x402`, which keeps the log instead",
        debugcon.probes(),
        debugcon.written(),
        run.drains.bytes("debug"),
        debugcon.dropped(),
    );
    // Reported, not asserted, and the difference is deliberate: this test's
    // subject is COM1, and a drop here would say a *slice* got long enough to
    // fill 64 KiB rather than that the board did anything wrong. The place that
    // holds the accounting to zero is the test whose subject is the log.
    if debugcon.dropped() != 0 {
        println!(
            "q35-uefi: and {} of those byte(s) never reached the host at all — a millisecond \
             slice of this firmware filled the 64 KiB a `CharPort` holds, which is the one way \
             a drained port can still overflow",
            debugcon.dropped()
        );
    }
}

/// The firmware's own `DEBUG()` log, off the port at `0x402`, whole.
///
/// The gap this fills: every other piece of firmware evidence in this file is a
/// byte the guest wrote to **COM1**, because that is the port `x86boot::run`
/// drains. EDK II's `PlatformDebugLibIoPort` writes somewhere else entirely,
/// and until now nothing committed asserted a single byte of it — the device
/// had unit tests driving it by hand, and the board had a `map port 0x0402`,
/// and the only thing that had ever put the two together was a person running
/// `rsemu run --console debug` and reading the result.
///
/// # What is asserted, and why each holds for *any* image
///
/// A firmware image cannot be committed (`CLAUDE.md`, testing), and the one
/// `scripts/fetch-testdata.sh ovmf` copies — a distribution's 2 MiB split
/// `edk2-ovmf` pair — is a **`RELEASE`** build, which compiles `DEBUG()` out
/// entirely and therefore writes nothing here at all. So the assertions are
/// written to be true of both kinds of build, and to say which kind arrived:
///
/// * **A read of `0x402` answers `0xe9`**, asked of the board through its own
///   I/O space, with `MemAttrs::debug` — which the device answers without
///   counting, so asking does not disturb the count the rest of the test reads.
/// * **The accounting closes**: what the device says the guest wrote is what
///   this loop captured plus what it dropped, and it dropped nothing, because
///   the port is drained every slice. A firmware log outgrows the 64 KiB a
///   `CharPort` holds — the 4 MiB debug build has written 95,726 bytes of it by
///   the time the shell prompt arrives — so a run that does *not* drain it
///   keeps the first 64 KiB and the device counts the rest away.
/// * **A firmware that asked, talked.** `probes` counts
///   `PlatformDebugLibIoPortDetect`, and EDK II calls it from `DebugPrintMarker`
///   immediately before the `IoWriteFifo8` that is the log itself
///   (`src/dev/pc/debugcon.rs` has both, quoted). So `probes > 0` with an empty
///   log is a real failure — the port answered the detect and then swallowed
///   the output — and `probes == 0` is a `RELEASE` image, which is reported
///   rather than failed.
/// * **The string the image carries is the string that came out**, when the
///   image carries it: `SecCoreStartupWithStack(0x%x, 0x%x)` lives uncompressed
///   in the SEC firmware volume of a debug build, because SEC is executed in
///   place out of flash. An image with that format string in it must produce
///   that line on this port, which is end to end from the reset vector: the
///   first `DEBUG()` a UEFI machine ever executes, through the detect, through
///   the device, onto a host stream.
///
/// `RSEMU_OVMF_DEBUG` names a file to write the captured log to. Nothing here
/// needs it; it is how a person gets the log out of a `cargo test` run, the way
/// `RSEMU_OVMF_VARS_OUT` gets the variable store out.
///
/// `scripts/fetch-testdata.sh ovmf-debug` copies a pair that has debug strings
/// in it — verifying that it really does before putting the name on it — or
/// point the two variables straight at the package:
///
/// ```console
/// RSEMU_OVMF_CODE=/usr/share/qemu/edk2-x86_64-code.fd \
/// RSEMU_OVMF_VARS=/usr/share/qemu/edk2-i386-vars.fd \
/// RSEMU_OVMF_DEBUG=boot.log \
///     cargo test --release --features machine-q35-uefi --test q35_uefi -- \
///         --nocapture the_firmwares_debug_log
/// ```
#[test]
fn the_firmwares_debug_log_reaches_the_port_at_0x402() {
    let Ok(path) = std::env::var("RSEMU_OVMF_CODE") else {
        println!(
            "q35-uefi: set RSEMU_OVMF_CODE to a split UEFI firmware image to capture its \
             DEBUG() log off the port at 0x402; an image with debug strings in it narrates the \
             whole boot there, and a RELEASE image writes nothing. See the module docs"
        );
        return;
    };
    let code = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let vars = std::env::var("RSEMU_OVMF_VARS")
        .ok()
        .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
        .unwrap_or_default();
    let mut params: Vec<(&str, String)> = vec![
        ("flash", format!("{}", code.len() + vars.len())),
        ("vars", format!("{}", vars.len().max(0x1000))),
    ];
    if let Ok(extmem) = std::env::var("RSEMU_OVMF_EXTMEM") {
        params.push(("extmem", extmem));
    }
    let disk = disk_from_env();
    if !disk.is_empty() {
        params.push(("disk", format!("{}", disk.len())));
    }

    let Board {
        machine: mut m,
        cpu,
        console,
        debugcon,
        hosts,
        ..
    } = match board(code.clone(), vars.clone(), disk, &params) {
        Ok(built) => built,
        Err(e) => panic!("the board does not realize: {e}"),
    };

    // The detect, before the firmware gets to it, and through the board rather
    // than through the device: `MemAttrs::DEBUG` because a monitor's peek must
    // not look like a firmware's question, which is the whole of what the
    // device's `debug` handling is for.
    let port = m.space("port").expect("the board declares `port`");
    assert_eq!(
        port.read(0x402, Width::U8, MemAttrs::DEBUG)
            .expect("the debug console answers a byte read") as u8,
        0xe9,
        "the board maps a debug console at 0x402 and it answers EDK II's \
         BOCHS_DEBUG_PORT_MAGIC; anything else and PlatformDebugLibIoPortDetect fails and the \
         whole log is dropped"
    );
    assert_eq!(
        debugcon.probes(),
        0,
        "a debug read is not a detect: the guest has not asked yet"
    );

    let ms: u64 = std::env::var("RSEMU_OVMF_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_MS);
    let stop_at = std::env::var("RSEMU_OVMF_STOP_AT").unwrap_or_else(|_| String::from("Shell>"));
    println!("q35-uefi: what the guest wrote to the debug port at 0x402:");
    // Nothing is typed here. What a firmware puts on its debug port is decided
    // long before a shell prompt exists, and a run that types nothing is one
    // less thing between an assertion and the log — so the script is a stop
    // marker on COM1 and no steps at all.
    let script = Script {
        steps: Vec::new(),
        stop_at: stop_at.clone(),
    };
    let run = x86boot::run(
        &mut m,
        &cpu,
        &console,
        // The whole difference between this run and every other one on this
        // board: `debug` is **kept** rather than discarded. It is drained every
        // slice either way — that is what stopped a firmware log from being cut
        // down to the first 64 KiB — and `keeping` is what says a test wants to
        // read it afterwards.
        Drains::open(&hosts, &console).keeping("debug"),
        GlobalTime::from_nanos(ms * 1_000_000),
        &script,
    );
    let log = run.drains.kept("debug").to_vec();
    println!(
        "q35-uefi: {} byte(s) of DEBUG() log in {} ms of virtual time, and {} byte(s) on COM1{}",
        log.len(),
        run.at.as_nanos() / 1_000_000,
        run.text.len(),
        if run.reached {
            format!(" (stopped at {stop_at:?})")
        } else {
            String::new()
        }
    );
    if let Ok(out) = std::env::var("RSEMU_OVMF_DEBUG") {
        std::fs::write(&out, &log).unwrap_or_else(|e| panic!("{out}: {e}"));
        println!("q35-uefi: wrote the log to {out}");
    }

    // The accounting, which is the assertion the *host* side has to pass: a
    // port nobody drains holds 64 KiB and the device drops the rest.
    assert_eq!(
        debugcon.dropped(),
        0,
        "the loop drains the port every slice and the device still had to drop {} byte(s): the \
         log in hand is a truncated one",
        debugcon.dropped()
    );
    assert_eq!(
        debugcon.written(),
        log.len() as u64,
        "the device counted {} byte(s) written by the guest and dropped none, and this loop \
         captured {}",
        debugcon.written(),
        log.len()
    );

    if debugcon.probes() == 0 {
        // Not a failure. `DEBUG()` is compiled out of a RELEASE build, so
        // nothing ever calls the detect and the port is never touched — which
        // is the image speaking, not the board, and the board has already
        // answered 0xe9 above.
        println!(
            "q35-uefi: this image never ran PlatformDebugLibIoPortDetect, so it is a RELEASE \
             build with DEBUG() compiled out and there is no log to capture. The 2 MiB \
             edk2-ovmf pair `scripts/fetch-testdata.sh ovmf` copies is one of those; a build \
             with debug strings in it — the 4 MiB pair in a distribution's qemu firmware \
             package — narrates the boot from the reset vector"
        );
        assert_eq!(
            log.len(),
            0,
            "the firmware never asked whether a debug console was fitted and wrote to it anyway"
        );
        return;
    }

    println!(
        "q35-uefi: the firmware ran the detect {} time(s) — the SEC instance of the library \
         re-reads the port before every DEBUG(), because it has no writable globals to cache \
         the answer in",
        debugcon.probes()
    );
    assert!(
        !log.is_empty(),
        "the firmware read 0x402, got 0xe9 back — which is the only answer that makes it use \
         the port at all — and then wrote nothing to it. EDK II's DebugPrintMarker calls \
         PlatformDebugLibIoPortDetect immediately before the IoWriteFifo8 that is the log"
    );

    // And the end-to-end one: the format string that is in the image is the
    // line that came out of the port.
    const SEC: &[u8] = b"SecCoreStartupWithStack(";
    if contains(&code, SEC) {
        assert!(
            contains(&log, SEC),
            "the image carries SEC's own DEBUG() format string, so it is a build with debug \
             strings in it, and the very first thing it prints did not reach the port. What did \
             is:\n{}",
            String::from_utf8_lossy(&log[..log.len().min(2048)])
        );
        println!(
            "q35-uefi: and the first DEBUG() a UEFI machine executes — SEC's own, out of the \
             uncompressed SEC volume — is in it"
        );
    }
}

/// The whole point of a variable store: what one boot writes, the next boot
/// reads.
///
/// Two boots of the same firmware, and the **only** thing carried from the
/// first to the second is the variable bank's bytes. In the first, the UEFI
/// Shell is told to create a non-volatile variable; in the second it is asked
/// for it, and prints it back.
///
/// This is the assertion the board could not make until the flash answered
/// `QemuFlashDetected`. Before that, everything in the first boot below still
/// happened — the shell set the variable, read it back in the same run and
/// reported the bytes — because the firmware had quietly fallen back to a
/// variable store in RAM, and the second boot found nothing. What makes this a
/// *reboot* rather than a second look at the same machine is that the second
/// `board_on_a_medium` builds a fresh one from the machine file, so the only
/// state that crosses is the medium.
///
/// The bank is bound as a **medium** rather than filled from the media table,
/// so the bytes that cross are the ones [`Machine::flush`] wrote back — the
/// path `--drive flash1=vars.fd` takes, and the one that would silently lose
/// everything the guest never barriered if `flash.cfi` had inherited the no-op
/// `Device::flush`.
///
/// Costs two whole boots, so it is gated on both images being present and
/// skips cleanly otherwise, like everything else here.
#[test]
fn a_variable_written_at_the_shell_is_there_after_a_reboot() {
    let (Ok(code_path), Ok(vars_path)) = (
        std::env::var("RSEMU_OVMF_CODE"),
        std::env::var("RSEMU_OVMF_VARS"),
    ) else {
        println!(
            "q35-uefi: set RSEMU_OVMF_CODE and RSEMU_OVMF_VARS to boot a firmware twice and \
             watch a variable survive the restart; see the module docs"
        );
        return;
    };
    let code = std::fs::read(&code_path).unwrap_or_else(|e| panic!("{code_path}: {e}"));
    let vars = std::fs::read(&vars_path).unwrap_or_else(|e| panic!("{vars_path}: {e}"));
    let params: Vec<(&str, String)> = vec![
        ("flash", format!("{}", code.len() + vars.len())),
        ("vars", format!("{}", vars.len().max(0x1000))),
    ];
    let ms: u64 = std::env::var("RSEMU_OVMF_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_MS);

    // A GUID of this test's own, and deliberately not the UEFI global variable
    // namespace: EDK II's `VarCheckUefiLib` refuses a name under
    // `gEfiGlobalVariableGuid` that the specification does not define, so
    // `setvar rsemu` with no `-guid` answers "Unable to set" whatever the flash
    // does. Nothing else on this board uses it.
    const GUID: &str = "8f1d4a52-6b3c-4e19-9d20-72736656d757";
    /// What the shell prints for the eight bytes below, and the marker that
    /// ends both runs.
    const VALUE: &str = "01 02 03 04 05 06 07 08";

    // -- the first boot: create it, and read it back in the same run ---------
    let first = run_the_shell(
        &code,
        &vars,
        &[],
        &params,
        ms,
        &[
            format!("setvar rsemu -guid {GUID} -nv -bs -rt =0102030405060708\r"),
            format!("setvar rsemu -guid {GUID}\r"),
        ],
        VALUE,
    );
    assert!(
        first.reached,
        "the shell never printed the variable it had just been told to set; it printed:\n{}",
        first.text
    );
    assert_ne!(
        first.store, vars,
        "the run reached the shell and set a non-volatile variable, and the variable store came \
         back byte-identical to the image it started from: nothing wrote the flash"
    );

    // The variable driver's own housekeeping is in there too, and `BootOrder`
    // is the one `docs/platforms/q35-uefi.md` named: a BDS that selected and
    // started a boot option writes it, and for a long time this board's store
    // did not have it.
    for name in ["BootOrder", "Boot0001", "rsemu"] {
        assert!(
            contains(&first.store, &utf16(name)),
            "the variable store the first boot left has no {name:?} in it"
        );
    }
    println!(
        "q35-uefi: the first boot left {} programmed byte(s) in the variable store, against {} \
         in the image it started from",
        first.store.iter().filter(|b| **b != 0xff).count(),
        vars.iter().filter(|b| **b != 0xff).count()
    );

    // -- and the second: a fresh machine, and only those bytes carried over --
    let second = run_the_shell(
        &code,
        &first.store,
        &[],
        &params,
        ms,
        &[format!("setvar rsemu -guid {GUID}\r")],
        VALUE,
    );
    assert!(
        second.reached,
        "a variable a previous boot wrote to the flash did not come back after a restart; the \
         second boot printed:\n{}",
        second.text
    );
    println!("q35-uefi: and the second boot read it back out of the flash");
}

/// One boot to the UEFI Shell with a script typed at it, and the variable bank
/// as [`Machine::flush`] left it.
struct Shell {
    /// Everything the guest wrote to COM1.
    text: String,
    /// Whether it printed the marker the run was waiting for.
    reached: bool,
    /// The variable bank's bytes, read out of the medium the run flushed to.
    store: Vec<u8>,
}

/// Boot the board with `vars` in the variable bank, type each of `lines` at the
/// `Shell>` prompt, and stop when the guest prints `stop_at`.
fn run_the_shell(
    code: &[u8],
    vars: &[u8],
    disk: &[u8],
    params: &[(&str, String)],
    ms: u64,
    lines: &[String],
    stop_at: &str,
) -> Shell {
    // The bank's backing store, exactly its size: `flash.cfi` refuses a medium
    // of any other, because a short one would take back only a prefix and
    // leave a firmware that boots once and never again.
    let store = Arc::new(RamStore::new(vars.len() as u64));
    Medium::write_at(&*store, 0, vars).expect("a fresh store takes the image");
    let Board {
        machine: mut m,
        cpu,
        console,
        hosts,
        ..
    } = match board_on_a_medium(
        code.to_vec(),
        Vec::new(),
        disk.to_vec(),
        Some(Arc::clone(&store)),
        params,
    ) {
        Ok(built) => built,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let script = Script {
        steps: lines
            .iter()
            .map(|line| (String::from("Shell> "), line.clone()))
            .collect(),
        stop_at: String::from(stop_at),
    };
    let run = x86boot::run(
        &mut m,
        &cpu,
        &console,
        Drains::open(&hosts, &console),
        GlobalTime::from_nanos(ms * 1_000_000),
        &script,
    );
    println!(
        "q35-uefi: a boot of {} ms stopped at {} ms having typed {} of {} line(s)",
        ms,
        run.at.as_nanos() / 1_000_000,
        run.typed,
        script.steps.len()
    );
    // The run is over: this is what `rsemu run … --drive flash1=vars.fd` does
    // when the machine stops, and it is the one call that puts what the guest
    // programmed where the next boot will look for it.
    m.flush().expect("the flash writes its bank back");
    let mut out = vec![0u8; vars.len()];
    Medium::read_at(&*store, 0, &mut out).expect("reading the medium back");
    Shell {
        text: run.text,
        reached: run.reached,
        store: out,
    }
}

/// A variable name as the store holds it: UTF-16LE, no terminator.
fn utf16(name: &str) -> Vec<u8> {
    name.encode_utf16().flat_map(u16::to_le_bytes).collect()
}

/// Whether `needle` appears anywhere in `haystack`.
fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.windows(needle.len()).any(|w| w == needle)
}

/// What a run of a UEFI firmware has to have shown to count.
///
/// `x86boot::assert_booted` is the kernel's version of this and looks for
/// `Linux version`. A firmware cannot be held to a fixed string, because what
/// it prints is the *build's* choice: a `RELEASE` EDK II says nothing at all
/// until its console driver comes up, and its `DEBUG()` output goes to I/O port
/// `0x402` behind a detect that this board fails.
///
/// So the standing assertions are the two that hold for any image. The reset
/// vector executed out of the flash and the processor reached **long mode**,
/// which is SEC's whole job and which nothing but a working code bank can
/// produce; and the guest **said something on COM1**, which for an `OvmfPkg`
/// build means BDS reached the terminal `PlatformBootManagerLib` puts on the
/// serial port. That second one was not assertable until the exception path
/// worked — `docs/platforms/q35-uefi.md` has the three architectural gaps that
/// stood between this board and its shell, and the ledger of what is left.
///
/// `RSEMU_OVMF_EXPECT` adds a string the guest must have printed, and
/// `RSEMU_OVMF_STOP_AT` with `RSEMU_OVMF_INPUT` turns the run into a
/// conversation.
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
    }
    assert!(
        !run.text.is_empty(),
        "the firmware printed nothing on COM1; it never reached the terminal \
         PlatformBootManagerLib puts on the serial port, and \
         docs/platforms/q35-uefi.md says what that has meant before"
    );
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

/// What the firmware asked the board about itself, in order.
///
/// The one thing about this board that no address space can be asked: a
/// selector write leaves nothing behind, so "which files did OVMF read" is
/// otherwise answerable only from a `DEBUG()` log on a debug port this board
/// does not have. `q35.fwcfg` keeps the last sixty-four selectors, and the test
/// holds a handle to the device because the binding that built it kept one.
///
/// It is worth printing on every boot: a firmware that takes no tables looks
/// exactly like one that was never asked, and this says which.
fn report_fwcfg(fwcfg: &FwCfg) {
    println!("q35-uefi: what the firmware asked fw_cfg for:");
    for (key, name) in fwcfg.trace() {
        let what = match (key, name) {
            (_, Some(file)) => file,
            (0x0000, _) => String::from("<signature>"),
            (0x0001, _) => String::from("<interface version>"),
            (0x0005, _) => String::from("<boot cpu count>"),
            (0x000f, _) => String::from("<max cpu count>"),
            (0x0019, _) => String::from("<the file directory>"),
            _ => String::from("<not served: reads as zeroes>"),
        };
        println!("q35-uefi:   {key:#06x}  {what}");
    }
    let (selector, offset) = fwcfg.cursor();
    println!("q35-uefi:   it stopped {offset} byte(s) into item {selector:#06x}");
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
    // The task-priority register, which **is** `CR8` (SDM Vol 3A §11.8.6.1):
    // the processor has no copy of its own, so this one byte is what a
    // `MOV CR8` left behind and what a `MOV RAX, CR8` would read back. Zero
    // here after a run that took an exception is itself a fact — EDK II's
    // `CommonInterruptEntry` saves `CR8` and restores it.
    println!(
        "q35-uefi:   local APIC TPR  = {:#010x}",
        mem.read(0xfee0_0080, Width::U32, MemAttrs::DEBUG)
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

/// What the firmware did with the disk controller.
///
/// Three registers and one bit, and between them they say how far the storage
/// stack got without any output from the guest at all:
///
/// * `BAR0` non-zero means **`PciBusDxe` enumerated the function and allocated
///   it a window** out of the 32-bit aperture `PlatformInitLib` published.
/// * `COMMAND.MSE` (bit 1) and `COMMAND.BME` (bit 2) mean a driver called
///   `PciIo->Attributes()` — that is `NvmExpressDxe` starting, and bus
///   mastering is what lets the controller fetch its own submission queue.
/// * `CC.EN` and `CSTS.RDY` mean the driver **brought the controller up** and
///   the controller answered (NVM Express 1.4 §3.1.5): admin queues placed,
///   doorbells written, identify issued.
///
/// A run that reaches the shell and prints `map: No mapping found.` is telling
/// you the same thing from the guest's side; this says which of those steps
/// was the one that did not happen.
fn report_nvme(m: &Machine, cpu: &X86) {
    let mem = m.space("mem").expect("the board declares `mem`");
    let port = m.space("port").expect("the board declares `port`");
    let cfg = |offset: u32| -> u32 {
        // `MemAttrs::DEFAULT` for the address latch, as in `report_chipset`:
        // mechanism #1 cannot be reached without writing one, and the run is
        // over.
        let _ = port.write(
            0xcf8,
            Width::U32,
            u64::from(0x8000_2000 | (offset & 0xfc)),
            MemAttrs::DEFAULT,
        );
        port.read(0xcfc, Width::U32, MemAttrs::DEBUG).unwrap_or(!0) as u32
    };
    let command = cfg(0x04);
    let bar = u64::from(cfg(0x10) & !0xf_u32) | (u64::from(cfg(0x14)) << 32);
    println!(
        "q35-uefi:   nvme 00:04.0 command={command:#06x} bar0={bar:#x} (MSE={}, BME={})",
        (command >> 1) & 1,
        (command >> 2) & 1
    );
    if bar == 0 || command & 0x2 == 0 {
        println!(
            "q35-uefi:   no memory window: the firmware's PCI bus driver never placed one, so \
             nothing could have bound"
        );
        return;
    }
    // Whether the *guest* can reach the window, which is a different question
    // from whether the board decodes it: a 64-bit BAR the firmware placed above
    // 4 GiB is only reachable if the firmware's own page tables cover it, and a
    // driver that reads all-ones from a register block it cannot see waits for
    // a bit that will never change.
    match read_debug(cpu, mem, bar, Width::U32) {
        Some(word) => println!(
            "q35-uefi:   the firmware's page tables map {bar:#x}, and CAP reads {word:#010x} \
             through them"
        ),
        None => println!(
            "q35-uefi:   {bar:#x} is NOT mapped by the page tables in CR3: a guest access to the \
             register block would fault"
        ),
    }
    let reg = |offset: u64| {
        mem.read(bar + offset, Width::U32, MemAttrs::DEBUG)
            .unwrap_or(!0) as u32
    };
    let (cc, csts) = (reg(0x14), reg(0x1c));
    println!(
        "q35-uefi:   cap={:08x}_{:08x} cc={cc:#010x} csts={csts:#010x} aqa={:#010x} asq={:#x}",
        reg(4),
        reg(0),
        reg(0x24),
        u64::from(reg(0x28)) | (u64::from(reg(0x2c)) << 32),
    );
    println!(
        "q35-uefi:   controller {} and {}",
        if cc & 1 == 1 { "enabled" } else { "disabled" },
        if csts & 1 == 1 { "ready" } else { "not ready" }
    );
}

// ---------------------------------------------------------------------------
// naming what stopped it
// ---------------------------------------------------------------------------

/// The first exception the firmware takes, and the instruction that raised it.
///
/// Opt-in through `RSEMU_OVMF_PROBE`, because it costs a second boot.
///
/// A `RELEASE` EDK II says nothing on any console this board has, so the only
/// way to name what stopped it is to watch the processor. The trouble is that
/// an exception whose handler faults on itself **destroys the evidence**: the
/// recursion pushes frames until the stack walks out of the identity map, and
/// on the way down it writes over the handler it was executing, so the
/// post-mortem's disassembly at `RIP` is nonsense and the vector is lost.
///
/// So this re-runs the board — the machine is deterministic, which is what
/// makes a second run the same run — up to `RSEMU_OVMF_PROBE_MS` (default 150)
/// virtual milliseconds before the first run stopped making progress, and from
/// there advances **one processor clock at a time**, reading the guest's own
/// interrupt descriptor table whenever the register moves.
///
/// What it prints when the processor lands on one of that table's gates is the
/// **frame the processor just pushed**, not the sample before it. The frame is
/// the processor's own account of the fault — the faulting `CS:RIP`, the error
/// code, the flags and the stack pointer — and it is right even when several
/// instructions ran between two samples, which the sample before it is not.
fn probe_first_exception(params: &[(&str, String)], stopped: GlobalTime) {
    if std::env::var("RSEMU_OVMF_PROBE").is_err() {
        return;
    }
    let (Ok(code_path), vars_path) = (
        std::env::var("RSEMU_OVMF_CODE"),
        std::env::var("RSEMU_OVMF_VARS").ok(),
    ) else {
        return;
    };
    let code = std::fs::read(&code_path).unwrap_or_default();
    let vars = vars_path
        .and_then(|p| std::fs::read(p).ok())
        .unwrap_or_default();
    let window: u64 = std::env::var("RSEMU_OVMF_PROBE_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(150);
    let Ok(Board {
        machine: mut m,
        cpu,
        ..
    }) = board(code, vars, disk_from_env(), params)
    else {
        return;
    };
    let fine_from = stopped.as_nanos().saturating_sub(window * 1_000_000);
    println!(
        "q35-uefi: probe: replaying to {} ms, then one clock at a time",
        fine_from / 1_000_000
    );
    while m.now().as_nanos() < fine_from {
        if m.run_for(GlobalTime::from_nanos(1_000_000)).is_err() {
            return;
        }
    }
    /// How many exceptions to report before giving up on the run.
    ///
    /// One is the interesting number when a handler works; when it does not,
    /// the second and third say so — and the first is still the one that
    /// names what the firmware could not do.
    const PROBE_DEPTH: usize = 4;

    // One processor clock at 25 MHz. Every instruction costs at least one, and
    // a core that has overspent its slice is held off until virtual time
    // catches up — so no instruction boundary goes unsampled.
    //
    // Reached through `step_until` rather than `run_for`, and the difference is
    // the whole probe: `run_for` **declines to split a scheduler round**, so a
    // forty-nanosecond span inside a one-millisecond quantum runs the whole
    // quantum and steps over everything in it. `step_until` is the debugger's
    // entry point and cuts the round, which is what makes one sample one
    // instruction.
    const CLOCK_NS: u64 = 40;

    let mut gates: Vec<u64> = Vec::new();
    let mut table = (0u64, 0u32);
    let mut prev = cpu.regs();
    let mut inside = false;
    let mut seen = 0usize;
    while m.now() < stopped {
        let next = GlobalTime::from_nanos(m.now().as_nanos() + CLOCK_NS);
        if m.step_until(next).is_err() {
            break;
        }
        let regs = cpu.regs();
        if regs.rip == prev.rip && regs.rsp == prev.rsp {
            continue;
        }
        // Borrowed inside the loop rather than outside it: stepping takes the
        // machine by `&mut`, so a space held across the call would not compile.
        let mem = m.space("mem").expect("the board declares `mem`");
        let sys = cpu.sys();
        if (sys.idtr.base, sys.idtr.limit) != table {
            table = (sys.idtr.base, sys.idtr.limit);
            gates = read_gates(&cpu, mem, table.0, table.1);
        }
        let hit = gates.iter().position(|gate| *gate == regs.rip);
        if let Some(vector) = hit
            && !inside
        {
            // The frame the processor has just pushed, which is the only
            // account of the fault that survives: `RIP` in the sample before
            // this one is the instruction that raised it *if* nothing else ran
            // in between, and the frame says so without the *if*.
            //
            // Five or six eight-byte words, low to high: an error code for the
            // vectors that have one, then `RIP`, `CS`, `RFLAGS`, `RSP`, `SS`
            // (*Intel SDM* volume 3A §6.14.2 — long mode pushes `SS:RSP`
            // whether or not the privilege level changed).
            let word = |i: u64| -> u64 {
                let at = regs.rsp + i * 8;
                cpu.translate_debug(at)
                    .phys(at)
                    .and_then(|pa| mem.read(pa, Width::U64, MemAttrs::DEBUG).ok())
                    .unwrap_or(0)
            };
            let has_error = matches!(vector, 8 | 10..=14 | 17 | 21 | 29 | 30);
            let base = u64::from(has_error);
            let (faulted, cs, rflags, rsp) =
                (word(base), word(base + 1), word(base + 2), word(base + 3));
            println!(
                "q35-uefi: probe: {} ms: vector {vector} at handler {:#x}, faulting \
                 {cs:#x}:{faulted:#x} err {:#x} rflags {rflags:#x} rsp {rsp:#x} cr2 {:#x}",
                m.now().as_nanos() / 1_000_000,
                regs.rip,
                if has_error { word(0) } else { 0 },
                sys.cr2
            );
            for line in cpu.disassemble(cs as u16, faulted, 1) {
                println!("q35-uefi: probe:   {line}");
            }
            if let Some(pa) = cpu.translate_debug(faulted).phys(faulted) {
                let hex: Vec<String> = (0..16)
                    .map(|i| {
                        mem.read(pa + i, Width::U8, MemAttrs::DEBUG)
                            .map_or_else(|_| "??".to_string(), |b| format!("{b:02x}"))
                    })
                    .collect();
                println!("q35-uefi: probe:   bytes {}", hex.join(" "));
            }
            println!(
                "q35-uefi: probe:   the sample before it was {:#x}:{:#x} with rsp {:#x}",
                prev.cs, prev.rip, prev.rsp
            );
            seen += 1;
            // A handler that faults on itself is the interesting shape, so a
            // few more are printed before the run is abandoned: the first
            // frame names what the firmware could not do, and the second names
            // what its handler could not do about it.
            if seen >= PROBE_DEPTH {
                return;
            }
        }
        inside = hit.is_some();
        prev = regs;
    }
    println!("q35-uefi: probe: no exception was taken in the window");
}

/// The entry point of each of the first thirty-two interrupt gates, read out of
/// the guest's own table.
///
/// A 64-bit gate is sixteen bytes and its offset is split into three: bytes
/// 0-1, bytes 6-7 and bytes 8-11 (*Intel SDM* volume 3A §6.14.1).
fn read_gates(cpu: &X86, mem: &AddressSpace, base: u64, limit: u32) -> Vec<u64> {
    let count = (u64::from(limit) + 1) / 16;
    (0..count.min(32))
        .map(|vector| {
            let at = base + vector * 16;
            let Some(pa) = cpu.translate_debug(at).phys(at) else {
                return 0;
            };
            let read = |offset: u64, width: Width| {
                mem.read(pa + offset, width, MemAttrs::DEBUG).unwrap_or(0)
            };
            read(0, Width::U16) | (read(6, Width::U16) << 16) | (read(8, Width::U32) << 32)
        })
        .collect()
}

/// The NUL-terminated ASCII at each address `RSEMU_OVMF_ASCII` names.
///
/// One line of code, and it is the difference between "the firmware stopped"
/// and "the firmware said why". EDK II's `DebugAssert` formats
/// `ASSERT [<driver>] <file>(<line>): <expression>` into a 512-byte buffer on
/// its own stack, offers it to `SerialPortWrite` **only if the debug port
/// answered**, and then calls `CpuDeadLoop`. On this board the port at `0x402`
/// does not answer, so the message is never printed — but it is still sitting
/// in that stack frame when the run ends, and the register that pointed at it
/// is in the post-mortem dump.
fn report_ascii(m: &Machine, cpu: &X86) {
    let Ok(list) = std::env::var("RSEMU_OVMF_ASCII") else {
        return;
    };
    let mem = m.space("mem").expect("the board declares `mem`");
    for item in list.split(',').filter(|s| !s.trim().is_empty()) {
        let text = item.trim().trim_start_matches("0x");
        let Ok(at) = u64::from_str_radix(text, 16) else {
            continue;
        };
        let mut out = String::new();
        for i in 0..512 {
            match read_debug(cpu, mem, at + i, Width::U8).map(|b| b as u8) {
                None | Some(0) => break,
                Some(byte) if byte.is_ascii_graphic() || byte == b' ' => out.push(char::from(byte)),
                Some(_) => out.push('.'),
            }
        }
        println!("q35-uefi: the string at {at:#x}: {out:?}");
    }
}

/// A hex dump of whatever `RSEMU_OVMF_HEX=addr[:length]` names.
///
/// The companion to [`report_ascii`]: an assertion says *what* was wrong with a
/// value and this says what the value was. Reads are `MemAttrs::DEBUG` and
/// through the guest's own page tables, so a driver's private structure can be
/// read at the pointer a register was left holding.
fn report_hex(m: &Machine, cpu: &X86) {
    let Ok(list) = std::env::var("RSEMU_OVMF_HEX") else {
        return;
    };
    let mem = m.space("mem").expect("the board declares `mem`");
    for item in list.split(',').filter(|s| !s.trim().is_empty()) {
        let (address, length) = item.split_once(':').unwrap_or((item, "64"));
        let Ok(at) = u64::from_str_radix(address.trim().trim_start_matches("0x"), 16) else {
            continue;
        };
        let length: u64 = length.trim().parse().unwrap_or(64);
        for row in 0..length.div_ceil(16) {
            let base = at + row * 16;
            let bytes: Vec<String> = (0..16)
                .map(|i| {
                    read_debug(cpu, mem, base + i, Width::U8)
                        .map_or_else(|| "??".to_string(), |b| format!("{:02x}", b as u8))
                })
                .collect();
            println!("q35-uefi:   {base:#012x}  {}", bytes.join(" "));
        }
    }
}

/// Which firmware driver an address is in, asked of the firmware.
///
/// `RSEMU_OVMF_WHOIS=1` resolves the processor's final `RIP` and every word on
/// the stack that lands inside a loaded image; `RSEMU_OVMF_WHOIS=0x7e80347,…`
/// adds addresses of its own. It costs nothing and needs no replay, because
/// everything it reads is still resident when the run ends.
///
/// **This is the answer to "a `RELEASE` build prints nothing".** A firmware
/// that hangs is a hexadecimal address and no name — and the driver that owns
/// that address is written down *inside the image itself*, because every EDK II
/// build carries a PE/COFF debug directory whose CodeView record is the path of
/// the `.pdb` it was linked against. `PeCoffLoaderGetPdbPointer` is how EDK II
/// prints `Loading driver at 0x0007E7E000 EntryPoint=… NvmExpressDxe.efi`; this
/// reads exactly the same field, from outside, after the fact.
///
/// PE/COFF is the *UEFI Specification*'s own image format (UEFI 2.10 §2.1.1),
/// so the layout is Microsoft's published PE32+ one: `MZ` at the image base,
/// `e_lfanew` at 0x3c, `PE\0\0`, then the optional header with `SizeOfImage` at
/// +0x38 and the debug data directory at +0xa0.
///
/// A `LoadImage` allocates pages, so a loaded image always starts on a 4 KiB
/// boundary and the search below is a walk down from the address.
fn report_modules(m: &Machine, cpu: &X86) {
    let Ok(list) = std::env::var("RSEMU_OVMF_WHOIS") else {
        return;
    };
    let mem = m.space("mem").expect("the board declares `mem`");
    let regs = cpu.regs();
    let mut wanted: Vec<(String, u64)> = vec![(String::from("rip"), regs.rip)];
    for item in list.split(',').filter(|s| !s.trim().is_empty()) {
        let text = item.trim().trim_start_matches("0x");
        if let Ok(at) = u64::from_str_radix(text, 16) {
            wanted.push((format!("{at:#x}"), at));
        }
    }
    // The stack, which is the poor relation of a backtrace and enough: a
    // firmware's frames are not walkable without unwind data, but every return
    // address a chain of calls left behind is still sitting there, and naming
    // the *images* they fall in says which drivers are on the stack.
    for i in 0..64u64 {
        let at = regs.rsp + i * 8;
        if let Some(word) = read_debug(cpu, mem, at, Width::U64)
            && word > 0x10_0000
            && word < 0x1_0000_0000
        {
            wanted.push((format!("[rsp+{:#x}]", i * 8), word));
        }
    }
    println!("q35-uefi: which image each address is in:");
    let mut said: Vec<(u64, String)> = Vec::new();
    for (label, at) in wanted {
        let Some(base) = image_base(cpu, mem, at) else {
            continue;
        };
        let name = said
            .iter()
            .find(|(b, _)| *b == base)
            .map(|(_, n)| n.clone())
            .unwrap_or_else(|| {
                let name = module_name(cpu, mem, base)
                    .unwrap_or_else(|| String::from("<no debug directory>"));
                said.push((base, name.clone()));
                name
            });
        println!(
            "q35-uefi:   {label:>14} = {at:#012x}  {name} + {:#x}",
            at - base
        );
    }
}

/// A debug read of `width` bytes at a guest *virtual* address, or `None` if it
/// is not mapped.
fn read_debug(cpu: &X86, mem: &AddressSpace, at: u64, width: Width) -> Option<u64> {
    let pa = cpu.translate_debug(at).phys(at)?;
    mem.read(pa, width, MemAttrs::DEBUG).ok()
}

/// The base of the PE/COFF image containing `at`, found by walking down.
///
/// Bounded at 64 MiB, which is more than the largest thing a UEFI firmware
/// loads and short enough that an address in no image costs nothing.
fn image_base(cpu: &X86, mem: &AddressSpace, at: u64) -> Option<u64> {
    let mut base = at & !0xfff;
    for _ in 0..16384 {
        if read_debug(cpu, mem, base, Width::U16) == Some(0x5a4d) {
            let lfanew = read_debug(cpu, mem, base + 0x3c, Width::U32)?;
            if lfanew >= 0x1000 {
                base = base.checked_sub(0x1000)?;
                continue;
            }
            let pe = base + lfanew;
            if read_debug(cpu, mem, pe, Width::U32) == Some(0x0000_4550) {
                // The optional header follows the 24-byte file header, and
                // `SizeOfImage` is 0x38 into it. An address past the end is an
                // image that merely sits below this one.
                let opt = pe + 0x18;
                let size = read_debug(cpu, mem, opt + 0x38, Width::U32)?;
                if at - base < size {
                    return Some(base);
                }
                return None;
            }
        }
        base = base.checked_sub(0x1000)?;
    }
    None
}

/// The name the image was linked under, out of its own CodeView record.
fn module_name(cpu: &X86, mem: &AddressSpace, base: u64) -> Option<String> {
    let lfanew = read_debug(cpu, mem, base + 0x3c, Width::U32)?;
    let opt = base + lfanew + 0x18;
    // DataDirectory[6] is the debug directory: 0x70 for the sixteen directory
    // entries, plus six of them at eight bytes each.
    let rva = read_debug(cpu, mem, opt + 0xa0, Width::U32)?;
    let size = read_debug(cpu, mem, opt + 0xa4, Width::U32)?;
    if rva == 0 || size < 28 {
        return None;
    }
    for entry in 0..size / 28 {
        let at = base + rva + entry * 28;
        if read_debug(cpu, mem, at + 12, Width::U32)? != 2 {
            continue; // not IMAGE_DEBUG_TYPE_CODEVIEW
        }
        let data = read_debug(cpu, mem, at + 20, Width::U32)?;
        if data == 0 {
            continue;
        }
        let record = base + data;
        // `RSDS` (16-byte GUID + age), `NB10` (offset + two dwords) and `MTOC`
        // (a 16-byte UUID) are the three EDK II toolchains emit, and each is
        // followed by a NUL-terminated path.
        let text = match read_debug(cpu, mem, record, Width::U32)? {
            0x5344_5352 => record + 24,
            0x3031_424e => record + 16,
            0x434f_544d => record + 20,
            _ => continue,
        };
        let mut name = String::new();
        for i in 0..256 {
            match read_debug(cpu, mem, text + i, Width::U8)? as u8 {
                0 => break,
                b'/' | b'\\' => name.clear(),
                byte => name.push(char::from(byte)),
            }
        }
        if !name.is_empty() {
            return Some(name);
        }
    }
    None
}

/// Disassemble whatever `RSEMU_OVMF_DISASM` names, after the run.
///
/// A comma-separated list of guest addresses, hexadecimal. It exists because a
/// firmware that *does* reach a console names its own faulting address and then
/// keeps running: EDK II's exception handler prints `RIP - 00000000080C655D`
/// and dead-loops, so the instruction is still sitting in memory when the run
/// ends and there is nothing to catch in the act. Reading it back is a whole
/// probe cheaper than replaying the boot.
fn report_disassembly(m: &Machine, cpu: &X86) {
    let Ok(list) = std::env::var("RSEMU_OVMF_DISASM") else {
        return;
    };
    let mem = m.space("mem").expect("the board declares `mem`");
    let cs = cpu.regs().cs;
    for item in list.split(',').filter(|s| !s.trim().is_empty()) {
        let text = item.trim().trim_start_matches("0x");
        let Ok(at) = u64::from_str_radix(text, 16) else {
            println!("q35-uefi: RSEMU_OVMF_DISASM: {item:?} is not a hexadecimal address");
            continue;
        };
        println!("q35-uefi: what is at {at:#x}:");
        for line in cpu.disassemble(cs, at, 6) {
            println!("q35-uefi:   {line}");
        }
        if let Some(pa) = cpu.translate_debug(at).phys(at) {
            let hex: Vec<String> = (0..24)
                .map(|i| {
                    mem.read(pa + i, Width::U8, MemAttrs::DEBUG)
                        .map_or_else(|_| "??".to_string(), |b| format!("{b:02x}"))
                })
                .collect();
            println!("q35-uefi:   bytes {}", hex.join(" "));
        }
    }
}
