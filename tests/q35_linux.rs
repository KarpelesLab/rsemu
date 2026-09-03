//! How far does a modern Linux kernel get on the `q35-linux` board, and does
//! it find the disk?
//!
//! [`tests/pc64_linux.rs`](pc64_linux.rs) asks the first half of that question
//! on the smallest machine a kernel can be entered on at all. This file asks
//! the second half, which needs a **PCI bus**: `q35-linux` is
//! [`q35`](q35_board.rs)'s chipset with `x86.linuxboot` where the firmware
//! socket was, and an NVM Express controller on the bus for the kernel to bind
//! a driver to.
//!
//! Every test but one is hermetic and always runs — they are about what the
//! board publishes, and what its interrupt controllers do, before anything
//! executes on it. The last is gated on `RSEMU_KERNEL`, exactly as
//! `pc64_linux` is: point it at a `bzImage` and it runs one, and without it the
//! test prints why and returns. No kernel is vendored, downloaded by
//! `cargo test`, or required for it (`CLAUDE.md`, Testing).
//!
//! ```text
//! scripts/fetch-testdata.sh initramfs-x86
//!
//! RSEMU_KERNEL=/boot/vmlinuz \
//! RSEMU_INITRD=testdata/x86/initramfs-x86.cpio \
//! RSEMU_KERNEL_MS=3000000 \
//! RSEMU_KERNEL_INPUT='rsemu# =>head -c 40 /dev/nvme0n1\n' \
//! RSEMU_KERNEL_STOP_AT='LBA 0' \
//!     cargo test --release --features machine-q35-linux --test q35_linux -- --nocapture
//! ```
//!
//! **There is no `RSEMU_KERNEL_CMDLINE` on that command any more.** The board's
//! own default line is what the kernel gets, and it carries no `nolapic`, no
//! `noapic` and no `hpet=disable`. Measured with a 6.6 kernel, that line now
//! takes the symmetric I/O path all the way through `check_timer()`:
//!
//! ```text
//! APIC: Switch to symmetric I/O mode setup
//! ..TIMER: vector=0x30 apic1=0 pin1=2 apic2=-1 pin2=-1
//! tsc: PIT calibration matches HPET. 1 loops
//! ```
//!
//! with no `..MP-BIOS bug: 8254 timer not connected to IO-APIC` and no
//! `Kernel panic - not syncing: IO-APIC + timer doesn't work!` after it.
//!
//! What that panic had been was **a missing route, not a missing delivery**,
//! and it was in this board's own machine file rather than in either APIC.
//! `pc.hpet` advertises `LEG_RT_CAP`, so `hpet_enable()` takes the legacy
//! replacement route: it registers the HPET as the global clock event, sets
//! `LEG_RT_CNF`, and — the half that matters — Linux therefore never calls
//! `pit_timer_init()`, because `hpet_time_init()` programs the 8254 only when
//! `hpet_enable()` fails. From then on the tick is supposed to arrive from HPET
//! comparator 0 on IRQ0, which is the 8259A's `IR0` and I/O APIC input 2. The
//! board wired `hpet0.t0` to `ioapic.irq16` and left the `legacy` pin
//! unconnected, so counter 0 was never loaded and comparator 0 was elsewhere:
//! nothing at all drove IRQ0, and all three of the kernel's fallbacks failed
//! because every one of them is another way of asking for it.
//!
//! `machines/q35-linux.machine` now builds the multiplexer §2.3.5 describes out
//! of one `wire.not` and five `wire.and`s.
//! [`docs/platforms/q35-linux.md`](../docs/platforms/q35-linux.md) has the
//! whole ledger, including the two items that were refuted rather than fixed.
//!
//! **It does not stop any more.** On that line, with the I/O APIC in use, a
//! stock Gentoo 6.6.67 kernel now reaches userspace and reads off the disk:
//!
//! ```text
//! Run /init as init process
//!
//! rsemu initramfs on Linux 6.6.67-gentoo-x86_64 x86_64
//! rsemu# head -c 40 /dev/nvme0n1
//! rsemu q35-linux nvme namespace, LBA 0
//! ```
//!
//! What had stopped it was **one bit in a redirection entry**, and `noapic`
//! being the control is what located it. The kernel programs a PCI interrupt
//! level-triggered and **active low** — PCI Local Bus 3.0 §2.2.6 — and the I/O
//! APIC was exclusive-oring that polarity bit into the level its input net
//! resolved to. A `core::wire` net carries an *assertion*, not a voltage
//! (`ROADMAP.md` §4.3: a fresh fan-in holds every source low and an undriven
//! wire sits low), so the idle line read as asserted the instant
//! `irq_startup()` unmasked the entry, and a level-triggered entry re-arms on
//! every end-of-interrupt: the processor took the vector, ended it, took it
//! again, and never retired the instruction after the `sti` that let it in.
//! The 8259A has no polarity bit, which is exactly why `noapic` booted.
//!
//! `report_apics` below is where that was read off the machine — the entry the
//! guest had written, and the local APIC's request register beside it:
//!
//! ```text
//! q35-linux:   irq11 01000000_0000e822 vector=0x22 level low  open remote-irr
//! q35-linux:   irr=..._00000004_00000000     isr=all zero
//! ```
//!
//! `the_entry_a_kernel_writes_for_this_boards_disk_is_quiet_while_the_disk_is`
//! asks the same question of the shipped board in milliseconds, and
//! [`tests/pc_apic.rs`](pc_apic.rs) drives the whole level-triggered lifecycle
//! through real guest instructions.
//!
//! | Variable | What it does |
//! | --- | --- |
//! | `RSEMU_KERNEL` | The `bzImage`. Without it the boot test skips. |
//! | `RSEMU_INITRD` | An initramfs. Optional; without one the kernel panics for want of a root, which is still a complete boot. |
//! | `RSEMU_DISK` | A raw image for the NVMe namespace. Without one the test writes its own signature into a blank namespace, which is what the `head -c 40` above reads back. |
//! | `RSEMU_KERNEL_MS` | How long to run, in virtual milliseconds. |
//! | `RSEMU_KERNEL_CMDLINE` | The command line, replacing the board's default. |
//! | `RSEMU_KERNEL_EXTMEM` | How much extended memory to give it, e.g. `512M`. |
//! | `RSEMU_KERNEL_INPUT` | Types at the guest: one `marker=>text` step per line. |
//! | `RSEMU_KERNEL_STOP_AT` | Ends the run when the guest prints this. |
//!
//! # What a run proves that `pc64` cannot
//!
//! `pc64` proves the *core* survives early boot. This board adds everything
//! between a kernel and a disk, and every step of it is a thing that can fail
//! on its own: the RSDP found by scanning `0xe0000`-`0xfffff` with no firmware
//! having staged it, the MADT read and the IMCR switched, PCI enumerated
//! through `0xcf8` and then through the ECAM window `MCFG` names, `BAR0` sized
//! and placed by the kernel's own resource allocator, `_PRT` consulted and an
//! I/O APIC redirection entry programmed for the interrupt the controller's
//! `INTA#` swizzles onto, admin and I/O queues built in this board's RAM, a
//! doorbell rung, a completion taken as a level-triggered interrupt, and a
//! block device published.
//!
//! **Everything printed as evidence is a byte the guest itself wrote to its own
//! serial port.** The image is run, never read, and never vendored
//! (`ROADMAP.md` §1).

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-q35",
    feature = "dev-nvme",
    feature = "dev-linuxboot",
    feature = "machine-q35-linux"
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
/// Half again as long as `pc64`'s ceiling, and the extra is not slack: the
/// kernel that reaches a shell there has, on this board, also to enumerate a
/// PCI bus, walk an ACPI namespace, and probe a controller. A ceiling rather
/// than a target — the run stops early when the processor stops making
/// progress, or when the guest prints `RSEMU_KERNEL_STOP_AT`.
const DEFAULT_MS: u64 = 1_500_000;

/// What the test stamps over the front of a blank namespace.
///
/// Forty bytes, so `head -c 40 /dev/nvme0n1` in the guest reads exactly this
/// and nothing after it. It is the same shape `riscv-virt`'s virtio test uses
/// (`RSEMU_RISCV_INPUT='rsemu# =>head -c 34 /dev/vda'`) and for the same
/// reason: a signature the guest reads back off the medium is the proof that
/// the whole path from a block-layer request to this test's own bytes is
/// joined up, and it needs no filesystem the guest's kernel may not have.
const SIGNATURE: &[u8] = b"rsemu q35-linux nvme namespace, LBA 0\n\0\0";

/// How big the namespace is when nothing is bound to it.
const DEFAULT_DISK: u64 = 16 * 1024 * 1024;

/// Everything the board needs to construct, with a `cpu.x86` that pushes what
/// it builds into `cpus`.
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

/// Build the board from its own machine file.
///
/// `disk` is the namespace's contents; the `disk` parameter is set from its
/// length so the controller is exactly as big as what was handed to it, which
/// is what a `--drive` on the command line would do.
fn board(
    kernel: Vec<u8>,
    initrd: Vec<u8>,
    disk: Vec<u8>,
    params: &[(&str, String)],
) -> Result<(Machine, Arc<X86>, Arc<CharPort>), String> {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings(&cpus));
    let capacity = format!("{}", (disk.len() as u64).max(DEFAULT_DISK));
    options = options.with_param("disk", capacity.as_str());
    for (name, value) in params {
        options = options.with_param(*name, value.as_str());
    }
    options.realize.media.insert("kernel", kernel);
    options.realize.media.insert("initrd", initrd);
    options.realize.media.insert("nvme0", disk);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = build(
        "q35-linux.machine",
        rsemu::machine::catalog::Q35_LINUX.source,
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

/// The board with nothing in any slot, which is what the hermetic tests want.
fn bare_board() -> Machine {
    match board(Vec::new(), Vec::new(), Vec::new(), &[]) {
        Ok((machine, _cpu, _console)) => machine,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Read a dword out of a space the way a guest would.
fn read32(space: &AddressSpace, at: u64) -> u32 {
    space
        .read(at, Width::U32, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("read of {at:#x} faulted: {e:?}")) as u32
}

/// Read a run of bytes out of a space the way a guest would.
fn read_bytes(space: &AddressSpace, at: u64, len: usize) -> Vec<u8> {
    let mut out = vec![0u8; len];
    space
        .read_bytes(at, &mut out, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("read of {at:#x} faulted: {e:?}"));
    out
}

// ---------------------------------------------------------------------------
// what the board publishes before anything executes
// ---------------------------------------------------------------------------

/// The question this board was built to answer, asked without a kernel.
///
/// A kernel on a q35 that cannot find an RSDP has no MADT, no `_PRT` and no
/// MCFG, and a firmware is what normally stages them. There is no firmware
/// here — so the claim is that the *machine file* stages them, by mapping a
/// generator's region inside ACPI §5.2.5.1's own search window, and that a
/// scan of `0xe0000`-`0xfffff` on sixteen-byte boundaries finds one.
#[test]
fn a_kernel_scanning_for_an_rsdp_finds_one_on_a_board_with_no_firmware() {
    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");

    // The scan an operating system actually performs, rather than a read of
    // the address the machine file happens to name.
    let found = (0xe_0000u64..0x10_0000)
        .step_by(16)
        .find(|at| read_bytes(mem, *at, 8) == b"RSD PTR ");
    assert_eq!(
        found,
        Some(0xe_0000),
        "ACPI §5.2.5.1 has an operating system search 0xe0000-0xfffff on 16-byte boundaries, \
         and on this board nothing else has staged anything there"
    );

    // And the half of the window that used to be a firmware socket decodes
    // nothing at all, which is what makes the scan above unambiguous.
    assert_eq!(
        read32(mem, 0xf_0000),
        0xffff_ffff,
        "0xf0000-0xfffff is the BIOS socket on `q35` and open bus here"
    );
}

/// The `sci-en` stand-in, seen from where a guest sees it.
///
/// `q35.acpi` publishes an FADT with `SMI_CMD` zero, which ACPI §5.2.9 defines
/// as *this system does not support ACPI mode transitions*. An operating system
/// that reads that and then finds `PM1_CNT.SCI_EN` clear has been told two
/// contradictory things — there is no way into ACPI mode, and we are not in it
/// — and gives up on ACPI entirely, which on this board would cost it the PCI
/// interrupt routing the disk arrives on. The board states the bit a POST would
/// have set; this is that bit, read through the window `PMBASE` placed.
#[test]
fn the_board_comes_up_in_acpi_mode_because_no_firmware_can_put_it_there() {
    let machine = bare_board();
    let port = machine.space("port").expect("the board declares `port`");
    // PMBASE is 0x600 and PM1_CNT is at offset 4 (ICH9 Table 13-11).
    let cnt = read32(port, 0x604);
    assert_eq!(cnt & 1, 1, "PM1_CNT.SCI_EN, ICH9 §13.8.3.3");
}

/// `LEG_RT_CNF` takes IRQ0 off the 8254 and gives it to HPET comparator 0.
///
/// This is the board's own multiplexer, asked without a kernel. The kernel boot
/// at the bottom of this file exercises it too, but that one is gated on
/// `RSEMU_KERNEL` and takes five minutes; this asks the same question of the
/// shipped machine file in a few milliseconds of virtual time, and it is what
/// stands between the wiring and a quiet deletion.
///
/// The observable is the master 8259A's **interrupt request register**, which a
/// read of port 0x20 returns when the last OCW3 did not select ISR. It latches
/// a request whether or not the line is masked, which is what makes it usable
/// here: nothing has to be allowed through to the processor, and the
/// processor's own interrupt flag is clear throughout.
///
/// Sources: IA-PC HPET Specification rev 1.0a §2.3.5 for what the bit selects,
/// the Intel 8254 data sheet for mode 2, and the 8259A data sheet for OCW3.
#[test]
fn the_legacy_replacement_route_moves_irq0_from_the_8254_to_the_hpet() {
    /// The HPET's general configuration register, and its two bits.
    const HPET_CONF: u64 = 0xfed0_0010;
    const HPET_ENABLE: u64 = 1 << 0;
    const HPET_LEGACY: u64 = 1 << 1;
    /// Comparator 0's configuration register, its enable, and the comparator.
    const HPET_T0_CONF: u64 = 0xfed0_0100;
    const HPET_T0_ENABLE: u64 = 1 << 2;
    const HPET_T0_COMPARATOR: u64 = 0xfed0_0108;
    /// The main counter.
    const HPET_COUNTER: u64 = 0xfed0_00f0;
    /// How many counts of each timer one half of this test waits for.
    const BUDGET_NS: u64 = 5_000_000;

    let mut m = bare_board();

    // Put the master 8259A into a known state. The initialization sequence also
    // recomputes the request register from the pins it can see, which is how
    // this test drops a latched request between the two halves below — an
    // 8259A has no other way to lower one without an acknowledge cycle.
    let init_master = |m: &Machine| {
        let port = m.space("port").expect("the board declares `port`");
        for (at, value) in [
            (0x20u64, 0x11u64), // ICW1: edge triggered, ICW4 to follow
            (0x21, 0x08),       // ICW2: vector base
            (0x21, 0x04),       // ICW3: a slave on IR2
            (0x21, 0x01),       // ICW4: 8086 mode
            (0x21, 0xff),       // OCW1: every line masked, which IRR ignores
        ] {
            port.write(at, Width::U8, value, MemAttrs::DEFAULT)
                .expect("the master 8259A decodes 0x20 and 0x21");
        }
    };
    let irr = |m: &Machine| {
        m.space("port")
            .expect("the board declares `port`")
            .read(0x20, Width::U8, MemAttrs::DEFAULT)
            .expect("a read of port 0x20 answers with IRR") as u8
    };
    let poke = |m: &Machine, at: u64, value: u64| {
        m.space("mem")
            .expect("the board declares `mem`")
            .write(at, Width::U32, value, MemAttrs::DEFAULT)
            .unwrap_or_else(|e| panic!("write of {at:#x} faulted: {e:?}"));
    };

    // -- the line as it is on every PC that has ever booted -------------------
    //
    // Counter 0 in mode 2 at a rate that gives several periods inside the
    // budget: the 8254 runs at 105/88 MHz, so 1000 counts is about 838
    // microseconds.
    init_master(&m);
    assert_eq!(irr(&m) & 1, 0, "nothing has asked for IRQ0 yet");
    {
        let port = m.space("port").expect("the board declares `port`");
        for (at, value) in [(0x43u64, 0x34u64), (0x40, 1000 & 0xff), (0x40, 1000 >> 8)] {
            port.write(at, Width::U8, value, MemAttrs::DEFAULT)
                .expect("the 8254 decodes 0x40-0x43");
        }
    }
    let _ = m.run_for(GlobalTime::from_nanos(BUDGET_NS));
    assert_eq!(
        irr(&m) & 1,
        1,
        "with LEG_RT_CNF clear the 8254's counter 0 owns IRQ0"
    );

    // -- and the line once the HPET takes it over -----------------------------
    //
    // `LEG_RT_CNF` set, the counter left running exactly as it was. The 8254 is
    // now disconnected, so a fresh initialization must find nothing on IR0 and
    // must still find nothing after several more of the counter's periods.
    poke(&m, HPET_CONF, HPET_ENABLE | HPET_LEGACY);
    init_master(&m);
    assert_eq!(irr(&m) & 1, 0, "the initialization dropped the latch");
    let _ = m.run_for(GlobalTime::from_nanos(BUDGET_NS));
    assert_eq!(
        irr(&m) & 1,
        0,
        "the 8254 is still counting and must no longer reach IRQ0"
    );

    // Comparator 0, one shot, a hundred microseconds out: a 10 MHz counter, so
    // 1000 ticks. This is the half that says the HPET has the line rather than
    // that nobody has it.
    let now = u64::from(read32(
        m.space("mem").expect("the board declares `mem`"),
        HPET_COUNTER,
    ));
    poke(&m, HPET_T0_COMPARATOR, now + 1000);
    poke(&m, HPET_T0_CONF, HPET_T0_ENABLE);
    let _ = m.run_for(GlobalTime::from_nanos(BUDGET_NS));
    assert_eq!(
        irr(&m) & 1,
        1,
        "with LEG_RT_CNF set comparator 0 drives IRQ0, and nothing else does"
    );
}

/// The controller is on the bus at the address the machine file names, and it
/// is reachable through both routes to configuration space.
#[test]
fn the_nvme_controller_answers_at_device_four_through_both_windows() {
    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    let port = machine.space("port").expect("the board declares `port`");

    // Mechanism #1: 00:04.0's identification and class.
    port.write(0xcf8, Width::U32, 0x8000_2000, MemAttrs::DEFAULT)
        .expect("CONFADD is a dword register");
    let id = read32(port, 0xcfc);
    assert_eq!(
        id & 0xffff,
        0x1234,
        "the vendor `nvme.controller` defaults to"
    );
    port.write(0xcf8, Width::U32, 0x8000_2008, MemAttrs::DEFAULT)
        .expect("CONFADD");
    assert_eq!(
        read32(port, 0xcfc) >> 8,
        0x0001_0802,
        "NVM Express is class 010802h (NVMe 1.4 §2.1.5)"
    );

    // And ECAM, at the address `PCIEXBAR` comes up pointing at: bus 0 is 0,
    // device 4 is 4 * 32 KiB, function 0 is 0 (datasheet §5.1.16).
    assert_eq!(
        read32(mem, 0xe000_0000 + 4 * 0x8000),
        id,
        "the same function through the window the (G)MCH placed"
    );
}

/// What a kernel does to a controller before it talks to it, and the proof that
/// the register block answers afterwards.
///
/// The interesting half is where `0x10100000` comes from: it is what Linux's own
/// resource allocator picked out of the `_CRS` window this board's DSDT
/// declares — the first megabyte-aligned address above the top of RAM. A board
/// whose `_CRS` was missing got `BAR 0: failed to assign` instead, and the
/// controller was never reached at all.
#[test]
fn the_controllers_register_block_answers_where_a_kernel_puts_it() {
    /// Where the kernel's allocator put `BAR0` on this board.
    const BAR0: u64 = 0x1010_0000;

    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    let port = machine.space("port").expect("the board declares `port`");
    let cfg = |offset: u32| {
        port.write(
            0xcf8,
            Width::U32,
            u64::from(0x8000_2000 | offset),
            MemAttrs::DEFAULT,
        )
        .expect("CONFADD is a dword register");
    };
    let write = |value: u32| {
        port.write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("CONFDATA");
    };

    // `BAR0` is a 64-bit memory base address register, so it takes two writes;
    // the low four bits are read-only type bits and a driver writes through
    // them.
    cfg(0x10);
    write(BAR0 as u32);
    cfg(0x14);
    write(0);
    // Memory space and bus mastering, which is what `pci_enable_device_mem`
    // and `pci_set_master` set between them.
    cfg(0x04);
    write(0x0006);

    // `CAP`, at offset 0 (NVM Express 1.4 §3.1.1). Open bus would read as ones,
    // which is exactly what the driver checks for before it goes any further.
    let cap_low = read32(mem, BAR0);
    assert_ne!(
        cap_low, 0xffff_ffff,
        "the register block does not decode at the address the base address \
         register was given"
    );
    assert_ne!(
        cap_low & 0xffff,
        0,
        "`CAP.MQES` is the maximum queue size and a controller must support at \
         least two entries"
    );
    // And the version, at offset 8, which is what tells the driver which of the
    // optional registers it may look at.
    assert_ne!(read32(mem, BAR0 + 8), 0xffff_ffff, "`VS`");
}

/// The sequence Linux's own driver puts a controller through before it sends a
/// command, replayed with the values it used on this board.
///
/// This began as an `#[ignore]`d ledger entry: the boot below reached
/// `nvme 0000:00:04.0: enabling device` and then the probe never finished, with
/// the controller left holding `CSTS = 0x2` — `CSTS.CFS` set, `CSTS.RDY` clear.
/// It asked the same question in five seconds instead of five minutes, and
/// answered it: `src/dev/nvme/ctrl.rs` placed three `CC` fields one bit too
/// high. It now runs.
///
/// *NVM Express* base specification, `CC` (Figure "Controller Configuration"):
/// `MPS` is bits **10:07**, `AMS` **13:11**, `SHN` **15:14**, `IOSQES`
/// **19:16** and `IOCQES` **23:20**, with 31:24 reserved. `ctrl.rs` had
/// `CC_MPS_SHIFT` 7 over five bits, `CC_SHN_SHIFT` 15, `CC_IOSQES_SHIFT` 17
/// and `CC_IOCQES_SHIFT` 21 — every field from `SHN` up shifted one bit — and
/// a `CC_MASK` of `0x01ff_f8f1` that dropped the top three bits of `MPS`.
///
/// The guest's own value settles which layout is right. A 6.6 kernel writes
/// `CC = 0x00460001`; read with the specification's field positions that is
/// `IOSQES` 6 (64-byte submission entries) and `IOCQES` 4 (16-byte completion
/// entries), which are the only two values NVMe defines. Read with the old
/// shifts they were 3 and 2, `Controller::enable`'s `iosqes == 6 && iocqes == 4`
/// was false, and the controller reported a fatal status instead of coming
/// ready.
#[test]
fn the_controller_comes_ready_for_the_configuration_a_current_kernel_writes() {
    /// Where the kernel's allocator put `BAR0` on this board.
    const BAR0: u64 = 0x1010_0000;
    /// `AQA` for a 32-entry admin queue pair, zero-based in both halves — which
    /// is `NVME_AQ_DEPTH`, and what the boot below was measured writing.
    const AQA: u32 = 0x001f_001f;
    /// `CC` as a 6.6 kernel builds it: `IOSQES` 6, `IOCQES` 4, `AMS` round
    /// robin, `SHN` none, `MPS` 0 (4 KiB), `CSS` the NVM command set, `EN`.
    const CC: u32 = 0x0046_0001;

    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    let port = machine.space("port").expect("the board declares `port`");
    let cfg = |offset: u32| {
        port.write(
            0xcf8,
            Width::U32,
            u64::from(0x8000_2000 | offset),
            MemAttrs::DEFAULT,
        )
        .expect("CONFADD");
    };
    let write_cfg = |value: u32| {
        port.write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("CONFDATA");
    };
    let reg = |offset: u64, value: u32| {
        mem.write(
            BAR0 + offset,
            Width::U32,
            u64::from(value),
            MemAttrs::DEFAULT,
        )
        .expect("the register block decodes");
    };

    cfg(0x10);
    write_cfg(BAR0 as u32);
    cfg(0x14);
    write_cfg(0);
    cfg(0x04);
    write_cfg(0x0006);

    // `nvme_disable_ctrl`: `CC` with `EN` clear, and `CSTS.RDY` must follow.
    reg(0x14, 0);
    assert_eq!(
        read32(mem, BAR0 + 0x1c) & 1,
        0,
        "`CSTS.RDY` after a disable"
    );

    // The admin queue pair, in this board's own RAM and page aligned, then
    // `CC` with `EN` set — the four writes, in the order the driver makes them.
    reg(0x24, AQA);
    reg(0x28, 0x0020_0000);
    reg(0x2c, 0);
    reg(0x30, 0x0020_1000);
    reg(0x34, 0);
    reg(0x14, CC);

    let csts = read32(mem, BAR0 + 0x1c);
    assert_eq!(
        csts & 0b10,
        0,
        "`CSTS.CFS`: the controller refused a configuration a current kernel \
         writes (CSTS={csts:#010x})"
    );
    assert_eq!(
        csts & 1,
        1,
        "`CSTS.RDY`: the controller never came ready (CSTS={csts:#010x})"
    );
}

/// Where the guest left the controller, printed after a run.
///
/// The board's own post-mortem, beside the processor's. A kernel that stops
/// while probing a disk says nothing about the disk, and these eight registers
/// say exactly how far its driver got: whether it was given an address at all,
/// whether it turned the controller on, whether the controller answered that it
/// was ready, and where it left the queues it built in this board's RAM.
///
/// Everything is read with `MemAttrs::DEBUG`, so nothing here can move the
/// machine (`CLAUDE.md`, devices).
fn report_nvme(m: &Machine) {
    let mem = m.space("mem").expect("the board declares `mem`");
    let port = m.space("port").expect("the board declares `port`");
    let debug = |space: &AddressSpace, at: u64| {
        space
            .read(at, Width::U32, MemAttrs::DEBUG)
            .map_or(0xffff_ffff, |v| v as u32)
    };
    let cfg = |offset: u32| {
        port.write(
            0xcf8,
            Width::U32,
            u64::from(0x8000_2000 | offset),
            MemAttrs::DEFAULT,
        )
        .expect("CONFADD");
        debug(port, 0xcfc)
    };
    let command = cfg(0x04);
    let bar = u64::from(cfg(0x10) & !0xfu32) | (u64::from(cfg(0x14)) << 32);
    let irq = cfg(0x3c);
    println!(
        "q35-linux: nvme 00:04.0 command={command:#06x} bar0={bar:#x} \
         interrupt line={:#04x} pin={:#04x}",
        irq & 0xff,
        (irq >> 8) & 0xff
    );
    if bar == 0 || command & 0x2 == 0 {
        println!("q35-linux:   no memory window: the guest never gave it one");
        return;
    }
    let reg = |offset: u64| debug(mem, bar + offset);
    println!(
        "q35-linux:   cap={:08x}_{:08x} vs={:#x} cc={:#010x} csts={:#010x} intms={:#010x}",
        reg(4),
        reg(0),
        reg(8),
        reg(0x14),
        reg(0x1c),
        reg(0x0c)
    );
    println!(
        "q35-linux:   aqa={:#010x} asq={:#x} acq={:#x}",
        reg(0x24),
        u64::from(reg(0x28)) | (u64::from(reg(0x2c)) << 32),
        u64::from(reg(0x30)) | (u64::from(reg(0x34)) << 32)
    );
}

/// The redirection entry a kernel writes for this board's disk, on this board,
/// with the disk quiet — and the local APIC that must therefore stay quiet too.
///
/// This is the boot below asked in a few milliseconds instead of sixteen
/// minutes. The value written is the one the guest itself wrote, read back out
/// of the part after a run that stopped inside `request_threaded_irq`:
///
/// ```text
/// q35-linux:   irq11 01000000_0000e822 vector=0x22 level low  open remote-irr
/// q35-linux:   irr=...  bit 0x22 set,  isr=0
/// ```
///
/// Bit 15 is the trigger mode and bit 13 the input pin polarity — level, active
/// low — because PCI Local Bus 3.0 §2.2.6 defines `INTA#`-`INTD#` that way and
/// an operating system handed a global system interrupt by `_PRT` with no
/// override programs every PCI interrupt so. The board's own `pirq-routes` put
/// device 4's `INTA#` on IRQ11, and nothing else on the board drives that net
/// while the controller is idle.
///
/// So after this write the local APIC must have nothing requested. An I/O APIC
/// that exclusive-ored the polarity bit into the net level instead found the
/// idle line asserted, latched remote IRR, and interrupted — and then again
/// after every end-of-interrupt, which is a processor that never retires
/// another instruction. `tests/pc_apic.rs` drives the whole lifecycle through
/// real guest instructions; this one pins the number to *this* board's input.
#[test]
fn the_entry_a_kernel_writes_for_this_boards_disk_is_quiet_while_the_disk_is() {
    /// The I/O APIC's register window, and the local APIC's page.
    const IOAPIC: u64 = 0xfec0_0000;
    const LAPIC: u64 = 0xfee0_0000;
    /// Which input the board's `pirq-routes` put device 4's `INTA#` on.
    const LINE: u64 = 11;
    /// The vector the 6.6 kernel gave it, measured.
    const VECTOR: u32 = 0x22;

    let machine = bare_board();
    let mem = machine.space("mem").expect("the board declares `mem`");
    let poke = |at: u64, value: u32| {
        mem.write(at, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .unwrap_or_else(|e| panic!("write of {at:#x} faulted: {e:?}"));
    };
    let indirect = |index: u32| {
        poke(IOAPIC, index);
        read32(mem, IOAPIC + 0x10)
    };

    // The high half first — physical destination, APIC 0 — then the low half,
    // which is the write that unmasks. That is the order a driver uses and the
    // order the entry is never briefly live in.
    poke(IOAPIC, 0x10 + 2 * LINE as u32 + 1);
    poke(IOAPIC + 0x10, 0);
    poke(IOAPIC, 0x10 + 2 * LINE as u32);
    poke(IOAPIC + 0x10, (1 << 15) | (1 << 13) | VECTOR);

    let irr = read32(mem, LAPIC + 0x200 + 0x10 * u64::from(VECTOR >> 5));
    assert_eq!(
        irr & (1 << (VECTOR & 31)),
        0,
        "unmasking the disk's redirection entry interrupted the processor \
         with the controller idle: nothing is driving I/O APIC input {LINE} \
         (local APIC IRR {irr:#010x})"
    );
    let entry = indirect(0x10 + 2 * LINE as u32);
    assert_eq!(entry & (1 << 14), 0, "and the entry latched no remote IRR");
    assert_eq!(
        entry & (1 << 13),
        1 << 13,
        "while the polarity bit the guest wrote still reads back"
    );
}

/// Where the guest left the two APICs, printed after a run.
///
/// The redirection table is the board's own record of what the kernel asked
/// the I/O APIC for — vector, trigger mode, polarity, mask and remote IRR —
/// and the local APIC's request, in-service and trigger-mode registers say
/// what became of it. Both are read with `MemAttrs::DEBUG` except the I/O
/// APIC's index register, which is half of a two-step protocol and has to be
/// moved to reach the window at all; it is put back afterwards, so a guest
/// caught mid-sequence sees nothing.
fn report_apics(m: &Machine) {
    /// The I/O APIC's register window and the local APIC's page, where this
    /// board's machine file maps them.
    const IOAPIC: u64 = 0xfec0_0000;
    const LAPIC: u64 = 0xfee0_0000;

    let mem = m.space("mem").expect("the board declares `mem`");
    let debug = |at: u64| {
        mem.read(at, Width::U32, MemAttrs::DEBUG)
            .map_or(0xffff_ffffu32, |v| v as u32)
    };
    let select = |index: u32| {
        mem.write(IOAPIC, Width::U32, u64::from(index), MemAttrs::DEFAULT)
            .expect("the index register is a dword");
    };
    let saved = debug(IOAPIC);
    select(1);
    let version = debug(IOAPIC + 0x10);
    let entries = ((version >> 16) & 0xff) + 1;
    println!("q35-linux: ioapic version={version:#010x}");
    for line in 0..entries {
        select(0x10 + 2 * line);
        let low = debug(IOAPIC + 0x10);
        select(0x10 + 2 * line + 1);
        let high = debug(IOAPIC + 0x10);
        if low == 0x0001_0000 && high == 0 {
            continue;
        }
        println!(
            "q35-linux:   irq{line:<2} {high:08x}_{low:08x} vector={:#04x} {} {} {}{}",
            low & 0xff,
            if low & (1 << 15) != 0 {
                "level"
            } else {
                "edge "
            },
            if low & (1 << 13) != 0 { "low " } else { "high" },
            if low & (1 << 16) != 0 {
                "masked"
            } else {
                "open"
            },
            if low & (1 << 14) != 0 {
                " remote-irr"
            } else {
                ""
            },
        );
    }
    select(saved);
    println!(
        "q35-linux: lapic id={:#010x} svr={:#010x} tpr={:#010x} ppr={:#010x}",
        debug(LAPIC + 0x20),
        debug(LAPIC + 0xf0),
        debug(LAPIC + 0x80),
        debug(LAPIC + 0xa0)
    );
    for (name, base) in [("isr", 0x100u64), ("tmr", 0x180), ("irr", 0x200)] {
        let words: Vec<String> = (0..8)
            .rev()
            .map(|i| format!("{:08x}", debug(LAPIC + base + 0x10 * i)))
            .collect();
        println!("q35-linux:   {name}={}", words.join("_"));
    }
}

// ---------------------------------------------------------------------------
// and the kernel
// ---------------------------------------------------------------------------

#[test]
fn a_linux_kernel_boots_and_finds_the_disk_on_the_q35_linux_board() {
    let Ok(path) = std::env::var("RSEMU_KERNEL") else {
        println!(
            "q35-linux: set RSEMU_KERNEL to a Linux/x86 bzImage to run one on this board; \
             see the module docs"
        );
        return;
    };
    let kernel = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let initrd = std::env::var("RSEMU_INITRD")
        .ok()
        .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
        .unwrap_or_default();
    // A raw image if one was named, and otherwise a blank namespace with this
    // file's own signature at LBA 0 — so that a run with nothing supplied
    // still has something the guest can read back and be believed about.
    let disk = match std::env::var("RSEMU_DISK") {
        Ok(p) => std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")),
        Err(_) => SIGNATURE.to_vec(),
    };
    println!(
        "q35-linux: {} bytes of kernel, {} bytes of initramfs, {} bytes into the namespace",
        kernel.len(),
        initrd.len(),
        disk.len()
    );

    let mut params: Vec<(&str, String)> = Vec::new();
    if let Ok(cmdline) = std::env::var("RSEMU_KERNEL_CMDLINE") {
        params.push(("cmdline", cmdline));
    }
    if let Ok(extmem) = std::env::var("RSEMU_KERNEL_EXTMEM") {
        params.push(("extmem", extmem));
    }
    let (mut m, cpu, console) = match board(kernel, initrd, disk, &params) {
        Ok(built) => built,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let ms: u64 = std::env::var("RSEMU_KERNEL_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_MS);

    let script = Script::from_env();
    println!("q35-linux: what the guest wrote to its serial port at 0x3f8:");
    let run = x86boot::run(
        &mut m,
        &cpu,
        &console,
        GlobalTime::from_nanos(ms * 1_000_000),
        &script,
    );
    x86boot::report("q35-linux", &m, &cpu, &run, &script);
    report_nvme(&m);
    report_apics(&m);
    x86boot::assert_booted(&run, &script);
}
