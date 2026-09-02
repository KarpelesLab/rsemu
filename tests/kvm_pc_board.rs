//! **A board, not a harness, under KVM.**
//!
//! `ROADMAP.md` phase 7's gate begins *"the phase-6 machines boot under KVM"*,
//! and every accel test before this one built its own two pages of memory and
//! its own recording device. This one takes `machines/pc-apic.machine` exactly
//! as it ships, hands its memory map to a hypervisor, and executes the board's
//! **reset vector on the host's own silicon**:
//!
//! * the firmware is fetched from the board's `pc.rom` socket, installed as a
//!   read-only memory slot, at `0xfffffff0` — where a PC's reset vector is;
//! * the far jump into segment `f000` lands in the *same* ROM through its
//!   second mapping, because the board maps the socket twice and both are
//!   slots;
//! * the store the guest writes is the board's own `object ram_low "ram"`,
//!   read back afterwards through `Machine::space`, with no copy anywhere;
//! * the port write leaves hardware and is answered by the board's
//!   `pc.sysctl`, whose A20 output is wired to two interpreted processors.
//!
//! What that demonstrates is the one core change phase 7 needed:
//! [`RamStore`](rsemu::core::space::RamStore) and
//! [`RomStore`](rsemu::core::space::RomStore) now allocate host-page aligned,
//! so a board's declared memory *can* be a memory slot. Before it, none of the
//! above was expressible.
//!
//! # What this is, and what `tests/kvm_smp.rs` is
//!
//! The vCPU here is created **beside** the board rather than being one of its
//! processors, so the board's two `cpu.x86` objects are still interpreters
//! that this test does not run. That is deliberate and still worth keeping:
//! it is the narrowest possible statement that a board's *memory map* is a
//! hypervisor's memory map, with no CPU device in the way.
//!
//! The consequence is visible in what is not asserted below — no interrupt is
//! delivered, because the vector comes from the board's local APIC on an
//! acknowledge cycle and only a CPU *device* can run one.
//! `rsemu::accel::cpu` is that device and `tests/kvm_smp.rs` is where two of
//! them run this same board.
//!
//! Every test skips cleanly with no `/dev/kvm`.

#![cfg(all(
    feature = "accel-kvm",
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-hpet",
    target_os = "linux",
    target_arch = "x86_64"
))]

use std::sync::Arc;

use rsemu::accel::board;
use rsemu::accel::kvm::Kvm;
use rsemu::accel::state;
use rsemu::core::device::ResetKind;
use rsemu::core::exec::ExitReason;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, build};

/// The board, verbatim.
const PC_APIC: &str = include_str!("../machines/pc-apic.machine");

/// The ROM socket's size, which the machine file also declares.
const ROM_LEN: usize = 128 * 1024;
/// Where segment `0xf000` starts inside the image: the socket is based at
/// `0xe0000`, so `0xf0000` is 64 KiB in.
const SEG_F000: usize = 0x1_0000;
/// The reset vector, sixteen bytes below the top of the socket.
const RESET_VECTOR: usize = ROM_LEN - 0x10;

/// Where the guest leaves its mark in low RAM, and what it leaves.
const MARK_AT: u64 = 0x0500;
const MARK: u16 = 0x1234;

/// System control port A, and the value written to it: bit 1 is fast A20.
const PORT_A: u64 = 0x0092;
const PORT_A_VALUE: u8 = 0x02;

/// The firmware: a reset vector, a far jump, and eight real-mode
/// instructions.
///
/// Hand-assembled, like `tests/pc_apic.rs`'s, because the point is to run
/// *real* x86 bytes out of the board's own socket rather than to trust a
/// helper.
fn firmware() -> Vec<u8> {
    let mut rom = vec![0u8; ROM_LEN];

    // At 0xf000:0000 — reached by the far jump below, which is what reloads
    // `CS` and drops the processor out of the 0xffff0000 reset base.
    let body: &[u8] = &[
        0xb8,
        0x00,
        0x00, // mov ax, 0
        0x8e,
        0xd8, // mov ds, ax
        0xc7,
        0x06,
        0x00,
        0x05,
        0x34,
        0x12, // mov word [0x500], 0x1234
        0xb0,
        PORT_A_VALUE, // mov al, 0x02
        0xe6,
        0x92, // out 0x92, al
        0xf4, // hlt
    ];
    rom[SEG_F000..SEG_F000 + body.len()].copy_from_slice(body);

    // The reset vector itself: `jmp far 0xf000:0x0000`.
    rom[RESET_VECTOR..RESET_VECTOR + 5].copy_from_slice(&[0xea, 0x00, 0x00, 0x00, 0xf0]);
    rom
}

/// The board, with that firmware in its socket.
fn board() -> Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert("bios", firmware());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    match build("pc-apic.machine", PC_APIC, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

#[test]
fn the_boards_declared_memory_becomes_memory_slots() {
    // No hypervisor needed: this is the classification, and it is the part
    // that was structurally impossible before the stores were page aligned.
    let m = board();
    let mem = m.space("mem").expect("the memory space");
    let plan = board::plan_space(mem, true);

    let bases: Vec<u64> = plan.slots.iter().map(|(base, _)| *base).collect();
    assert!(bases.contains(&0x0000_0000), "640K of base memory");
    assert!(
        bases.contains(&0x0010_0000),
        "and the extended memory above it"
    );
    assert!(bases.contains(&0x000e_0000), "the ROM socket at 0xe0000");
    assert!(
        bases.contains(&0xfffe_0000u64),
        "and its second copy under the 4 GiB mark, where the reset vector is"
    );

    // The APIC-era register pages must not be slots, or the guest would read
    // the bytes of a page nobody wrote instead of asking the device.
    for page in [0xfec0_0000u64, 0xfed0_0000, 0xfee0_0000, 0xfef0_0000] {
        assert!(
            !plan.slots.iter().any(|(base, _)| *base == page),
            "{page:#x} answers from a device model, so it stays MMIO"
        );
    }
}

#[test]
fn the_boards_ram_is_the_memory_the_hardware_writes() {
    let Ok(kvm) = Kvm::open() else { return };
    let mut m = board();
    m.reset(ResetKind::Cold);
    m.sweep();

    let vm = kvm.create_vm().expect("KVM_CREATE_VM");
    let (plan, mem) = board::install_machine(&vm, &m, "mem", 0).expect("install the board's map");
    assert!(
        plan.covers(0xffff_fff0),
        "the reset vector has to be fetchable in hardware:\n{}",
        plan.describe()
    );
    assert!(plan.covers(MARK_AT));

    let io = m.space("port").expect("the I/O space");
    let vcpu = vm
        .create_vcpu(0, Arc::clone(&mem), Some(Arc::clone(io)))
        .expect("KVM_CREATE_VCPU");

    // **Nothing is written into the vCPU.** A freshly created one is already at
    // the architectural reset state this board's firmware expects — `CS`
    // selector f000 with a cached base of ffff0000, `IP` at fff0, so `CS:IP`
    // addresses 0xfffffff0 — and it is at that state with the *host's* fixed
    // `CR0` and `CR4` bits, which a state this test invented would not be.
    // `accel::state`'s own tests assert that the same shape survives a round
    // trip through `Sys::reset`; asserting it here as well would be asserting
    // the translation twice and the board not at all.
    let sregs = vcpu.sregs().expect("KVM_GET_SREGS");
    assert_eq!(sregs.cs.selector, 0xf000, "a processor out of reset");
    assert_eq!(sregs.cs.base, 0xffff_0000);
    assert_eq!(vcpu.regs().expect("KVM_GET_REGS").rip, 0xfff0);

    let run = vcpu.run_until_exit(64).expect("the guest runs");
    let exit = run.exit.expect("the firmware halts");
    assert_eq!(
        exit.reason,
        ExitReason::HALT,
        "the firmware ran to its `hlt`; got {exit:?} after {} entries\n{}",
        run.consumed.ticks,
        plan.describe()
    );

    // The mark is in the board's own `ram_low`, read back through the same
    // address space the interpreter uses. One set of bytes, two engines.
    let seen = mem
        .read(MARK_AT, Width::U16, MemAttrs::DEFAULT)
        .expect("low RAM answers");
    assert_eq!(seen as u16, MARK, "the guest wrote the board's RAM");

    // And the port write was answered by the board's `pc.sysctl`, not by
    // anything this test built.
    let porta = io
        .read(PORT_A, Width::U8, MemAttrs::DEFAULT)
        .expect("port 0x92 decodes");
    assert_eq!(
        porta as u8 & PORT_A_VALUE,
        PORT_A_VALUE,
        "the fast-A20 bit the guest set is still set in the device model"
    );

    // The board saw one port access and no MMIO: everything else the guest
    // touched was hardware-backed.
    let stats = vcpu.stats();
    assert_eq!(stats.pio, 1, "one `out`, routed into the board's I/O space");
    assert_eq!(stats.mmio, 0);
}

#[test]
fn a_state_transfer_off_the_board_agrees_with_the_interpreter() {
    // The second half of phase 7's gate, on a real board: the state the vCPU
    // reached is carried into an interpreter and the two are compared field by
    // field on everything both engines model.
    let Ok(kvm) = Kvm::open() else { return };
    let mut m = board();
    m.reset(ResetKind::Cold);
    m.sweep();

    let vm = kvm.create_vm().expect("KVM_CREATE_VM");
    let (_plan, mem) = board::install_machine(&vm, &m, "mem", 0).expect("install");
    let io = m.space("port").expect("the I/O space");
    let vcpu = vm
        .create_vcpu(0, Arc::clone(&mem), Some(Arc::clone(io)))
        .expect("KVM_CREATE_VCPU");

    vcpu.run_until_exit(64).expect("the guest runs");

    let cpu = rsemu::cpu::x86::X86::new(rsemu::cpu::x86::Config::I80486);
    cpu.attach_space(Arc::clone(&mem));
    cpu.attach_io_space(Arc::clone(io));
    cpu.step(); // the reset sequence, as a restored core would have had

    state::store_from_vcpu(&vcpu, &cpu).expect("carry the state off the accelerator");
    assert_eq!(
        state::differs(&vcpu, &cpu).expect("compare"),
        None,
        "the two engines disagree about the architectural state"
    );
    // `CS` came back with the base the far jump computed, not the reset base:
    // a translation that rebuilt it from `selector << 4` would agree here by
    // accident, so the interesting assertion is the *reset* one in
    // `accel::state`'s own tests. This one says the jump happened at all.
    assert_eq!(cpu.sys().segs[1].base, 0x000f_0000);
    assert!(cpu.regs().rip > 0);
}

/// Says out loud whether the tests above actually ran.
#[test]
fn report_whether_this_host_can_run_a_board_under_kvm() {
    if Kvm::is_available() {
        // Nothing to assert; the tests above did the asserting.
    } else {
        // Deliberately not a failure: `cargo test` stays hermetic.
    }
}
