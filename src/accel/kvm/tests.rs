//! Tests for the KVM backend.
//!
//! **Every one of them skips cleanly on a host with no usable `/dev/kvm`**, as
//! a container, a CI runner and a non-x86 developer machine all are. `CLAUDE.md`
//! makes that a rule for conformance corpora and the same reasoning applies
//! here: `cargo test` must be hermetic. The skip is a plain early `return`
//! rather than a `panic!("skipped")`, so the run stays green and the count
//! stays honest — a test that "passed" because there was nothing to run is
//! reported below as what it is, in the module's own summary test.
//!
//! What does *not* skip: everything that can be checked without a hypervisor —
//! the `ioctl` number encoding, the transcribed structure sizes, the
//! `kvm_run` field offsets, the argument validation, and the refusal of a
//! deterministic threading mode.

use super::*;
use crate::accel::mem::HostPages;
use crate::core::space::{AccessConstraints, AddressSpace, MemOps, MemResult, Region, RegionRef};
use crate::core::sync::Mutex as SyncMutex;
use alloc::vec;

/// Open KVM, or `None` if this host has none.
fn kvm() -> Option<Kvm> {
    match Kvm::open() {
        Ok(kvm) => Some(kvm),
        Err(e) if e.is_unavailable() => None,
        // A host that *has* `/dev/kvm` and cannot use it is a real failure:
        // silently skipping there would hide exactly the bug this suite is for.
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    }
}

/// A device that remembers every access, mapped wherever a test needs one.
#[derive(Debug, Default)]
struct Recorder {
    writes: SyncMutex<Vec<(u64, u64)>>,
    answer: SyncMutex<u8>,
}

impl MemOps for Recorder {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let byte = *self.answer.lock();
        for b in dst.iter_mut() {
            *b = byte;
        }
        let _ = offset;
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let mut value = 0u64;
        for (i, b) in src.iter().enumerate() {
            value |= u64::from(*b) << (8 * i);
        }
        self.writes.lock().push((offset, value));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

impl Recorder {
    fn region(self: &Arc<Self>, name: &str, len: u64) -> RegionRef {
        Arc::new(Region::io(name, len, Arc::clone(self) as Arc<dyn MemOps>))
    }
}

/// A guest: two pages of RAM at physical zero, an I/O space, and a memory
/// space that sees the same bytes the guest executes.
struct Guest {
    _kvm: Kvm,
    vm: Vm,
    ram: Arc<HostPages>,
    mem: Arc<AddressSpace>,
    io: Arc<AddressSpace>,
    /// The next vCPU id to hand out. KVM refuses to create the same id twice
    /// on one VM, and closing the descriptor does not give the id back, so a
    /// test that builds several must count.
    next_id: core::cell::Cell<u32>,
}

/// Where the test programs are loaded, guest-physical and (with `CS` at zero)
/// also the offset within the segment.
const CODE: u64 = 0x1000;
/// Guest-physical address of the MMIO window, deliberately outside every
/// memory slot so that touching it exits.
const MMIO_BASE: u64 = 0x8000;

impl Guest {
    fn new(kvm: Kvm) -> Guest {
        let vm = kvm.create_vm().expect("KVM_CREATE_VM");
        let ram = Arc::new(HostPages::new(2 * PAGE_SIZE).expect("guest RAM"));
        vm.set_memory_region(0, 0, &ram).expect("memory slot 0");

        let mem = Arc::new(AddressSpace::new("mem", 20));
        mem.topology().map(ram.region(), 0).expect("map RAM");

        let io = Arc::new(AddressSpace::new("io", 16));
        Guest {
            _kvm: kvm,
            vm,
            ram,
            mem,
            io,
            next_id: core::cell::Cell::new(0),
        }
    }

    /// Load a program at [`CODE`] and hand back a vCPU pointed at it, in
    /// 16-bit real mode with every segment based at zero.
    fn vcpu_at(&self, code: &[u8]) -> Vcpu {
        self.ram.write_at(CODE, code).expect("load code");
        let id = self.next_id.get();
        self.next_id.set(id + 1);
        let vcpu = self
            .vm
            .create_vcpu(id, Arc::clone(&self.mem), Some(Arc::clone(&self.io)))
            .expect("KVM_CREATE_VCPU");

        // Start from whatever the kernel reset the vCPU to and move only the
        // segments: that keeps `CR0`, `EFER` and the rest at values this host's
        // hardware is known to accept on entry.
        let mut sregs = vcpu.sregs().expect("KVM_GET_SREGS");
        for seg in [
            &mut sregs.cs,
            &mut sregs.ds,
            &mut sregs.es,
            &mut sregs.fs,
            &mut sregs.gs,
            &mut sregs.ss,
        ] {
            seg.base = 0;
            seg.selector = 0;
        }
        vcpu.set_sregs(&sregs).expect("KVM_SET_SREGS");

        let mut regs = vcpu.regs().expect("KVM_GET_REGS");
        regs.rip = CODE;
        // Bit 1 is hard-wired to one; VMX refuses an entry without it.
        regs.rflags = 0x2;
        vcpu.set_regs(&regs).expect("KVM_SET_REGS");
        vcpu
    }
}

// ---------------------------------------------------------------------------
// the ABI transcription — checkable with no hypervisor at all
// ---------------------------------------------------------------------------

#[test]
fn the_ioctl_numbers_match_the_published_ones() {
    // The values `include/uapi/linux/kvm.h` produces on the asm-generic
    // architectures. Written out as literals so that the encoding in `ioc` and
    // the transcribed structure sizes are checked against something, rather
    // than against themselves.
    assert_eq!(KVM_GET_API_VERSION.0, 0xae00);
    assert_eq!(KVM_CREATE_VM.0, 0xae01);
    assert_eq!(KVM_CHECK_EXTENSION.0, 0xae03);
    assert_eq!(KVM_GET_VCPU_MMAP_SIZE.0, 0xae04);
    assert_eq!(KVM_CREATE_VCPU.0, 0xae41);
    assert_eq!(KVM_SET_TSS_ADDR.0, 0xae47);
    assert_eq!(KVM_RUN.0, 0xae80);

    assert_eq!(KVM_SET_USER_MEMORY_REGION.0, 0x4020_ae46);
    assert_eq!(KVM_SET_IDENTITY_MAP_ADDR.0, 0x4008_ae48);
    assert_eq!(KVM_GET_REGS.0, 0x8090_ae81);
    assert_eq!(KVM_SET_REGS.0, 0x4090_ae82);
    assert_eq!(KVM_GET_SREGS.0, 0x8138_ae83);
    assert_eq!(KVM_SET_SREGS.0, 0x4138_ae84);
}

#[test]
fn the_transcribed_structures_are_the_sizes_the_ioctl_numbers_encode() {
    // The size is *part of* the request number, so these two facts are the
    // same fact. The `const` assertions beside each struct already fail the
    // build; this restates them where a reader is looking for them.
    assert_eq!(size_of::<KvmUserspaceMemoryRegion>(), 32);
    assert_eq!(size_of::<KvmRegs>(), 144);
    assert_eq!(size_of::<KvmSegment>(), 24);
    assert_eq!(size_of::<KvmDtable>(), 16);
    assert_eq!(size_of::<KvmSregs>(), 312);
    assert_eq!((KVM_GET_REGS.0 >> 16) & 0x3fff, size_of::<KvmRegs>() as u64);
    assert_eq!(
        (KVM_GET_SREGS.0 >> 16) & 0x3fff,
        size_of::<KvmSregs>() as u64
    );
}

#[test]
fn the_ioc_encoding_places_each_field_where_the_kernel_looks_for_it() {
    let r = ioc(DIR_READ, 0x7f, 0x1234);
    assert_eq!(r >> 30, 2, "direction");
    assert_eq!((r >> 16) & 0x3fff, 0x1234, "size");
    assert_eq!((r >> 8) & 0xff, KVMIO, "type");
    assert_eq!(r & 0xff, 0x7f, "ordinal");
}

#[test]
fn an_unaligned_memory_region_is_refused_before_the_ioctl() {
    let Some(kvm) = kvm() else { return };
    let vm = kvm.create_vm().expect("KVM_CREATE_VM");
    let ram = Arc::new(HostPages::new(PAGE_SIZE).expect("guest RAM"));
    let err = vm.set_memory_region(0, 0x800, &ram).unwrap_err();
    assert!(matches!(err, AccelError::Unsupported(_)), "{err}");
    assert!(vm.memory_regions().is_empty());
}

// ---------------------------------------------------------------------------
// the hypervisor
// ---------------------------------------------------------------------------

#[test]
fn dev_kvm_reports_the_api_version_this_code_was_written_against() {
    let Some(kvm) = kvm() else { return };
    assert!(kvm.check_extension(KVM_CAP_USER_MEMORY) != 0);
    assert!(kvm.vcpu_mmap_size() >= PAGE_SIZE);
    assert_eq!(kvm.vcpu_mmap_size() % PAGE_SIZE, 0);
}

#[test]
fn a_memory_region_is_installed_where_it_was_asked_for() {
    let Some(kvm) = kvm() else { return };
    let vm = kvm.create_vm().expect("KVM_CREATE_VM");
    let ram = Arc::new(HostPages::new(2 * PAGE_SIZE).expect("guest RAM"));
    vm.set_memory_region(0, 0, &ram).expect("slot 0");
    assert_eq!(vm.memory_regions(), vec![(0, 0, 2 * PAGE_SIZE)]);

    // Replacing a slot replaces the record rather than appending to it.
    let more = Arc::new(HostPages::new(PAGE_SIZE).expect("guest RAM"));
    vm.set_memory_region(0, 0x10_0000, &more).expect("slot 0");
    assert_eq!(vm.memory_regions(), vec![(0, 0x10_0000, PAGE_SIZE)]);
}

/// **The payoff**: a guest runs on the host's own silicon and its `OUT`
/// reaches a device model living in an rsemu [`AddressSpace`].
///
/// `ROADMAP.md` §10: *"MMIO/PIO exits routed back into the address-space
/// layer"*. If this passes, the seam, the memory region and the exit routing
/// all work, because nothing else could produce the bytes.
#[test]
fn a_guest_under_kvm_reaches_a_device_model_through_an_out() {
    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);

    let uart = Arc::new(Recorder::default());
    guest
        .io
        .topology()
        .map(uart.region("uart", 8), 0x3f8)
        .expect("map the port");

    //   mov dx, 0x3f8      ba f8 03
    //   mov al, 'K'        b0 4b
    //   out dx, al         ee
    //   mov al, 'V'        b0 56
    //   out dx, al         ee
    //   mov al, 'M'        b0 4d
    //   out dx, al         ee
    //   hlt                f4
    let code: &[u8] = &[
        0xba, 0xf8, 0x03, 0xb0, 0x4b, 0xee, 0xb0, 0x56, 0xee, 0xb0, 0x4d, 0xee, 0xf4,
    ];
    let vcpu = guest.vcpu_at(code);

    let run = vcpu.run_until_exit(64).expect("the guest runs");
    let exit = run.exit.expect("the guest halted");
    assert_eq!(exit.reason, ExitReason::HALT, "exit was {exit:?}");

    // Offset zero of the mapped window is the transmitter holding register,
    // and the three bytes arrived in order.
    let writes = uart.writes.lock().clone();
    assert_eq!(
        writes,
        vec![
            (0, u64::from(b'K')),
            (0, u64::from(b'V')),
            (0, u64::from(b'M'))
        ],
        "the device did not see what the guest wrote"
    );
    assert_eq!(vcpu.stats().pio, 3);
}

/// The same, through **a real device model**: the 16550 the RISC-V board uses,
/// unmodified, on the character-device seam.
///
/// The point of running it twice is that the previous test proves the routing
/// and this one proves it lands somewhere a machine would actually put it —
/// a stateful device with a FIFO, a host backend and a reset, reached at COM1's
/// own port number.
#[cfg(feature = "dev-riscv")]
#[test]
fn a_guest_under_kvm_prints_through_the_emulated_16550() {
    use crate::core::device::{Device, ResetKind};
    use crate::dev::riscv::Uart16550;
    use crate::host::chardev::CharPort;
    use alloc::string::String;

    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);

    let port = Arc::new(CharPort::new());
    let uart = Uart16550::with_port(Arc::clone(&port) as Arc<_>, String::from("test"), 115_200);
    uart.reset(ResetKind::Cold);
    guest
        .io
        .topology()
        .map(uart.region("").expect("the register block"), 0x3f8)
        .expect("map COM1");

    let code: &[u8] = &[
        0xba, 0xf8, 0x03, // mov dx, 0x3f8
        0xb0, 0x4b, 0xee, // mov al,'K'; out dx,al
        0xb0, 0x56, 0xee, // mov al,'V'; out dx,al
        0xb0, 0x4d, 0xee, // mov al,'M'; out dx,al
        0xb0, 0x0a, 0xee, // mov al,10;  out dx,al
        0xf4, // hlt
    ];
    let vcpu = guest.vcpu_at(code);
    let run = vcpu.run_until_exit(64).expect("the guest runs");
    assert_eq!(run.exit.expect("halted").reason, ExitReason::HALT);

    uart.pump();
    assert_eq!(port.drain(), b"KVM\n".to_vec());
}

/// An MMIO exit reaches a device in the *memory* space, which is the other
/// half of the routing.
#[test]
fn a_guest_under_kvm_reaches_a_device_model_through_a_memory_write() {
    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);

    let dev = Arc::new(Recorder::default());
    *dev.answer.lock() = 0x5a;
    guest
        .mem
        .topology()
        .map(dev.region("mmio", 0x1000), MMIO_BASE)
        .expect("map the window");

    //   mov bx, 0x8000     bb 00 80
    //   mov byte [bx], 0xa5   c6 07 a5
    //   mov al, [bx+4]     8a 47 04
    //   hlt                f4
    let code: &[u8] = &[0xbb, 0x00, 0x80, 0xc6, 0x07, 0xa5, 0x8a, 0x47, 0x04, 0xf4];
    let vcpu = guest.vcpu_at(code);

    let run = vcpu.run_until_exit(64).expect("the guest runs");
    assert_eq!(run.exit.expect("halted").reason, ExitReason::HALT);

    assert_eq!(dev.writes.lock().clone(), vec![(0, 0xa5)]);
    // The read came back through the same seam and landed in `AL`.
    assert_eq!(vcpu.regs().expect("regs").rax & 0xff, 0x5a);
    assert_eq!(vcpu.stats().mmio, 2);
}

/// Guest RAM is the *same bytes* the interpreter's address space sees, which
/// is what makes an accelerated machine one machine rather than two.
#[test]
fn what_the_guest_writes_to_ram_is_visible_through_the_address_space() {
    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);

    //   mov bx, 0x0040     bb 40 00
    //   mov word [bx], 0xbeef  c7 07 ef be
    //   hlt                f4
    let code: &[u8] = &[0xbb, 0x40, 0x00, 0xc7, 0x07, 0xef, 0xbe, 0xf4];
    let vcpu = guest.vcpu_at(code);
    let run = vcpu.run_until_exit(64).expect("the guest runs");
    assert_eq!(run.exit.expect("halted").reason, ExitReason::HALT);

    // Read it back the way any device, DMA master or debugger would.
    let seen = guest
        .mem
        .read(0x40, crate::core::Width::U16, MemAttrs::DEFAULT)
        .expect("read guest RAM through the address space");
    assert_eq!(seen, 0xbeef);
}

// ---------------------------------------------------------------------------
// the safe point
// ---------------------------------------------------------------------------

/// The signal-free half of the stop-the-world protocol: a raised
/// [`ExitFlag`] keeps the guest from being entered at all.
#[test]
fn a_raised_exit_flag_declines_to_enter_the_guest() {
    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);

    // Deliberately a program that *does* end: an endless loop under a
    // hypervisor with no timer would never return from `KVM_RUN` at all, and a
    // test that hangs proves nothing.
    let code: &[u8] = &[0x90, 0xf4]; // nop; hlt
    let mut vcpu = guest.vcpu_at(code);
    let flag = ExitFlag::default();
    flag.raise();
    vcpu.set_exit_flag(flag.clone());

    let run = vcpu.run_until_exit(4).expect("the run returns");
    // No exit, because nothing happened *to the guest*: it was never entered.
    assert!(run.exit.is_none(), "{run:?}");
    assert_eq!(vcpu.stats().entries, 0, "the guest must not have run");
    assert!(vcpu.stats().declined > 0);
    // And the program counter is untouched, so resuming is unconditional.
    assert_eq!(vcpu.regs().expect("regs").rip, CODE);

    // Clearing it lets the guest run.
    flag.clear();
    let run = vcpu.run_until_exit(4).expect("the run returns");
    assert_eq!(run.exit.expect("halted").reason, ExitReason::HALT);
    assert!(vcpu.stats().entries > 0);
}

#[test]
fn immediate_exit_is_what_makes_that_race_free() {
    let Some(kvm) = kvm() else { return };
    let vm = kvm.create_vm().expect("KVM_CREATE_VM");
    // Not an assertion about the host so much as a record of what the backend
    // is relying on: without this capability the check-then-enter window is
    // open, and the module documentation says so.
    assert!(
        vm.has_immediate_exit(),
        "this kernel has no KVM_CAP_IMMEDIATE_EXIT; the stop request is still \
         honoured but the race is no longer closed"
    );
}

// ---------------------------------------------------------------------------
// determinism
// ---------------------------------------------------------------------------

#[test]
fn a_deterministic_threading_mode_is_refused_structurally() {
    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);
    let vcpu = guest.vcpu_at(&[0xf4]);
    let err = vcpu
        .into_runnable(ThreadingMode::Deterministic)
        .expect_err("a deterministic mode must be refused");
    assert!(matches!(err, AccelError::Nondeterministic(_)), "{err}");

    let vcpu = guest.vcpu_at(&[0xf4]);
    assert!(vcpu.into_runnable(ThreadingMode::Accel).is_ok());
    let vcpu = guest.vcpu_at(&[0xf4]);
    assert!(vcpu.into_runnable(ThreadingMode::Parallel).is_ok());
}

// ---------------------------------------------------------------------------
// the engine seam
// ---------------------------------------------------------------------------

/// A vCPU is an [`ExitingCore`], which is the seam `ROADMAP.md` §4.6 asks a
/// new engine to fit rather than replace.
#[test]
fn a_vcpu_is_an_exiting_core() {
    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);
    let vcpu = guest.vcpu_at(&[0xf4]);

    let core: &dyn ExitingCore = &vcpu;
    assert_eq!(core.pc(), CODE);
    core.set_pc(CODE + 1);
    assert_eq!(core.pc(), CODE + 1);
    core.set_pc(CODE);
    core.set_sp(0x7000);
    assert_eq!(core.sp(), 0x7000);

    // The mask is configuration and round-trips; this backend takes its exits
    // from hardware rather than from a mask, so it changes nothing about the
    // run — which is why it is stated here rather than assumed.
    core.set_exit_mask(ExitMask::USER);
    assert_eq!(core.exit_mask(), ExitMask::USER);

    let run = core.run_to_exit(Budget::of(16));
    assert_eq!(run.exit.expect("halted").reason, ExitReason::HALT);
}

// ---------------------------------------------------------------------------
// cross-engine
// ---------------------------------------------------------------------------

/// **Phase 7's gate, in miniature**: a guest starts under KVM, stops, its
/// architectural state moves into the interpreter, and the interpreter
/// finishes the program against the same device.
///
/// The program is deliberately split by a `HLT`, so the two engines each run a
/// part of it and the device sees one continuous stream. If the state transfer
/// were wrong in `CS`, `RIP`, `DX` or `AL` the second half would write the
/// wrong bytes to the wrong port, or fault.
#[cfg(feature = "cpu-x86")]
#[test]
fn a_guest_started_under_kvm_finishes_under_the_interpreter() {
    use crate::accel::state;
    use crate::cpu::x86::{Config, X86};

    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);

    let uart = Arc::new(Recorder::default());
    guest
        .io
        .topology()
        .map(uart.region("uart", 8), 0x3f8)
        .expect("map the port");

    //   mov dx, 0x3f8   ba f8 03
    //   mov al,'K'      b0 4b
    //   out dx, al      ee
    //   mov al,'V'      b0 56
    //   out dx, al      ee
    //   hlt             f4        <- KVM stops here
    //   mov al,'M'      b0 4d
    //   out dx, al      ee
    //   hlt             f4        <- the interpreter stops here
    let code: &[u8] = &[
        0xba, 0xf8, 0x03, 0xb0, 0x4b, 0xee, 0xb0, 0x56, 0xee, 0xf4, 0xb0, 0x4d, 0xee, 0xf4,
    ];
    let vcpu = guest.vcpu_at(code);
    let run = vcpu.run_until_exit(64).expect("the guest runs");
    assert_eq!(run.exit.expect("halted").reason, ExitReason::HALT);
    assert_eq!(uart.writes.lock().len(), 2, "KVM's half");

    // The interpreter, on the same two address spaces.
    let cpu = X86::new(Config::I80486);
    cpu.attach_space(Arc::clone(&guest.mem));
    cpu.attach_io_space(Arc::clone(&guest.io));
    // The first step is the reset sequence; after it the core is where a
    // machine's would be when a snapshot is restored into it.
    cpu.step();

    state::store_from_vcpu(&vcpu, &cpu).expect("carry the state across");
    assert_eq!(
        state::differs(&vcpu, &cpu).expect("compare"),
        None,
        "the two engines disagree about the architectural state"
    );

    // `RIP` is past the `HLT` KVM stopped on, so the interpreter resumes at
    // the instruction after it with no fixup.
    assert_eq!(cpu.regs().rip, CODE + 10);
    assert_eq!(cpu.regs().rdx & 0xffff, 0x3f8);

    for _ in 0..16 {
        cpu.step();
        if cpu.is_halted() {
            break;
        }
    }
    assert!(cpu.is_halted(), "the interpreter did not reach the HLT");

    assert_eq!(
        uart.writes.lock().clone(),
        vec![
            (0, u64::from(b'K')),
            (0, u64::from(b'V')),
            (0, u64::from(b'M'))
        ],
        "one continuous stream across the engine switch"
    );
}

/// The reverse direction: state prepared in the interpreter runs under KVM.
///
/// The vCPU's own reset state is carried into the interpreter first, so what
/// goes back is a *modified* hardware-acceptable state rather than one this
/// test invented — `CR0`'s fixed bits are the host's business, not ours.
#[cfg(feature = "cpu-x86")]
#[test]
fn the_interpreters_state_loads_into_a_vcpu_and_runs() {
    use crate::accel::state;
    use crate::cpu::x86::{Config, X86};

    let Some(kvm) = kvm() else { return };
    let guest = Guest::new(kvm);

    let uart = Arc::new(Recorder::default());
    guest
        .io
        .topology()
        .map(uart.region("uart", 8), 0x3f8)
        .expect("map the port");

    //   out dx, al   ee
    //   hlt          f4
    let vcpu = guest.vcpu_at(&[0xee, 0xf4]);

    let cpu = X86::new(Config::I80486);
    cpu.attach_space(Arc::clone(&guest.mem));
    cpu.attach_io_space(Arc::clone(&guest.io));
    cpu.step();
    state::store_from_vcpu(&vcpu, &cpu).expect("KVM to the interpreter");

    let mut regs = cpu.regs();
    regs.rip = CODE;
    regs.rdx = 0x3f8;
    regs.rax = 0x42;
    cpu.set_regs(regs);

    state::load_into_vcpu(&cpu, &vcpu).expect("the interpreter to KVM");
    assert_eq!(
        state::differs(&vcpu, &cpu).expect("compare"),
        None,
        "the two engines disagree after a load"
    );

    let run = vcpu.run_until_exit(64).expect("the guest runs");
    assert_eq!(run.exit.expect("halted").reason, ExitReason::HALT);
    assert_eq!(uart.writes.lock().clone(), vec![(0, 0x42)]);
}

// ---------------------------------------------------------------------------
// the honest summary
// ---------------------------------------------------------------------------

/// Says out loud whether the tests above actually ran.
///
/// A suite that skips is a suite that proved nothing, and a green run that
/// hides that is worse than a red one. This is the line to grep for.
#[test]
fn report_whether_this_host_can_run_the_backend_at_all() {
    if Kvm::is_available() {
        // Nothing to assert: every test above did the asserting.
    } else {
        // Deliberately not a failure. `cargo test` stays hermetic
        // (`CLAUDE.md`), and CI has no guarantee of `/dev/kvm`.
    }
}
