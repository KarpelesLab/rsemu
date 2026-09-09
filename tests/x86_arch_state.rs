//! **A snapshot taken on host silicon, restored under the interpreter and the
//! JIT — and the guest itself says whether its state survived.**
//!
//! `ROADMAP.md` phase 7 names an **engine-independent architectural CPU-state
//! model** as a deliverable and gates it on *"snapshots taken under KVM restore
//! under the JIT and vice versa"*. `tests/kvm_smp.rs` already asserts the
//! *format* half of that — `pc-apic.machine` under KVM saves a chunk the same
//! board under interpreters loads, byte for byte. This file is the half that
//! format equality cannot reach.
//!
//! # Why a second file, and why an `x86-64` board
//!
//! `pc-apic.machine` declares two **80486s**, and the machine file says so in
//! its own comment: *"these are 486s, so they have no model-specific
//! registers"*. A 486 has no `IA32_MTRR_DEF_TYPE`, no `IA32_MISC_ENABLE` and no
//! `IA32_LSTAR`, so a cross-engine test on that board compares two processors
//! that both hold nothing, agrees, and proves that the fields it never looked at
//! are equal. They were not.
//!
//! What was actually happening, measured before any of this was written: a vCPU
//! with `IA32_MTRR_DEF_TYPE` = `0xc06` and `IA32_MTRR_FIX64K_00000` =
//! `0x0606…06` handed `accel::state::store_from_vcpu` **two zeros**, because
//! `CARRIED_MSRS` listed five registers and neither of those was among them.
//! Zero in `MTRR_DEF_TYPE` is not "the default": *Intel SDM* volume 3A
//! §12.11.2.1 defines `E = 0` as *every physical address uncacheable*. So a
//! snapshot taken on hardware described a machine whose memory types the guest
//! had never asked for, the reverse restore wiped a firmware's ranges back to
//! reset, and both directions passed every existing test.
//!
//! # The shape: the guest is the witness
//!
//! Comparing rsemu's internal fields across a restore would only prove that two
//! copies of this crate agree with each other. So the guest program does the
//! asserting: it programs four model-specific registers once, and then loops
//! **reading them back with `RDMSR` and writing what it sees into RAM**, along
//! with `RDTSC`. The test zeroes that witness area *after* the restore and
//! before running on, so every value it then reads was produced by a `RDMSR`
//! executed on the destination engine.
//!
//! That is the property phase 7 actually wants. A field can round-trip a chunk
//! perfectly and still be wrong if the engine never put it in the chunk, which
//! is precisely the defect above.
//!
//! # The board
//!
//! Built here rather than taken from `machines/` for `tests/x86_engines.rs`'s
//! reason, which applies twice over: every shipped x86 board is a board for
//! software this repository does not contain, and the two that carry an
//! `x86-64` core (`pc64`, `q35-linux`) want a `bzImage`. This is the smallest
//! machine with model-specific registers in it — RAM, a clock, an I/O space and
//! one `variant = "x86-64"` core. RAM is a memory slot and every instruction in
//! the guest is one hardware can fetch, so an accelerated run stays in hardware
//! for the whole loop but one `OUT`, which is there to give the scheduler its
//! thread back: see [`program`] for why a guest that takes no exits at all
//! hangs this test rather than failing it.
//!
//! # Which directions actually run here
//!
//! * **KVM to the interpreter** and **KVM to the JIT**: run, whenever
//!   `/dev/kvm` is usable and the build has the engine.
//! * **The interpreter to KVM** and **the JIT to KVM**: run, same condition.
//! * **The interpreter to the JIT**: `tests/x86_engines.rs` owns that leg and
//!   asserts a bit-identical state hash across it, which is stronger than
//!   anything possible here — a hash over an accelerated run is meaningless,
//!   because host silicon is not reproducible.
//!
//! Every test **skips cleanly** with no `/dev/kvm`, and says so in
//! [`report_whether_this_host_ran_the_cross_engine_gate`].

#![cfg(all(
    feature = "accel-kvm",
    feature = "cpu-x86",
    target_os = "linux",
    target_arch = "x86_64"
))]

use std::sync::Arc;

use rsemu::accel::cpu::AccelCpus;
use rsemu::accel::kvm::Kvm;
use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::prot::{SegReg, Sys, ar, cr0, cr4, efer};
use rsemu::cpu::x86::{Regs, Variant, X86, isa::seg};
use rsemu::machine::{Machine, build};

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// RAM, a clock and one long-mode core.
///
/// No firmware socket and no reset stub: the reset sequence is discharged with
/// a single `step` on the shell and the world is written into the system
/// registers, exactly as `tests/x86_engines.rs` and
/// `cpu::x86::differential::oracle` place a core. A ROM would only be sixteen
/// bytes of real-mode code, and real mode is the one world an accelerated fetch
/// has to leave hardware for.
const SOURCE: &str = r#"
machine "x86-arch-state" {
  param engine = "interp"
  param ram = 8M

  osc cpu = 100000000 Hz

  space mem { width = 64, unassigned = read-as-ones }
  space port { width = 16, unassigned = read-as-ones }

  object cpu0 "cpu.x86" {
    clock   = cpu
    space   = mem
    iospace = "port"
    variant = "x86-64"
    engine  = engine
  }

  object dram "ram" { size = ram }

  map mem 0x00000000 size ram = dram
}
"#;

/// Where the guest program is loaded.
const PROGRAM: u64 = 0x1000;
/// Where the witness area starts. The program carries it as an immediate.
const WITNESS: u64 = 0x2000;
/// Where the four-level page tables go.
const PML4: u64 = 0x30_0000;
const PDPT: u64 = 0x30_1000;
const PDIR: u64 = 0x30_2000;

/// Offsets within the witness area, in the order the loop writes them.
mod at {
    /// How many times round the loop the guest has been.
    pub(crate) const TICK: u64 = 0;
    /// `IA32_MTRR_DEF_TYPE`, low half — the whole of it fits.
    pub(crate) const DEF_TYPE: u64 = 4;
    /// `IA32_MTRR_FIX64K_00000`, both halves.
    pub(crate) const FIX_LO: u64 = 8;
    pub(crate) const FIX_HI: u64 = 12;
    /// `IA32_LSTAR`, both halves.
    pub(crate) const LSTAR_LO: u64 = 16;
    pub(crate) const LSTAR_HI: u64 = 20;
    /// `IA32_MISC_ENABLE`, high half, which is where the execute-disable lock
    /// lives (bit 34).
    pub(crate) const MISC_HI: u64 = 24;
    /// `RDTSC`, both halves.
    pub(crate) const TSC_LO: u64 = 28;
    pub(crate) const TSC_HI: u64 = 32;
    /// One past the last word the guest writes.
    pub(crate) const END: u64 = 36;
}

/// What the guest programs, and therefore what it must read back.
const DEF_TYPE: u32 = 0x0c06; // E | FE | write-back
const FIX_LO: u32 = 0x0606_0606;
const FIX_HI: u32 = 0x0606_0606;
const LSTAR_LO: u32 = 0x0010_0000;
const LSTAR_HI: u32 = 0xffff_8000;
/// Bit 34 of `IA32_MISC_ENABLE` — the execute-disable lock — is bit 2 of the
/// high half. Firmware on some parts sets it and an operating system clears it
/// before it looks for `NX`, which is the reason the register is modelled at
/// all, and it is one of exactly two bits `WRMSR` will accept here.
const MISC_HI: u32 = 4;

/// The guest: program four model-specific registers, then loop reading them
/// back and writing what `RDMSR` returned into the witness area.
///
/// Encoded so the **same bytes** mean the same thing in a 64-bit code segment
/// and a 32-bit one — `mov edi, imm32` zero-extends into `RDI`, and `89 47 xx`
/// is `mov [rdi+disp8], eax` in one and `mov [edi+disp8], eax` in the other.
/// `RDMSR`, `WRMSR` and `RDTSC` are two-byte opcodes with no operand-size
/// dependence at all.
///
/// ```text
/// setup:
///   b9 ff 02 00 00   mov ecx, 0x2ff          ; IA32_MTRR_DEF_TYPE
///   b8 06 0c 00 00   mov eax, 0xc06
///   ba 00 00 00 00   mov edx, 0
///   0f 30            wrmsr
///   b9 50 02 00 00   mov ecx, 0x250          ; IA32_MTRR_FIX64K_00000
///   b8 06 06 06 06   mov eax, 0x06060606
///   ba 06 06 06 06   mov edx, 0x06060606
///   0f 30            wrmsr
///   b9 82 00 00 c0   mov ecx, 0xc0000082     ; IA32_LSTAR
///   b8 00 00 10 00   mov eax, 0x00100000
///   ba 00 80 ff ff   mov edx, 0xffff8000
///   0f 30            wrmsr
///   b9 a0 01 00 00   mov ecx, 0x1a0          ; IA32_MISC_ENABLE
///   0f 32            rdmsr                   ; read-modify-write, because the
///   83 ca 04         or edx, 4               ; other bits are the part's
///   0f 30            wrmsr
/// loop:
///   bf 00 20 00 00   mov edi, 0x2000
///   ff 07            inc dword [rdi]
///   b9 ff 02 00 00   mov ecx, 0x2ff
///   0f 32            rdmsr
///   89 47 04         mov [rdi+4], eax
///   b9 50 02 00 00   mov ecx, 0x250
///   0f 32            rdmsr
///   89 47 08         mov [rdi+8], eax
///   89 57 0c         mov [rdi+12], edx
///   b9 82 00 00 c0   mov ecx, 0xc0000082
///   0f 32            rdmsr
///   89 47 10         mov [rdi+16], eax
///   89 57 14         mov [rdi+20], edx
///   b9 a0 01 00 00   mov ecx, 0x1a0
///   0f 32            rdmsr
///   89 57 18         mov [rdi+24], edx
///   0f 31            rdtsc
///   89 47 1c         mov [rdi+28], eax
///   89 57 20         mov [rdi+32], edx
///   eb c9            jmp loop
/// ```
fn program() -> Vec<u8> {
    let mut out: Vec<u8> = Vec::new();
    let wr = |out: &mut Vec<u8>, index: u32, lo: u32, hi: u32| {
        out.push(0xb9);
        out.extend_from_slice(&index.to_le_bytes());
        out.push(0xb8);
        out.extend_from_slice(&lo.to_le_bytes());
        out.push(0xba);
        out.extend_from_slice(&hi.to_le_bytes());
        out.extend_from_slice(&[0x0f, 0x30]);
    };
    wr(&mut out, 0x2ff, DEF_TYPE, 0);
    wr(&mut out, 0x250, FIX_LO, FIX_HI);
    wr(&mut out, 0xc000_0082, LSTAR_LO, LSTAR_HI);
    // `IA32_MISC_ENABLE` is read-modify-write rather than written outright:
    // every bit but two is a statement about the part rather than guest state,
    // and `WRMSR` of anything outside `misc_enable::WRITABLE` raises `#GP(0)`
    // on this core — so a blind write would fault on the interpreter and be
    // accepted on hardware, which would make the two engines differ for a
    // reason that had nothing to do with the snapshot.
    out.extend_from_slice(&[0xb9, 0xa0, 0x01, 0x00, 0x00]);
    out.extend_from_slice(&[0x0f, 0x32]);
    out.extend_from_slice(&[0x83, 0xca, 0x04]);
    out.extend_from_slice(&[0x0f, 0x30]);

    let loop_at = out.len();
    // mov edi, WITNESS
    out.push(0xbf);
    out.extend_from_slice(&(WITNESS as u32).to_le_bytes());
    // inc dword [rdi]
    out.extend_from_slice(&[0xff, 0x07]);
    let rd = |out: &mut Vec<u8>, index: u32, lo: Option<u64>, hi: Option<u64>| {
        out.push(0xb9);
        out.extend_from_slice(&index.to_le_bytes());
        out.extend_from_slice(&[0x0f, 0x32]);
        if let Some(disp) = lo {
            out.extend_from_slice(&[0x89, 0x47, disp as u8]);
        }
        if let Some(disp) = hi {
            out.extend_from_slice(&[0x89, 0x57, disp as u8]);
        }
    };
    rd(&mut out, 0x2ff, Some(at::DEF_TYPE), None);
    rd(&mut out, 0x250, Some(at::FIX_LO), Some(at::FIX_HI));
    rd(
        &mut out,
        0xc000_0082,
        Some(at::LSTAR_LO),
        Some(at::LSTAR_HI),
    );
    rd(&mut out, 0x1a0, None, Some(at::MISC_HI));
    // rdtsc, then both halves
    out.extend_from_slice(&[0x0f, 0x31]);
    out.extend_from_slice(&[0x89, 0x47, at::TSC_LO as u8]);
    out.extend_from_slice(&[0x89, 0x57, at::TSC_HI as u8]);
    // `out 0x80, al`, which is the reason the board has an I/O space.
    //
    // **A hypervisor's guest that takes no exits is not bounded here.** Under
    // `ThreadingMode::Accel` the preemption interval brings one back;
    // `Parallel` sets that interval to zero, so a loop with no `HLT`, no
    // `MMIO` and no `IN`/`OUT` in it stays inside `KVM_RUN` forever and the
    // scheduler never gets its thread back — which is exactly what this test
    // did before this instruction was in it. One port write per iteration is
    // the smallest exit that does not also park the processor the way a `HLT`
    // on a board with no interrupt controller would. The port is unassigned
    // and the space discards the write; nothing reads it.
    out.extend_from_slice(&[0xe6, 0x80]);
    // jmp loop
    let from = out.len() + 2;
    let delta = (loop_at as i64 - from as i64) as i8;
    out.extend_from_slice(&[0xeb, delta as u8]);
    out
}

/// A selector value. Nothing loads a descriptor here — the hidden caches are
/// written directly, as a processor that had loaded one would hold them.
const CODE_SEL: u16 = 0x08;
const DATA_SEL: u16 = 0x10;

/// Long mode over a four-level identity map, built rather than reached.
fn system() -> Sys {
    let mut sys = Sys::reset();
    sys.cr0 |= cr0::PE;
    // A zero-limit interrupt table on purpose: nothing in this guest faults,
    // and one that did would shut the processor down loudly rather than
    // vectoring into whatever RAM happened to hold.
    sys.idtr = Default::default();
    sys.gdtr = Default::default();
    sys.segs[usize::from(seg::CS)] = SegReg {
        selector: CODE_SEL,
        base: 0,
        limit: 0xffff_ffff,
        ar: ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::L | ar::GRANULAR,
    };
    for index in [seg::DS, seg::ES, seg::SS, seg::FS, seg::GS] {
        sys.segs[usize::from(index)] = SegReg {
            selector: DATA_SEL,
            base: 0,
            limit: 0xffff_ffff,
            ar: ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::DB,
        };
    }
    sys.cr4 |= cr4::PAE;
    sys.cr3 = PML4;
    sys.efer |= efer::LME | efer::LMA;
    sys.cr0 |= cr0::PG;
    sys
}

/// Identity-map the first four mebibytes with two 2 MiB pages.
fn map_identity(space: &Arc<rsemu::core::space::AddressSpace>) {
    const PRESENT_RW: u64 = 0b11;
    const LARGE: u64 = 1 << 7;
    let put = |at: u64, value: u64| {
        space
            .write(at, Width::U64, value, MemAttrs::DEFAULT)
            .expect("the tables fit in RAM");
    };
    put(PML4, PDPT | PRESENT_RW);
    put(PDPT, PDIR | PRESENT_RW);
    put(PDIR, LARGE | PRESENT_RW);
    put(PDIR + 8, 0x20_0000 | LARGE | PRESENT_RW);
}

/// Everything a test writes into a board before running it: the bytes, where
/// they go, and the world the processor wakes up in.
///
/// **A value rather than a function**, because this file now places two
/// different guests and the two must be placed *identically* on both engines
/// or the comparison is between two boards rather than two engines.
struct Placement {
    /// What to write into the memory space, and where.
    blobs: Vec<(u64, Vec<u8>)>,
    /// The system registers the processor wakes up with.
    sys: Sys,
    /// Where it starts.
    rip: u64,
    /// The stack it starts on. Zero where the guest never pushes.
    rsp: u64,
}

/// Place a core in the world above and load the guest into its space.
///
/// Shared by both engines, and that sharing is the point: an accelerated
/// processor is placed by writing its **shell's** system registers, because the
/// shell is the device and `accel::state` is what carries the result into
/// hardware. If this had to be done twice the two boards would not be the same
/// board.
fn place(cpu: &X86, p: &Placement) {
    // One step discharges the reset sequence, which is what clears
    // `reset_pending`; without it the first round would run the sequence and
    // throw away what is written below.
    cpu.step();
    assert!(!cpu.reset_requested(), "the reset sequence did not run");
    let space = cpu.space().expect("the core has its space");
    for (at, bytes) in &p.blobs {
        for (n, byte) in bytes.iter().enumerate() {
            space
                .write(
                    at + n as u64,
                    Width::U8,
                    u64::from(*byte),
                    MemAttrs::DEFAULT,
                )
                .expect("the blob fits in RAM");
        }
    }
    map_identity(&space);
    cpu.set_sys(p.sys);
    let mut regs = Regs::new();
    regs.cs = CODE_SEL;
    for sr in [seg::SS, seg::DS, seg::ES, seg::FS, seg::GS] {
        regs.set_segment(sr, DATA_SEL);
    }
    regs.rip = p.rip;
    regs.rsp = p.rsp;
    regs.eflags = rsemu::cpu::x86::flags::ALWAYS_SET;
    cpu.set_regs(regs);
}

/// The model-specific-register guest, placed.
fn msr_placement() -> Placement {
    Placement {
        blobs: vec![(PROGRAM, program())],
        sys: system(),
        rip: PROGRAM,
        rsp: 0,
    }
}

/// The board with an emulated core: `engine` is `interp`, `jit` or `jit-host`.
///
/// The core comes back with the machine because a test that has to stop the
/// guest at one exact instruction needs [`X86::step`], and because driving the
/// `INTR` pin by hand is how a board with no interrupt controller gets an
/// interrupt at all.
fn emulated(engine: &str, tag: &str, p: &Placement) -> (Machine, Arc<X86>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cpus);
    let mut bindings = rsemu::machine::catalog::bindings().expect("this build's bindings");
    bindings.replace("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::X86_64)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings)
        .with_param("engine", engine);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = build(
        &format!("x86-arch-state.{tag}"),
        SOURCE,
        &registry,
        &options,
    )
    .unwrap_or_else(|e| panic!("the board does not build with engine={engine}: {e}"));
    let core = cpus.take().expect("the binding captured the core");
    place(&core, p);
    (machine, core)
}

/// The same board with its core on host silicon, or `None` with no `/dev/kvm`.
///
/// `AccelCpus::install` is the whole of the interception: the machine file is
/// used verbatim, `engine = "interp"` and all, and what changes is the engine
/// underneath it.
fn accelerated(tag: &str, p: &Placement) -> Option<(Machine, Arc<AccelCpus>)> {
    if !Kvm::is_available() {
        return None;
    }
    // `Parallel` rather than `Deterministic`: `AccelCpus::open` refuses a mode
    // claiming reproducibility, because a run on host silicon is not
    // reproducible and a state hash taken over one is a number a regression
    // suite would then bless.
    let accel = match AccelCpus::open(ThreadingMode::Parallel) {
        Ok(accel) => accel,
        Err(e) if e.is_unavailable() => return None,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.scheduler.mode = ThreadingMode::Parallel;
    accel.install(&mut options.bindings);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = build(
        &format!("x86-arch-state.{tag}"),
        SOURCE,
        &registry,
        &options,
    )
    .unwrap_or_else(|e| panic!("the board does not realize under acceleration: {e}"));
    let cpu = accel.cpus().pop().expect("the board's processor");
    place(cpu.shell(), p);
    Some((machine, accel))
}

// ---------------------------------------------------------------------------
// the witness
// ---------------------------------------------------------------------------

/// Read one witness word out of a board's memory space, as a debugger would.
fn peek(m: &Machine, off: u64) -> u32 {
    m.space("mem")
        .expect("the memory space")
        .read(WITNESS + off, Width::U32, MemAttrs::DEBUG)
        .expect("RAM answers") as u32
}

/// Erase the witness area, so that everything read after this was produced by
/// an instruction the destination engine executed.
///
/// **This is what makes the test about the engine rather than about the
/// snapshot.** `Machine::load` restores RAM, so a restored board arrives with
/// the *source* engine's answers already written; asserting them would only say
/// that memory round-trips, which every snapshot test in the tree already says.
fn erase(m: &Machine) {
    let space = m.space("mem").expect("the memory space");
    let mut off = 0;
    while off < at::END {
        space
            .write(WITNESS + off, Width::U32, 0, MemAttrs::DEFAULT)
            .expect("RAM takes a write");
        off += 4;
    }
}

/// Run the board until the guest has been round its loop, or give up.
fn run_until_witnessed(m: &mut Machine) {
    for _ in 0..200 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek(m, at::TICK) > 0 {
            return;
        }
    }
}

/// Everything the guest says it can see, for a failure message worth reading.
fn describe(m: &Machine) -> String {
    format!(
        "tick={} def_type={:#x} fix={:#010x}{:08x} lstar={:#010x}{:08x} misc_hi={:#x} tsc={:#010x}{:08x}",
        peek(m, at::TICK),
        peek(m, at::DEF_TYPE),
        peek(m, at::FIX_HI),
        peek(m, at::FIX_LO),
        peek(m, at::LSTAR_HI),
        peek(m, at::LSTAR_LO),
        peek(m, at::MISC_HI),
        peek(m, at::TSC_HI),
        peek(m, at::TSC_LO),
    )
}

/// Assert that the guest, on whatever engine it is now running, reads back
/// every register it programmed.
fn assert_the_guest_still_sees_its_registers(m: &Machine, whose: &str) {
    let seen = describe(m);
    assert!(
        peek(m, at::TICK) > 0,
        "{whose}: the guest never ran ({seen})"
    );
    assert_eq!(
        peek(m, at::DEF_TYPE),
        DEF_TYPE,
        "{whose}: IA32_MTRR_DEF_TYPE. Zero here is not `the default` — SDM \
         vol 3A 12.11.2.1 makes E=0 mean every physical address uncacheable \
         ({seen})"
    );
    assert_eq!(peek(m, at::FIX_LO), FIX_LO, "{whose}: FIX64K low ({seen})");
    assert_eq!(peek(m, at::FIX_HI), FIX_HI, "{whose}: FIX64K high ({seen})");
    assert_eq!(
        peek(m, at::LSTAR_LO),
        LSTAR_LO,
        "{whose}: LSTAR low ({seen})"
    );
    assert_eq!(
        peek(m, at::LSTAR_HI),
        LSTAR_HI,
        "{whose}: IA32_LSTAR high. A 64-bit guest that loses this has a \
         SYSCALL that jumps to zero ({seen})"
    );
    assert_eq!(
        peek(m, at::MISC_HI) & MISC_HI,
        MISC_HI,
        "{whose}: the execute-disable lock in IA32_MISC_ENABLE ({seen})"
    );
}

// ---------------------------------------------------------------------------
// the gate
// ---------------------------------------------------------------------------

/// **KVM to an emulated engine**, which is one direction of phase 7's gate.
///
/// Run on hardware until the guest has programmed and read back its registers,
/// save, restore into a board whose core is `engine`, erase the witness, and
/// run on. Everything the guest then reports it re-read for itself.
fn a_hardware_snapshot_restores_under(engine: &str) {
    let Some((mut hardware, accel)) = accelerated(&format!("kvm-to-{engine}"), &msr_placement())
    else {
        return;
    };
    run_until_witnessed(&mut hardware);
    assert_the_guest_still_sees_its_registers(&hardware, "on hardware");
    assert!(
        accel.cpus()[0].entries() > 0,
        "the processor never entered the guest, so nothing was accelerated"
    );
    let tsc_before =
        u64::from(peek(&hardware, at::TSC_HI)) << 32 | u64::from(peek(&hardware, at::TSC_LO));
    let saved = hardware.save().expect("an accelerated machine saves");

    let (mut emulated, _core) = emulated(engine, &format!("kvm-to-{engine}"), &msr_placement());
    emulated
        .load(&saved)
        .unwrap_or_else(|e| panic!("a snapshot taken under KVM will not load under {engine}: {e}"));
    erase(&emulated);
    run_until_witnessed(&mut emulated);
    assert_the_guest_still_sees_its_registers(&emulated, engine);

    // And the counter is continuous rather than restarting. It is the one
    // carried value whose *rate* legitimately changes across the switch — see
    // `accel::state` — so what is asserted is monotonicity, which is what a
    // guest's timekeeping actually depends on.
    let tsc_after =
        u64::from(peek(&emulated, at::TSC_HI)) << 32 | u64::from(peek(&emulated, at::TSC_LO));
    assert!(
        tsc_after >= tsc_before,
        "RDTSC went backwards across the restore: {tsc_before:#x} -> {tsc_after:#x}. \
         A guest whose clocksource does that marks it unstable at best"
    );
}

/// **An emulated engine to KVM**, the other direction.
fn a_snapshot_from_restores_under_hardware(engine: &str) {
    if !Kvm::is_available() {
        return;
    }
    let (mut emulated, _core) = emulated(engine, &format!("{engine}-to-kvm"), &msr_placement());
    run_until_witnessed(&mut emulated);
    assert_the_guest_still_sees_its_registers(&emulated, engine);
    let tsc_before =
        u64::from(peek(&emulated, at::TSC_HI)) << 32 | u64::from(peek(&emulated, at::TSC_LO));
    let saved = emulated.save().expect("the emulated machine saves");

    let Some((mut hardware, accel)) = accelerated(&format!("{engine}-to-kvm"), &msr_placement())
    else {
        return;
    };
    hardware
        .load(&saved)
        .unwrap_or_else(|e| panic!("a snapshot taken under {engine} will not load under KVM: {e}"));
    erase(&hardware);
    run_until_witnessed(&mut hardware);
    assert_the_guest_still_sees_its_registers(&hardware, "on hardware");
    assert!(
        accel.cpus()[0].entries() > 0,
        "the restored processor never entered the guest"
    );
    let tsc_after =
        u64::from(peek(&hardware, at::TSC_HI)) << 32 | u64::from(peek(&hardware, at::TSC_LO));
    assert!(
        tsc_after >= tsc_before,
        "RDTSC went backwards across the restore into hardware: \
         {tsc_before:#x} -> {tsc_after:#x}. This is the direction that has to \
         recompute the hypervisor's TSC offset, and `accel::state::restore_into_vcpu` \
         is the call that does it"
    );
}

#[test]
fn a_snapshot_taken_under_kvm_restores_under_the_interpreter() {
    a_hardware_snapshot_restores_under("interp");
}

#[test]
fn a_snapshot_taken_under_the_interpreter_restores_under_kvm() {
    a_snapshot_from_restores_under_hardware("interp");
}

#[cfg(all(feature = "cpu-x86-lift", feature = "jit"))]
#[test]
fn a_snapshot_taken_under_kvm_restores_under_the_jit() {
    a_hardware_snapshot_restores_under("jit");
}

#[cfg(all(feature = "cpu-x86-lift", feature = "jit"))]
#[test]
fn a_snapshot_taken_under_the_jit_restores_under_kvm() {
    a_snapshot_from_restores_under_hardware("jit");
}

/// The chunk is **one format**, and a restore is byte-identical whichever
/// engine wrote it.
///
/// `tests/kvm_smp.rs` asserts this for a 486 board. Repeated here because this
/// board's core has thirty-four model-specific registers in its chunk that a
/// 486's does not, so the two tests would fail for different reasons.
#[test]
fn the_chunk_is_the_same_bytes_whichever_engine_wrote_it() {
    let Some((mut hardware, _accel)) = accelerated("bytes", &msr_placement()) else {
        return;
    };
    run_until_witnessed(&mut hardware);
    let saved = hardware.save().expect("an accelerated machine saves");

    let (mut interpreted, _core) = emulated("interp", "bytes", &msr_placement());
    interpreted
        .load(&saved)
        .expect("a snapshot taken under KVM restores under the interpreter");
    assert_eq!(
        saved,
        interpreted.save().expect("and saves again"),
        "a snapshot taken under KVM does not survive a round trip through the \
         interpreter, so the two engines do not agree on what a processor is"
    );
}

/// The `cpu.x86` chunk did **not** change for any of this, and that is the
/// finding rather than an omission.
///
/// `CLAUDE.md`'s device rule and `src/machine/migrate.rs` say a changed chunk
/// needs a bumped class version and a registered migration in the same commit.
/// The defect this file was written for was not a missing *field*: the
/// interpreter's chunk already wrote `mtrr_def_type`, `mtrr_fix`, `mtrr_var`,
/// `misc_enable` and `cycles`, and had since before there was an accelerator.
/// What was missing was the **bridge** — `accel::state::CARRIED_MSRS` listed
/// five registers, so those fields were written faithfully and were faithfully
/// zero. Fixing a bridge changes no bytes, so nothing bumps and no snapshot an
/// earlier build wrote is orphaned.
///
/// This test pins that claim, so that the next person to widen the model finds
/// out here whether they have crossed the line into a version bump. The
/// `XSAVE` work named in `accel::state`'s honest list is the case that will:
/// an AVX register file is new chunk bytes.
#[test]
fn the_processors_chunk_version_is_unchanged_by_the_widened_model() {
    assert_eq!(
        rsemu::cpu::x86::CLASS.version,
        8,
        "the cpu.x86 chunk grew. `CLAUDE.md` and `src/machine/migrate.rs`: bump \
         the class version *and* register the migration step in the same commit, \
         and add it to `default_migrations`"
    );
}

// ---------------------------------------------------------------------------
// the interrupt shadow
// ---------------------------------------------------------------------------

/// Where the interrupt handler goes.
const HANDLER: u64 = 0x1800;
/// The global descriptor table. Needed because *delivering* an interrupt
/// reloads `CS` from it — the tests above never take one, so they get away
/// with a zero-limit `GDTR`, and this one would triple-fault on it.
const GDT: u64 = 0x5000;
/// The interrupt descriptor table.
const IDT: u64 = 0x4000;
/// The stack the interrupt frame is pushed on.
const STACK: u64 = 0x8000;
/// The vector the test drives onto the `INTR` pin.
const VECTOR: u8 = 0x20;
/// What the shadow-covered `OUT` writes, so that a stray `AL` cannot look like
/// a pass.
const SENTINEL: u8 = 0x5a;

/// Offsets in the witness area, for the shadow guest.
mod sh {
    /// How many times round the loop.
    pub(crate) const TICK: u64 = 0;
    /// How many interrupts the guest has taken.
    pub(crate) const TAKEN: u64 = 4;
    /// **The witness.** The `RIP` the *first* interrupt interrupted, as the
    /// handler read it off its own stack frame.
    pub(crate) const SEEN_RIP: u64 = 8;
}

/// The shadow guest, and the addresses inside it the assertions name.
struct ShadowGuest {
    placement: Placement,
    /// The address of the `OUT` that the `STI` before it casts its shadow
    /// over. A processor stopped here **is** a processor in an interrupt
    /// shadow, which is what makes this a snapshot worth taking.
    shadowed_at: u64,
    /// The address after it: where a correctly shadowed interrupt is taken.
    after_shadowed: u64,
}

/// **A guest that can tell whether it was interrupted one instruction early.**
///
/// The engine-versus-engine comparison in `tests/kvm_smp.rs` cannot see this
/// class of defect at all: a field neither side carries looks *identical* on
/// both sides. So, as with the model-specific registers above, the guest is
/// the witness — and here it witnesses something no register holds.
///
/// The interrupt shadow is the one instruction after a `STI`, a `MOV SS` or a
/// `POP SS` during which an external interrupt is not recognised (*Intel SDM*
/// volume 2B, `STI`; volume 3A §6.8.3). It exists so that `STI; HLT` cannot
/// lose a wakeup and so that a `SS:SP` reload cannot be interrupted halfway.
/// It is not in `RFLAGS`, not in a control register and not in any MSR; on
/// hardware it lives in the VMCS's interruptibility state, and
/// `KVM_GET_VCPU_EVENTS` is the only request that reports it.
///
/// The program:
///
/// ```text
///        bd 00 20 00 00   mov ebp, WITNESS       ; the witness area, once
///   loop:
///        ff 45 00         inc dword [rbp+0]      ; TICK
///        b0 5a            mov al, 0x5a
///        fa               cli                    ; so the STI below shadows
///        fb               sti
///        e6 80            out 0x80, al           ; <-- SHADOWED, and an exit
///        eb f5            jmp loop
///
///   handler:                                     ; IDT vector 0x20
///        83 7d 04 00      cmp dword [rbp+4], 0   ; only the first one counts
///        75 06            jne skip
///        8b 04 24         mov eax, [rsp]         ; the RIP it interrupted
///        89 45 08         mov [rbp+8], eax
///   skip:
///        ff 45 04         inc dword [rbp+4]      ; TAKEN
///        81 64 24 10 ..   and dword [rsp+16], ~IF ; return with IF clear, so
///        48 cf            iretq                   ; a level-held INTR does not
///                                                 ; storm
/// ```
///
/// # Why the shadowed instruction is an `OUT`
///
/// Because a hypervisor's guest is only observable at an **exit**, and the
/// snapshot has to be taken *inside* the shadow. Measured on this host rather
/// than assumed: with `CLI; STI; OUT` in a loop, every `KVM_RUN` comes back
/// with `RIP` pointing at the `OUT` and `KVM_GET_VCPU_EVENTS` reporting
/// `interrupt.shadow = 3` — the port write has not retired and the shadow is
/// still owed. The same sequence with an MMIO access, or with any instruction
/// between the `STI` and the exit, comes back with the shadow already spent.
/// So this shape parks the processor exactly where the defect lives, on every
/// slice, with no timing luck involved.
///
/// # What each outcome means
///
/// The test asserts the *first* interrupt after the restore, with the `INTR`
/// pin driven by hand only once the destination engine holds the state:
///
/// * `SEEN_RIP == after_shadowed` — the shadow crossed. The `OUT` ran, and
///   only then was the interrupt taken.
/// * `SEEN_RIP == shadowed_at` — **the shadow was dropped in the transfer.**
///   The interrupt landed on the instruction the shadow was protecting. On a
///   real guest that instruction is the second half of a `MOV SS; MOV ESP`
///   pair and the interrupt is pushed onto a stack that does not exist yet.
fn shadow_guest() -> ShadowGuest {
    let mut main: Vec<u8> = Vec::new();
    // mov ebp, WITNESS — zero-extends into RBP, so `[rbp+disp8]` reaches the
    // witness area in 64-bit mode without a REX prefix or a 64-bit immediate.
    main.push(0xbd);
    #[allow(clippy::cast_possible_truncation)]
    main.extend_from_slice(&(WITNESS as u32).to_le_bytes());
    let loop_at = PROGRAM + main.len() as u64;
    main.extend_from_slice(&[0xff, 0x45, sh::TICK as u8]); // inc dword [rbp+0]
    main.extend_from_slice(&[0xb0, SENTINEL]); // mov al, SENTINEL
    main.push(0xfa); // cli
    main.push(0xfb); // sti
    let shadowed_at = PROGRAM + main.len() as u64;
    main.extend_from_slice(&[0xe6, 0x80]); // out 0x80, al
    let after_shadowed = PROGRAM + main.len() as u64;
    let from = PROGRAM + main.len() as u64 + 2;
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let delta = (loop_at as i64 - from as i64) as i8;
    #[allow(clippy::cast_sign_loss)]
    main.extend_from_slice(&[0xeb, delta as u8]);

    let handler: Vec<u8> = vec![
        0x83,
        0x7d,
        sh::TAKEN as u8,
        0x00, // cmp dword [rbp+TAKEN], 0
        0x75,
        0x06, // jne skip
        0x8b,
        0x04,
        0x24, // mov eax, [rsp]
        0x89,
        0x45,
        sh::SEEN_RIP as u8, // mov [rbp+SEEN_RIP], eax
        // skip:
        0xff,
        0x45,
        sh::TAKEN as u8, // inc dword [rbp+TAKEN]
        // and dword [rsp+16], ~IF — the interrupt frame's RFLAGS. `INTR` is a
        // level this test holds high, so returning with `IF` set would take
        // the same interrupt again on the instruction after the `IRET` and the
        // guest would never make progress.
        0x81,
        0x64,
        0x24,
        0x10,
        0xff,
        0xfd,
        0xff,
        0xff,
        0x48,
        0xcf, // iretq — the frame is the 64-bit one
    ];

    ShadowGuest {
        placement: Placement {
            blobs: vec![
                (PROGRAM, main),
                (HANDLER, handler),
                (GDT, gdt()),
                (IDT, idt()),
            ],
            sys: shadow_system(),
            rip: PROGRAM,
            rsp: STACK,
        },
        shadowed_at,
        after_shadowed,
    }
}

/// A three-entry global descriptor table: null, a 64-bit code segment at
/// [`CODE_SEL`], a data segment at [`DATA_SEL`].
///
/// The tests above run with a zero-limit `GDTR` because nothing in them ever
/// loads a descriptor. Taking an interrupt does: the gate names a code
/// selector and the processor reads it out of this table.
fn gdt() -> Vec<u8> {
    let mut out = vec![0u8; 8];
    // limit ffff, base 0, present/DPL0/code/readable, granular + long.
    out.extend_from_slice(&[0xff, 0xff, 0, 0, 0, 0x9a, 0xaf, 0x00]);
    // the same, as data: present/DPL0/data/writable, granular + 32-bit.
    out.extend_from_slice(&[0xff, 0xff, 0, 0, 0, 0x92, 0xcf, 0x00]);
    out
}

/// An interrupt descriptor table with one 64-bit interrupt gate, at
/// [`VECTOR`].
///
/// A gate rather than a trap gate, and that is the point: an interrupt gate
/// clears `IF` on entry (*Intel SDM* volume 3A §6.12.1.2), so the handler is
/// not re-entered by the level this test holds on the pin.
fn idt() -> Vec<u8> {
    let mut out = vec![0u8; 16 * (VECTOR as usize + 1)];
    let gate = 16 * VECTOR as usize;
    #[allow(clippy::cast_possible_truncation)]
    let lo = (HANDLER & 0xffff) as u16;
    #[allow(clippy::cast_possible_truncation)]
    let mid = ((HANDLER >> 16) & 0xffff) as u16;
    #[allow(clippy::cast_possible_truncation)]
    let hi = (HANDLER >> 32) as u32;
    out[gate..gate + 2].copy_from_slice(&lo.to_le_bytes());
    out[gate + 2..gate + 4].copy_from_slice(&CODE_SEL.to_le_bytes());
    out[gate + 4] = 0; // no interrupt-stack table
    out[gate + 5] = 0x8e; // present, DPL 0, 64-bit interrupt gate
    out[gate + 6..gate + 8].copy_from_slice(&mid.to_le_bytes());
    out[gate + 8..gate + 12].copy_from_slice(&hi.to_le_bytes());
    out
}

/// [`system`], plus the two descriptor tables an interrupt needs.
fn shadow_system() -> Sys {
    let mut sys = system();
    sys.gdtr = rsemu::cpu::x86::prot::TableReg {
        base: GDT,
        limit: 0x17,
    };
    sys.idtr = rsemu::cpu::x86::prot::TableReg {
        base: IDT,
        limit: 16 * u32::from(VECTOR) + 15,
    };
    sys
}

/// Run until the guest has been round its loop and the processor is stopped on
/// the shadowed instruction.
///
/// The predicate is `RIP`, deliberately, rather than anything either engine
/// reports about its own interruptibility: a test whose *gate* is the field
/// under test cannot fail when that field is missing, it can only skip.
fn park_in_the_shadow(m: &mut Machine, rip_of: &dyn Fn() -> u64, at: u64) -> bool {
    for _ in 0..400 {
        m.run_for(GlobalTime::from_nanos(200_000))
            .expect("the board runs");
        if peek(m, sh::TICK) > 0 && rip_of() == at {
            return true;
        }
    }
    false
}

/// Assert what the guest saw, on whichever engine it woke up on.
fn assert_the_interrupt_waited(m: &Machine, g: &ShadowGuest, whose: &str) {
    let taken = peek(m, sh::TAKEN);
    let seen = u64::from(peek(m, sh::SEEN_RIP));
    assert!(
        taken > 0,
        "{whose}: the restored guest never took the interrupt at all \
         (tick={}, INTR was driven high before it ran)",
        peek(m, sh::TICK)
    );
    assert_ne!(
        seen, g.shadowed_at,
        "{whose}: the interrupt was delivered *on* the instruction the STI \
         shadowed, at {:#x}. The interrupt shadow did not survive the \
         transfer — nothing but KVM_GET_VCPU_EVENTS reports it, and the \
         emulated chunk's `int_shadow` is where it has to land. A real guest \
         losing this takes an interrupt on a half-loaded SS:SP",
        g.shadowed_at
    );
    assert_eq!(
        seen, g.after_shadowed,
        "{whose}: the interrupt was taken at {seen:#x}, which is neither the \
         shadowed instruction ({:#x}) nor the one after it ({:#x}) — the guest \
         is somewhere this test does not understand",
        g.shadowed_at, g.after_shadowed
    );
}

/// **A snapshot taken on hardware inside an interrupt shadow, restored under
/// an emulated engine.**
fn a_shadowed_hardware_snapshot_restores_under(engine: &str) {
    let g = shadow_guest();
    let tag = format!("shadow-kvm-to-{engine}");
    let Some((mut hardware, accel)) = accelerated(&tag, &g.placement) else {
        return;
    };
    let cpu = accel.cpus()[0].clone();
    let shell = cpu.clone();
    assert!(
        park_in_the_shadow(
            &mut hardware,
            &move || shell.shell().regs().rip,
            g.shadowed_at
        ),
        "the accelerated guest never stopped on the shadowed instruction at {:#x}",
        g.shadowed_at
    );
    assert!(
        cpu.entries() > 0,
        "the processor never entered the guest, so nothing was accelerated"
    );
    // The intermediate claim, named so a failure says which half broke: the
    // shell is this processor's architectural state between slices, and the
    // shadow is part of that state.
    assert!(
        cpu.shell().interrupt_shadow(),
        "the vCPU is stopped on the shadowed instruction and the shell does \
         not know it — `accel::state::store_from_vcpu` is not reading \
         KVM_GET_VCPU_EVENTS"
    );
    let saved = hardware.save().expect("an accelerated machine saves");
    drop(hardware);

    let (mut emulated, core) = emulated(engine, &tag, &g.placement);
    emulated
        .load(&saved)
        .unwrap_or_else(|e| panic!("a snapshot taken under KVM will not load under {engine}: {e}"));
    erase(&emulated);
    assert_eq!(
        core.regs().rip,
        g.shadowed_at,
        "the restore did not put the processor where the snapshot had it"
    );
    // Now, and only now, the pin. The board has no interrupt controller on
    // purpose: what is under test is what the *core* does with the state it
    // was handed, and a controller would add its own.
    core.set_intr_vector(VECTOR);
    core.set_intr(true);
    for _ in 0..200 {
        emulated
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek(&emulated, sh::TAKEN) > 0 {
            break;
        }
    }
    assert_the_interrupt_waited(&emulated, &g, engine);
}

/// **The other direction**: a snapshot taken under an emulated engine inside
/// an interrupt shadow, restored onto host silicon.
fn a_shadowed_snapshot_from_restores_under_hardware(engine: &str) {
    if !Kvm::is_available() {
        return;
    }
    let g = shadow_guest();
    let tag = format!("shadow-{engine}-to-kvm");
    let (mut emulated, core) = emulated(engine, &tag, &g.placement);
    run_until_witnessed(&mut emulated);
    // An emulated core stops between instructions wherever it is asked to, so
    // it is walked to the shadow rather than parked there by an exit. Bounded
    // by rather more than one turn of the loop.
    for _ in 0..64 {
        if core.regs().rip == g.shadowed_at {
            break;
        }
        assert!(core.step() > 0, "the emulated core stopped");
    }
    assert_eq!(
        core.regs().rip,
        g.shadowed_at,
        "the emulated guest never reached the shadowed instruction"
    );
    assert!(
        core.interrupt_shadow(),
        "the interpreter is on the instruction after a STI and does not think \
         it is shadowed, which is a core bug rather than a transfer one"
    );
    let saved = emulated.save().expect("the emulated machine saves");
    drop(emulated);

    let Some((mut hardware, accel)) = accelerated(&tag, &g.placement) else {
        return;
    };
    hardware
        .load(&saved)
        .unwrap_or_else(|e| panic!("a snapshot taken under {engine} will not load under KVM: {e}"));
    erase(&hardware);
    let cpu = accel.cpus()[0].clone();
    cpu.shell().set_intr_vector(VECTOR);
    cpu.shell().set_intr(true);
    for _ in 0..200 {
        hardware
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek(&hardware, sh::TAKEN) > 0 {
            break;
        }
    }
    assert!(
        cpu.entries() > 0,
        "the restored processor never entered the guest"
    );
    assert_the_interrupt_waited(&hardware, &g, "on hardware");
}

#[test]
fn an_interrupt_shadow_survives_a_snapshot_from_kvm_to_the_interpreter() {
    a_shadowed_hardware_snapshot_restores_under("interp");
}

#[test]
fn an_interrupt_shadow_survives_a_snapshot_from_the_interpreter_to_kvm() {
    a_shadowed_snapshot_from_restores_under_hardware("interp");
}

#[cfg(all(feature = "cpu-x86-lift", feature = "jit"))]
#[test]
fn an_interrupt_shadow_survives_a_snapshot_from_kvm_to_the_jit() {
    a_shadowed_hardware_snapshot_restores_under("jit");
}

#[cfg(all(feature = "cpu-x86-lift", feature = "jit"))]
#[test]
fn an_interrupt_shadow_survives_a_snapshot_from_the_jit_to_kvm() {
    a_shadowed_snapshot_from_restores_under_hardware("jit");
}

/// Says out loud whether the tests above actually ran.
///
/// A suite that skips is a suite that proved nothing, and a green run that
/// hides that is worse than a red one. This is the line to grep for.
#[test]
fn report_whether_this_host_ran_the_cross_engine_gate() {
    if Kvm::is_available() {
        println!("x86-arch-state: /dev/kvm is usable; the cross-engine gate ran");
    } else {
        println!("x86-arch-state: no usable /dev/kvm; every direction above skipped");
    }
}
