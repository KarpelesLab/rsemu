//! One x86 board under all three execution engines, and the claim that they
//! are the same machine.
//!
//! `ROADMAP.md` §0: *"a bit-identical state hash … across the interpreter and
//! the JIT for the same guest"*. `src/cpu/x86/differential.rs` asserts
//! something one level below that — a block, a register, a tick — and
//! `src/cpu/x86/engine.rs`'s own tests assert it for a bare core; until
//! `engine = "jit"` reached the dispatcher there was no *machine* to hash.
//! There is now, and this is the assertion the roadmap actually wrote down.
//! It is `tests/riscv_virt_engines.rs` for the other core, deliberately in the
//! same shape:
//!
//! * the same board, the same guest, the same number of quanta, under
//!   `engine = "interp"`, `engine = "jit"` and `engine = "jit-host"`, hashes to
//!   the same number at **every** checkpoint — not at the end only, because a
//!   divergence that shows up at the last checkpoint is a different bug from
//!   one that shows up at the first;
//! * a snapshot taken under one engine restores under the other and carries on
//!   to the same hash, which is a property of the snapshot rather than of the
//!   engines: nothing engine-specific is in one, which is also what keeps it
//!   interchangeable with `accel::state`.
//!
//! # Why the board is built here rather than taken from `machines/`
//!
//! Every shipped x86 board is a board for *software this repository does not
//! contain*: `pc-at` and `q35` start in real mode at `0xfffffff0` and want a
//! firmware, and `pc64` and `q35-linux` want a `bzImage`. Real mode is outside
//! the lifted subset by construction (`cpu::x86::lift::World::of`), so a board
//! that spends its first hundred thousand instructions there would compare two
//! interpreters and pass.
//!
//! So this file builds the smallest machine that is *in* the subset: RAM, a
//! clock and a core, with the processor placed in the world under test the way
//! `cpu::x86::differential::oracle` places it — by writing the system
//! registers rather than by executing the twenty instructions that reach them.
//! Both worlds the frontend accepts are covered, because they are different
//! block keys, different store policies and different memory paths:
//!
//! | world | `Origin` | store policy | what a data access goes through |
//! | --- | --- | --- | --- |
//! | 32-bit protected, `CR0.PG` clear | `Flat` | the in-block page guard | the segment check and one bus cycle |
//! | 64-bit long mode, four-level paging | `Paged` | a store ends its block | the segment check, a walk, its accessed and dirty bits, and the page-crossing split |
//!
//! The guest is a loop rather than a program: forty bytes of adds, a store, a
//! load, two shifts, a compare and two branches, encoded so the **same bytes**
//! run in both worlds. Every instruction in it is inside the lifted subset, so
//! the JIT really does translate, cache, chain and re-serve blocks here —
//! asserted through `X86::jit_stats`, not assumed.

#![cfg(all(feature = "cpu-x86", feature = "cpu-x86-lift", feature = "jit"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::prot::{SegReg, Sys, ar, cr0, cr4, efer};
use rsemu::cpu::x86::{Regs, Variant, X86, isa::seg};
use rsemu::machine::{Machine, build};

/// The smallest machine that is inside the lifted subset: RAM, a clock and a
/// core.
///
/// No firmware socket and no reset stub, because nothing here executes from
/// one: the reset sequence is discharged with a single `step` and the world is
/// then written into the system registers. A ROM would only be sixteen bytes
/// of real-mode code that the frontend refuses anyway.
const SOURCE: &str = r#"
machine "x86-engines" {
  param engine = "interp"
  param ram = 8M

  osc cpu = 100000000 Hz

  space mem { width = 64, unassigned = read-as-ones }

  object cpu0 "cpu.x86" {
    clock   = cpu
    space   = mem
    variant = "x86-64"
    engine  = engine
  }

  object dram "ram" { size = ram }

  map mem 0x00000000 size ram = dram
}
"#;

/// Where the guest program is loaded. Its scratch word is at `0x2000`, which
/// the program carries as an immediate.
const PROGRAM: u64 = 0x1000;

/// Where the four-level page tables go in the long-mode world.
const PML4: u64 = 0x30_0000;
const PDPT: u64 = 0x30_1000;
const PDIR: u64 = 0x30_2000;

/// A loop whose forty bytes mean the same thing in a 32-bit code segment and
/// in a 64-bit one.
///
/// ```text
///   bf 00 20 00 00   mov edi, 0x2000     ; the scratch word
///   b8 01 00 00 00   mov eax, 1
///   b9 00 00 00 00   mov ecx, 0
/// loop:
///   01 c1            add ecx, eax        ; every flag written
///   89 0f            mov [edi], ecx      ; the store that ends a paged block
///   8b 17            mov edx, [edi]      ; and the load that reads it back
///   c1 e2 03         shl edx, 3
///   c1 ea 03         shr edx, 3
///   01 d1            add ecx, edx
///   ff c0            inc eax             ; not 0x40, which is a REX prefix
///   3d 00 01 00 00   cmp eax, 0x100
///   75 e9            jne loop            ; the back edge a trace merges
///   eb dd            jmp 0x05            ; and the outer loop, forever
/// ```
///
/// `inc eax` is spelled `ff c0` rather than `40` deliberately: `40` is a `REX`
/// prefix in 64-bit mode, so the one-byte form would be two different programs
/// and the comparison would stop being between two engines.
const LOOP: [u8; 40] = [
    0xbf, 0x00, 0x20, 0x00, 0x00, // mov edi, 0x2000
    0xb8, 0x01, 0x00, 0x00, 0x00, // mov eax, 1
    0xb9, 0x00, 0x00, 0x00, 0x00, // mov ecx, 0
    0x01, 0xc1, // add ecx, eax
    0x89, 0x0f, // mov [edi], ecx
    0x8b, 0x17, // mov edx, [edi]
    0xc1, 0xe2, 0x03, // shl edx, 3
    0xc1, 0xea, 0x03, // shr edx, 3
    0x01, 0xd1, // add ecx, edx
    0xff, 0xc0, // inc eax
    0x3d, 0x00, 0x01, 0x00, 0x00, // cmp eax, 0x100
    0x75, 0xe9, // jne loop
    0xeb, 0xdd, // jmp 0x05
];

/// Which world the core is placed in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum World {
    /// 32-bit protected mode, flat segments, `CR0.PG` clear.
    Flat,
    /// 64-bit long mode over a four-level identity map.
    Long,
}

/// A selector value. Nothing loads a descriptor here — the hidden caches are
/// written directly, exactly as a processor that had loaded one would hold
/// them — so the numbers only have to be non-null and distinct.
const CODE_SEL: u16 = 0x08;
const DATA_SEL: u16 = 0x10;

/// Build the board with `engine`, place the core in `world`, and load the
/// guest.
///
/// The order matters and is the machine's own: `build` realizes and resets,
/// which zeroes RAM, so everything written into memory is written after it.
fn board(engine: &str, world: World, tag: &str) -> (Machine, Arc<X86>) {
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
    let machine = build(&format!("x86-engines.{tag}"), SOURCE, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not build with engine={engine}: {e}"));
    let cpu = cpus.take().expect("the binding captured the core");

    // One step discharges the reset sequence, which is what clears
    // `reset_pending`; without it the first quantum would run the sequence and
    // throw away the `CS:RIP` written below.
    cpu.step();
    assert!(!cpu.reset_requested(), "the reset sequence did not run");

    let space = cpu.space().expect("the core has its space");
    for (n, byte) in LOOP.iter().enumerate() {
        space
            .write(
                PROGRAM + n as u64,
                Width::U8,
                u64::from(*byte),
                MemAttrs::DEFAULT,
            )
            .expect("the program fits in RAM");
    }
    if world == World::Long {
        map_identity(&space);
    }
    cpu.set_sys(system(world));
    let mut regs = Regs::new();
    regs.cs = CODE_SEL;
    for sr in [seg::SS, seg::DS, seg::ES, seg::FS, seg::GS] {
        regs.set_segment(sr, DATA_SEL);
    }
    regs.rip = PROGRAM;
    regs.eflags = rsemu::cpu::x86::flags::ALWAYS_SET;
    cpu.set_regs(regs);
    (machine, cpu)
}

/// The system registers for `world`.
fn system(world: World) -> Sys {
    let long = world == World::Long;
    let mut sys = Sys::reset();
    sys.cr0 |= cr0::PE;
    // A zero-limit interrupt table, on purpose: nothing in this guest faults,
    // and if something ever does it shuts the processor down rather than
    // vectoring into whatever RAM happens to hold — which both engines then
    // report identically and loudly instead of quietly diverging.
    sys.idtr.base = 0;
    sys.idtr.limit = 0;
    sys.gdtr.base = 0;
    sys.gdtr.limit = 0;
    sys.segs[usize::from(seg::CS)] = SegReg {
        selector: CODE_SEL,
        base: 0,
        limit: 0xffff_ffff,
        ar: ar::PRESENT
            | ar::S
            | ar::CODE
            | ar::RW
            | ar::ACCESSED
            | if long { ar::L | ar::GRANULAR } else { ar::DB },
    };
    for index in [seg::DS, seg::ES, seg::SS, seg::FS, seg::GS] {
        sys.segs[usize::from(index)] = SegReg {
            selector: DATA_SEL,
            base: 0,
            limit: 0xffff_ffff,
            ar: ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::DB,
        };
    }
    if long {
        // The manual's order, minus the instructions that would have performed
        // it: `CR4.PAE`, `CR3`, `EFER.LME`, then `CR0.PG` — at which point the
        // processor sets `EFER.LMA`. Both bits are written here because this
        // builds the state rather than reaching it; `cpu::x86::tests` is where
        // the transition is executed as real instructions.
        sys.cr4 |= cr4::PAE;
        sys.cr3 = PML4;
        sys.efer |= efer::LME | efer::LMA;
        sys.cr0 |= cr0::PG;
    }
    sys
}

/// Identity-map the first four mebibytes with two 2 MiB pages.
///
/// Three tables and three entries: `PML4[0]` to the pointer table, `PDPT[0]` to
/// the directory, and the directory's first two entries as large pages. That
/// covers the program, the scratch word and the tables themselves, so a walk
/// of any address the guest touches succeeds and its accessed and dirty bits
/// land in memory the state hash covers.
fn map_identity(space: &Arc<rsemu::core::space::AddressSpace>) {
    // present | writable | user | accessed, and `PAGE_SIZE` on the leaves.
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

/// Run `quanta` quanta, hashing every `every`.
fn hashes(machine: &mut Machine, quanta: usize, every: usize) -> Vec<u64> {
    let mut out = Vec::new();
    for n in 1..=quanta {
        machine.run_quantum().expect("the machine advances");
        if n % every == 0 {
            out.push(
                machine
                    .state_hash()
                    .expect("a deterministic machine hashes"),
            );
        }
    }
    out
}

/// The three engines, on one world, at every checkpoint.
fn every_engine_agrees(world: World, quanta: usize, every: usize) {
    let (mut interp, _cpu) = board("interp", world, "hash.interp");
    let want = hashes(&mut interp, quanta, every);
    assert_eq!(
        want.len(),
        quanta / every,
        "the checkpoint arithmetic changed and the test stopped checking"
    );
    for engine in ["jit", "jit-host"] {
        let (mut board, cpu) = board(engine, world, &format!("hash.{engine}"));
        let got = hashes(&mut board, quanta, every);
        for (n, (want, got)) in want.iter().zip(&got).enumerate() {
            assert_eq!(
                want, got,
                "{world:?}, checkpoint {n}: `engine = \"interp\"` hashes to {want:#018x} and \
                 `engine = \"{engine}\"` to {got:#018x}. A cache hit, a cache miss, an \
                 interpreted run and a compiled run must be indistinguishable to the \
                 guest, including cycle counts (ROADMAP.md §0)"
            );
        }
        let stats = cpu.jit_stats().expect("a JIT core keeps statistics");
        assert!(
            stats.blocks > 0,
            "{world:?} under `engine = \"{engine}\"` executed no translated block at all, \
             so the comparison above compared two interpreters"
        );
        assert!(
            stats.retired > stats.interpreted,
            "{world:?} under `engine = \"{engine}\"` retired {} guest instructions in \
             blocks against {} interpreted, which is not a translated run",
            stats.retired,
            stats.interpreted,
        );
        if engine == "jit" {
            assert_eq!(
                stats.compiled, 0,
                "`engine = \"jit\"` is the portable backend and must compile nothing"
            );
        }
    }
}

#[test]
fn every_engine_hashes_to_the_same_machine_with_paging_off() {
    every_engine_agrees(World::Flat, 40, 4);
}

#[test]
fn every_engine_hashes_to_the_same_machine_in_long_mode() {
    every_engine_agrees(World::Long, 40, 4);
}

#[test]
fn the_engine_property_is_read_rather_than_accepted_and_ignored() {
    // The comparison above passes trivially if `engine` reaches nothing, which
    // is exactly the state this work found the crate in — every `cpu.x86` in
    // the tree accepted `engine` and threw it away. So the property has to be
    // shown to be *load-bearing*: a value nothing implements is refused, and
    // the two that are implemented reach different backends.
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_param("engine", "tier1");
    let err = build("x86-engines.bad", SOURCE, &registry, &options)
        .expect_err("an engine nothing implements must be refused, not ignored");
    let text = err.to_string();
    assert!(
        text.contains("engine") && text.contains("interp") && text.contains("jit"),
        "the refusal has to name the property and what it will take: {text}"
    );

    let (_m, plain) = board("jit", World::Long, "prop.jit");
    let (_m2, host) = board("jit-host", World::Long, "prop.host");
    for _ in 0..8 {
        plain.run_budget(10_000);
        host.run_budget(10_000);
    }
    let plain_compiled = plain.jit_stats().expect("statistics").compiled;
    let host_compiled = host.jit_stats().expect("statistics").compiled;
    assert_eq!(plain_compiled, 0, "the portable backend compiles nothing");
    if cfg!(all(
        feature = "jit-x86",
        target_os = "linux",
        target_arch = "x86_64"
    )) {
        assert!(
            host_compiled > 0,
            "`engine = \"jit-host\"` on a host with the code generator must run compiled \
             blocks, or the two JIT values are the same measurement twice"
        );
    }
    assert_eq!(
        plain.regs().rip,
        host.regs().rip,
        "the two JIT engines are the same guest and a different backend"
    );
    assert_eq!(plain.cycles(), host.cycles());
}

#[test]
fn a_snapshot_crosses_from_one_engine_to_the_other_and_carries_on_the_same() {
    // Phase 7's gate, minus the accelerator: a snapshot is a property of the
    // guest rather than of the engine that produced it, so it must move both
    // ways and the run afterwards must be the same run. The x86 chunk is the
    // one another agent is also standing on this round (`accel::state`), and
    // nothing in this work touched it — this is what says so.
    //
    // **The flat world rather than the paged one, and the reason is the core
    // rather than the engines.** `cpu::x86::exec::State::tlb` is derived state
    // and is deliberately not serialized (`ROADMAP.md` §4.5), so a restored
    // processor comes back with an empty translation buffer and its next few
    // accesses pay for walks the machine that took the snapshot had already
    // paid for. Those are guest-visible clocks, so a *paged* guest restored
    // from a snapshot legitimately runs on a slightly different tick schedule
    // from the one it was taken off — under either engine, identically, which
    // is exactly why measuring it here would be measuring something else. With
    // `CR0.PG` clear the buffer is never consulted at all (every caller of
    // `Exec::translate` checks `Sys::paging` first), so the comparison is
    // about the engines and nothing else.
    let (mut interp, _a) = board("interp", World::Flat, "snap.interp");
    let (mut jit, _b) = board("jit-host", World::Flat, "snap.jit");
    for _ in 0..12 {
        interp.run_quantum().expect("advances");
    }
    let taken = interp.save().expect("the machine snapshots");

    jit.load(&taken).expect("the JIT board takes it");
    assert_eq!(
        jit.state_hash().expect("hashes"),
        interp.state_hash().expect("hashes"),
        "restoring an interpreter's snapshot into a JIT board did not reproduce it"
    );

    let want = hashes(&mut interp, 20, 5);
    let got = hashes(&mut jit, 20, 5);
    assert_eq!(want, got, "the two diverged after the restore");

    // And the other direction, which is the one that finds derived state a
    // restore forgot to invalidate: the JIT board's block cache was filled
    // from the *old* RAM, and the snapshot it takes must carry none of it.
    let taken = jit.save().expect("the JIT board snapshots");
    let (mut fresh, _c) = board("interp", World::Flat, "snap.fresh");
    fresh.load(&taken).expect("the interpreter board takes it");
    assert_eq!(
        fresh.state_hash().expect("hashes"),
        jit.state_hash().expect("hashes"),
    );
    assert_eq!(hashes(&mut fresh, 20, 5), hashes(&mut jit, 20, 5));
}
