//! What a second core can see of a store that is still happening — and which
//! threading mode it has to be running in to see it.
//!
//! # The two claims this file separates
//!
//! `AddressSpace::bus_lock` makes a locked read-modify-write indivisible
//! against *another locked one*, and `buslock`'s own "What it does not make
//! atomic" says the residual out loud: a **plain** store by another observer
//! can still land inside the window. That residual is stated there as a lost
//! update, which is hard to catch — a lost update leaves a value that looks
//! like a legal one. It has a second and much sharper form, and this file is
//! the instrument for it.
//!
//! `RamStore` is a `Vec<AtomicU8>` and every access to it is a **byte loop**
//! (`space::store`, "Ordering is `Relaxed` throughout … the store provides
//! per-byte atomicity … and nothing more"). So a four-byte guest store is four
//! independent byte stores, and a four-byte guest load that overlaps it reads a
//! *mixture of the old and the new word*. All three architectures rsemu
//! emulates forbid that outright — *Intel SDM* volume 3 §9.1.1 ("the processor
//! guarantees that … reading or writing a doubleword aligned on a 32-bit
//! boundary" is carried out atomically), ARM DDI 0487 B2.2.1 (single-copy
//! atomicity for a naturally aligned access up to 64 bits), RISC-V
//! Unprivileged ISA §1.4 for the same. A guest can therefore observe a value
//! that never existed in memory.
//!
//! The two meet: a `LOCK XADD` whose read half comes back torn is the bus
//! lock's residual, caught in the act. The plain store did not merely land
//! *before* the locked write, it landed *between* the locked read's own bytes.
//!
//! # The answer to "which mode"
//!
//! **Neither is reachable under [`ThreadingMode::Deterministic`]**, and the
//! argument is structural rather than statistical: that mode runs every
//! runnable on one host thread, so the finest interleaving it can produce is
//! one whole instruction. Nothing at all executes between a locked
//! instruction's read and its write, and nothing executes between the bytes of
//! a plain store. `interleaved` below is that mode's *worst case* — a context
//! switch after every single instruction, finer than any quantum the scheduler
//! will ever hand out — and it tears zero times in sixty thousand tries while
//! demonstrably seeing the writer's other value thousands of times.
//!
//! The one exception, and it is written here because it is the only way the
//! claim can fail: a **lazily advanced device** is caught up from inside the
//! access that dispatches to it (`ROADMAP.md` §4.2), and `advance_to` is free
//! to touch its own bus. So a locked read-modify-write whose operand is *MMIO*
//! can have another master's write land inside it even on one thread. A locked
//! read-modify-write whose operand is **RAM** cannot: a RAM access dispatches
//! to no device, so there is no place for a catch-up to happen.
//!
//! Under [`ThreadingMode::Parallel`] both are reachable and this file catches
//! them. Under [`ThreadingMode::Accel`] the question does not arise: the host's
//! silicon performs the guest's accesses, so an accelerated SMP run is evidence
//! about the host and not about this tree.
//!
//! # Measured
//!
//! Sixty thousand loads racing sixty thousand alternating stores, debug build,
//! two host threads:
//!
//! | reader | torn reads | loads that saw the *other* value |
//! | --- | --- | --- |
//! | plain `mov eax, [W]` | 181, 361, 247 | ~17 000 |
//! | `lock xadd [W], eax` (adds zero) | 102, 138, 96 | ~13 000 |
//!
//! The second row is the bus lock's residual with a number on it. The bus was
//! taken sixty thousand times in that run — the lock was working, and a plain
//! store went through it anyway, because a plain store does not ask.
//!
//! # Why the concurrent halves assert so little
//!
//! How many a run tears is a property of the host's scheduler, exactly as
//! `x86_bus_lock.rs` says of its own control: on a machine that happens to run
//! the two threads one after the other it tears none. So the concurrent tests
//! assert only what is true on every host and print the count; the assertion
//! with teeth is the deterministic one, and it is the claim that matters —
//! **the mode whose state hash is a golden cannot observe either gap.**
//!
//! # `std::thread` here rather than `core::sync::Pool`
//!
//! The same call `x86_bus_lock.rs` and `tests/kvm_freedos.rs` make, for the
//! same reason: a test that exists to make two cores collide has to be able to
//! say what a thread is.
//!
//! [`ThreadingMode::Deterministic`]: rsemu::core::sched::ThreadingMode::Deterministic
//! [`ThreadingMode::Parallel`]: rsemu::core::sched::ThreadingMode::Parallel
//! [`ThreadingMode::Accel`]: rsemu::core::sched::ThreadingMode::Accel

#![cfg(all(feature = "cpu-x86", feature = "std"))]

use std::sync::Arc;

use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Config, Variant, X86};

/// The contended word. Four-byte aligned, so every architecture in the tree
/// promises an access to it is indivisible.
const W: u16 = 0x2000;
/// Where the reader counts values that are neither of the two the writer
/// stores.
const TORN: u16 = 0x2010;
/// Where it counts the times it saw zero — the writer's *other* value, and the
/// proof that the two programs really did overlap. Without it a run that tore
/// nothing would be indistinguishable from a run in which the reader never saw
/// the writer at all.
const WITNESS: u16 = 0x2014;

/// Iterations a side. Sixty thousand fits the sixteen bits `CX` and both
/// counters have.
const N: u16 = 60_000;

/// Where each program is placed.
const WRITER_AT: u64 = 0x0100;
const READER_AT: u64 = 0x0200;

/// The writer: `[W]` alternates between `0x00000000` and `0xffffffff`, with
/// plain 32-bit stores.
///
/// ```text
///   mov cx, N
/// top:
///   mov dword [0x2000], 0x00000000
///   mov dword [0x2000], 0xffffffff
///   loop top
///   hlt
/// ```
///
/// `C7 /0 iz` is `MOV Ev, Iz` and the ModRM byte `06` is the 16-bit
/// direct-address form, so the destination is `ds:0x2000` with `DS` zero; the
/// `66` prefix makes the operand — and so the store — four bytes wide.
fn writer() -> Vec<u8> {
    let mut c = vec![0xb9, (N & 0xff) as u8, (N >> 8) as u8];
    for value in [0x0000_0000u32, 0xffff_ffff] {
        c.extend_from_slice(&[0x66, 0xc7, 0x06, (W & 0xff) as u8, (W >> 8) as u8]);
        c.extend_from_slice(&value.to_le_bytes());
    }
    c.extend_from_slice(&[0xe2, 0xec]); // loop top
    c.push(0xf4); // hlt
    c
}

/// The reader: load `[W]`, and count anything that is neither of the writer's
/// two values.
///
/// ```text
///   mov cx, N
/// top:
///   mov eax, [0x2000]          ; or: xor eax, eax / lock xadd [0x2000], eax
///   cmp eax, 0
///   jne have
///   inc word [0x2014]          ; witnessed the other value
/// have:
///   cmp eax, 0
///   je next
///   cmp eax, -1
///   je next
///   inc word [0x2010]          ; a value that was never in memory
/// next:
///   loop top
///   hlt
/// ```
///
/// The locked form adds **zero**, so it writes back exactly what it read and
/// the word's contents stay the writer's. What it changes is that the read is
/// the read half of a read-modify-write the architecture requires to be
/// indivisible — and `AddressSpace::bus_lock` is held across it.
fn reader(locked: bool) -> Vec<u8> {
    let mut c = vec![0xb9, (N & 0xff) as u8, (N >> 8) as u8];
    if locked {
        // `xor eax, eax`, then `0F C1 /r` — `XADD Ev, Gv`, a 486 addition —
        // with the `F0` LOCK prefix.
        c.extend_from_slice(&[0x66, 0x31, 0xc0]);
        c.extend_from_slice(&[
            0xf0,
            0x66,
            0x0f,
            0xc1,
            0x06,
            (W & 0xff) as u8,
            (W >> 8) as u8,
        ]);
    } else {
        // `A1 moffs` — `MOV eAX, moffs`; the address size is still sixteen, so
        // the displacement is two bytes.
        c.extend_from_slice(&[0x66, 0xa1, (W & 0xff) as u8, (W >> 8) as u8]);
    }
    c.extend_from_slice(&[0x66, 0x83, 0xf8, 0x00]); // cmp eax, 0
    c.extend_from_slice(&[0x75, 0x04]); // jne have
    c.extend_from_slice(&[0xff, 0x06, (WITNESS & 0xff) as u8, (WITNESS >> 8) as u8]);
    c.extend_from_slice(&[0x66, 0x83, 0xf8, 0x00]); // have: cmp eax, 0
    c.extend_from_slice(&[0x74, 0x0a]); // je next
    c.extend_from_slice(&[0x66, 0x83, 0xf8, 0xff]); // cmp eax, -1
    c.extend_from_slice(&[0x74, 0x04]); // je next
    c.extend_from_slice(&[0xff, 0x06, (TORN & 0xff) as u8, (TORN >> 8) as u8]);
    // The backward displacement counts from the byte after the `loop`, and the
    // body's length depends on which read form was assembled.
    let back = -((c.len() + 2 - 3) as i64);
    c.extend_from_slice(&[0xe2, back as i8 as u8]);
    c.push(0xf4); // hlt
    c
}

/// One 32-bit space with four megabytes of RAM at zero, holding both programs.
fn space(locked: bool) -> Arc<AddressSpace> {
    let space = Arc::new(AddressSpace::new("mem", 32));
    space
        .topology()
        .map(Region::ram("ram", Arc::new(RamStore::new(0x40_0000))), 0)
        .expect("4 MiB at zero");
    space
        .write_bytes(WRITER_AT, &writer(), MemAttrs::DEFAULT)
        .expect("the writer lands");
    space
        .write_bytes(READER_AT, &reader(locked), MemAttrs::DEFAULT)
        .expect("the reader lands");
    space
}

/// A 486 attached to `space` and placed at `entry` in real mode.
///
/// The first `step` performs the power-on reset, which discards any register
/// file written before it — so it is spent here, and only then is the core
/// placed. Doing it the other way round leaves both cores wandering out of the
/// reset vector into whatever RAM holds, which is not a program either of them
/// was meant to run.
fn core(space: &Arc<AddressSpace>, entry: u64) -> Arc<X86> {
    let cpu = Arc::new(X86::new(Config::default().with_variant(Variant::I80486)));
    cpu.attach_space(Arc::clone(space));
    cpu.step();
    let mut regs = cpu.regs();
    regs.cs = 0;
    regs.ds = 0;
    regs.es = 0;
    regs.ss = 0;
    regs.rip = entry;
    cpu.set_regs(regs);
    cpu
}

/// What a run leaves behind.
#[derive(Debug)]
struct Outcome {
    /// Loads that returned a value the writer never stored.
    torn: u64,
    /// Loads that saw the writer's zero — the proof the two overlapped.
    witnessed: u64,
    /// How many locked transactions the space served.
    bus: u64,
}

fn outcome(space: &Arc<AddressSpace>) -> Outcome {
    let at = |a: u16| {
        space
            .read(u64::from(a), Width::U16, MemAttrs::DEFAULT)
            .expect("the counter reads back")
    };
    Outcome {
        torn: at(TORN),
        witnessed: at(WITNESS),
        bus: space.bus_lock().taken(),
    }
}

/// A generous step ceiling: the reader is nine instructions an iteration.
fn ceiling() -> u64 {
    u64::from(N) * 16 + 64
}

/// Run both cores on **one** host thread, one instruction at a time, choosing
/// which of them runs next from a fixed pseudo-random sequence.
///
/// This is `ThreadingMode::Deterministic` with its quantum set to one — finer
/// than the scheduler will ever cut it, and therefore the strongest form of the
/// claim. Whatever this cannot produce, that mode cannot produce.
///
/// The choice is randomised rather than strictly alternating because strict
/// alternation *phase-locks*: the writer's loop is three instructions and the
/// locked reader's is ten, and a fixed one-for-one schedule sampled the same
/// point of the writer's loop every time, so the reader never once saw the
/// writer's zero and the run proved nothing. The generator is a plain LCG with
/// a fixed seed, so the schedule is the same on every host and in every build —
/// this stays a deterministic test of the deterministic mode.
fn interleaved(space: &Arc<AddressSpace>) -> Outcome {
    let cores = [core(space, WRITER_AT), core(space, READER_AT)];
    let mut live = [true, true];
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut steps = 0u64;
    while live[0] || live[1] {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let which = ((seed >> 33) & 1) as usize;
        let which = if live[which] { which } else { 1 - which };
        if cores[which].step() == 0 {
            live[which] = false;
        }
        steps += 1;
        assert!(steps < ceiling() * 2, "a core never reached its `hlt`");
    }
    outcome(space)
}

/// Run both cores on two host threads: the shape `ThreadingMode::Parallel`
/// gives them, and the shape `x86_bus_lock.rs` already uses.
fn concurrent(space: &Arc<AddressSpace>) -> Outcome {
    let cores = [core(space, WRITER_AT), core(space, READER_AT)];
    std::thread::scope(|s| {
        for cpu in &cores {
            let cpu = Arc::clone(cpu);
            s.spawn(move || {
                for _ in 0..ceiling() {
                    if cpu.step() == 0 {
                        return;
                    }
                }
                panic!("a core never reached its `hlt`");
            });
        }
    });
    outcome(space)
}

/// The gate. One thread cannot tear a store, however finely it interleaves.
#[test]
fn instruction_boundary_interleaving_never_tears_a_plain_load() {
    let space = space(false);
    let out = interleaved(&space);
    println!(
        "interleaved plain: {} torn of {N} loads, {} witnessed the other value",
        out.torn, out.witnessed
    );
    assert!(
        out.witnessed > 0,
        "the reader never saw the writer at all, so the run proves nothing"
    );
    assert_eq!(
        out.torn, 0,
        "one host thread produced a value that was never in memory, which \
         would mean something interleaves inside an instruction"
    );
    assert_eq!(out.bus, 0, "no `LOCK` prefix, no bus lock");
}

/// The same, for the locked read — the case `buslock`'s residual is about.
#[test]
fn instruction_boundary_interleaving_never_tears_a_locked_read() {
    let space = space(true);
    let out = interleaved(&space);
    println!(
        "interleaved locked: {} torn of {N} locked reads, {} witnessed the other value",
        out.torn, out.witnessed
    );
    assert!(out.witnessed > 0, "the two programs must overlap");
    assert_eq!(out.torn, 0, "nothing runs inside a locked instruction here");
    assert_eq!(
        out.bus,
        u64::from(N),
        "and every one of those reads took the bus"
    );
}

/// Two host threads, plain against plain: single-copy atomicity is not kept.
///
/// The count is printed rather than asserted, for the reason the module
/// documentation gives.
#[test]
fn concurrent_cores_may_tear_a_plain_aligned_load() {
    let space = space(false);
    let out = concurrent(&space);
    println!(
        "plain: {} torn of {N} loads, {} witnessed the other value",
        out.torn, out.witnessed
    );
    assert!(
        out.torn <= u64::from(N),
        "a load cannot tear more often than it was executed"
    );
    assert_eq!(out.bus, 0, "and none of it took the bus");
}

/// Two host threads, plain against **locked**: the bus lock's residual, caught.
///
/// The lock was held — `bus` says so — and a plain store landed inside the
/// window anyway, because a plain store does not ask for it. Closing this would
/// mean every store in the machine taking the bus lock, which is the cost the
/// design exists to avoid; `buslock`'s "What it does not make atomic" has the
/// argument and now has this file's numbers behind it.
#[test]
fn concurrent_cores_may_tear_the_read_half_of_a_locked_instruction() {
    let space = space(true);
    let out = concurrent(&space);
    println!(
        "locked: {} torn of {N} locked reads, {} witnessed the other value",
        out.torn, out.witnessed
    );
    assert!(
        out.torn <= u64::from(N),
        "a read cannot tear more often than it was executed"
    );
    assert_eq!(
        out.bus,
        u64::from(N),
        "the bus really was taken for every one of them"
    );
}
