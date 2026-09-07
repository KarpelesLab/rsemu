//! Whether an A64 atomic read-modify-write is indivisible against a sibling
//! core's — measured as updates that go missing.
//!
//! # The claim
//!
//! DDI 0487 B2.2.1 and B2.9: a `FEAT_LSE` atomic (`LDADD`, `SWP`, `CAS` and
//! the rest) is one single-copy-atomic access, and a `STXR` succeeds only if
//! nothing has written the reservation granule since the `LDXR`. Being
//! indivisible is the entire reason a guest reaches for either of them — a
//! spinlock, a refcount and a futex are exactly these two shapes.
//!
//! An emulator that issues the read and the write as separate bus accesses
//! keeps neither promise unless something holds the two together. Two cores
//! incrementing one word then interleave:
//!
//! ```text
//! core 0                        core 1
//! ldadd: read  [x0] -> 5
//!                               ldadd: read  [x0] -> 5
//! ldadd: write [x0] <- 6
//!                               ldadd: write [x0] <- 6
//! ```
//!
//! and one increment is simply gone.
//!
//! # Why this one is catchable where an ordering bug is not
//!
//! Most of the memory-model work in this tree cannot be gated by a test on an
//! x86-64 host, and `memory_model_litmus.rs` and `cpu::arm::a64::exec`'s
//! `host_fence` both say so: a host strong enough to hide the reordering hides
//! it before *and* after the fix. **A lost update is not a reordering.** It is
//! a value that no ordering of the two programs could have produced, so any
//! host that runs the two threads at once shows it, and the assertion can be an
//! equality rather than a printed count.
//!
//! That is why both counters are here. The atomic one is asserted exactly; the
//! plain one — three instructions, a load, an add and a store, on a word the
//! architecture promises nothing about — is *printed*, and it is the witness
//! that the two programs really did overlap. A run where the plain counter
//! reaches its full `2N` is a run where the host scheduler never let the two
//! threads collide, and it proves nothing whichever way the atomic counter
//! came out.
//!
//! # Measured, debug build, two host threads
//!
//! | | lost of 120 000 | runs that lost any |
//! | --- | --- | --- |
//! | `STADD`, before `Exec::lock_bus` | 3 410, 8 497, 8 850 | every one |
//! | `LDXR`/`STXR`, before it | 318 | most |
//! | `LDXR`/`STXR`, with the bus lock on `STXR` but the reservation taken after the read | 46 | most |
//! | `LDXR`/`STXR`, with both of those and the load-exclusive still unlocked | 1 to 3 | **33 of 60** |
//! | any of them, as the tree stands | 0 | 0 of 126 |
//!
//! Four separate windows, and the counts are worth keeping apart because they
//! measure how wide each one is. The `FEAT_LSE` window is a whole
//! read-modify-write. The `STXR` window is one monitor check and one store —
//! narrower, and two orders of magnitude rarer. The third is narrower still: a
//! sibling's store landing between a load-exclusive's *read* and the moment it
//! claims the granule, where the monitor cannot see it because the slot is not
//! live yet. The fourth is a few host instructions wide — the gap inside a
//! committing `STXR` between telling the monitor and writing the bytes, which
//! `Exec::exclusive`'s load arm draws — and it is the one this file exists to
//! keep closed.
//!
//! **Read that last row as a failure *rate*, not a count.** One lost update in
//! 120 000 is invisible to a single run and to three; it took twenty-four to
//! see it and sixty to price it, and it appeared only while the host was busy,
//! because what widens the window is a preemption inside it. A green run of
//! this file proves nothing on its own. If it ever fails again, the number to
//! report is how many runs of how many, not that it failed.
//!
//! # Which threading mode
//!
//! `ThreadingMode::Parallel`, and the same structural argument
//! `smp_single_copy_atomicity.rs` makes applies here: `Deterministic` runs
//! every runnable on one host thread, so its finest interleaving is one whole
//! instruction and nothing at all executes between an instruction's read and
//! its write. [`instruction_boundary_interleaving_never_loses_an_lse_update`]
//! is that mode's worst case — a context switch after every single instruction
//! — and it passed before any of this was fixed. It is here as the claim, not
//! as the gate. No machine file selects `parallel`; `--threading parallel`
//! does.
//!
//! # Two details of the layout that are deliberate
//!
//! The two counters share one 16-byte reservation granule, so every plain
//! store breaks whatever reservation is outstanding and the `LDXR`/`STXR` loop
//! really does go round again — 7 000-odd times in 120 000 here. A test whose
//! store-conditional always succeeds is not testing the retry path.
//!
//! The two cores run *the same program* from two different pages. Nothing in
//! the tree makes a core's behaviour depend on where its code sits, and it
//! halves what has to be read to see what the two are doing.
//!
//! # `std::thread` rather than `core::sync::Pool`
//!
//! The same call `smp_single_copy_atomicity.rs` and `x86_bus_lock.rs` make: a
//! test that exists to make two cores collide has to be able to say what a
//! thread is.

#![cfg(all(feature = "cpu-arm-a64", feature = "std"))]

use std::sync::Arc;

use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
use rsemu::core::value::Width;
use rsemu::cpu::arm::a64::{Config, Cpu};

/// The word both cores increment atomically.
const WORD: u64 = 0x8000;
/// The word both cores increment with a plain load, add and store — in the
/// same reservation granule, deliberately.
const PLAIN: u64 = 0x8008;

/// Iterations a core.
const N: u64 = 60_000;

/// Where each core's copy of the loop starts.
const FIRST_AT: u64 = 0x0000;
const SECOND_AT: u64 = 0x1000;

const fn movz(rd: u32, imm: u32) -> u32 {
    0xd280_0000 | ((imm & 0xffff) << 5) | rd
}

const fn b_ne(back: i32) -> u32 {
    0x5400_0000 | (((back as u32) & 0x7_ffff) << 5) | 1
}

const fn cbnz_w(rt: u32, back: i32) -> u32 {
    0x3500_0000 | (((back as u32) & 0x7_ffff) << 5) | rt
}

/// `LDADD X1, XZR, [X0]` — the `STADD` spelling, which discards the old value.
const STADD: u32 = 0xf821_001f;
/// `LDXR X5, [X0]`.
const LDXR: u32 = 0xc840_7c05;
/// `STXR W6, X5, [X0]`.
const STXR: u32 = 0xc806_7c05;
/// `subs x2, x2, #1`.
const SUBS: u32 = 0xf100_0442;
/// `b .` — where a finished core parks.
const PARK: u32 = 0x1400_0000;

/// The prologue both programs share: the two addresses, the addend and the
/// iteration count.
fn prologue() -> [u32; 4] {
    [
        movz(0, WORD as u32),
        movz(3, PLAIN as u32),
        movz(1, 1),
        movz(2, N as u32),
    ]
}

/// The plain counter: three instructions, so a sibling has somewhere to land,
/// and what it does when it lands there is this file's witness.
const PLAIN_BODY: [u32; 3] = [
    0xf940_0064, // ldr x4, [x3]
    0x9100_0484, // add x4, x4, #1
    0xf900_0064, // str x4, [x3]
];

/// The `FEAT_LSE` program.
///
/// ```text
///   <prologue>
/// top:
///   ldr   x4, [x3]
///   add   x4, x4, #1
///   str   x4, [x3]
///   stadd x1, [x0]
///   subs  x2, x2, #1
///   b.ne  top
///   b .
/// ```
fn lse_program() -> Vec<u32> {
    let mut c = prologue().to_vec();
    c.extend_from_slice(&PLAIN_BODY);
    c.push(STADD);
    c.push(SUBS);
    c.push(b_ne(-5));
    c.push(PARK);
    c
}

/// The same increment in the Armv8.0 spelling: the exclusive pair, with the
/// retry loop it is defined around.
///
/// ```text
///   <prologue>
/// top:
///   ldr   x4, [x3]
///   add   x4, x4, #1
///   str   x4, [x3]
/// retry:
///   ldxr  x5, [x0]
///   add   x5, x5, #1
///   stxr  w6, x5, [x0]
///   cbnz  w6, retry
///   subs  x2, x2, #1
///   b.ne  top
///   b .
/// ```
fn llsc_program() -> Vec<u32> {
    let mut c = prologue().to_vec();
    c.extend_from_slice(&PLAIN_BODY);
    c.push(LDXR);
    c.push(0x9100_04a5); // add x5, x5, #1
    c.push(STXR);
    c.push(cbnz_w(6, -3));
    c.push(SUBS);
    c.push(b_ne(-8));
    c.push(PARK);
    c
}

/// One 64-bit space with a megabyte of RAM at zero, holding both copies of
/// `words`.
fn space(words: &[u32]) -> Arc<AddressSpace> {
    let space = Arc::new(AddressSpace::new("mem", 64));
    space
        .topology()
        .map(Region::ram("ram", Arc::new(RamStore::new(0x10_0000))), 0)
        .expect("1 MiB at zero");
    let mut bytes = Vec::new();
    for word in words {
        bytes.extend_from_slice(&word.to_le_bytes());
    }
    for at in [FIRST_AT, SECOND_AT] {
        space
            .write_bytes(at, &bytes, MemAttrs::DEFAULT)
            .expect("the program lands");
    }
    space
}

/// A Neoverse N1 — `FEAT_LSE` is why, and a Cortex-A53 would take `UNDEFINED`
/// on the `STADD` — entered at `entry`.
fn core(space: &Arc<AddressSpace>, entry: u64) -> Arc<Cpu> {
    let cpu = Arc::new(Cpu::new(Config::neoverse_n1().with_reset_vector(entry)));
    cpu.attach_space(Arc::clone(space));
    cpu
}

/// Where a finished core parks: the last instruction of the program.
fn park(words: &[u32], entry: u64) -> u64 {
    entry + 4 * (words.len() as u64 - 1)
}

/// A step ceiling generous enough for any amount of `STXR` retrying.
fn ceiling(words: &[u32]) -> u64 {
    words.len() as u64 * N * 4
}

#[derive(Debug)]
struct Outcome {
    /// What the atomically incremented word holds. Anything but `2 * N` is a
    /// lost update.
    atomic: u64,
    /// What the plainly incremented word holds. Anything *below* `2 * N` is
    /// the proof that the two programs overlapped.
    plain: u64,
    /// How many locked transactions the space served.
    bus: u64,
}

fn outcome(space: &Arc<AddressSpace>) -> Outcome {
    let at = |a: u64| {
        space
            .read(a, Width::U64, MemAttrs::DEFAULT)
            .expect("the counter reads back")
    };
    Outcome {
        atomic: at(WORD),
        plain: at(PLAIN),
        bus: space.bus_lock().taken(),
    }
}

/// Both cores on **one** host thread, one instruction at a time, choosing
/// which of them runs next from a fixed pseudo-random sequence.
///
/// `ThreadingMode::Deterministic` with its quantum set to one. The choice is
/// randomised rather than strictly alternating for the reason
/// `smp_single_copy_atomicity.rs` gives: two loops of similar length
/// phase-lock under strict alternation, and a phase-locked schedule can sample
/// the same point of the other program every time and witness nothing. The
/// generator is a plain LCG with a fixed seed, so the schedule is identical on
/// every host and in every build.
fn interleaved(words: &[u32]) -> Outcome {
    let space = space(words);
    let cores = [core(&space, FIRST_AT), core(&space, SECOND_AT)];
    let parks = [park(words, FIRST_AT), park(words, SECOND_AT)];
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    for _ in 0..2 * ceiling(words) {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let live = [cores[0].pc() != parks[0], cores[1].pc() != parks[1]];
        if !live[0] && !live[1] {
            break;
        }
        let which = ((seed >> 33) & 1) as usize;
        let which = if live[which] { which } else { 1 - which };
        cores[which].step();
    }
    for (cpu, at) in cores.iter().zip(parks) {
        assert_eq!(cpu.pc(), at, "a core never finished its loop");
    }
    outcome(&space)
}

/// Both cores on two host threads: the shape `ThreadingMode::Parallel` gives
/// them.
fn concurrent(words: &[u32]) -> Outcome {
    let space = space(words);
    let cores = [core(&space, FIRST_AT), core(&space, SECOND_AT)];
    let parks = [park(words, FIRST_AT), park(words, SECOND_AT)];
    let ceiling = ceiling(words);
    std::thread::scope(|s| {
        for (cpu, at) in cores.iter().zip(parks) {
            let cpu = Arc::clone(cpu);
            s.spawn(move || {
                for _ in 0..ceiling {
                    if cpu.pc() == at {
                        return;
                    }
                    cpu.step();
                }
                panic!("a core never finished its loop");
            });
        }
    });
    outcome(&space)
}

fn report(what: &str, out: &Outcome) {
    println!(
        "{what}: atomic {} of {}, plain {} of {}, bus taken {}",
        out.atomic,
        2 * N,
        out.plain,
        2 * N,
        out.bus
    );
}

/// The structural claim for `FEAT_LSE`: one host thread cannot interleave
/// inside an instruction, however finely it cuts.
///
/// This passed before any of the fixes and is not the gate. What it rules out
/// is the possibility that the deterministic mode — the one whose state hash
/// is a golden — was ever exposed to this.
#[test]
fn instruction_boundary_interleaving_never_loses_an_lse_update() {
    let out = interleaved(&lse_program());
    report("interleaved lse", &out);
    assert!(
        out.plain < 2 * N,
        "the two programs never interleaved, so the run proves nothing"
    );
    assert_eq!(out.atomic, 2 * N, "an update was lost on one host thread");
}

/// The gate. `Exec::lock_bus` is what holds the read and the write of a
/// `FEAT_LSE` atomic together; without it this loses thousands.
#[test]
fn concurrent_cores_never_lose_an_lse_update() {
    let out = concurrent(&lse_program());
    report("concurrent lse", &out);
    assert_eq!(
        out.bus,
        2 * N,
        "every `STADD` should have taken the bus lock"
    );
    assert_eq!(out.atomic, 2 * N, "a sibling core's LSE atomic was lost");
}

/// The same structural claim for the exclusive pair.
#[test]
fn instruction_boundary_interleaving_never_loses_an_exclusive_update() {
    let out = interleaved(&llsc_program());
    report("interleaved ll/sc", &out);
    assert!(
        out.plain < 2 * N,
        "the two programs never interleaved, so the run proves nothing"
    );
    assert_eq!(out.atomic, 2 * N, "an update was lost on one host thread");
}

/// The second gate, and it is three claims at once: a `STXR`'s monitor check
/// and its store are one transaction, a `LDXR`'s claim and its read are one
/// transaction (both the bus lock), and the claim comes before the read rather
/// than after it (`Exec::reserve_then_read`). Removing any one of the three
/// puts lost updates back, in descending order of how often — see the table at
/// the top, and note that the last one costs a single update in 120 000 and
/// needs dozens of runs to see.
#[test]
fn concurrent_cores_never_lose_an_exclusive_update() {
    let out = concurrent(&llsc_program());
    report("concurrent ll/sc", &out);
    assert!(
        out.bus > 4 * N,
        "both halves of every pair take the bus, and a contended run retries \
         some of the store-conditionals on top"
    );
    assert_eq!(
        out.atomic,
        2 * N,
        "two store-exclusives on one granule both succeeded"
    );
}
