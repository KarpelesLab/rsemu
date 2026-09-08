//! Whether a RISC-V `A` extension read-modify-write is indivisible against a
//! sibling hart's — measured as updates that go missing.
//!
//! # The claim
//!
//! *The RISC-V Instruction Set Manual, Volume I*, "A" Standard Extension:
//!
//! * **AMOs.** "These AMO instructions atomically load a data value from the
//!   address in `rs1`, place the value into register `rd`, apply a binary
//!   operator to the loaded value and the original value in `rs2`, then store
//!   the result back to the address in `rs1`." *Atomically* is the whole word:
//!   an AMO has no status register and the guest has no retry loop, so an AMO
//!   that is interleaved with does not fail — it returns and stores a wrong
//!   answer.
//! * **`LR`/`SC`.** RVWMO's **Atomicity Axiom**: if `r` and `w` are the paired
//!   load and store of an aligned `LR`/`SC` in hart *h*, `s` is a store to byte
//!   *x* whose value `r` returns, then `s` precedes `w` in the global memory
//!   order and **no store from a hart other than *h* to byte *x* lies between
//!   them**. Two harts whose store-conditionals both succeed against one word
//!   is exactly the forbidden shape.
//!
//! An emulator that issues each of those as separate bus accesses keeps
//! neither promise unless something holds the accesses together. Two harts
//! incrementing one word then interleave:
//!
//! ```text
//! hart 0                          hart 1
//! amoadd.w: read  [a0] -> 5
//!                                 amoadd.w: read  [a0] -> 5
//! amoadd.w: write [a0] <- 6
//!                                 amoadd.w: write [a0] <- 6
//! ```
//!
//! and one increment is simply gone.
//!
//! # Why this one is catchable where an ordering bug is not
//!
//! Most of the memory-model work in this tree cannot be gated by a test on an
//! x86-64 host — `memory_model_litmus.rs` says so at length: a host strong
//! enough to hide a reordering hides it before *and* after the fix. **A lost
//! update is not a reordering.** It is a value no ordering of the two programs
//! could have produced, so any host that runs the two threads at once shows
//! it, and the assertion can be an equality rather than a printed count.
//!
//! That is why both counters are here. The atomic one is asserted exactly; the
//! plain one — a `lw`, an `addi` and an `sw` on a word the architecture
//! promises nothing about — is *printed*, and it is the witness that the two
//! programs really did overlap. A run where the plain counter reaches its full
//! `2N` is a run where the host scheduler never let the two harts collide, and
//! it proves nothing whichever way the atomic counter came out.
//!
//! # Measured, debug build, two host threads
//!
//! The numbers, the method and the failure *rate* are in
//! `docs/platforms/riscv-virt.md`, "The reservation set was necessary and not
//! sufficient". **Read them as a rate, not a count.** A defect worth one
//! update in 120 000 is invisible to a single run and to three; the AArch64
//! twin of this file took twenty-four runs to see its narrowest window and
//! sixty to price it, and it appeared only while the host was busy, because
//! what widens the window is a preemption inside it. A green run of this file
//! proves nothing on its own. If it ever fails again, the number to report is
//! how many runs of how many.
//!
//! # Which threading mode
//!
//! `ThreadingMode::Parallel`, and the same structural argument
//! `smp_single_copy_atomicity.rs` and `a64_lse_atomicity.rs` make applies:
//! `Deterministic` runs every runnable on one host thread, so its finest
//! interleaving is one whole instruction and nothing at all executes between
//! an instruction's read and its write.
//! [`instruction_boundary_interleaving_never_loses_an_amo_update`] is that
//! mode's worst case — a context switch after every single instruction — and
//! it passed before any of this was fixed. It is here as the claim, not as the
//! gate. No machine file selects `parallel`; `--threading parallel` does.
//!
//! # Two details of the layout that are deliberate
//!
//! The counters are 32-bit and adjacent, so they share one eight-byte
//! reservation granule (`cpu::riscv`'s `RESERVATION_SHIFT`) and every plain
//! store breaks whatever reservation is outstanding — the `lr`/`sc` loop
//! really does go round again. A test whose store-conditional always succeeds
//! is not testing the retry path.
//!
//! The two harts run *the same program* from two different pages. Nothing in
//! the tree makes a hart's behaviour depend on where its code sits, and it
//! halves what has to be read to see what the two are doing.
//!
//! # `std::thread` rather than `core::sync::Pool`
//!
//! The same call `smp_single_copy_atomicity.rs`, `x86_bus_lock.rs` and
//! `a64_lse_atomicity.rs` make: a test that exists to make two harts collide
//! has to be able to say what a thread is.

#![cfg(all(feature = "cpu-riscv", feature = "std"))]

use std::sync::Arc;

use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
use rsemu::core::value::Width;
use rsemu::cpu::riscv::{Config, Hart};

/// The word both harts increment atomically.
const WORD: u64 = 0x600;
/// The word both harts increment with a plain load, add and store — in the
/// same reservation granule, deliberately.
const PLAIN: u64 = 0x604;

/// Iterations a hart.
const N: u64 = 60_000;

/// Where each hart's copy of the loop starts.
const FIRST_AT: u64 = 0x0000;
const SECOND_AT: u64 = 0x1000;

// Register numbers, under the ABI names the listings use.
const A0: u32 = 10;
const A1: u32 = 11;
const A2: u32 = 12;
const A3: u32 = 13;
const A4: u32 = 14;
const A5: u32 = 15;
const A6: u32 = 16;

/// `addi rd, rs1, imm` — and, with `rs1 == x0`, the `li` of a small constant.
const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    (((imm as u32) & 0xfff) << 20) | (rs1 << 15) | (rd << 7) | 0x13
}

/// `lui rd, imm20`.
const fn lui(rd: u32, imm: u32) -> u32 {
    ((imm & 0xf_ffff) << 12) | (rd << 7) | 0x37
}

/// `li rd, value` for a 32-bit signed constant: `lui` plus `addi`, with the
/// upper half rounded up so the sign-extended low twelve bits land right.
fn li(rd: u32, value: i32) -> Vec<u32> {
    let hi = ((value as u32).wrapping_add(0x800) >> 12) & 0xf_ffff;
    let lo = value.wrapping_sub((hi << 12) as i32);
    if hi == 0 {
        return vec![addi(rd, 0, lo)];
    }
    vec![lui(rd, hi), addi(rd, rd, lo)]
}

/// `lw rd, off(rs1)`.
const fn lw(rd: u32, rs1: u32, off: i32) -> u32 {
    (((off as u32) & 0xfff) << 20) | (rs1 << 15) | (0b010 << 12) | (rd << 7) | 0x03
}

/// `sw rs2, off(rs1)`.
const fn sw(rs2: u32, rs1: u32, off: i32) -> u32 {
    let imm = (off as u32) & 0xfff;
    ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (0b010 << 12) | ((imm & 0x1f) << 7) | 0x23
}

/// `bne rs1, rs2, off` — `off` in bytes, relative to this instruction.
const fn bne(rs1: u32, rs2: u32, off: i32) -> u32 {
    let imm = (off as u32) & 0x1fff;
    (((imm >> 12) & 1) << 31)
        | (((imm >> 5) & 0x3f) << 25)
        | (rs2 << 20)
        | (rs1 << 15)
        | (0b001 << 12)
        | (((imm >> 1) & 0xf) << 8)
        | (((imm >> 11) & 1) << 7)
        | 0x63
}

/// One word-width `A` extension encoding: `funct5`, `rs2`, `rs1`, `rd`, with
/// `aq` and `rl` both clear.
const fn amo_w(funct5: u32, rs2: u32, rs1: u32, rd: u32) -> u32 {
    (funct5 << 27) | (rs2 << 20) | (rs1 << 15) | (0b010 << 12) | (rd << 7) | 0x2f
}

/// `amoadd.w x0, a1, (a0)` — the old value discarded, which is how a guest
/// spells "increment this word and tell me nothing".
const AMOADD: u32 = amo_w(0b00000, A1, A0, 0);
/// `lr.w a5, (a0)`.
const LR: u32 = amo_w(0b00010, 0, A0, A5);
/// `sc.w a6, a5, (a0)`.
const SC: u32 = amo_w(0b00011, A5, A0, A6);
/// `jal x0, 0` — where a finished hart parks.
const PARK: u32 = 0x0000_006f;

/// The prologue both programs share: the two addresses, the addend and the
/// iteration count.
fn prologue() -> Vec<u32> {
    let mut c = li(A0, WORD as i32);
    c.extend(li(A3, PLAIN as i32));
    c.extend(li(A1, 1));
    c.extend(li(A2, N as i32));
    c
}

/// The plain counter: three instructions, so a sibling has somewhere to land,
/// and what it does when it lands there is this file's witness.
fn plain_body() -> [u32; 3] {
    [lw(A4, A3, 0), addi(A4, A4, 1), sw(A4, A3, 0)]
}

/// The AMO program.
///
/// ```text
///   <prologue>
/// top:
///   lw       a4, 0(a3)
///   addi     a4, a4, 1
///   sw       a4, 0(a3)
///   amoadd.w x0, a1, (a0)
///   addi     a2, a2, -1
///   bne      a2, x0, top
///   jal      x0, 0
/// ```
fn amo_program() -> Vec<u32> {
    let mut c = prologue();
    c.extend_from_slice(&plain_body());
    c.push(AMOADD);
    c.push(addi(A2, A2, -1));
    c.push(bne(A2, 0, -5 * 4));
    c.push(PARK);
    c
}

/// The same increment built out of the pair, with the retry loop it is defined
/// around.
///
/// ```text
///   <prologue>
/// top:
///   lw   a4, 0(a3)
///   addi a4, a4, 1
///   sw   a4, 0(a3)
/// retry:
///   lr.w a5, (a0)
///   addi a5, a5, 1
///   sc.w a6, a5, (a0)
///   bne  a6, x0, retry
///   addi a2, a2, -1
///   bne  a2, x0, top
///   jal  x0, 0
/// ```
fn llsc_program() -> Vec<u32> {
    let mut c = prologue();
    c.extend_from_slice(&plain_body());
    c.push(LR);
    c.push(addi(A5, A5, 1));
    c.push(SC);
    c.push(bne(A6, 0, -3 * 4));
    c.push(addi(A2, A2, -1));
    c.push(bne(A2, 0, -8 * 4));
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

/// An RV64GC hart entered at `entry`.
fn hart(space: &Arc<AddressSpace>, entry: u64, id: u64) -> Arc<Hart> {
    let hart = Arc::new(Hart::new(
        Config::rv64gc().with_reset_vector(entry).with_hartid(id),
    ));
    hart.attach_space(Arc::clone(space));
    hart
}

/// Where a finished hart parks: the last instruction of the program.
fn park(words: &[u32], entry: u64) -> u64 {
    entry + 4 * (words.len() as u64 - 1)
}

/// A step ceiling generous enough for any amount of `sc` retrying.
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
            .read(a, Width::U32, MemAttrs::DEFAULT)
            .expect("the counter reads back")
    };
    Outcome {
        atomic: at(WORD),
        plain: at(PLAIN),
        bus: space.bus_lock().taken(),
    }
}

/// Both harts on **one** host thread, one instruction at a time, choosing
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
    let harts = [hart(&space, FIRST_AT, 0), hart(&space, SECOND_AT, 1)];
    let parks = [park(words, FIRST_AT), park(words, SECOND_AT)];
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    for _ in 0..2 * ceiling(words) {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let live = [harts[0].pc() != parks[0], harts[1].pc() != parks[1]];
        if !live[0] && !live[1] {
            break;
        }
        let which = ((seed >> 33) & 1) as usize;
        let which = if live[which] { which } else { 1 - which };
        harts[which].step();
    }
    for (hart, at) in harts.iter().zip(parks) {
        assert_eq!(hart.pc(), at, "a hart never finished its loop");
    }
    outcome(&space)
}

/// Both harts on two host threads: the shape `ThreadingMode::Parallel` gives
/// them.
fn concurrent(words: &[u32]) -> Outcome {
    let space = space(words);
    let harts = [hart(&space, FIRST_AT, 0), hart(&space, SECOND_AT, 1)];
    let parks = [park(words, FIRST_AT), park(words, SECOND_AT)];
    let ceiling = ceiling(words);
    std::thread::scope(|s| {
        for (hart, at) in harts.iter().zip(parks) {
            let hart = Arc::clone(hart);
            s.spawn(move || {
                for _ in 0..ceiling {
                    if hart.pc() == at {
                        return;
                    }
                    hart.step();
                }
                panic!("a hart never finished its loop");
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

/// The structural claim for the AMOs: one host thread cannot interleave inside
/// an instruction, however finely it cuts.
///
/// This passed before any of the fixes and is not the gate. What it rules out
/// is the possibility that the deterministic mode — the one whose state hash
/// is a golden — was ever exposed to this.
#[test]
fn instruction_boundary_interleaving_never_loses_an_amo_update() {
    let out = interleaved(&amo_program());
    report("interleaved amo", &out);
    assert!(
        out.plain < 2 * N,
        "the two programs never interleaved, so the run proves nothing"
    );
    assert_eq!(out.atomic, 2 * N, "an update was lost on one host thread");
}

/// The gate. `Exec::lock_bus` is what holds the read and the write of an AMO
/// together; without it this loses thousands.
#[test]
fn concurrent_harts_never_lose_an_amo_update() {
    let out = concurrent(&amo_program());
    report("concurrent amo", &out);
    assert_eq!(
        out.bus,
        2 * N,
        "every `amoadd.w` should have taken the bus lock"
    );
    assert_eq!(out.atomic, 2 * N, "a sibling hart's AMO was lost");
}

/// The same structural claim for the pair.
#[test]
fn instruction_boundary_interleaving_never_loses_a_reserved_update() {
    let out = interleaved(&llsc_program());
    report("interleaved lr/sc", &out);
    assert!(
        out.plain < 2 * N,
        "the two programs never interleaved, so the run proves nothing"
    );
    assert_eq!(out.atomic, 2 * N, "an update was lost on one host thread");
}

/// The second gate, and it is three claims at once: an `sc`'s reservation
/// check and its store are one transaction, an `lr`'s claim and its read are
/// one transaction (both the bus lock), and the claim comes before the read
/// rather than after it (`Exec::reserve_then_read`). Removing any one of the
/// three puts lost updates back, in descending order of how often — see the
/// table `docs/platforms/riscv-virt.md` carries, and note that the last one
/// costs a single-digit number of updates in 120 000 and needs dozens of runs
/// to see.
#[test]
fn concurrent_harts_never_lose_a_reserved_update() {
    let out = concurrent(&llsc_program());
    report("concurrent lr/sc", &out);
    assert!(
        out.bus > 4 * N,
        "both halves of every pair take the bus, and a contended run retries \
         some of the store-conditionals on top"
    );
    assert_eq!(
        out.atomic,
        2 * N,
        "two store-conditionals on one reservation both succeeded"
    );
}
