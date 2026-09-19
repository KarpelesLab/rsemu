//! Does a processor that shares its crystal with another still see **its own
//! clock move**?
//!
//! # The defect this file is the proof against
//!
//! On `riscv-virt-smp` every printk timestamp Linux wrote was a whole
//! millisecond — `[    0.045000]`, `[    0.048000]` — which is the length of a
//! scheduler round. Two harts on one oscillator each execute the round from a
//! position of their own, one after the other, and the scheduler's rule for
//! that (`Scheduler::arm_live_cursors`, *Not on a shared crystal*) is that
//! neither gets a live view of any lazily-advanced device: the one that runs
//! first must not drag a device into the future of the one that runs second.
//! So the CLINT stood where the round began for the whole of it, and `time` —
//! which is architecturally a view of the CLINT's `mtime` — was frozen for a
//! million instructions at a stretch. A `udelay` spun to the next round,
//! `sched_clock` had a resolution of a millisecond, and two reads of the
//! counter a few thousand cycles apart returned the same number.
//!
//! The rule is right for a device with side effects and needlessly strict for
//! a register that is a pure function of time. `mtime` is one: reading it at
//! the reader's own position moves nothing and cannot show the reader another
//! processor's future. `core::sched`'s read views (`TickCursor::tick_in`,
//! `LazyHandle::reader_tick`, `LiveCounter`) are that read path, and this file
//! is what they have to satisfy:
//!
//! * each hart's `time` — and a load of `mtime`, one instruction later — moves
//!   by the cycles *that hart* executed, read after read, across rounds, and
//!   never backwards;
//! * the same program gives the same numbers under
//!   [`ThreadingMode::Parallel`], because a read at one's own position
//!   involves nobody else;
//! * a comparator still fires where it always did: never before `mtime`
//!   reaches it, and — once it is armed beyond the round in which it was
//!   written — on the tick it names.
//!
//! The single-hart board is the control. Its hart has a live view of the
//! CLINT, so a load of `mtime` was already current; `time` was not, because a
//! CSR read never reaches the device and nothing caught it up.
//!
//! # Sources
//!
//! *The RISC-V Instruction Set Manual, Volume II: Privileged Architecture*
//! (CC-BY-4.0), "Machine Timer Registers (`mtime` and `mtimecmp`)" — the
//! interrupt is pending whenever `mtime >= mtimecmp` — and Volume I's `Zicntr`
//! chapter for `time` as a view of `mtime` that software must observe to be
//! monotonic. The ACLINT specification for the register map. Encodings are
//! Volume I's RV64I base.

#![cfg(all(feature = "machine-riscv-virt", feature = "std"))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Where the boot ROM hands every hart over, with `a0` holding `mhartid`.
const ENTRY: u64 = 0x8000_0000;

/// Each hart's record, `STRIDE` apart.
const BUF: u64 = 0x8010_0000;
const STRIDE: u64 = 0x1_0000;

/// The CLINT, as `machines/riscv-virt.machine` maps it.
const CLINT: u64 = 0x0200_0000;
const MTIME: u64 = CLINT + 0xbff8;
const MTIMECMP: u64 = CLINT + 0x4000;

/// `mtime` ticks per core cycle, inverted: the core crystal is 1 GHz and the
/// `rtc` crystal 10 MHz.
const CYCLES_PER_TICK: u64 = 100;

/// A scheduler round, in `mtime` ticks: the default millisecond.
const ROUND_TICKS: u64 = 10_000;

fn board(name: &str, program: &[u32], mode: ThreadingMode) -> Machine {
    let entry = catalog::machine(name).expect("this build ships the board");
    let mut code = Vec::with_capacity(program.len() * 4);
    for word in program {
        code.extend_from_slice(&word.to_le_bytes());
    }
    let mut options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_media("firmware", code)
        .with_media("flash0", Vec::new())
        .with_media("flash1", Vec::new())
        .with_media("disk", Vec::new())
        .with_media("initrd", Vec::new())
        .with_param("ram", "16M".to_string());
    options.realize.threading = Some(mode);
    options.realize.scheduler.workers = 2;
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{name} does not build: {e}"))
}

fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the board's memory space")
        .read(addr, Width::U64, MemAttrs::DEBUG)
        .expect("readable RAM")
}

// ---------------------------------------------------------------------------
// an RV64I assembler, just big enough
// ---------------------------------------------------------------------------

const T0: u32 = 5;
const T1: u32 = 6;
const T2: u32 = 7;
const S0: u32 = 8;
const S1: u32 = 9;
const A0: u32 = 10;
const A2: u32 = 12;
const A3: u32 = 13;
const A4: u32 = 14;
const A5: u32 = 15;
const A6: u32 = 16;

const TIME: u32 = 0xc01;
const MCYCLE: u32 = 0xb00;
const MSTATUS: u32 = 0x300;
const MIE: u32 = 0x304;
const MTVEC: u32 = 0x305;
const MCAUSE: u32 = 0x342;

const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    (((imm as u32) & 0xfff) << 20) | (rs1 << 15) | (rd << 7) | 0x13
}

const fn lui(rd: u32, imm: u32) -> u32 {
    ((imm & 0xf_ffff) << 12) | (rd << 7) | 0x37
}

const fn add(rd: u32, rs1: u32, rs2: u32) -> u32 {
    (rs2 << 20) | (rs1 << 15) | (rd << 7) | 0x33
}

const fn slli(rd: u32, rs1: u32, shamt: u32) -> u32 {
    ((shamt & 0x3f) << 20) | (rs1 << 15) | (0b001 << 12) | (rd << 7) | 0x13
}

const fn srli(rd: u32, rs1: u32, shamt: u32) -> u32 {
    ((shamt & 0x3f) << 20) | (rs1 << 15) | (0b101 << 12) | (rd << 7) | 0x13
}

const fn ld(rd: u32, rs1: u32, off: i32) -> u32 {
    (((off as u32) & 0xfff) << 20) | (rs1 << 15) | (0b011 << 12) | (rd << 7) | 0x03
}

const fn sd(rs2: u32, rs1: u32, off: i32) -> u32 {
    let imm = (off as u32) & 0xfff;
    ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (0b011 << 12) | ((imm & 0x1f) << 7) | 0x23
}

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

/// `jal x0, off`.
const fn j(off: i32) -> u32 {
    let imm = off as u32;
    (((imm >> 20) & 1) << 31)
        | (((imm >> 1) & 0x3ff) << 21)
        | (((imm >> 11) & 1) << 20)
        | (((imm >> 12) & 0xff) << 12)
        | 0x6f
}

/// `csrr rd, csr`.
const fn csrr(rd: u32, csr: u32) -> u32 {
    (csr << 20) | (0b010 << 12) | (rd << 7) | 0x73
}

/// `csrw csr, rs1`.
const fn csrw(csr: u32, rs1: u32) -> u32 {
    (csr << 20) | (rs1 << 15) | (0b001 << 12) | 0x73
}

/// `csrsi csr, uimm`.
const fn csrsi(csr: u32, uimm: u32) -> u32 {
    (csr << 20) | ((uimm & 0x1f) << 15) | (0b110 << 12) | 0x73
}

/// Load a 32-bit value **zero**-extended: RV64's `lui` sign-extends, and every
/// address on this board above 2 GiB would otherwise come out negative.
fn li(rd: u32, value: u32) -> Vec<u32> {
    let hi = value.wrapping_add(0x800) >> 12;
    let lo = value.wrapping_sub(hi << 12) as i32;
    let mut out = if hi == 0 {
        vec![addi(rd, 0, lo)]
    } else {
        vec![lui(rd, hi), addi(rd, rd, lo)]
    };
    if value & 0x8000_0000 != 0 {
        out.push(slli(rd, rd, 32));
        out.push(srli(rd, rd, 32));
    }
    out
}

/// Branch from the instruction about to be pushed back to `target`, both as
/// word indices.
fn back(from: usize, target: usize) -> i32 {
    (target as i32 - from as i32) * 4
}

// ---------------------------------------------------------------------------
// reading the counter across rounds
// ---------------------------------------------------------------------------

/// How many times each hart samples.
const SAMPLES: u64 = 120;

/// Iterations of the two-instruction spin between samples. A round trip is
/// four cycles, so this is twenty microseconds at 1 GHz and fifty samples in
/// every round.
const SPIN: u32 = 5_000;

/// Offsets inside a hart's sample record.
const REC_CYCLE: u64 = 0;
const REC_TIME: u64 = 8;
const REC_MTIME: u64 = 16;
const REC: u64 = 24;
/// Where a hart says it has finished.
const DONE: u64 = 0xfff8;

/// Both harts, out of one image:
///
/// ```text
///   ; a0 = mhartid, from the boot ROM
///   s0 = BUF + a0 * STRIDE
///   s1 = MTIME
///   a2 = SAMPLES
/// sample:
///   csrr a3, mcycle
///   csrr a4, time
///   ld   a5, 0(s1)          ; mtime, through the CLINT's window
///   sd   a3, 0(s0)
///   sd   a4, 8(s0)
///   sd   a5, 16(s0)
///   addi s0, s0, 24
///   a6 = SPIN
/// spin:
///   addi a6, a6, -1
///   bne  a6, x0, spin
///   addi a2, a2, -1
///   bne  a2, x0, sample
///   sd   a2 -> DONE          ; any nonzero word
/// park:
///   j park
/// ```
fn sampler() -> Vec<u32> {
    let mut c = Vec::new();
    c.extend(li(S0, BUF as u32));
    c.extend(li(T0, STRIDE as u32));
    // a0 × 64 KiB, by shifting: STRIDE is a power of two.
    c.push(slli(T1, A0, STRIDE.trailing_zeros()));
    c.push(add(S0, S0, T1));
    c.push(add(T2, S0, 0)); // the record's base, for DONE
    c.extend(li(S1, MTIME as u32));
    c.extend(li(A2, SAMPLES as u32));
    let sample = c.len();
    c.push(csrr(A3, MCYCLE));
    c.push(csrr(A4, TIME));
    c.push(ld(A5, S1, 0));
    c.push(sd(A3, S0, REC_CYCLE as i32));
    c.push(sd(A4, S0, REC_TIME as i32));
    c.push(sd(A5, S0, REC_MTIME as i32));
    c.push(addi(S0, S0, REC as i32));
    c.extend(li(A6, SPIN));
    let spin = c.len();
    c.push(addi(A6, A6, -1));
    c.push(bne(A6, 0, back(c.len(), spin)));
    c.push(addi(A2, A2, -1));
    c.push(bne(A2, 0, back(c.len(), sample)));
    // DONE is past the end of the 12-bit store offset, so aim at it through
    // a register of its own.
    c.extend(li(T0, DONE as u32));
    c.push(add(T0, T0, T2));
    c.push(addi(T1, 0, 1));
    c.push(sd(T1, T0, 0));
    c.push(j(0));
    c
}

/// One hart's samples: `(mcycle, time, mtime)`.
fn samples(m: &Machine, hart: u64) -> Vec<(u64, u64, u64)> {
    let base = BUF + hart * STRIDE;
    assert_eq!(
        peek(m, base + DONE),
        1,
        "hart {hart} did not finish sampling"
    );
    (0..SAMPLES)
        .map(|i| {
            let at = base + i * REC;
            (
                peek(m, at + REC_CYCLE),
                peek(m, at + REC_TIME),
                peek(m, at + REC_MTIME),
            )
        })
        .collect()
}

fn run_sampler(name: &str, mode: ThreadingMode, harts: u64) -> Vec<Vec<(u64, u64, u64)>> {
    let mut m = board(name, &sampler(), mode);
    // 120 samples of 20 µs is 2.4 ms of the counter; the rest is margin.
    m.run_for(GlobalTime::from_nanos(4_000_000))
        .expect("the board runs");
    (0..harts).map(|h| samples(&m, h)).collect()
}

/// The claim, for one hart's record: `time` moves by the cycles this hart
/// executed between two reads, every read, and a load of `mtime` one
/// instruction later agrees with it.
fn check_follows_own_cycles(board: &str, hart: usize, s: &[(u64, u64, u64)]) {
    let mut crossed_a_round = false;
    for pair in s.windows(2) {
        let ((c0, t0, _), (c1, t1, _)) = (pair[0], pair[1]);
        assert!(
            t1 > t0,
            "{board} hart {hart}: `time` went from {t0} to {t1}"
        );
        // `time` is floor(position / 100) and so is its difference to within
        // one tick; the two `csrr`s of a sample are a cycle apart, and the
        // whole sample is read at one position.
        let expect = (c1 - c0) / CYCLES_PER_TICK;
        assert!(
            (t1 - t0).abs_diff(expect) <= 1,
            "{board} hart {hart}: {} cycles went by between two reads and `time` moved \
             {} ticks rather than {expect}",
            c1 - c0,
            t1 - t0
        );
        crossed_a_round |= t0 / ROUND_TICKS != t1 / ROUND_TICKS;
    }
    assert!(
        crossed_a_round,
        "{board} hart {hart}: the samples never crossed a round, so they prove nothing \
         about one"
    );
    for (i, &(_, time, mtime)) in s.iter().enumerate() {
        assert!(
            mtime >= time && mtime - time <= 1,
            "{board} hart {hart}, sample {i}: `time` read {time} and the load of `mtime` one \
             instruction later {mtime}"
        );
    }
}

/// Two harts on one crystal each see `time` advance by their own cycles, read
/// after read and across round boundaries.
///
/// Before the read path existed, every one of these reads inside a round
/// returned the value the round began with: forty-nine differences of zero
/// and then one of ten thousand, per round per hart.
#[test]
fn each_hart_on_a_shared_crystal_reads_its_own_time() {
    let records = run_sampler("riscv-virt-smp", ThreadingMode::Deterministic, 2);
    for (hart, s) in records.iter().enumerate() {
        check_follows_own_cycles("riscv-virt-smp", hart, s);
    }
}

/// The same program, the same numbers, with the two harts running at once on
/// two host threads.
///
/// Neither hart's program looks at the other, so everything each one records
/// is a function of its own execution — and a read of the counter at one's
/// own position is too, which is the property this pins down. A read that
/// depended on which hart the scheduler ran first, or on what another thread
/// had caught a device up to, would differ here.
#[test]
fn a_parallel_round_reads_the_same_counter() {
    let deterministic = run_sampler("riscv-virt-smp", ThreadingMode::Deterministic, 2);
    let parallel = run_sampler("riscv-virt-smp", ThreadingMode::Parallel, 2);
    for (hart, s) in parallel.iter().enumerate() {
        check_follows_own_cycles("riscv-virt-smp (parallel)", hart, s);
    }
    assert_eq!(
        deterministic, parallel,
        "a hart's reads of its own counter depend on the threading mode"
    );
}

/// The control: one hart, a crystal of its own, a live view of the CLINT.
///
/// A load of `mtime` was already current here. `time` was not — a CSR read
/// never reaches the device, so it read whatever the last access or the last
/// round had caught the CLINT up to — and it is now the same number.
#[test]
fn the_single_hart_board_reads_the_same_way() {
    let records = run_sampler("riscv-virt", ThreadingMode::Deterministic, 1);
    check_follows_own_cycles("riscv-virt", 0, &records[0]);
}

/// Two runs of one deterministic machine are one run.
#[test]
fn reading_the_counter_is_deterministic() {
    let hash = || {
        let mut m = board("riscv-virt-smp", &sampler(), ThreadingMode::Deterministic);
        m.run_for(GlobalTime::from_nanos(3_000_000))
            .expect("the board runs");
        m.state_hash().expect("a deterministic board hashes")
    };
    assert_eq!(hash(), hash());
}

// ---------------------------------------------------------------------------
// the comparator
// ---------------------------------------------------------------------------

/// Offsets in a hart's comparator record.
const CMP_AT: u64 = 0;
const CMP_LAST: u64 = 8;
const CMP_TRAP: u64 = 16;
const CMP_CAUSE: u64 = 24;

/// Where the trap handler sits, as a word index into the image.
const HANDLER: usize = 128;

/// Arm this hart's comparator `delay` ticks from `now`, then spin reading
/// `time` until the interrupt arrives:
///
/// ```text
///   s0 = BUF + a0 * STRIDE
///   s1 = MTIMECMP + a0 * 8
///   mtvec = handler
///   mie   = MTIE
///   t0 = time + delay
///   sd t0, 0(s1)             ; mtimecmp
///   sd t0, CMP_AT(s0)
///   csrsi mstatus, MIE
/// spin:
///   csrr t1, time
///   sd   t1, CMP_LAST(s0)
///   j    spin
///
/// handler:
///   csrr t1, time
///   sd   t1, CMP_TRAP(s0)
///   csrr t1, mcause
///   sd   t1, CMP_CAUSE(s0)
///   park
/// ```
///
/// `CMP_LAST` is therefore the last value this hart read before it took the
/// interrupt, and `CMP_TRAP` the first one after.
fn comparator(delay: u32) -> Vec<u32> {
    let mut c = Vec::new();
    c.extend(li(S0, BUF as u32));
    c.push(slli(T1, A0, STRIDE.trailing_zeros()));
    c.push(add(S0, S0, T1));
    c.extend(li(S1, MTIMECMP as u32));
    c.push(slli(T1, A0, 3));
    c.push(add(S1, S1, T1));
    c.extend(li(T0, (ENTRY + 4 * HANDLER as u64) as u32));
    c.push(csrw(MTVEC, T0));
    c.extend(li(T0, 1 << 7));
    c.push(csrw(MIE, T0));
    c.extend(li(T1, delay));
    c.push(csrr(T0, TIME));
    c.push(add(T0, T0, T1));
    c.push(sd(T0, S1, 0));
    c.push(sd(T0, S0, CMP_AT as i32));
    c.push(csrsi(MSTATUS, 1 << 3));
    let spin = c.len();
    c.push(csrr(T1, TIME));
    c.push(sd(T1, S0, CMP_LAST as i32));
    c.push(j(back(c.len(), spin)));
    assert!(c.len() <= HANDLER, "the program ran into its handler");
    c.resize(HANDLER, 0);
    c.push(csrr(T1, TIME));
    c.push(sd(T1, S0, CMP_TRAP as i32));
    c.push(csrr(T1, MCAUSE));
    c.push(sd(T1, S0, CMP_CAUSE as i32));
    c.push(j(0));
    c
}

/// `(armed for, last read before, first read after, mcause)`.
fn fired(name: &str, delay: u32, hart: u64) -> (u64, u64, u64, u64) {
    let mut m = board(name, &comparator(delay), ThreadingMode::Deterministic);
    m.run_for(GlobalTime::from_nanos(6_000_000))
        .expect("the board runs");
    let base = BUF + hart * STRIDE;
    let record = (
        peek(&m, base + CMP_AT),
        peek(&m, base + CMP_LAST),
        peek(&m, base + CMP_TRAP),
        peek(&m, base + CMP_CAUSE),
    );
    assert_eq!(
        record.3,
        (1 << 63) | 7,
        "{name} hart {hart} took something other than its machine timer interrupt: {record:?}"
    );
    record
}

/// Where a comparator fires, on each board, for a `delay` armed at the very
/// start of the run: never before `mtime` reaches it, and exactly where it
/// fired before the counter could be read inside a round.
///
/// Armed a few microseconds out, the comparator falls inside the round that is
/// already running. That round's end was fixed when it began, and nothing in
/// it catches the CLINT up past the comparator — on the two-hart board nothing
/// may, and on the one-hart board the hart never touches the CLINT while it
/// waits — so the interrupt is delivered when the round closes: late by up to
/// a round, exactly as it always was, and **never before `mtime` reaches it**.
/// What the read path changed is that the hart now *sees* the counter pass
/// its comparator while it waits, rather than a counter frozen below it.
///
/// Armed further out than the round that is running, the comparator is the
/// next round's end (`Scheduler::natural_target`) and it fires on its tick.
fn check_comparator(name: &str, harts: u64) {
    for hart in 0..harts {
        let (at, last, trap, _) = fired(name, 500, hart);
        assert!(
            trap >= at,
            "{name} hart {hart}'s interrupt arrived at {trap}, before `mtime` reached {at}"
        );
        assert!(
            trap.div_ceil(ROUND_TICKS) == at.div_ceil(ROUND_TICKS),
            "{name} hart {hart}'s interrupt, armed for {at}, waited past the round it fell in \
             (it arrived at {trap})"
        );
        assert!(last < trap, "{name} hart {hart}: {last} then {trap}");

        let (at, last, trap, _) = fired(name, 15_000, hart);
        assert!(
            last < at,
            "{name} hart {hart} read {last}, at or past its comparator {at}, before the interrupt"
        );
        assert!(
            trap >= at && trap - at <= 1,
            "{name} hart {hart}'s interrupt, armed for {at}, arrived at {trap}"
        );
    }
}

#[test]
fn a_comparator_on_a_shared_crystal_never_fires_early() {
    check_comparator("riscv-virt-smp", 2);
}

/// The control. Before the read path the lone hart took these two interrupts
/// at `mtime` 10 000 and 15 000 as well; the only difference is what it could
/// read while it waited.
#[test]
fn a_comparator_on_the_single_hart_board_fires_where_it_did() {
    check_comparator("riscv-virt", 1);
}
