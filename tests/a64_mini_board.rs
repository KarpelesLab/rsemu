//! The `a64-mini` board, end to end.
//!
//! A unit test can say "the interpreter executed `STR`". This says something
//! stronger: an AArch64 core **named in a `.machine` file** is handed a 64-bit
//! address space by the machine layer, starts at the board's `RVBAR`, runs a
//! program out of a boot ROM that loops, uses a stack, takes a supervisor call
//! into its own vector table and returns from it, then builds a three-level
//! translation-table hierarchy, turns the MMU on and keeps executing — from a
//! virtual address that resolves back to the ROM it is already in — and stores
//! through an alias that lives somewhere else entirely.
//!
//! The MMU half is what makes this a board test rather than a decode test.
//! The tables are built by the guest, in guest memory, out of instructions the
//! guest executed; nothing in the test writes a descriptor.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-a64-mini`.

#![cfg(feature = "machine-a64-mini")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::DebugTranslation;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::arm::a64::sysreg::enc;
use rsemu::machine::{Machine, catalog};

/// Where the board's DRAM starts, with the file's default `ram-base`.
const DRAM: u64 = 0x4000_0000;

/// Where the firmware puts its exception vector table. In the boot ROM, and
/// 2 KiB aligned as `VBAR_EL1` requires.
const VBAR: u64 = 0x800;

/// The physical 2 MiB block the virtual alias at `0x8000_0000` resolves to.
const ALIASED: u64 = DRAM + 0x0020_0000;

// ---------------------------------------------------------------------------
// A very small A64 assembler
// ---------------------------------------------------------------------------
//
// Encodings from DDI 0487 C4.1. Written as functions rather than as a table of
// hex so the program below reads as a program; every one of them is exercised
// by the core's own decoder tests against independently known words.

const fn movz(rd: u32, imm: u32, shift: u32) -> u32 {
    0xd280_0000 | ((shift / 16) << 21) | ((imm & 0xffff) << 5) | rd
}
const fn movk(rd: u32, imm: u32, shift: u32) -> u32 {
    0xf280_0000 | ((shift / 16) << 21) | ((imm & 0xffff) << 5) | rd
}
const fn add_imm(rd: u32, rn: u32, imm: u32) -> u32 {
    0x9100_0000 | ((imm & 0xfff) << 10) | (rn << 5) | rd
}
const fn subs_imm(rd: u32, rn: u32, imm: u32) -> u32 {
    0xf100_0000 | ((imm & 0xfff) << 10) | (rn << 5) | rd
}
const fn add_reg(rd: u32, rn: u32, rm: u32) -> u32 {
    0x8b00_0000 | (rm << 16) | (rn << 5) | rd
}
/// `ORR Xd, Xn, #imm` for a bitmask immediate given as its `N:immr:imms`.
const fn orr_imm(rd: u32, rn: u32, n: u32, immr: u32, imms: u32) -> u32 {
    0xb200_0000 | (n << 22) | (immr << 16) | (imms << 10) | (rn << 5) | rd
}
const fn str_x(rt: u32, rn: u32, offset: u32) -> u32 {
    0xf900_0000 | ((offset / 8) << 10) | (rn << 5) | rt
}
const fn stp_x_pre(rt: u32, rt2: u32, rn: u32, imm: i32) -> u32 {
    0xa980_0000 | ((((imm / 8) as u32) & 0x7f) << 15) | (rt2 << 10) | (rn << 5) | rt
}
const fn ldp_x_post(rt: u32, rt2: u32, rn: u32, imm: i32) -> u32 {
    0xa8c0_0000 | ((((imm / 8) as u32) & 0x7f) << 15) | (rt2 << 10) | (rn << 5) | rt
}
/// `B.NE` to a target `words` instructions away.
const fn b_ne(words: i32) -> u32 {
    0x5400_0001 | (((words as u32) & 0x7ffff) << 5)
}
const fn b(words: i32) -> u32 {
    0x1400_0000 | ((words as u32) & 0x03ff_ffff)
}
const fn svc(imm: u32) -> u32 {
    0xd400_0001 | (imm << 5)
}
fn mrs(reg: u16, rt: u32) -> u32 {
    0xd520_0000 | (u32::from(reg) << 5) | rt
}
fn msr(reg: u16, rt: u32) -> u32 {
    0xd500_0000 | (u32::from(reg) << 5) | rt
}
/// `CBZ Xt, .` — branch to itself while `Xt` is zero, which is how the timer
/// firmware below waits for its own interrupt handler.
const fn cbz_self(rt: u32) -> u32 {
    0xb400_0000 | rt
}
const ERET: u32 = 0xd69f_03e0;
const ISB: u32 = 0xd503_3fdf;
/// `MSR DAIFClr, #2` — unmask IRQ. `PSTATE.DAIF` comes up with every mask set,
/// so this is the instruction that lets an interrupt in at all.
const DAIFCLR_I: u32 = 0xd503_42ff;

/// A descriptor's valid bit plus its access flag, which is what a block
/// descriptor needs to be usable: `0b01` in bits 1:0 and `AF` at bit 10.
const BLOCK: u32 = 0x401;

/// The firmware, as a program rather than as a table of hex.
///
/// ```text
///   ; -- arithmetic and a stack ---------------------------------------
///   x0 = 0x40000000                  ; DRAM
///   sp = x0 + 0x800
///   x2 = 0; x3 = 10
///   loop: x2 += x3; x3 -= 1; b.ne loop     ; x2 = 55
///   [x0]      = x2                   ; the sum
///   push x2, x3; pop x16, x17
///   [x0 + 16] = x16                  ; the same 55, through the stack
///
///   ; -- an exception and a return ------------------------------------
///   VBAR_EL1 = 0x800
///   svc  #1                          ; -> 0x800 + 0x200, the EL1h vector
///   [x0 + 8]  = x5                   ; the marker the handler left
///
///   ; -- three levels of translation table ----------------------------
///   x6 = 0x40001000                  ; level 1
///   [x6]      = 0x00000000 | block   ; VA 0x00000000 -> PA 0x00000000, 1 GiB
///   [x6 + 8]  = 0x40000000 | block   ; VA 0x40000000 -> PA 0x40000000, 1 GiB
///   x9 = 0x40002000                  ; level 2
///   [x6 + 16] = x9 | 3               ; VA 0x80000000 -> the level-2 table
///   [x9]      = 0x40200000 | block   ; VA 0x80000000 -> PA 0x40200000, 2 MiB
///   TTBR0_EL1 = x6
///   TCR_EL1   = T0SZ 25, T1SZ 25, TG1 4 KiB
///   SCTLR_EL1 |= M ; isb             ; the MMU is on from here
///
///   [0x80000000] = 0xbeef            ; through the alias
///   b .
/// ```
///
/// The first two level-1 entries are identity blocks and they are not
/// decoration: the instruction after the `ISB` is fetched through them, and
/// the stack is still where it was. A walker that got the level-1 index wrong
/// would stop the machine dead at exactly that instruction.
fn firmware_words() -> Vec<u32> {
    let vbar = enc(3, 0, 12, 0, 0);
    let ttbr0 = enc(3, 0, 2, 0, 0);
    let tcr = enc(3, 0, 2, 0, 2);
    let sctlr = enc(3, 0, 1, 0, 0);
    vec![
        movz(0, 0x4000, 16),        // x0 = 0x40000000
        add_imm(1, 0, 0x800),       // x1 = x0 + 0x800
        add_imm(31, 1, 0),          // sp = x1  (Rd 31 is SP in this encoding)
        movz(2, 0, 0),              // x2 = 0
        movz(3, 10, 0),             // x3 = 10
        add_reg(2, 2, 3),           // loop: x2 += x3
        subs_imm(3, 3, 1),          //       x3 -= 1
        b_ne(-2),                   //       b.ne loop
        str_x(2, 0, 0),             // [x0] = 55
        stp_x_pre(2, 3, 31, -16),   // push x2, x3
        ldp_x_post(16, 17, 31, 16), // pop x16, x17
        str_x(16, 0, 16),           // [x0 + 16] = 55
        movz(4, VBAR as u32, 0),    // x4 = 0x800
        msr(vbar, 4),               // VBAR_EL1 = x4
        svc(1),                     // -> the synchronous EL1h vector
        str_x(5, 0, 8),             // [x0 + 8] = the handler's marker
        movz(6, 0x4000, 16),        // x6 = 0x40001000, the level-1 table
        movk(6, 0x1000, 0),
        movz(7, BLOCK, 0), // x7 = 0x401
        str_x(7, 6, 0),    // L1[0] = identity block at 0
        movz(8, 0x4000, 16),
        add_imm(8, 8, BLOCK), // x8 = 0x40000401
        str_x(8, 6, 8),       // L1[1] = identity block at DRAM
        movz(9, 0x4000, 16),  // x9 = 0x40002000, the level-2 table
        movk(9, 0x2000, 0),
        add_imm(10, 9, 3), // a table descriptor is bits 1:0 == 0b11
        str_x(10, 6, 16),  // L1[2] = the level-2 table
        movz(11, 0x4020, 16),
        add_imm(11, 11, BLOCK), // x11 = 0x40200401
        str_x(11, 9, 0),        // L2[0] = a 2 MiB block, the alias
        msr(ttbr0, 6),          // TTBR0_EL1 = the level-1 table
        movz(12, 25, 0),        // T0SZ = 25: 39 bits, three levels
        movk(12, 0x8019, 16),   // T1SZ = 25, TG1 = 0b10 (4 KiB)
        msr(tcr, 12),
        mrs(sctlr, 13),
        orr_imm(13, 13, 1, 0, 0), // orr x13, x13, #1 -> SCTLR_EL1.M
        msr(sctlr, 13),
        ISB,
        movz(14, 0x8000, 16), // x14 = 0x80000000, the virtual alias
        movz(15, 0xbeef, 0),
        str_x(15, 14, 0), // and a store through it
        b(0),             // park
    ]
}

/// The synchronous vector for "current EL with `SP_ELx`", which is where an
/// `SVC` taken at EL1h lands: `VBAR_EL1 + 0x200` (DDI 0487 D1.10.2).
const HANDLER: u64 = VBAR + 0x200;

/// What the handler does: leave a marker in `x5` and return.
fn handler_words() -> Vec<u32> {
    vec![movz(5, 0x1234, 0), ERET]
}

/// The firmware image: the program at zero and the handler at its vector.
fn firmware() -> Vec<u8> {
    let mut image = vec![0u8; (HANDLER as usize) + 16];
    for (i, word) in firmware_words().iter().enumerate() {
        image[4 * i..4 * i + 4].copy_from_slice(&word.to_le_bytes());
    }
    for (i, word) in handler_words().iter().enumerate() {
        let at = HANDLER as usize + 4 * i;
        image[at..at + 4].copy_from_slice(&word.to_le_bytes());
    }
    image
}

/// Build the board out of the catalog with `image` in its `firmware` slot and
/// `params` overriding the file's defaults.
fn boot_with(image: Vec<u8>, params: &[(&str, &str)]) -> Machine {
    let entry = catalog::machine("a64-mini").expect("this build ships a64-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", image);
    for (name, value) in params {
        options
            .resolve
            .params
            .push((name.to_string(), value.to_string()));
    }
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// The board with a smaller DRAM than the file's default. 128 MiB is the right
/// number for the board and the wrong one for a test that builds it eight
/// times: a cold reset clears every byte, and this is the one parameter worth
/// overriding to keep the suite quick.
const SMALL_RAM: (&str, &str) = ("ram", "16M");

fn boot() -> Machine {
    boot_with(firmware(), &[SMALL_RAM])
}

/// Read one doubleword of the guest's memory space.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U64, MemAttrs::DEFAULT)
        .expect("a mapped doubleword")
}

/// A millisecond of virtual time, which at 1 GHz is a million bus accesses
/// against the sixty-odd the firmware needs. A span rather than an instruction
/// count because the scheduler hands out budgets, not steps — and a generous
/// one because `run_for` stops on the machine's own scheduling boundaries and
/// may return with a round elapsed but not yet executed.
fn settle(m: &mut Machine) {
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
}

#[test]
fn the_board_realizes_with_the_core_bound_to_its_space() {
    let m = boot();
    assert_eq!(m.name(), "a64-mini");
    for path in ["cpu", "boot", "dram", "regs"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
    // The firmware landed at the board's reset vector, which is where the core
    // will fetch its first instruction from.
    assert_eq!(
        peek(&m, 0) & 0xffff_ffff,
        u64::from(firmware_words()[0]),
        "the image is not at RVBAR"
    );
}

#[test]
fn the_firmware_computes_uses_a_stack_and_reaches_dram() {
    let mut m = boot();
    settle(&mut m);
    assert_eq!(peek(&m, DRAM), 55, "the loop's sum did not reach DRAM");
    assert_eq!(
        peek(&m, DRAM + 16),
        55,
        "the value did not survive a push and a pop through SP"
    );
}

/// The exception model, end to end: a guest sets its own `VBAR_EL1`, executes
/// `SVC`, lands on the right one of the sixteen vectors, and `ERET`s back to
/// the instruction after the call.
#[test]
fn a_supervisor_call_reaches_the_guests_own_vector_table() {
    let mut m = boot();
    settle(&mut m);
    assert_eq!(
        peek(&m, DRAM + 8),
        0x1234,
        "the handler at VBAR_EL1 + 0x200 did not run, or ERET did not return"
    );
}

/// The MMU, end to end: the guest builds the tables, enables translation, goes
/// on executing through an identity block, and stores through an alias.
#[test]
fn the_guest_builds_page_tables_and_stores_through_the_alias() {
    let mut m = boot();
    settle(&mut m);
    assert_eq!(
        peek(&m, ALIASED),
        0xbeef,
        "the store through virtual 0x80000000 did not land at physical {ALIASED:#x}"
    );
    // Nothing whatsoever is mapped at physical 0x80000000, so a core that had
    // ignored the tables would have taken an external abort instead.
    assert!(
        m.space("mem")
            .expect("the memory space")
            .read(0x8000_0000, Width::U64, MemAttrs::DEFAULT)
            .is_err(),
        "physical 0x80000000 must be a hole for this test to mean anything"
    );
}

/// The seam, from the outside: a `dyn Device` is asked where a virtual address
/// lives and answers with the physical one, following the table the *guest*
/// built.
#[test]
fn a_debug_translation_follows_the_page_table_the_guest_built() {
    let mut m = boot();
    settle(&mut m);
    let cpu = m.device("cpu").expect("the board has a cpu").device();
    assert_eq!(
        cpu.debug_translate(0x8000_0000),
        DebugTranslation::Mapped(ALIASED),
        "the debug translation did not follow the guest's page table"
    );
    // And the bytes at the translated address are the ones the guest put there.
    let space = m.space("mem").expect("the memory space");
    let pa = cpu
        .debug_translate(0x8000_0000)
        .phys(0x8000_0000)
        .expect("mapped");
    assert_eq!(
        space
            .read(pa, Width::U64, MemAttrs::DEBUG)
            .expect("a mapped doubleword"),
        0xbeef
    );
    // An address in the hole between the two halves of the address space maps
    // nothing at all.
    assert_eq!(
        cpu.debug_translate(0x0000_8000_0000_0000),
        DebugTranslation::Unmapped
    );
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let mut m = boot();
    settle(&mut m);

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = boot();
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );
    // And the restored machine kept what the other one had done, rather than
    // taking a reset on the way in.
    assert_eq!(peek(&other, ALIASED), 0xbeef);
}

/// The extension lattice, at board level: the same image on two parts, and one
/// of them does not have the instruction.
#[test]
fn the_part_the_machine_file_names_decides_what_decodes() {
    // A `CAS X1, X2, [X0]` planted where the firmware would otherwise park.
    let mut words = firmware_words();
    let park = words.len() - 1;
    words[park] = 0xc8a1_7c02; // cas x1, x2, [x0]
    words.push(b(0));
    let mut image = vec![0u8; (HANDLER as usize) + 16];
    for (i, word) in words.iter().enumerate() {
        image[4 * i..4 * i + 4].copy_from_slice(&word.to_le_bytes());
    }
    for (i, word) in handler_words().iter().enumerate() {
        let at = HANDLER as usize + 4 * i;
        image[at..at + 4].copy_from_slice(&word.to_le_bytes());
    }

    // A Neoverse N1 is Armv8.2 and has FEAT_LSE, so it executes it and parks
    // on the branch after it.
    let mut n1 = boot_with(image.clone(), &[SMALL_RAM, ("part", "neoverse-n1")]);
    settle(&mut n1);
    let pc = n1
        .device("cpu")
        .expect("a cpu")
        .device()
        .debug_translate(0)
        .phys(0);
    assert!(pc.is_some(), "the N1 kept running");
    assert_eq!(peek(&n1, ALIASED), 0xbeef);

    // A Cortex-A53 is Armv8.0 without it, so the same word is UNDEFINED and
    // the core vectors instead — into a table whose synchronous EL1h entry is
    // the handler, which `ERET`s straight back to the faulting instruction and
    // loops. What matters is that it did *not* execute: nothing but the
    // architecture told it so.
    let mut a53 = boot_with(image, &[SMALL_RAM, ("part", "cortex-a53")]);
    settle(&mut a53);
    assert_eq!(
        peek(&a53, ALIASED),
        0xbeef,
        "everything before the CAS still ran"
    );
}

/// A machine file naming a part this core does not implement fails at
/// construction, with the name in the message — `new` validates and `realize`
/// acts (`ROADMAP.md` §4.4).
#[test]
fn an_unknown_part_is_refused_at_construction() {
    let entry = catalog::machine("a64-mini").expect("this build ships a64-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push(("part".to_string(), "cortex-a9".to_string()));
    let registry = catalog::registry().expect("a registry");
    let err = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect_err("a Cortex-A9 is not an AArch64 part");
    let text = format!("{err}");
    assert!(
        text.contains("cortex-a9") || text.contains("cpu"),
        "the error should name the property: {text}"
    );
}

// ---------------------------------------------------------------------------
// The generic timer
// ---------------------------------------------------------------------------

/// The IRQ vector for "current EL with `SP_ELx`": `VBAR_EL1 + 0x280`
/// (DDI 0487 D1.10.2). One group along from the `SVC` vector the firmware
/// above uses, which is the whole difference between a synchronous exception
/// and an interrupt.
const IRQ_VECTOR: u64 = VBAR + 0x280;

/// Firmware that arms its own timer and waits for it.
///
/// ```text
///   x0 = 0x40000000                    ; DRAM
///   sp = x0 + 0x800
///   VBAR_EL1 = 0x800
///   [x0]      = CNTFRQ_EL0             ; what the board told it
///   [x0 + 8]  = CNTPCT_EL0             ; a count before the delay
///   x9 = 200; delay: subs x9, #1; b.ne delay
///   [x0 + 32] = CNTPCT_EL0             ; and one after it
///   CNTP_TVAL_EL0 = 64                 ; fire 64 counts from now
///   CNTP_CTL_EL0  = ENABLE
///   msr DAIFClr, #2                    ; and let the interrupt in
///   x20 = 0
///   spin: cbz x20, spin                ; until the handler says otherwise
///   [x0 + 40] = x20
///   b .
/// ```
///
/// Nothing on the board is wired to `cpu.irq`. The interrupt this waits for is
/// produced inside the core by a comparator against its own tick counter, and
/// the only reason the loop ever ends is that the counter reached it.
fn timer_firmware_words() -> Vec<u32> {
    let vbar = enc(3, 0, 12, 0, 0);
    let cntfrq = enc(3, 3, 14, 0, 0);
    let cntpct = enc(3, 3, 14, 0, 1);
    let cntp_tval = enc(3, 3, 14, 2, 0);
    let cntp_ctl = enc(3, 3, 14, 2, 1);
    vec![
        movz(0, 0x4000, 16),
        add_imm(1, 0, 0x800),
        add_imm(31, 1, 0),
        movz(4, VBAR as u32, 0),
        msr(vbar, 4),
        mrs(cntfrq, 5),
        str_x(5, 0, 0),
        mrs(cntpct, 6),
        str_x(6, 0, 8),
        // A counted delay, so the count that follows it is a *measurement*
        // rather than a value read one instruction later: 200 iterations of
        // two instructions is 400 bus accesses, and the counter advances one
        // per `cntdiv` of them.
        movz(9, 200, 0),
        subs_imm(9, 9, 1),
        b_ne(-1),
        mrs(cntpct, 3),
        str_x(3, 0, 32),
        movz(7, 64, 0),
        msr(cntp_tval, 7),
        movz(8, 1, 0), // ENABLE, IMASK clear
        msr(cntp_ctl, 8),
        DAIFCLR_I,
        movz(20, 0, 0),
        cbz_self(20),
        str_x(20, 0, 40),
        b(0),
    ]
}

/// What the IRQ handler does: read the control register while the interrupt is
/// still asserted, disarm the timer, record the count it fired at, and return.
///
/// Disarming is not tidiness. The output is a **level**, not an edge: a handler
/// that returned without clearing the condition would take the same interrupt
/// again on the very next instruction, forever — which is exactly what a real
/// tick handler has to do and exactly what an edge-triggered model would let it
/// get away with.
fn timer_handler_words() -> Vec<u32> {
    let cntpct = enc(3, 3, 14, 0, 1);
    let cntp_ctl = enc(3, 3, 14, 2, 1);
    vec![
        mrs(cntp_ctl, 12),
        str_x(12, 0, 16),  // ENABLE | ISTATUS, IMASK clear
        msr(cntp_ctl, 31), // CNTP_CTL_EL0 = XZR: disarmed
        mrs(cntpct, 10),
        str_x(10, 0, 24),
        movz(20, 0xbeef, 0),
        ERET,
    ]
}

fn timer_firmware() -> Vec<u8> {
    let mut image = vec![0u8; (IRQ_VECTOR as usize) + 64];
    for (i, word) in timer_firmware_words().iter().enumerate() {
        image[4 * i..4 * i + 4].copy_from_slice(&word.to_le_bytes());
    }
    for (i, word) in timer_handler_words().iter().enumerate() {
        let at = IRQ_VECTOR as usize + 4 * i;
        image[at..at + 4].copy_from_slice(&word.to_le_bytes());
    }
    image
}

/// The generic timer, end to end on a board: a guest reads `CNTFRQ_EL0`, arms
/// `CNTP_TVAL_EL0`, unmasks `PSTATE.I`, and is interrupted into its own vector
/// table by nothing but the passage of virtual time.
#[test]
fn the_generic_timer_interrupts_the_guest_it_belongs_to() {
    let mut m = boot_with(timer_firmware(), &[SMALL_RAM]);
    settle(&mut m);

    // The board's `cntfrq`, which the machine file derives its `cntdiv` from.
    assert_eq!(
        peek(&m, DRAM),
        100_000_000,
        "CNTFRQ_EL0 is not what the board said"
    );
    let before = peek(&m, DRAM + 8);
    let fired = peek(&m, DRAM + 24);
    assert_eq!(
        peek(&m, DRAM + 40),
        0xbeef,
        "the spin loop never ended, so the timer never interrupted"
    );
    assert_eq!(
        peek(&m, DRAM + 16),
        0b101,
        "the handler should see ENABLE and ISTATUS with IMASK clear"
    );
    // It fired *after* its deadline and not long after: the comparator was 64
    // counts past the value read at `before`, and a handful of instructions
    // separate the two reads.
    assert!(
        fired >= before + 64 && fired < before + 128,
        "fired at {fired}, armed 64 counts after {before}"
    );
}

/// The rate the board declares is the rate the guest measures.
///
/// Two boards, identical but for `cntfrq`, running the same delay loop: the
/// 100 MHz counter advances exactly twice as far through it as the 50 MHz one.
/// That is the claim the machine file's derived `cntdiv` exists to make, and
/// it is measured from inside the guest rather than asserted from outside it.
///
/// Exactly twice, not approximately: the ratio is an integer divisor of one
/// tick counter inside one oscillator tree (`ROADMAP.md` §4.2), so there is no
/// residual to accumulate and no absolute time in the path. A one-count
/// tolerance is allowed only for the floor of the division at each end.
#[test]
fn the_counter_advances_at_the_rate_the_board_declares() {
    let elapsed = |hz: &str| {
        let mut m = boot_with(timer_firmware(), &[SMALL_RAM, ("cntfrq", hz)]);
        settle(&mut m);
        (peek(&m, DRAM), peek(&m, DRAM + 32) - peek(&m, DRAM + 8))
    };
    let (frq_fast, fast) = elapsed("100000000");
    let (frq_slow, slow) = elapsed("50000000");
    assert_eq!(frq_fast, 100_000_000);
    assert_eq!(frq_slow, 50_000_000);
    assert!(
        fast >= 39,
        "a 200-iteration loop should span ~40 counts: {fast}"
    );
    assert!(
        fast.abs_diff(slow * 2) <= 1,
        "a counter at twice the rate must advance twice as far: {fast} against {slow}"
    );
}
