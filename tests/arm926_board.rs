//! The `arm926` board, end to end.
//!
//! A unit test can say "the interpreter executed `STR`". This says something
//! stronger: an ARMv5TE core **named in a `.machine` file** is handed an
//! address space by the machine layer, resets to the vector, runs a program out
//! of a boot ROM, writes into DRAM and into the peripheral aperture, and the
//! bytes are there afterwards.
//!
//! That is the thing that did not exist before: `cpu.arm` had a `DeviceClass`
//! and could be constructed, but no `Instance` impl, no `bind` and no `schema`,
//! so no machine file could give it a space or wire a line to it.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-arm926`.

#![cfg(feature = "machine-arm926")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// Where the board's peripheral aperture is, with the file's default `periph`.
const PERIPH: u64 = 0xf000_0000;

/// Where the board's DRAM starts, with the file's default `ram-base`.
const DRAM: u64 = 0x0200_0000;

/// A firmware image: eight ARM instructions, hand-assembled.
///
/// Written out as words with their mnemonics rather than assembled by anything,
/// because the crate has no assembler and a table of eight encodings is
/// checkable by hand against the ARM ARM (DDI 0100, A3.4 data processing, A5.2
/// load/store, A4.1.5 branch).
///
/// ```text
///   0x00: mov r0, #0xf0000000     ; the peripheral aperture
///   0x04: mov r1, #0x02000000     ; DRAM
///   0x08: mov r2, #42
///   0x0c: str r2, [r1]            ; 42 -> DRAM
///   0x10: ldr r3, [r1]            ; and back out again
///   0x14: add r3, r3, #1          ; 43
///   0x18: str r3, [r0]            ; 43 -> the peripheral window
///   0x1c: b   .                   ; park
/// ```
///
/// The load and the store are what make this a *board* test rather than a
/// decode test: they only work if the machine layer mapped ROM, DRAM and the
/// aperture into one space and handed that space to the core.
const FIRMWARE: [u32; 8] = [
    0xe3a0_04f0,
    0xe3a0_1402,
    0xe3a0_202a,
    0xe581_2000,
    0xe591_3000,
    0xe283_3001,
    0xe580_3000,
    0xeaff_fffe,
];

/// The firmware as bytes, little-endian, which is the byte order the board's
/// `big-endian = false` selects.
fn firmware() -> Vec<u8> {
    FIRMWARE.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// Build the board out of the catalog with `image` in its `firmware` slot and
/// `params` overriding the file's defaults.
fn boot_with(image: Vec<u8>, params: &[(&str, &str)]) -> Machine {
    let entry = catalog::machine("arm926").expect("this build ships arm926");
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

/// Build the board out of the catalog with the firmware in its `firmware` slot.
fn boot() -> Machine {
    boot_with(firmware(), &[])
}

/// Read one word of the guest's memory space.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped word")
}

#[test]
fn the_board_realizes_with_the_core_bound_to_its_space() {
    let m = boot();
    assert_eq!(m.name(), "arm926");
    for path in ["cpu", "boot", "dram", "regs"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
    // The firmware landed at the reset vector, which is where the core will
    // fetch its first instruction from.
    assert_eq!(peek(&m, 0x0000_0000), u64::from(FIRMWARE[0]));
}

#[test]
fn the_firmware_runs_and_reaches_dram_and_the_peripheral_window() {
    let mut m = boot();

    // Eight instructions and a reset sequence, with room to spare: a millisecond
    // of virtual time at 200 MHz is 200,000 ticks. A span rather than an
    // instruction count because the scheduler hands out budgets, not steps.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    assert_eq!(
        peek(&m, DRAM),
        42,
        "the `STR` into DRAM did not reach the mapped RAM"
    );
    assert_eq!(
        peek(&m, PERIPH),
        43,
        "the `STR` into the peripheral aperture did not reach it"
    );
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = boot();
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );

    // And the restored machine keeps running from where the other one was,
    // rather than from a reset it silently took on the way in.
    assert_eq!(peek(&other, PERIPH), 43);
}

// ---------------------------------------------------------------------------
// The MMU
// ---------------------------------------------------------------------------

/// Where the MMU firmware puts the first-level translation table.
///
/// The base of DRAM, which is 16 KiB aligned, which is what `TTBR` requires.
const TABLE: u64 = DRAM;

/// Where the MMU firmware writes its results, clear of the 16 KiB table.
const SCRATCH: u64 = DRAM + 0x0001_0000;

/// The physical megabyte the virtual alias at `0x10000000` resolves to.
const ALIASED: u64 = DRAM + 0x0010_0000;

/// A firmware that builds a page table, turns the MMU on, runs code from a
/// virtual address that is somewhere else entirely, and then takes a fault.
///
/// Hand-assembled, like [`FIRMWARE`], and for the same reason. The layout:
///
/// ```text
///   0x00  b start                       ; the reset vector
///   0x10  b abort                       ; the data abort vector
///
///   start:
///   0x20  mov  r0, #0x02000000          ; the first-level table, 16 KiB aligned
///   0x24  mov  r1, #0xc00               ; AP = 0b11, full access
///   0x28  orr  r1, r1, #2               ;  ... as a section descriptor
///   0x2c  str  r1, [r0]                 ; VA 0x00000000 -> PA 0x00000000 (this ROM)
///   0x30  mov  r2, #0x02000000
///   0x34  orr  r2, r2, r1
///   0x38  str  r2, [r0, #0x80]          ; VA 0x02000000 -> PA 0x02000000 (DRAM)
///   0x3c  mov  r3, #0x02000000
///   0x40  orr  r3, r3, #0x00100000
///   0x44  orr  r3, r3, r1
///   0x48  mov  r4, #0x400
///   0x4c  str  r3, [r0, r4]             ; VA 0x10000000 -> PA 0x02100000  <- the point
///   0x50  mov  r5, #1                   ; domain 0 = client
///   0x54  mcr  p15, 0, r5, c3, c0, 0
///   0x58  mcr  p15, 0, r0, c2, c0, 0    ; TTBR
///   0x5c  mrc  p15, 0, r6, c1, c0, 0
///   0x60  orr  r6, r6, #1               ; the M bit
///   0x64  mcr  p15, 0, r6, c1, c0, 0    ; the MMU is on from here
///   0x68  nop x3                        ; the pipeline an ARM926 would be draining
///
///   0x74  mov  r7, #0x10000000          ; copy three words of code through
///   0x78  mov  r8, #0x100               ;  the alias, so they land at 0x02100000
///   0x7c  ldr  r9, [r8], #4
///   0x80  str  r9, [r7], #4
///   0x84  ldr  r9, [r8], #4
///   0x88  str  r9, [r7], #4
///   0x8c  ldr  r9, [r8], #4
///   0x90  str  r9, [r7], #4
///
///   0x94  mov  r11, #0x02000000
///   0x98  orr  r11, r11, #0x00010000    ; the scratch word
///   0x9c  mov  r7, #0x10000000
///   0xa0  mov  lr, pc
///   0xa4  bx   r7                       ; execute from the alias
///
///   0xa8  mov  r12, #0x20000000         ; nothing is mapped there
///   0xac  ldr  r0, [r12]                ; -> data abort
///   0xb0  b .
///
///   abort:
///   0xb4  mrc  p15, 0, r0, c5, c0, 0    ; the fault status
///   0xb8  mrc  p15, 0, r1, c6, c0, 0    ; the fault address
///   0xbc  mov  r2, #0x02000000
///   0xc0  orr  r2, r2, #0x00010000
///   0xc4  str  r0, [r2, #0x10]
///   0xc8  str  r1, [r2, #0x14]
///   0xcc  b .
///
///   0x100 mov  r10, #43                 ; the three words that get copied
///   0x104 str  r10, [r11]
///   0x108 bx   lr
/// ```
const MMU_FIRMWARE: [(usize, u32); 55] = [
    (0x00, 0xea00_0006),
    (0x04, 0xeaff_fffe),
    (0x08, 0xeaff_fffe),
    (0x0c, 0xeaff_fffe),
    (0x10, 0xea00_0027),
    (0x14, 0xeaff_fffe),
    (0x18, 0xeaff_fffe),
    (0x1c, 0xeaff_fffe),
    (0x20, 0xe3a0_0402),
    (0x24, 0xe3a0_1b03),
    (0x28, 0xe381_1002),
    (0x2c, 0xe580_1000),
    (0x30, 0xe3a0_2402),
    (0x34, 0xe182_2001),
    (0x38, 0xe580_2080),
    (0x3c, 0xe3a0_3402),
    (0x40, 0xe383_3601),
    (0x44, 0xe183_3001),
    (0x48, 0xe3a0_4b01),
    (0x4c, 0xe780_3004),
    (0x50, 0xe3a0_5001),
    (0x54, 0xee03_5f10),
    (0x58, 0xee02_0f10),
    (0x5c, 0xee11_6f10),
    (0x60, 0xe386_6001),
    (0x64, 0xee01_6f10),
    (0x68, 0xe1a0_0000),
    (0x6c, 0xe1a0_0000),
    (0x70, 0xe1a0_0000),
    (0x74, 0xe3a0_7201),
    (0x78, 0xe3a0_8c01),
    (0x7c, 0xe498_9004),
    (0x80, 0xe487_9004),
    (0x84, 0xe498_9004),
    (0x88, 0xe487_9004),
    (0x8c, 0xe498_9004),
    (0x90, 0xe487_9004),
    (0x94, 0xe3a0_b402),
    (0x98, 0xe38b_b801),
    (0x9c, 0xe3a0_7201),
    (0xa0, 0xe1a0_e00f),
    (0xa4, 0xe12f_ff17),
    (0xa8, 0xe3a0_c202),
    (0xac, 0xe59c_0000),
    (0xb0, 0xeaff_fffe),
    (0xb4, 0xee15_0f10),
    (0xb8, 0xee16_1f10),
    (0xbc, 0xe3a0_2402),
    (0xc0, 0xe382_2801),
    (0xc4, 0xe582_0010),
    (0xc8, 0xe582_1014),
    (0xcc, 0xeaff_fffe),
    (0x100, 0xe3a0_a02b),
    (0x104, 0xe58b_a000),
    (0x108, 0xe12f_ff1e),
];

/// [`MMU_FIRMWARE`] as a little-endian image, zero-filled between the pieces.
fn mmu_firmware() -> Vec<u8> {
    let mut image = vec![0u8; 0x120];
    for (at, word) in MMU_FIRMWARE {
        image[at..at + 4].copy_from_slice(&word.to_le_bytes());
    }
    image
}

/// Run the MMU firmware long enough for it to finish and fault.
fn boot_mmu() -> Machine {
    let mut m = boot_with(mmu_firmware(), &[]);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    m
}

#[test]
fn the_machine_file_asks_for_a_cp15_and_gets_one() {
    // The whole mechanism, from the outside: a property on the CPU object. If
    // this fails, everything below it is measuring something else.
    let m = boot_with(mmu_firmware(), &[("cp15", "none")]);
    assert!(m.device("cpu").is_some());
    // And the name has to be one the core knows.
    let entry = catalog::machine("arm926").expect("this build ships arm926");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push(("cp15".to_string(), "cortex-a9".to_string()));
    let registry = catalog::registry().expect("a registry");
    let err = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect_err("`cortex-a9` is not a CP15 this core has");
    let message = format!("{err}");
    assert!(
        message.contains("cp15"),
        "the error should name the property: {message}"
    );
}

#[test]
fn the_guest_builds_a_page_table_and_executes_from_the_alias() {
    let m = boot_mmu();

    // The three words of code were written to virtual 0x10000000 and landed at
    // physical 0x02100000 — the store went through the page table.
    assert_eq!(
        peek(&m, ALIASED),
        0xe3a0_a02b,
        "the write through the virtual alias did not reach the physical page"
    );

    // And then the core *fetched* from 0x10000000 and ran what is at
    // 0x02100000, which is the whole claim: it wrote 43 to the scratch word.
    assert_eq!(
        peek(&m, SCRATCH),
        43,
        "the routine at the aliased virtual address did not execute"
    );
}

#[test]
fn an_unmapped_virtual_address_is_a_data_abort_with_a_status_and_an_address() {
    let m = boot_mmu();

    // A section translation fault: the first-level descriptor for virtual
    // megabyte 0x200 is zero (ARM ARM B4.6's `0b0101`), in domain 0.
    assert_eq!(
        peek(&m, SCRATCH + 0x10),
        0x05,
        "the fault status register should say `translation fault, section`"
    );
    assert_eq!(
        peek(&m, SCRATCH + 0x14),
        0x2000_0000,
        "the fault address register should hold the address that faulted"
    );
}

#[test]
fn the_table_the_guest_wrote_is_the_table_the_mmu_read() {
    let m = boot_mmu();
    // Section descriptor for virtual megabyte 0x100: physical base 0x02100000,
    // AP 0b11, domain 0, type `0b10`.
    assert_eq!(peek(&m, TABLE + 0x400), 0x0210_0c02);
}

#[test]
fn a_board_with_no_cp15_never_gets_past_the_first_mcr() {
    // The same firmware on the same board with `cp15 = none`: `MCR p15` is an
    // Undefined Instruction, so the MMU is never enabled, nothing is copied,
    // and the scratch word stays zero. This is what the property *does*.
    let mut m = boot_with(mmu_firmware(), &[("cp15", "none")]);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(peek(&m, SCRATCH), 0);
    assert_eq!(peek(&m, ALIASED), 0);
    // The page table was still built — those are ordinary stores — which is
    // what makes this a fair comparison rather than a broken guest.
    assert_eq!(peek(&m, TABLE + 0x400), 0x0210_0c02);
}

#[test]
fn a_machine_with_the_mmu_on_snapshots_and_restores_to_an_identical_hash() {
    let m = boot_mmu();
    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = boot_with(mmu_firmware(), &[]);
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );
    // The TLB is derived state and is not in the chunk; the restored machine
    // rebuilds it by walking, and must reach the same answers.
    assert_eq!(peek(&other, SCRATCH), 43);
}
