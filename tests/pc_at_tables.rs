//! **A guest walks the tables rsemu's own BIOS publishes and finds the
//! processors.**
//!
//! [`rsemu::fw::pcbios`] lays an MP 1.4 floating pointer and configuration
//! table, an ACPI RSDP/RSDT/XSDT/FADT/MADT/DSDT set and an SMBIOS structure
//! table into its ROM image, generated from the machine description the board
//! is built from. `src/fw/pcbios/tests.rs` checks the *bytes*; this checks the
//! thing that actually matters, which is that a program running on the board
//! can find them by the searches the specifications define and read the right
//! answer out of them.
//!
//! The guest is a boot sector assembled with [`rsemu::fw::asm16`] — the same
//! assembler the firmware is written with — and it does what an operating
//! system's early boot does:
//!
//! 1. searches `F000:0000` upward on 16-byte boundaries for `_MP_`
//!    (*MultiProcessor Specification* §4.1),
//! 2. follows its physical pointer to the `PCMP` configuration table, sums the
//!    declared base table length and checks it comes out zero,
//! 3. steps through `ENTRY COUNT` entries, 20 bytes for a processor entry and
//!    8 for everything else (§4.3), counting the ones whose `EN` flag is set,
//! 4. searches the same segment for `RSD PTR ` (*ACPI* §5.2.5.1), follows the
//!    RSDT to the table whose signature is `APIC`, and counts its enabled
//!    Processor Local APIC structures (§5.2.12.2),
//! 5. leaves both counts in low memory and halts.
//!
//! # The two boards, and why both are needed
//!
//! One processor on `machines/pc-at.machine` as it ships, and two on the same
//! text with the second processor `tests/kvm_pc_at_smp.rs` adds. The
//! two-processor run is the claim that the tables are generated; **the
//! one-processor run is the claim that they are generated from the machine**,
//! because a firmware that published "two processors" unconditionally would
//! pass the first test and fail this one.
//!
//! No accelerator: the guest reads tables out of ROM and never starts the
//! second processor, so the interpreter is enough and this runs everywhere.
//! `tests/kvm_pc_at_smp.rs` is where a second processor actually executes.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-pc-hpet",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::fw::asm16::{AH, AL, AX, Alu, Asm, BX, CX, Cc, DI, DS, DX, ES, Mem, SI, SP, SS, Shift};
use rsemu::machine::{Machine, build};

// ---------------------------------------------------------------------------
// where the guest leaves what it found
// ---------------------------------------------------------------------------

/// Where the boot sector lands, and what its labels are relative to.
const BOOT: u16 = 0x7c00;

/// The block at `0x0500` every PC has left free since 1981.
const SCRATCH: u16 = 0x0500;

/// It ran at all.
const OFF_STARTED: u16 = SCRATCH;
/// How many enabled processor entries the MP configuration table had.
const OFF_MP_CPUS: u16 = SCRATCH + 2;
/// What its header said `ENTRY COUNT` was.
const OFF_MP_ENTRIES: u16 = SCRATCH + 4;
/// Whether its checksum came out zero.
const OFF_MP_SUM: u16 = SCRATCH + 6;
/// How many enabled Processor Local APIC structures the MADT had.
const OFF_MADT_CPUS: u16 = SCRATCH + 8;
/// How many tables the RSDT listed.
const OFF_RSDT_LEN: u16 = SCRATCH + 10;
/// The address the MP table says a processor reaches its own local APIC at.
const OFF_LAPIC: u16 = SCRATCH + 12;
/// It finished.
const OFF_DONE: u16 = SCRATCH + 16;

/// What [`OFF_STARTED`] holds.
const STARTED: u16 = 0xb105;
/// What [`OFF_DONE`] holds.
const DONE: u16 = 0x600d;

/// The BIOS segment, which is the whole of what the guest searches.
const BIOS_SEGMENT: u16 = 0xf000;
/// One below the top of it, so a signature that would straddle the end is not
/// read past the segment.
const SEARCH_END: u16 = 0xfff0;

/// `_MP_`, as the little-endian double word a search compares (*MP* §4.1).
const SIG_MP: u32 = 0x5f50_4d5f;
/// The first half of `RSD PTR ` (*ACPI* §5.2.5.3). The trailing blank is part
/// of the signature.
const SIG_RSD: u32 = 0x2044_5352;
/// Its second half.
const SIG_PTR: u32 = 0x2052_5450;
/// `APIC`, the MADT's signature (*ACPI* §5.2.12).
const SIG_APIC: u32 = 0x4349_5041;

// ---------------------------------------------------------------------------
// the guest
// ---------------------------------------------------------------------------

/// Assemble the boot sector that walks the tables.
///
/// Assembled into an image that starts at zero and taken from `0x7c00`, so the
/// assembler's absolute labels are the addresses the sector actually runs at.
#[allow(clippy::too_many_lines)]
fn boot_sector() -> Vec<u8> {
    let mut a = Asm::new(usize::from(BOOT) + 512, 0x00);
    a.seek(BOOT);

    let done = a.label();

    // Segments and a stack. `ES` stays at zero for the whole program, because
    // that is where the answers go; `DS` moves to the BIOS segment, because
    // that is where the tables are.
    a.cli();
    a.movi(AX, 0);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT);
    a.movsr(DS, AX);
    a.movmi(Mem::abs(OFF_STARTED), STARTED);
    a.movi(AX, BIOS_SEGMENT);
    a.movsr(DS, AX);

    // -- the MP floating pointer (*MP* §4.1) ---------------------------------
    //
    // "It must span a minimum of 16 contiguous bytes, beginning on a 16-byte
    // boundary", which is what makes this search legal rather than a scan.
    let mp_found = a.label();
    a.movi(SI, 0);
    let mp_scan = a.here_label();
    a.alui32(Alu::CMP, Mem::si(0), SIG_MP);
    a.jcc(Cc::E, mp_found);
    a.alui(Alu::ADD, SI, 16);
    a.alui(Alu::CMP, SI, SEARCH_END);
    a.jcc(Cc::B, mp_scan);
    a.jmp(done);

    a.bind(mp_found);
    // PHYSICAL ADDRESS POINTER at offset 4. Every table this firmware
    // publishes is inside the segment already loaded, so the low half of the
    // physical address is the offset within it.
    a.mov(BX, Mem::si(4));
    // ENTRY COUNT at 34, and ADDRESS OF LOCAL APIC at 36 (*MP* §4.2).
    a.mov(AX, Mem::bx(34));
    a.movto(Mem::abs(OFF_MP_ENTRIES).seg(ES), AX);
    a.mov(AX, Mem::bx(36));
    a.movto(Mem::abs(OFF_LAPIC).seg(ES), AX);
    a.mov(AX, Mem::bx(38));
    a.movto(Mem::abs(OFF_LAPIC + 2).seg(ES), AX);

    // The checksum: BASE TABLE LENGTH bytes from the start, which "must add up
    // to zero".
    let bad_sum = a.label();
    a.mov(CX, Mem::bx(4));
    a.mov(DI, BX);
    a.movi8(AL, 0);
    let sum_loop = a.here_label();
    a.alu8(Alu::ADD, AL, Mem::di(0));
    a.inc(DI);
    a.dec(CX);
    a.jcc(Cc::NE, sum_loop);
    a.movi8(AH, 0);
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::NE, bad_sum);
    a.movi8(AH, 1);
    a.bind(bad_sum);
    a.movto8(Mem::abs(OFF_MP_SUM).seg(ES), AH);

    // -- the entries (*MP* §4.3) ---------------------------------------------
    //
    // "Software must step through each entry in the base table until it reaches
    // ENTRY COUNT", and the length of an entry depends on its type: twenty
    // bytes for a processor entry and eight for every other one.
    let other = a.label();
    let step = a.label();
    let not_enabled = a.label();
    a.mov(CX, Mem::bx(34));
    a.mov(DI, BX);
    a.alui(Alu::ADD, DI, 44);
    a.movi(DX, 0);
    let walk = a.here_label();
    a.alui8(Alu::CMP, Mem::di(0), 0);
    a.jcc(Cc::NE, other);
    // CPU FLAGS bit 0, EN: "If zero, this processor is unusable".
    a.testi8(Mem::di(3), 1);
    a.jcc(Cc::E, not_enabled);
    a.inc(DX);
    a.bind(not_enabled);
    a.alui(Alu::ADD, DI, 20);
    a.jmp(step);
    a.bind(other);
    a.alui(Alu::ADD, DI, 8);
    a.bind(step);
    a.dec(CX);
    a.jcc(Cc::NE, walk);
    a.movto(Mem::abs(OFF_MP_CPUS).seg(ES), DX);

    // -- the RSDP (*ACPI* §5.2.5.1) ------------------------------------------
    //
    // "OSPM finds the Root System Description Pointer structure by searching
    // physical memory ranges on 16-byte boundaries for a valid Root System
    // Description Pointer structure signature".
    let rsdp_found = a.label();
    let rsdp_next = a.label();
    a.movi(SI, 0);
    let rsdp_scan = a.here_label();
    a.alui32(Alu::CMP, Mem::si(0), SIG_RSD);
    a.jcc(Cc::NE, rsdp_next);
    a.alui32(Alu::CMP, Mem::si(4), SIG_PTR);
    a.jcc(Cc::E, rsdp_found);
    a.bind(rsdp_next);
    a.alui(Alu::ADD, SI, 16);
    a.alui(Alu::CMP, SI, SEARCH_END);
    a.jcc(Cc::B, rsdp_scan);
    a.jmp(done);

    a.bind(rsdp_found);
    // RsdtAddress at offset 16 (*ACPI* Table 5.3). The RSDT rather than the
    // XSDT, because this guest is 16-bit and the XSDT's pointers are 64-bit —
    // which is exactly why the specification still requires both.
    a.mov(BX, Mem::si(16));
    // Length at offset 4 of the description header, less the header, in
    // four-byte entries (*ACPI* §5.2.7).
    a.mov(CX, Mem::bx(4));
    a.alui(Alu::SUB, CX, 36);
    a.shift(Shift::SHR, CX, 2);
    a.movto(Mem::abs(OFF_RSDT_LEN).seg(ES), CX);
    a.mov(DI, BX);
    a.alui(Alu::ADD, DI, 36);
    let madt_found = a.label();
    let entries = a.here_label();
    a.mov(BX, Mem::di(0));
    a.alui32(Alu::CMP, Mem::bx(0), SIG_APIC);
    a.jcc(Cc::E, madt_found);
    a.alui(Alu::ADD, DI, 4);
    a.dec(CX);
    a.jcc(Cc::NE, entries);
    a.jmp(done);

    // -- the MADT (*ACPI* §5.2.12) -------------------------------------------
    //
    // A list of variable-length structures, each with its own length byte, from
    // the end of the header plus the local APIC address and the flags.
    a.bind(madt_found);
    let madt_next = a.label();
    let madt_done = a.label();
    a.mov(CX, Mem::bx(4));
    a.alu(Alu::ADD, CX, BX);
    a.mov(DI, BX);
    a.alui(Alu::ADD, DI, 44);
    a.movi(DX, 0);
    let madt_walk = a.here_label();
    a.alu(Alu::CMP, DI, CX);
    a.jcc(Cc::AE, madt_done);
    a.alui8(Alu::CMP, Mem::di(0), 0);
    a.jcc(Cc::NE, madt_next);
    // Table 5.23's Local APIC Flags, bit 0: Enabled.
    a.testi8(Mem::di(4), 1);
    a.jcc(Cc::E, madt_next);
    a.inc(DX);
    a.bind(madt_next);
    a.mov8(AL, Mem::di(1));
    a.movi8(AH, 0);
    a.alu(Alu::ADD, DI, AX);
    a.jmp(madt_walk);
    a.bind(madt_done);
    a.movto(Mem::abs(OFF_MADT_CPUS).seg(ES), DX);

    a.bind(done);
    a.movmi(Mem::abs(OFF_DONE).seg(ES), DONE);
    let spin = a.here_label();
    a.hlt();
    a.jmp(spin);

    assert!(
        a.here() <= BOOT + 510,
        "the boot sector is {} bytes and 510 is all a sector has",
        a.here() - BOOT
    );
    a.seek(BOOT + 510);
    a.db(&[0x55, 0xaa]);

    let image = a.finish();
    image[usize::from(BOOT)..].to_vec()
}

/// A 1.44 MB diskette with that sector on it.
fn diskette() -> Vec<u8> {
    let mut image = boot_sector();
    assert_eq!(image.len(), 512, "a boot sector is one sector");
    image.resize(1_474_560, 0);
    image
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Build `text` with `bios` in its socket and the walker on the diskette.
fn board(name: &str, text: &str, bios: Vec<u8>) -> Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert("bios", bios);
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", diskette());
    for slot in ["disk", "hd0", "hd1", "cd0", "cd1"] {
        options.realize.media.insert(slot, Vec::new());
    }
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build(name, text, &registry, &options)
        .unwrap_or_else(|e| panic!("{name} does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    m
}

/// A word of guest memory, read as a debugger reads.
fn peek16(m: &Machine, at: u16) -> u16 {
    m.space("mem")
        .expect("the memory space")
        .read(u64::from(at), Width::U16, MemAttrs::DEBUG)
        .unwrap_or(0) as u16
}

/// Run until the guest says it finished, or for `ms` milliseconds of virtual
/// time.
fn run(m: &mut Machine, ms: usize) {
    for _ in 0..ms {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek16(m, OFF_DONE) == DONE {
            return;
        }
    }
}

/// What the guest found, once it has run.
#[derive(Debug, PartialEq, Eq)]
struct Found {
    mp_cpus: u16,
    mp_entries: u16,
    mp_sum_ok: bool,
    madt_cpus: u16,
    rsdt_tables: u16,
    lapic: u32,
}

fn walk(name: &str, text: &str, bios: Vec<u8>) -> Found {
    let mut m = board(name, text, bios);
    run(&mut m, 2000);
    assert_eq!(
        peek16(&m, OFF_STARTED),
        STARTED,
        "the boot sector never ran: `INT 19h` did not reach it"
    );
    assert_eq!(
        peek16(&m, OFF_DONE),
        DONE,
        "the guest did not finish walking the tables"
    );
    Found {
        mp_cpus: peek16(&m, OFF_MP_CPUS),
        mp_entries: peek16(&m, OFF_MP_ENTRIES),
        mp_sum_ok: peek16(&m, OFF_MP_SUM) & 0xff == 1,
        madt_cpus: peek16(&m, OFF_MADT_CPUS),
        rsdt_tables: peek16(&m, OFF_RSDT_LEN),
        lapic: u32::from(peek16(&m, OFF_LAPIC)) | (u32::from(peek16(&m, OFF_LAPIC + 2)) << 16),
    }
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn a_guest_finds_one_processor_on_the_board_that_has_one() {
    let found = walk(
        "pc-at.machine",
        rsemu::dev::pc::PC_AT,
        rsemu::fw::pcbios::image(),
    );
    assert!(found.mp_sum_ok, "the MP configuration table's checksum");
    assert_eq!(found.mp_cpus, 1, "one processor entry, enabled");
    assert_eq!(found.mp_entries, 12, "the whole base table was stepped");
    assert_eq!(found.madt_cpus, 1, "and the MADT agrees");
    assert_eq!(found.rsdt_tables, 2, "the FADT and the MADT");
    assert_eq!(found.lapic, 0xfee0_0000, "where the board maps `lapic0`");
}

#[test]
fn a_guest_finds_two_processors_on_the_board_that_has_two() {
    // The same firmware source, assembled for a board with a second processor
    // in its description. Nothing about the guest changes.
    let text = two_processor_at();
    let bios = rsemu::fw::pcbios::image_for_machine("pc-at-smp.machine", &text)
        .expect("the two-processor board resolves");
    let found = walk("pc-at-smp.machine", &text, bios);
    assert!(found.mp_sum_ok, "the MP configuration table's checksum");
    assert_eq!(found.mp_cpus, 2, "two processor entries, both enabled");
    assert_eq!(found.mp_entries, 13, "one entry more than one processor");
    assert_eq!(found.madt_cpus, 2, "and the MADT agrees");
    assert_eq!(found.lapic, 0xfee0_0000);
}

#[test]
fn the_default_image_on_a_two_processor_board_finds_only_one() {
    // The negative control, and the reason `image_for_machine` exists: the
    // stock image describes the stock board, so putting it in a board that
    // grew a processor publishes a table that has not. A firmware that
    // hard-coded two would pass the test above and fail this one.
    let text = two_processor_at();
    let found = walk("pc-at-smp.machine", &text, rsemu::fw::pcbios::image());
    assert_eq!(found.mp_cpus, 1);
    assert_eq!(found.madt_cpus, 1);
}

/// `machines/pc-at.machine` with the second processor `tests/kvm_pc_at_smp.rs`
/// adds, and for the same reason: a second processor on the shipped board is a
/// board decision, and this is the evidence for taking it rather than the
/// taking of it.
fn two_processor_at() -> String {
    let mut text = String::from(rsemu::dev::pc::PC_AT);

    const CPU0: &str = "  object cpu0 \"cpu.x86\" {\n\
                        \x20   clock   = cpu\n\
                        \x20   space   = mem\n\
                        \x20   iospace = \"port\"\n\
                        \x20   model   = \"80486\"\n\
                        \x20   engine  = \"interp\"\n\
                        \x20 }\n";
    assert!(text.contains(CPU0), "the `cpu0` object moved");
    text = text.replace(CPU0, &format!("{CPU0}{}", CPU0.replace("cpu0", "cpu1")));

    const APICS: &str = "  object lapic0 \"pc.lapic\"  { clock = bus, id = 0, bus = \"apic\" }\n  \
                         object ioapic \"pc.ioapic\" { id = 1, bus = \"apic\" }";
    assert!(text.contains(APICS), "the APIC objects moved");
    text = text.replace(
        APICS,
        "  object lapic0 \"pc.lapic\"  { clock = bus, id = 0, bus = \"apic\" }\n  \
         object lapic1 \"pc.lapic\"  { clock = bus, id = 1, bus = \"apic\" }\n  \
         object ioapic \"pc.ioapic\" { id = 2, bus = \"apic\" }",
    );

    const LAPIC0_MAP: &str = "  map mem 0xfee00000 size 0x1000   = lapic0.regs";
    assert!(text.contains(LAPIC0_MAP), "the lapic0 mapping moved");
    text = text.replace(
        LAPIC0_MAP,
        "  map mem 0xfee00000 size 0x1000   = lapic0.regs\n  \
         map mem 0xfef00000 size 0x1000   = lapic1.regs",
    );

    const LAPIC0_WIRES: &str = "  wire lapic0.intr -> cpu0.intr\n  wire lapic0.nmi  -> cpu0.nmi";
    assert!(text.contains(LAPIC0_WIRES), "the lapic0 wires moved");
    text = text.replace(
        LAPIC0_WIRES,
        "  wire lapic0.intr -> cpu0.intr\n  \
         wire lapic0.nmi  -> cpu0.nmi\n  \
         wire lapic1.intr -> cpu1.intr\n  \
         wire lapic1.nmi  -> cpu1.nmi",
    );
    text
}
