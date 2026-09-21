//! The `amiga-a4000t` board, end to end, with a ROM built in this file.
//!
//! The one thing that proves the SCSI I/O processor on the *board* rather than
//! in a rig is a 68040 program doing what `scsi.device` does — and for this
//! chip that is almost nothing, which is the point. The processor writes four
//! bytes into `DSP` and the **chip** does the rest: it fetches its own
//! instructions out of the Kickstart socket, arbitrates for the cable, selects
//! the drive, sends an `IDENTIFY` and a command descriptor block, takes 512
//! bytes straight into chip RAM by mastering the bus, takes the status byte and
//! the `COMMAND COMPLETE`, waits for the bus to go free and raises an
//! interrupt. The interrupt has to come the whole way — the chip's `IRQ/`, the
//! `INT2` net, Paula's `INTREQ`, the processor's autovector — or the program
//! waits for ever and the test says so.
//!
//! It also pins the two things the model got wrong before a real ROM found
//! them, because both are visible from a program: which byte of `DSP` starts
//! the processor (the one at the highest address, which a `move.l` writes
//! last), and that the register file answers in all three of the copies the
//! board's decode makes.
//!
//! No Kickstart and no disk image from anywhere: the ROM is hand-assembled from
//! the MC68000 User's Manual's instruction formats and the SCRIPTS program from
//! the NCR 53C710 Data Manual's instruction formats, and the disk is bytes made
//! here. `tests/amiga_a4000t.rs` is where real ones run.

#![cfg(feature = "machine-amiga-a4000t")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// where things are
// ---------------------------------------------------------------------------

/// Where the program leaves its results in chip RAM. Above `$400`, which is
/// where the 68000 exception vectors stop.
const COUNT: u64 = 0x400; // interrupts taken
const RESULT: u64 = 0x402; // the low half of DSPS: the program's own vector
const DSTAT_SEEN: u64 = 0x404;
const SSTAT_SEEN: u64 = 0x405;

/// The buffers the SCRIPTS program names, also in chip RAM.
const IDENTIFY_AT: u64 = 0x410;
const CDB_AT: u64 = 0x420;
const STATUS_AT: u64 = 0x430;
const MESSAGE_AT: u64 = 0x432;
const BUFFER: u64 = 0x1000;

/// The level 2 handler's offset in the ROM, and the SCRIPTS program's.
const HANDLER: usize = 0x300;
const SCRIPT: usize = 0x400;

/// Where the ROM answers, so a SCRIPTS address in it is that plus the offset.
const ROM: u32 = 0x00F8_0000;

/// How many 512-byte blocks the test disk holds, and which one is read.
const BLOCKS: usize = 64;
const LBA: u8 = 5;

/// Where `machines/amiga-a4000t.machine` decodes the chip, and the two further
/// copies its decode makes.
const REGS: u64 = 0x00DD_0040;
const MIRROR1: u64 = 0x00DD_0080;
const MIRROR2: u64 = 0x00DD_00C0;

/// The drive's address on the cable, which the board's `scsi-id` parameter
/// defaults to.
const TARGET: u8 = 1;

/// The vectors the SCRIPTS program hands back in `DSPS`.
const GOOD: u16 = 0x600D;
const BAD: u16 = 0xBAD0;

/// Where a register the data manual numbers `n` answers, in the board's
/// big-endian lane order.
const fn reg(n: u64) -> u32 {
    (REGS + (n ^ 3)) as u32
}

// ---------------------------------------------------------------------------
// the 68040 program
// ---------------------------------------------------------------------------

/// One `move.b #imm,(xxx).L`.
fn poke(words: &mut Vec<u16>, value: u8, at: u32) {
    words.extend_from_slice(&[0x13FC, u16::from(value), (at >> 16) as u16, at as u16]);
}

/// One `move.l #imm,(xxx).L`.
fn poke_long(words: &mut Vec<u16>, value: u32, at: u32) {
    words.extend_from_slice(&[
        0x23FC,
        (value >> 16) as u16,
        value as u16,
        (at >> 16) as u16,
        at as u16,
    ]);
}

/// A 512 KiB ROM: the reset longwords, the program at `$F80010`, the level 2
/// handler at `$F80300` and the SCRIPTS program at `$F80400`.
///
/// ```text
///   ; --- the program -------------------------------------------------------
///   ; It runs from the ROM's own window, because the first CIA write takes the
///   ; overlay — and the ROM at zero with it — away.
///   move.b #0,$00BFE001        ; CIA-A PRA: PA0 low …
///   move.b #1,$00BFE201        ; … and DDRA makes it an output: OVL drops
///   clr.w  $400.w              ; interrupts taken
///   clr.w  $402.w              ; the vector the program came back with
///   move.l #$00F80300,$68.w    ; the level 2 autovector
///   move.w #$C008,$00DFF09A    ; INTENA: SET | INTEN | PORTS
///   move   #$2000,sr           ; and let them in
///
///   move.b #$C0,$410           ; IDENTIFY | DiscPriv, LUN 0
///   move.b #$08,$420           ; READ(6) of one block at LBA 5
///   …                          ; five more bytes
///   move.b #$FF,$430           ; the status byte, so GOOD is visible
///   move.b #$FF,$432           ; and the message
///
///   move.b #$FF,$00DD007A      ; DIEN: every DMA interrupt
///   move.b #$AF,$00DD0040      ; SIEN: every SCSI one but FCMP and SEL
///   move.l #$00F80400,$00DD006C ; DSP — and the fourth byte starts it
///
///   cmpi.w #1,$400.w           ; wait for the interrupt
///   blt.s  *-6
///   bra.s  *
///
///   ; --- the level 2 handler ----------------------------------------------
///   movem.l d0-d1,-(sp)
///   move.w $00DD0072,d0        ; DSPS, low half: the program's own vector
///   move.w d0,$402.w
///   move.b $00DD004E,d0        ; SSTAT0 — and reading it clears SIP
///   move.b d0,$405.w
///   move.b $00DD004F,d0        ; DSTAT  — and reading it clears DIP
///   move.b d0,$404.w
///   addq.w #1,$400.w
///   move.w #$0008,$00DFF09C    ; INTREQ: let go of PORTS
///   movem.l (sp)+,d0-d1
///   rte
/// ```
fn kickstart() -> Vec<u8> {
    let mut main: Vec<u16> = Vec::new();
    // The overlay: the level first, so the pin never glitches high.
    poke(&mut main, 0x00, 0x00BF_E001);
    poke(&mut main, 0x01, 0x00BF_E201);
    main.extend_from_slice(&[0x4278, COUNT as u16]);
    main.extend_from_slice(&[0x4278, RESULT as u16]);
    main.extend_from_slice(&[0x21FC, 0x00F8, 0x0300, 0x0068]);
    main.extend_from_slice(&[0x33FC, 0xC008, 0x00DF, 0xF09A]);
    main.extend_from_slice(&[0x46FC, 0x2000]);

    // The message-out byte, the command descriptor block and two bytes the
    // target will overwrite.
    poke(&mut main, 0xC0, IDENTIFY_AT as u32);
    for (i, byte) in [0x08u8, 0x00, 0x00, LBA, 0x01, 0x00]
        .into_iter()
        .enumerate()
    {
        poke(&mut main, byte, CDB_AT as u32 + i as u32);
    }
    poke(&mut main, 0xFF, STATUS_AT as u32);
    poke(&mut main, 0xFF, MESSAGE_AT as u32);

    // The chip: the two masks, and then the address that starts it.
    poke(&mut main, 0xFF, reg(0x39)); // DIEN
    poke(&mut main, 0xAF, reg(0x03)); // SIEN
    poke_long(&mut main, ROM + SCRIPT as u32, REGS as u32 + 0x2C); // DSP

    main.extend_from_slice(&[0x0C78, 0x0001, COUNT as u16, 0x6DF8]);
    main.push(0x60FE);

    let handler: &[u16] = &[
        0x48E7,
        0xC000, // movem.l d0-d1,-(sp)
        0x3039,
        0x00DD,
        0x0072, // move.w $00DD0072,d0
        0x31C0,
        RESULT as u16, // move.w d0,$402.w
        0x1039,
        0x00DD,
        0x004E, // move.b $00DD004E,d0
        0x11C0,
        SSTAT_SEEN as u16, // move.b d0,$405.w
        0x1039,
        0x00DD,
        0x004F, // move.b $00DD004F,d0
        0x11C0,
        DSTAT_SEEN as u16, // move.b d0,$404.w
        0x5278,
        COUNT as u16, // addq.w #1,$400.w
        0x33FC,
        0x0008,
        0x00DF,
        0xF09C, // move.w #$0008,$00DFF09C
        0x4CDF,
        0x0003, // movem.l (sp)+,d0-d1
        0x4E73, // rte
    ];

    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0020_0000u32.to_be_bytes()); // SSP: top of 2 MiB
    image[4..8].copy_from_slice(&0x00F8_0010u32.to_be_bytes()); // PC: the ROM's own window
    for (base, code) in [(0x10usize, &main[..]), (HANDLER, handler)] {
        for (i, word) in code.iter().enumerate() {
            let at = base + 2 * i;
            image[at..at + 2].copy_from_slice(&word.to_be_bytes());
        }
    }
    assert!(
        0x10 + 2 * main.len() < HANDLER,
        "the program fits below the handler"
    );
    assert!(
        HANDLER + 2 * handler.len() < SCRIPT,
        "the handler fits below the SCRIPTS program"
    );
    for (i, word) in scripts().iter().enumerate() {
        let at = SCRIPT + 4 * i;
        image[at..at + 4].copy_from_slice(&word.to_be_bytes());
    }
    image
}

// ---------------------------------------------------------------------------
// the SCRIPTS program
// ---------------------------------------------------------------------------

/// The whole SCSI operation, as the chip's own instruction set spells it.
///
/// ```text
///   Select with ATN, address 1, on failure jump $00F80440
///   Move 1   byte  when MSG_OUT  from $410
///   Move 6   bytes when CMD      from $420
///   Move 512 bytes when DATA_IN  to   $1000
///   Move 1   byte  when STATUS   to   $430
///   Move 1   byte  when MSG_IN   to   $432
///   Wait Disconnect
///   Interrupt $0000600D
///   Interrupt $0000BAD0          ; where a failed selection lands
/// ```
fn scripts() -> Vec<u32> {
    /// `DCMD` for a Block Move in the phase `MSG`, `C/D`, `I/O` spell.
    const MSG_OUT: u32 = 0b110;
    const CMD: u32 = 0b010;
    const DATA_IN: u32 = 0b001;
    const STATUS: u32 = 0b011;
    const MSG_IN: u32 = 0b111;
    /// `DBC` bit 19, "jump when the comparison is true", which with no
    /// comparison asked for is an unconditional instruction.
    const TRUE: u32 = 0x0008_0000;

    let fail = ROM + SCRIPT as u32 + 0x40;
    vec![
        // 01 000 0 0 1 : I/O, opcode 0 (Select), with ATN. The destination is
        // the *bus line*, in bits 23-16.
        0x4100_0000 | (u32::from(1u8 << TARGET) << 16),
        fail,
        MSG_OUT << 24 | 1,
        IDENTIFY_AT as u32,
        CMD << 24 | 6,
        CDB_AT as u32,
        DATA_IN << 24 | 512,
        BUFFER as u32,
        STATUS << 24 | 1,
        STATUS_AT as u32,
        MSG_IN << 24 | 1,
        MESSAGE_AT as u32,
        // 01 001 000 : I/O, opcode 1, Wait Disconnect.
        0x4800_0000,
        0,
        // 10 011 000 : Transfer Control, opcode 3, Interrupt.
        0x9800_0000 | TRUE,
        u32::from(GOOD),
        0x9800_0000 | TRUE,
        u32::from(BAD),
    ]
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// `BLOCKS` blocks whose every byte says which block it is, so a transfer that
/// lands on the wrong one is visible.
fn disk() -> Vec<u8> {
    let mut image = vec![0u8; BLOCKS * 512];
    for (n, block) in image.chunks_mut(512).enumerate() {
        block.fill(n as u8);
    }
    image[..4].copy_from_slice(b"RDSK");
    image
}

fn build(scsi0: Vec<u8>) -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", kickstart());
    options.realize.media.insert("scsi0", scsi0);
    options.realize.media.insert("hd0", Vec::new());
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a4000t")
        .expect("this build ships amiga-a4000t")
        .source;
    rsemu::machine::build("amiga-a4000t", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"))
}

fn peek(m: &Machine, at: u64, width: Width) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(at, width, MemAttrs::DEBUG)
        .expect("guest memory")
}

/// Run until the program has taken its interrupt, or give up.
fn run(m: &mut Machine) {
    for _ in 0..200 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        if peek(m, COUNT, Width::U16) != 0 {
            return;
        }
    }
}

#[test]
fn the_board_realizes_with_both_disk_ports_on_it() {
    let m = build(disk());
    assert_eq!(m.name(), "amiga-a4000t");
    for path in [
        "cpu", "chipram", "fastram", "kick", "custom", "paula", "agnus", "denise", "rtc", "ramsey",
        "cia_a", "cia_b", "sd0", "hd0", "ncr", "ide", "overlay", "df0", "kbd", "mouse",
    ] {
        assert!(m.device(path).is_some(), "no instance called `{path}`");
    }
}

#[test]
fn the_board_names_exactly_the_media_slots_it_documents() {
    use rsemu::machine::{ResolveOptions, resolve_file};
    let source = catalog::machine("amiga-a4000t").expect("shipped").source;
    let resolved =
        resolve_file("amiga-a4000t.machine", source, &ResolveOptions::new()).expect("it resolves");
    let mut slots: Vec<String> = resolved
        .objects
        .iter()
        .filter_map(|o| o.props.get("image"))
        .filter_map(|v| v.as_str().map(ToString::to_string))
        .collect();
    slots.sort();
    slots.dedup();
    // The Kickstart socket, both drives and the internal floppy.
    assert_eq!(slots, ["df0", "hd0", "kickstart", "scsi0"]);
    assert_eq!(
        catalog::machine("amiga-a4000t").expect("shipped").media,
        ["kickstart", "scsi0", "hd0", "df0"]
    );
}

/// The register file is decoded by six address lines, so it answers three
/// times over in the `$C0` bytes the board maps — which is not a nicety: the
/// A4000T's Kickstart writes `DSA` in the third copy and its own SCRIPTS
/// program writes it in the first, and both have to reach the same register.
#[test]
fn the_register_file_answers_in_all_three_copies() {
    let m = build(disk());
    let space = m.space("mem").expect("the memory space");
    // `SCRATCH` is a longword of scratch with nothing behind it, so writing it
    // is a question about the decode and nothing else. Manual number `34`,
    // which in big-endian lane order is the four bytes at offset `$34`.
    for (n, base) in [REGS, MIRROR1, MIRROR2].into_iter().enumerate() {
        let value = 0x1234_0000 + n as u64;
        space
            .write(base + 0x34, Width::U32, value, MemAttrs::DEFAULT)
            .expect("a register");
        for other in [REGS, MIRROR1, MIRROR2] {
            assert_eq!(
                space
                    .read(other + 0x34, Width::U32, MemAttrs::DEBUG)
                    .expect("a register"),
                value,
                "written at {base:#010x}, read at {other:#010x}"
            );
        }
    }
}

/// A write to `DSP` starts the processor, and it is the byte at the **highest
/// address** that does it — which for a 68040's `move.l` is the last one out.
///
/// Asserted by halves: the top word of the address on its own leaves the chip
/// alone, and the bottom word sets it going. A model that started on the first
/// byte would run from `$00F80000`, which is the Kickstart's stack pointer.
///
/// The witness that it started is `SSTAT0`'s `FCMP` — the chip arbitrated for
/// the cable and selected the drive — because that is the one thing the first
/// instruction does without reading a byte of chip RAM. The processor has not
/// run here, so `OVL` is still up and chip RAM is still the ROM; what the rest
/// of the program would find there is `a_program_reads_a_block_over_scsi…`'s
/// business.
#[test]
fn the_top_half_of_dsp_does_not_start_the_processor() {
    let m = build(disk());
    let space = m.space("mem").expect("the memory space");
    let at = REGS + 0x2C;
    space
        .write(at, Width::U16, 0x00F8, MemAttrs::DEFAULT)
        .expect("a register");
    // `DSTAT` reads `$80` — `DFE`, the empty FIFO — and nothing else, and
    // nothing has happened on the cable.
    assert_eq!(
        peek(&m, u64::from(reg(0x0c)), Width::U8),
        0x80,
        "the processor stayed put"
    );
    assert_eq!(peek(&m, u64::from(reg(0x0d)), Width::U8), 0x00);
    space
        .write(at + 2, Width::U16, SCRIPT as u64, MemAttrs::DEFAULT)
        .expect("a register");
    assert_eq!(
        peek(&m, u64::from(reg(0x0d)), Width::U8) & 0x40,
        0x40,
        "and the fourth byte set it going: it selected the drive"
    );
}

/// The whole of it, on the board: four bytes into `DSP` and the chip does a
/// SCSI operation by itself, all the way to the autovector.
#[test]
fn a_program_reads_a_block_over_scsi_with_four_bytes_of_setup() {
    let mut m = build(disk());
    run(&mut m);
    assert_ne!(
        peek(&m, COUNT, Width::U16),
        0,
        "no interrupt ever arrived: DSTAT {:#04x}, SSTAT0 {:#04x}",
        peek(&m, u64::from(reg(0x0c)), Width::U8),
        peek(&m, u64::from(reg(0x0d)), Width::U8),
    );
    assert_eq!(
        peek(&m, RESULT, Width::U16) as u16,
        GOOD,
        "the program came back by its own failure path"
    );
    // `DFE | SIR`: the FIFO is empty and a SCRIPTS `Interrupt` instruction is
    // why the pin came up.
    assert_eq!(peek(&m, DSTAT_SEEN, Width::U8), 0x84);
    // `FCMP` and nothing else: the selection completed, and every condition
    // that would have been an error is clear. The bit is set although `SIEN`
    // masks it, because a status register records what happened whether or not
    // it was asked to interrupt about it.
    assert_eq!(peek(&m, SSTAT_SEEN, Width::U8), 0x40);
    // The status byte and the message the target sent.
    assert_eq!(peek(&m, STATUS_AT, Width::U8), 0x00, "GOOD");
    assert_eq!(peek(&m, MESSAGE_AT, Width::U8), 0x00, "COMMAND COMPLETE");
    // Every byte of the block, where the instruction said to put it.
    for i in 0..512 {
        assert_eq!(
            peek(&m, BUFFER + i, Width::U8) as u8,
            LBA,
            "byte {i} of the block"
        );
    }
    // And nothing either side of it.
    assert_eq!(peek(&m, BUFFER - 2, Width::U16), 0);
    assert_eq!(peek(&m, BUFFER + 512, Width::U16), 0);
}

/// An empty cable: the selection times out, the chip takes the instruction's
/// own alternate path, and the interrupt still arrives — an address with
/// nothing at it is a report, not a hang.
#[test]
fn with_no_disk_the_selection_times_out_and_the_program_says_so() {
    let mut m = build(Vec::new());
    run(&mut m);
    assert_ne!(peek(&m, COUNT, Width::U16), 0, "an interrupt still arrived");
    // `SSTAT0`'s `STO`: "selection or reselection timed out".
    assert_eq!(peek(&m, SSTAT_SEEN, Width::U8), 0x20);
    // Nothing was read, and the buffer is as the board left it.
    assert_eq!(peek(&m, BUFFER, Width::U16), 0);
    assert_eq!(peek(&m, STATUS_AT, Width::U8), 0xFF, "no status came back");
}
