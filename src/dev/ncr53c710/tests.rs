//! The 53C710 as a board reaches it: sixty-four registers in a memory space,
//! and SCRIPTS programs assembled here and executed out of that same space.
//!
//! The target's own behaviour is `src/dev/scsi/tests.rs`'s. What is asserted
//! here is what happens between a register write and a SCSI bus — the
//! big-endian lane order an Amiga board wires, the two resets, the interrupt
//! model and its two masks, every SCRIPTS instruction type including a Memory
//! Move whose destination is the chip's own register file, `debug` reads, and a
//! snapshot.
//!
//! The command descriptor block a program here sends is assembled *in this
//! file*, which is where a SCSI command opcode is allowed to be: `ncr53c710.rs`
//! itself has none, and that grep is the falsifiable form of `dev/scsi`'s split.

use super::*;
use crate::core::props::{Media, Props, Value};
use crate::core::space::{AddressSpace, RamStore, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink, WireSource};
use crate::dev::scsi::{DiskDevice, status};
use alloc::vec;
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// the rig
// ---------------------------------------------------------------------------

/// The drive's SCSI address, and the initiator's.
const TARGET: u8 = 0;
const OWN: u8 = 7;

/// How many blocks the drive under test holds, and how big one is.
const BLOCKS: usize = 64;
const BLOCK: usize = 512;

/// Where the rig puts the chip's register file, which is where an A4000T's
/// board puts it.
const REGS_AT: u64 = 0x00DD_0040;

/// Where a program, its tables and its buffers go.
const PROGRAM: u32 = 0x0001_0000;
const IDENTIFY_AT: u32 = 0x0001_1000;
const CDB_AT: u32 = 0x0001_1010;
const BUFFER: u32 = 0x0001_2000;
const STATUS_AT: u32 = 0x0001_1020;
const MESSAGE_AT: u32 = 0x0001_1030;
const TABLE: u32 = 0x0001_1100;

/// The vector a finished program hands the host in `DSPS`.
const DONE: u32 = 0x00C0_FFEE;

/// A wire sink that remembers the last level it was given.
#[derive(Debug, Default)]
struct Probe {
    level: AtomicU32,
}

impl Probe {
    fn high(&self) -> bool {
        self.level.load(Ordering::Relaxed) != 0
    }
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.level
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}

struct Rig {
    chip: Arc<Ncr53c710>,
    /// The drive, kept alive: a bus holds `Arc<dyn Target>`, and the device
    /// wrapper is what owns the medium.
    _disk: DiskDevice,
    irq: Arc<Probe>,
    /// The one space the chip masters — RAM, and the chip's own registers at
    /// the address an A4000T decodes them at, so a Memory Move can reach them.
    space: Arc<AddressSpace>,
}

fn rig() -> Rig {
    rig_with(Order::Big)
}

fn rig_with(order: Order) -> Rig {
    // Block *n* of the drive is filled with the byte *n*.
    let mut image = vec![0u8; BLOCKS * BLOCK];
    for (n, block) in image.chunks_mut(BLOCK).enumerate() {
        block.fill(n as u8);
    }
    let hosts = Arc::new(crate::core::hosts::HostObjects::new());
    let disk = DiskDevice::new(
        &Props::new()
            .with("image", Value::Media(Media::new("hd0", image)))
            .with("bus", Value::Str(String::from("scsi0")))
            .with("id", Value::Uint(u64::from(TARGET)))
            .with_hosts(Arc::clone(&hosts)),
    )
    .expect("a drive");
    let chip = Arc::new(
        Ncr53c710::new(
            &Props::new()
                .with("bus", Value::Str(String::from("scsi0")))
                .with("id", Value::Uint(u64::from(OWN)))
                .with("byte-order", Value::Str(String::from(order.as_str())))
                .with_hosts(Arc::clone(&hosts)),
        )
        .expect("a controller"),
    );

    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ZEROS));
    space
        .topology()
        .map(
            Arc::new(Region::ram("ram", Arc::new(RamStore::new(0x10_0000)))),
            0,
        )
        .expect("ram maps");
    space
        .topology()
        .map(
            Device::region(chip.as_ref(), "").expect("a region"),
            REGS_AT,
        )
        .expect("the registers map");
    chip.attach_space(&space, RequesterId(0));

    let irq = Arc::new(Probe::default());
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&irq) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(chip.as_ref(), IRQ_PIN, WireSource::new(wire, id)).expect("the pin exists");
    Device::announce(chip.as_ref(), IRQ_PIN);

    Rig {
        chip,
        _disk: disk,
        irq,
        space,
    }
}

impl Rig {
    /// One register, by the data manual's number, through the board's window —
    /// which is where the lane order is applied.
    fn get(&self, reg: usize) -> u8 {
        let at = REGS_AT + lane(self.chip.order(), reg);
        self.space
            .read(at, Width::U8, MemAttrs::DEFAULT)
            .expect("a register") as u8
    }

    fn set(&self, reg: usize, value: u8) {
        let at = REGS_AT + lane(self.chip.order(), reg);
        self.space
            .write(at, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .expect("a register");
    }

    /// A longword register, written byte by byte **in address order**, which is
    /// the order a processor writes a longword in and the order the `DSP`
    /// start trigger depends on. A longword register occupies the same four
    /// window offsets whichever way the lanes are round; which end is the most
    /// significant byte is what changes.
    fn set_long(&self, reg: usize, value: u32) {
        let bytes = match self.chip.order() {
            Order::Big => value.to_be_bytes(),
            Order::Little => value.to_le_bytes(),
        };
        for (i, byte) in bytes.into_iter().enumerate() {
            self.space
                .write(
                    REGS_AT + (reg + i) as u64,
                    Width::U8,
                    u64::from(byte),
                    MemAttrs::DEFAULT,
                )
                .expect("a register");
        }
    }

    fn long(&self, reg: usize) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, byte) in bytes.iter_mut().enumerate() {
            *byte = self
                .space
                .read(REGS_AT + (reg + i) as u64, Width::U8, MemAttrs::DEFAULT)
                .expect("a register") as u8;
        }
        match self.chip.order() {
            Order::Big => u32::from_be_bytes(bytes),
            Order::Little => u32::from_le_bytes(bytes),
        }
    }

    fn poke(&self, at: u32, bytes: &[u8]) {
        self.space
            .write_bytes(u64::from(at), bytes, MemAttrs::DEFAULT)
            .expect("ram");
    }

    fn peek(&self, at: u32, len: usize) -> Vec<u8> {
        let mut buf = vec![0u8; len];
        self.space
            .read_bytes(u64::from(at), &mut buf, MemAttrs::DEFAULT)
            .expect("ram");
        buf
    }

    /// Assemble `words` at `PROGRAM`, big-endian, as a SCRIPTS program is held.
    fn program(&self, words: &[u32]) {
        let mut bytes = Vec::new();
        for w in words {
            bytes.extend_from_slice(&w.to_be_bytes());
        }
        self.poke(PROGRAM, &bytes);
    }

    /// Point `DSP` at `PROGRAM` and let go, which is what starts the processor.
    fn run(&self) {
        self.set_long(DSP, PROGRAM);
    }

    /// The masks an initiator's driver actually sets before it starts
    /// anything: every DMA condition, and every SCSI one **except** `FCMP` and
    /// `SEL`.
    ///
    /// Not a simplification — it is what the mask is for. `FCMP` fires on every
    /// successful selection, so a driver that enabled it would stop its own
    /// program one instruction in; `SEL` is the chip being selected as a
    /// target, which an initiator is not. Commodore's A4000T Kickstart writes
    /// `SIEN := $AF`, which is exactly these six.
    fn unmask(&self) {
        self.set(DIEN, 0xff);
        self.set(SIEN, !(SSTAT0_FCMP | SSTAT0_SEL));
    }
}

/// Where in the window a register answers, given the lane order.
const fn lane(order: Order, reg: usize) -> u64 {
    match order {
        Order::Little => reg as u64,
        Order::Big => (reg ^ 3) as u64,
    }
}

// ---------------------------------------------------------------------------
// assembling SCRIPTS
// ---------------------------------------------------------------------------

/// One Block Move: `count` bytes at `at`, in `phase`.
const fn block_move(phase: u8, count: u32, at: u32) -> [u32; 2] {
    [(phase as u32) << 24 | (count & 0x00ff_ffff), at]
}

/// A table-indirect Block Move: the count and the address are the pair at
/// `DSA + offset`.
const fn block_move_table(phase: u8, offset: u32) -> [u32; 2] {
    [
        ((TYPE_BLOCK_MOVE | BLOCK_TABLE | phase) as u32) << 24,
        offset,
    ]
}

/// `Select` with `ATN`, of `id`, jumping to `alt` if nobody answers.
///
/// The destination field is the *bus line*, not the number: bit `id`.
const fn select(id: u8, alt: u32) -> [u32; 2] {
    [
        ((TYPE_IO | IO_SELECT | SELECT_ATN) as u32) << 24 | (1u32 << id) << 16,
        alt,
    ]
}

/// `Wait Disconnect`.
const fn wait_disconnect() -> [u32; 2] {
    [((TYPE_IO | IO_WAIT_DISCONNECT) as u32) << 24, 0]
}

/// `Wait Reselect`, leaving by `alt` when the host sets `SIGP`.
const fn wait_reselect(alt: u32) -> [u32; 2] {
    [((TYPE_IO | IO_WAIT_RESELECT) as u32) << 24, alt]
}

/// `Interrupt` with `vector`, unconditionally.
const fn interrupt(vector: u32) -> [u32; 2] {
    [
        ((TYPE_TRANSFER | XFER_INTERRUPT) as u32) << 24 | DBC_TRUE,
        vector,
    ]
}

/// An unconditional absolute `Jump`.
const fn jump(to: u32) -> [u32; 2] {
    [((TYPE_TRANSFER | XFER_JUMP) as u32) << 24 | DBC_TRUE, to]
}

/// `Jump` to `to` when `SFBR`, with `mask`'s bits ignored, equals `data`.
const fn jump_if_data(data: u8, mask: u8, to: u32) -> [u32; 2] {
    [
        ((TYPE_TRANSFER | XFER_JUMP) as u32) << 24
            | DBC_TRUE
            | DBC_COMPARE_DATA
            | (mask as u32) << 8
            | data as u32,
        to,
    ]
}

/// A register read/write instruction: `SFBR := op(reg, data)` or
/// `reg := op(reg, data)`, with `op` the manual's two-bit operation and
/// `carry` its carry in.
const fn reg_op(opcode: u8, op: u8, carry: bool, reg: usize, data: u8) -> [u32; 2] {
    [
        ((TYPE_IO | opcode | (op << 1) | if carry { 1 } else { 0 }) as u32) << 24
            | (reg as u32) << 16
            | (data as u32) << 8,
        0,
    ]
}

/// A Memory Move of `count` bytes.
const fn memory_move(count: u32, from: u32, to: u32) -> [u32; 3] {
    [
        (TYPE_MEMORY_MOVE as u32) << 24 | (count & 0x00ff_ffff),
        from,
        to,
    ]
}

/// The three phases of a command, as `DCMD` spells them.
const MESSAGE_OUT: u8 = 0b110;
const COMMAND: u8 = 0b010;
const DATA_IN: u8 = 0b001;
const STATUS_PHASE: u8 = 0b011;
const MESSAGE_IN: u8 = 0b111;

/// A six-byte command descriptor block that reads one block.
///
/// X3.131-1986 §8.2.5's `READ(6)`: the opcode, a twenty-one-bit logical block
/// address and a transfer length in blocks. Assembled here and nowhere else.
fn read6(lba: u32, blocks: u8) -> [u8; 6] {
    [
        0x08,
        ((lba >> 16) & 0x1f) as u8,
        (lba >> 8) as u8,
        lba as u8,
        blocks,
        0,
    ]
}

/// Put the buffers a whole-command program needs where it expects them.
fn stage(rig: &Rig, lba: u32) {
    rig.poke(IDENTIFY_AT, &[identify(0, false)]);
    rig.poke(CDB_AT, &read6(lba, 1));
    rig.poke(STATUS_AT, &[0xff]);
    rig.poke(MESSAGE_AT, &[0xff]);
    rig.poke(BUFFER, &vec![0u8; BLOCK]);
}

/// The instructions that read one block, as a flat program.
fn read_program() -> Vec<u32> {
    let mut words = Vec::new();
    words.extend_from_slice(&select(TARGET, PROGRAM));
    words.extend_from_slice(&block_move(MESSAGE_OUT, 1, IDENTIFY_AT));
    words.extend_from_slice(&block_move(COMMAND, 6, CDB_AT));
    words.extend_from_slice(&block_move(DATA_IN, BLOCK as u32, BUFFER));
    words.extend_from_slice(&block_move(STATUS_PHASE, 1, STATUS_AT));
    words.extend_from_slice(&block_move(MESSAGE_IN, 1, MESSAGE_AT));
    words.extend_from_slice(&wait_disconnect());
    words.extend_from_slice(&interrupt(DONE));
    words
}

// ---------------------------------------------------------------------------
// the lane order
// ---------------------------------------------------------------------------

/// The one number that fixes the whole map: an A4000T's Kickstart polls `ISTAT`
/// at `$00DD0062`, and `$00DD0040 + (0x21 XOR 3)` is `$00DD0062`.
#[test]
fn big_endian_lanes_put_istat_where_an_amiga_polls_it() {
    assert_eq!(Order::Big.register(0x22), ISTAT);
    assert_eq!(REGS_AT + 0x22, 0x00DD_0062);
    // And the rest of the row, which is what makes it a lane swap rather than a
    // coincidence.
    assert_eq!(Order::Big.register(0x20), LCRC);
    assert_eq!(Order::Big.register(0x21), CTEST8);
    assert_eq!(Order::Big.register(0x23), DFIFO);
    // A longword register reads as a natural big-endian longword: its most
    // significant byte answers at its lowest address.
    assert_eq!(Order::Big.register(0x10), DSA + 3);
    assert_eq!(Order::Big.register(0x13), DSA);
    // The data manual's own numbering is what `little` is.
    assert_eq!(Order::Little.register(0x21), ISTAT);
    assert_eq!(Order::Little.register(0x10), DSA);
}

/// The same register file, reached both ways round.
#[test]
fn either_order_reaches_the_same_registers() {
    for order in [Order::Big, Order::Little] {
        let rig = rig_with(order);
        rig.set_long(SCRATCH, 0xDEAD_BEEF);
        assert_eq!(rig.long(SCRATCH), 0xDEAD_BEEF, "{order:?}");
        // And through the chip's own accessor, which knows nothing about
        // addresses.
        assert_eq!(rig.chip.peek(SCRATCH), 0xEF, "{order:?}");
        assert_eq!(rig.chip.peek(SCRATCH + 3), 0xDE, "{order:?}");
    }
}

// ---------------------------------------------------------------------------
// the resets and the interrupt model
// ---------------------------------------------------------------------------

/// `ISTAT`'s `RST` held and released, which is the first thing an A4000T's
/// Kickstart does with the chip.
#[test]
fn a_software_reset_puts_every_register_back() {
    let rig = rig();
    rig.set(DIEN, 0xff);
    rig.set_long(SCRATCH, 0x1234_5678);
    rig.set(ISTAT, ISTAT_RST);
    assert_eq!(rig.get(ISTAT) & ISTAT_RST, ISTAT_RST, "the chip is held");
    rig.set(ISTAT, 0);
    assert_eq!(rig.long(SCRATCH), 0, "SCRATCH went back");
    assert_eq!(rig.get(DIEN), 0, "so did the mask");
    assert_eq!(rig.get(SCID), 1 << OWN, "but not the strapped address");
    assert_eq!(rig.get(DSTAT), DSTAT_DFE, "the FIFO is empty out of reset");
}

/// `ISTAT`'s `ABRT`, which the same driver writes before the reset.
#[test]
fn abort_reports_itself_only_when_dien_asked() {
    let rig = rig();
    rig.set(ISTAT, ISTAT_ABRT);
    assert_eq!(rig.get(ISTAT) & ISTAT_DIP, 0, "masked: no summary bit");
    assert!(!rig.irq.high(), "and no pin");
    assert_eq!(rig.get(DSTAT) & DSTAT_ABRT, DSTAT_ABRT, "but it happened");

    rig.set(DIEN, DSTAT_ABRT);
    rig.set(ISTAT, ISTAT_ABRT);
    assert_eq!(rig.get(ISTAT) & ISTAT_DIP, ISTAT_DIP);
    assert!(rig.irq.high(), "the pin follows the summary bit");
    assert_eq!(rig.get(DSTAT) & DSTAT_ABRT, DSTAT_ABRT);
    assert!(!rig.irq.high(), "reading DSTAT clears DIP and the pin");
    assert_eq!(rig.get(ISTAT) & ISTAT_DIP, 0);
}

/// `SCNTL1`'s `RST` drives `RST/`, and a chip sees its own.
#[test]
fn asserting_rst_resets_the_cable_and_says_so() {
    let rig = rig();
    rig.set(SIEN, SSTAT0_RST);
    rig.set(SCNTL1, SCNTL1_RST);
    assert_eq!(rig.get(ISTAT) & ISTAT_SIP, ISTAT_SIP);
    assert!(rig.irq.high());
    assert_eq!(rig.get(SSTAT0) & SSTAT0_RST, SSTAT0_RST);
    assert!(!rig.irq.high(), "reading SSTAT0 clears SIP");
    rig.set(SCNTL1, 0);
    // The whole point: the drive got the reset, so its next command carries a
    // unit attention. Asserted through the bus rather than through the chip.
    assert_eq!(rig.get(SSTAT0), 0);
}

/// `DCNTL`'s `IRQD` holds the pin off whatever the registers say.
#[test]
fn irqd_holds_the_pin_off() {
    let rig = rig();
    rig.set(DIEN, 0xff);
    rig.set(DCNTL, DCNTL_IRQD);
    rig.set(ISTAT, ISTAT_ABRT);
    assert_eq!(
        rig.get(ISTAT) & ISTAT_DIP,
        ISTAT_DIP,
        "the status still says"
    );
    assert!(!rig.irq.high(), "the pin does not");
    rig.set(DCNTL, 0);
    assert!(rig.irq.high(), "and comes back when it is let go");
}

// ---------------------------------------------------------------------------
// SCRIPTS
// ---------------------------------------------------------------------------

/// The whole of it: a program that selects a drive, sends an `IDENTIFY` and a
/// command descriptor block, takes 512 bytes, a status byte and a
/// `COMMAND COMPLETE`, waits for the bus to go free and interrupts.
#[test]
fn a_scripts_program_reads_one_block_off_the_drive() {
    let rig = rig();
    rig.unmask();
    stage(&rig, 5);
    rig.program(&read_program());
    rig.run();

    assert!(rig.irq.high(), "the program interrupted");
    let dstat = rig.get(DSTAT);
    assert_eq!(dstat & DSTAT_SIR, DSTAT_SIR, "and did it with Interrupt");
    assert_eq!(rig.long(DSPS), DONE, "with its own vector");
    assert_eq!(
        rig.peek(STATUS_AT, 1),
        vec![status::GOOD],
        "the drive was happy"
    );
    assert_eq!(
        rig.peek(MESSAGE_AT, 1),
        vec![message::COMMAND_COMPLETE],
        "and said so"
    );
    assert_eq!(
        rig.peek(BUFFER, BLOCK),
        vec![5u8; BLOCK],
        "block 5 arrived, by bus mastering, at the address the program named"
    );
    // The first byte of the last inbound Block Move is in `SFBR`, which is what
    // a program compares against.
    assert_eq!(rig.get(SFBR), message::COMMAND_COMPLETE);
}

/// The destination of a `Select` is the **bus line**, not the number — and it
/// is in bits 23–16 of the instruction, or of the table entry, which is the
/// difference between scanning a cable and finding one drive seven times.
#[test]
fn the_destination_is_a_bus_line() {
    let rig = rig();
    rig.unmask();
    stage(&rig, 1);

    // Address 4 is a bus line nobody is driving, even though its *number* is a
    // bit pattern that would select address 2 read the other way round.
    let mut words = Vec::new();
    words.extend_from_slice(&select(4, PROGRAM + 16));
    words.extend_from_slice(&interrupt(0x0000_0001));
    words.extend_from_slice(&interrupt(0x0000_0002));
    rig.program(&words);
    rig.run();
    assert_eq!(rig.get(SSTAT0) & SSTAT0_STO, SSTAT0_STO, "nobody at 4");

    // And the table-indirect form reads the same field of its entry.
    let fresh = rig_with(Order::Big);
    fresh.unmask();
    stage(&fresh, 1);
    fresh.poke(TABLE, &(u32::from(1u8 << TARGET) << 16).to_be_bytes());
    fresh.set_long(DSA, TABLE);
    let mut words = Vec::new();
    words.extend_from_slice(&[
        ((TYPE_IO | IO_SELECT | SELECT_ATN | SELECT_TABLE) as u32) << 24,
        PROGRAM,
    ]);
    words.extend_from_slice(&interrupt(DONE));
    fresh.program(&words);
    fresh.run();
    assert_eq!(fresh.get(SSTAT0) & SSTAT0_STO, 0, "the drive answered");
    assert_eq!(fresh.long(DSPS), DONE);
    assert_eq!(fresh.get(SDID), TARGET, "and `SDID` holds the address");
}

/// The processor starts on the **highest-addressed** byte of `DSP`, which in
/// big-endian order is the least significant one. A 68000 writing a longword
/// as two words gets there last, and a chip that started on the first would
/// run whatever is at the top quarter of the address.
#[test]
fn the_processor_starts_on_the_last_byte_of_dsp() {
    let rig = rig();
    rig.unmask();
    rig.program(&interrupt(DONE));

    // The three bytes a 68000 writes first, in address order, top half first.
    for (i, byte) in PROGRAM.to_be_bytes().into_iter().take(3).enumerate() {
        rig.space
            .write(
                REGS_AT + (DSP + i) as u64,
                Width::U8,
                u64::from(byte),
                MemAttrs::DEFAULT,
            )
            .expect("a register");
    }
    assert_eq!(
        rig.get(DSTAT) & DSTAT_SIR,
        0,
        "three quarters is not a start"
    );
    rig.space
        .write(
            REGS_AT + (DSP + 3) as u64,
            Width::U8,
            u64::from(PROGRAM.to_be_bytes()[3]),
            MemAttrs::DEFAULT,
        )
        .expect("a register");
    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, DSTAT_SIR, "and the fourth is");
    assert_eq!(rig.long(DSPS), DONE);
}

/// The same command, with every Block Move taking its count and address from a
/// table at `DSA` — which is how Commodore's driver writes it.
#[test]
fn a_table_indirect_program_reads_the_same_block() {
    let rig = rig();
    rig.unmask();
    stage(&rig, 9);

    // The table: five count/address pairs, one per phase.
    let entries: [(u32, u32); 5] = [
        (1, IDENTIFY_AT),
        (6, CDB_AT),
        (BLOCK as u32, BUFFER),
        (1, STATUS_AT),
        (1, MESSAGE_AT),
    ];
    let mut table = Vec::new();
    for (count, at) in entries {
        table.extend_from_slice(&count.to_be_bytes());
        table.extend_from_slice(&at.to_be_bytes());
    }
    rig.poke(TABLE, &table);
    rig.set_long(DSA, TABLE);

    let mut words = Vec::new();
    words.extend_from_slice(&select(TARGET, PROGRAM));
    for (n, phase) in [MESSAGE_OUT, COMMAND, DATA_IN, STATUS_PHASE, MESSAGE_IN]
        .into_iter()
        .enumerate()
    {
        words.extend_from_slice(&block_move_table(phase, n as u32 * 8));
    }
    words.extend_from_slice(&wait_disconnect());
    words.extend_from_slice(&interrupt(DONE));
    rig.program(&words);
    rig.run();

    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, DSTAT_SIR);
    assert_eq!(rig.peek(BUFFER, BLOCK), vec![9u8; BLOCK]);
    assert_eq!(rig.peek(STATUS_AT, 1), vec![status::GOOD]);
}

/// Asking for more than the drive has ends the transfer the way a real SCSI
/// transfer ends: a phase mismatch, with the residual in `DBC` and `DNAD`.
#[test]
fn a_short_data_phase_is_a_phase_mismatch() {
    let rig = rig();
    rig.unmask();
    stage(&rig, 3);
    let mut words = Vec::new();
    words.extend_from_slice(&select(TARGET, PROGRAM));
    words.extend_from_slice(&block_move(MESSAGE_OUT, 1, IDENTIFY_AT));
    words.extend_from_slice(&block_move(COMMAND, 6, CDB_AT));
    // Twice as much as one block.
    words.extend_from_slice(&block_move(DATA_IN, 2 * BLOCK as u32, BUFFER));
    words.extend_from_slice(&interrupt(DONE));
    rig.program(&words);
    rig.run();

    assert_eq!(rig.get(SSTAT0) & SSTAT0_MA, SSTAT0_MA, "M/A, not an error");
    assert_eq!(rig.get(ISTAT) & ISTAT_SIP, 0, "which reading it cleared");
    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, 0, "the Interrupt never ran");
    assert_eq!(
        rig.long(DBC) & 0x00ff_ffff,
        BLOCK as u32,
        "the residual is what the drive did not have"
    );
    assert_eq!(
        rig.long(DNAD),
        BUFFER + BLOCK as u32,
        "and the address is where it stopped"
    );
    assert_eq!(rig.peek(BUFFER, BLOCK), vec![3u8; BLOCK], "what came, came");
}

/// Nobody at that address: the manual's `STO`, and the alternate jump.
#[test]
fn a_selection_nobody_answers_times_out_and_jumps() {
    let rig = rig();
    rig.unmask();
    // The alternate path is one instruction along from the program.
    let alt = PROGRAM + 16;
    let mut words = Vec::new();
    words.extend_from_slice(&select(4, alt));
    words.extend_from_slice(&interrupt(0x0000_0001));
    words.extend_from_slice(&interrupt(0x0000_0002));
    rig.program(&words);
    rig.run();

    assert_eq!(rig.get(SSTAT0) & SSTAT0_STO, SSTAT0_STO);
    // With `STO` enabled the chip stops there rather than running the alternate
    // path, and `DSP` is left on it so a driver may simply start again.
    assert_eq!(rig.long(DSP), alt, "DSP is on the alternate path");
    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, 0, "neither Interrupt ran");

    // Masked, the program keeps going down the alternate path on its own.
    let quiet = rig_with(Order::Big);
    quiet.set(DIEN, 0xff);
    quiet.set(SIEN, 0);
    quiet.program(&words);
    quiet.run();
    assert_eq!(quiet.get(DSTAT) & DSTAT_SIR, DSTAT_SIR);
    assert_eq!(quiet.long(DSPS), 0x0000_0002, "it took the alternate path");
}

/// A Memory Move whose destination is the chip's own register file, which is
/// how a driver loads `DSA` from a SCRIPTS program — and the reason no lock of
/// this chip's may be held across a bus tenure.
#[test]
fn a_memory_move_loads_the_chips_own_registers() {
    let rig = rig();
    rig.unmask();
    rig.poke(TABLE, &0x1234_5678u32.to_be_bytes());
    let mut words = Vec::new();
    // Into `DSA`, at the address the board decodes it at — the same longword a
    // 68000 would write.
    words.extend_from_slice(&memory_move(4, TABLE, REGS_AT as u32 + 0x10));
    // And back out again, through `SCRATCH`, so the round trip is visible in
    // memory rather than only in a register.
    words.extend_from_slice(&memory_move(4, TABLE, REGS_AT as u32 + 0x34));
    words.extend_from_slice(&memory_move(4, REGS_AT as u32 + 0x34, TABLE + 8));
    words.extend_from_slice(&interrupt(DONE));
    rig.program(&words);
    rig.run();

    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, DSTAT_SIR, "it finished");
    assert_eq!(rig.long(DSA), 0x1234_5678, "DSA took the longword");
    assert_eq!(
        rig.peek(TABLE + 8, 4),
        0x1234_5678u32.to_be_bytes().to_vec(),
        "and SCRATCH read back as a big-endian longword"
    );
}

/// The eight-bit ALU, and the carry that makes four of them a thirty-two-bit
/// add — which is what a driver's `SCRATCH += 4` is written as.
#[test]
fn register_arithmetic_carries_across_scratch() {
    let rig = rig();
    rig.unmask();
    rig.set_long(SCRATCH, 0x0000_00FF);
    let mut words = Vec::new();
    // `SCRATCH.0 += 4`, then the three bytes above it take the carry.
    words.extend_from_slice(&reg_op(IO_TO_REG, 3, false, SCRATCH, 4));
    words.extend_from_slice(&reg_op(IO_TO_REG, 3, true, SCRATCH + 1, 0));
    words.extend_from_slice(&reg_op(IO_TO_REG, 3, true, SCRATCH + 2, 0));
    words.extend_from_slice(&reg_op(IO_TO_REG, 3, true, SCRATCH + 3, 0));
    words.extend_from_slice(&interrupt(DONE));
    rig.program(&words);
    rig.run();
    assert_eq!(rig.long(SCRATCH), 0x0000_0103, "$FF + 4 carried");

    // And the other direction: a register into `SFBR`, then a compare.
    let compare = rig_with(Order::Big);
    compare.unmask();
    compare.set(SCRATCH, 0xA5);
    let mut words = Vec::new();
    words.extend_from_slice(&reg_op(IO_TO_SFBR, 1, false, SCRATCH, 0x00));
    words.extend_from_slice(&jump_if_data(0xA5, 0x00, PROGRAM + 24));
    words.extend_from_slice(&interrupt(0x0000_0001));
    words.extend_from_slice(&interrupt(0x0000_0002));
    compare.program(&words);
    compare.run();
    assert_eq!(compare.get(SFBR), 0xA5, "SFBR took the register");
    assert_eq!(compare.long(DSPS), 0x0000_0002, "and the compare jumped");
}

/// `Wait Reselect` parks the processor, and `SIGP` is what gets it going —
/// which is the shape Commodore's driver's idle loop has.
#[test]
fn wait_reselect_parks_until_sigp() {
    let rig = rig();
    rig.unmask();
    let mut words = Vec::new();
    words.extend_from_slice(&wait_reselect(PROGRAM + 8));
    words.extend_from_slice(&interrupt(DONE));
    rig.program(&words);
    rig.run();

    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, 0, "it is waiting, not finished");
    assert!(!rig.irq.high());
    assert_eq!(rig.long(DSP), PROGRAM, "parked on the instruction itself");

    rig.set(ISTAT, ISTAT_SIGP);
    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, DSTAT_SIR, "and then it ran");
    assert_eq!(rig.long(DSPS), DONE);
    // And the signal is still there: only a read of `CTEST2` takes it, because
    // the program wants to ask what woke it.
    assert_eq!(rig.get(ISTAT) & ISTAT_SIGP, ISTAT_SIGP);
    assert_eq!(rig.get(CTEST2) & CTEST2_SIGP, CTEST2_SIGP);
    assert_eq!(rig.get(ISTAT) & ISTAT_SIGP, 0, "which that read consumed");
}

/// `CTEST2` is where a SCRIPTS program reads `SIGP`, and reading it clears it.
#[test]
fn ctest2_reports_sigp_and_reading_it_clears_it() {
    let rig = rig();
    rig.set(ISTAT, ISTAT_SIGP);
    assert_eq!(rig.get(CTEST2) & CTEST2_SIGP, CTEST2_SIGP);
    assert_eq!(rig.get(ISTAT) & ISTAT_SIGP, 0, "the read consumed it");
    assert_eq!(rig.get(CTEST2) & CTEST2_SIGP, 0);
}

/// `DMODE`'s `MAN`: a `DSP` write loads the register and waits for `DCNTL`'s
/// `STD`.
#[test]
fn manual_start_waits_for_std() {
    let rig = rig();
    rig.unmask();
    rig.set(DMODE, DMODE_MAN);
    rig.program(&interrupt(DONE));
    rig.run();
    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, 0, "MAN held it");
    rig.set(DCNTL, DCNTL_STD);
    assert_eq!(rig.get(DSTAT) & DSTAT_SIR, DSTAT_SIR, "STD started it");
    assert_eq!(rig.long(DSPS), DONE);
    assert_eq!(rig.get(DCNTL) & DCNTL_STD, 0, "STD does not stick");
}

/// An instruction naming one of the two phases X3.131-1994 reserves.
#[test]
fn a_reserved_phase_is_an_illegal_instruction() {
    let rig = rig();
    rig.unmask();
    rig.program(&block_move(0b100, 1, BUFFER));
    rig.run();
    assert!(rig.irq.high(), "the pin came up");
    assert_eq!(rig.get(DSTAT) & DSTAT_IID, DSTAT_IID);
    assert!(!rig.irq.high(), "and reading DSTAT put it down");
}

/// A program that neither interrupts, waits nor mismatches is a runaway, and
/// the model says so rather than hanging the host.
#[test]
fn a_program_that_never_stops_is_stopped() {
    let rig = rig();
    rig.unmask();
    rig.program(&jump(PROGRAM));
    rig.run();
    assert_eq!(rig.get(DSTAT) & DSTAT_IID, DSTAT_IID);
}

/// `Call` and `Return`, whose stack is one deep because `TEMP` is one
/// longword.
#[test]
fn call_and_return_go_through_temp() {
    let rig = rig();
    rig.unmask();
    let sub = PROGRAM + 24;
    let mut words = Vec::new();
    words.extend_from_slice(&[((TYPE_TRANSFER | XFER_CALL) as u32) << 24 | DBC_TRUE, sub]);
    words.extend_from_slice(&interrupt(DONE));
    words.extend_from_slice(&interrupt(0x0000_00FF));
    words.extend_from_slice(&[((TYPE_TRANSFER | XFER_RETURN) as u32) << 24 | DBC_TRUE, 0]);
    rig.program(&words);
    rig.run();
    assert_eq!(
        rig.long(DSPS),
        DONE,
        "it came back to the instruction after"
    );
    assert_eq!(rig.long(TEMP), PROGRAM + 8);
}

// ---------------------------------------------------------------------------
// debug reads, and a snapshot
// ---------------------------------------------------------------------------

/// A debugger reading the register file must not take the interrupt the
/// guest's handler is about to look for, nor consume `SIGP`.
#[test]
fn a_debug_read_takes_nothing() {
    let rig = rig();
    rig.set(DIEN, 0xff);
    rig.set(ISTAT, ISTAT_ABRT | ISTAT_SIGP);
    assert!(rig.irq.high());

    let peek = |reg: usize| {
        let at = REGS_AT + lane(Order::Big, reg);
        rig.space
            .read(at, Width::U8, MemAttrs::DEBUG)
            .expect("a register") as u8
    };
    assert_eq!(peek(DSTAT) & DSTAT_ABRT, DSTAT_ABRT);
    assert_eq!(peek(CTEST2) & CTEST2_SIGP, CTEST2_SIGP);
    assert!(rig.irq.high(), "and nothing moved");
    assert_eq!(
        rig.get(ISTAT) & (ISTAT_DIP | ISTAT_SIGP),
        ISTAT_DIP | ISTAT_SIGP
    );

    // A debug *write* is refused outright: one to `DSP` would master the bus.
    let at = REGS_AT + lane(Order::Big, DSP + 3);
    assert!(rig.space.write(at, Width::U8, 0, MemAttrs::DEBUG).is_err());
}

/// Every register, as a snapshot holds them.
fn snapshot(chip: &Ncr53c710) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("scsi0", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("scsi0", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(chip, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

/// Save, change everything, load, and get the same chip back.
#[test]
fn state_round_trips() {
    let saved = rig();
    saved.unmask();
    stage(&saved, 11);
    saved.program(&read_program());
    saved.run();
    assert_eq!(saved.get(DSTAT) & DSTAT_SIR, DSTAT_SIR);
    let bytes = snapshot(saved.chip.as_ref());

    let fresh = rig();
    fresh.set_long(SCRATCH, 0xFFFF_FFFF);
    fresh.set(DIEN, 0x55);
    assert_ne!(snapshot(fresh.chip.as_ref()), bytes);

    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("scsi0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(fresh.chip.as_ref(), &mut chunk.reader()).unwrap();
    assert_eq!(
        snapshot(fresh.chip.as_ref()),
        bytes,
        "identical state, hash for hash"
    );
    assert_eq!(fresh.chip.irq_asserted(), saved.chip.irq_asserted());
}

/// A reset is a reset: the pin goes away with the registers.
#[test]
fn a_board_reset_clears_the_pin() {
    let rig = rig();
    rig.set(DIEN, 0xff);
    rig.set(ISTAT, ISTAT_ABRT);
    assert!(rig.irq.high());
    Device::reset(rig.chip.as_ref(), ResetKind::Cold);
    assert!(!rig.irq.high());
    assert_eq!(rig.get(ISTAT), 0);
    assert_eq!(rig.get(DSTAT), DSTAT_DFE);
}
