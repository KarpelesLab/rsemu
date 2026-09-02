//! The board, end to end: a hart, the chips around it, and programs that run.
//!
//! Every device in this tree has its own unit tests. What those cannot show is
//! whether the pieces fit — whether the device tree the guest is handed
//! describes the machine it is actually running on, whether a timer programmed
//! through the CLINT reaches `mtvec`, whether a keystroke crosses the UART, the
//! PLIC and `meip` and comes back out. That is what is here.
//!
//! # The programs are assembled, not vendored
//!
//! [`asm`] is forty lines of instruction encoders built from the formats in
//! *The RISC-V Instruction Set Manual, Volume I* (CC-BY-4.0). Writing the test
//! programs that way rather than committing `.bin` files keeps the fixtures
//! readable, keeps them in the same file as the assertion they support, and
//! means the suite needs no cross toolchain to run — which the
//! `cargo test` gate in `CLAUDE.md` requires.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::space::MemAttrs;
use crate::core::value::Width;
use crate::host::chardev::{CharPort, ports};
use crate::machine::{Machine, catalog};

use super::boot::DTB_OFFSET;
use super::syscon::{Request, signals};

/// Where the boot ROM sits, as `machines/riscv-virt.machine` maps it.
const BOOT_BASE: u64 = 0x1000;
/// Where DRAM starts.
const DRAM: u64 = 0x8000_0000;
/// The UART's transmit/receive register.
const UART: u64 = 0x1000_0000;
/// The system controller.
const SYSCON: u64 = 0x0010_0000;
/// The CLINT's first comparator.
const MTIMECMP: u64 = 0x0200_4000;
/// The CLINT's `mtime`, as a guest reads it through the register block.
const MTIME: u64 = 0x0200_bff8;
/// Where the `rdtime` programs below publish what they saw.
const RDTIME_SCRATCH: u64 = DRAM + 0x1000;
/// The PLIC.
const PLIC: u64 = 0x0c00_0000;

// ---------------------------------------------------------------------------
// a very small assembler
// ---------------------------------------------------------------------------

/// Instruction encoders, from *Volume I: Unprivileged ISA* §2.2-§2.5 and §5.2.
mod asm {
    /// `x5`.
    pub(super) const T0: u32 = 5;
    /// `x6`.
    pub(super) const T1: u32 = 6;
    /// `x7`.
    pub(super) const T2: u32 = 7;
    /// `x10`.
    pub(super) const A0: u32 = 10;
    /// `x11`.
    pub(super) const A1: u32 = 11;
    /// `x28`.
    pub(super) const T3: u32 = 28;
    /// `x0`.
    pub(super) const ZERO: u32 = 0;

    /// `mstatus`.
    pub(super) const CSR_MSTATUS: u32 = 0x300;
    /// `mie`.
    pub(super) const CSR_MIE: u32 = 0x304;
    /// `mtvec`.
    pub(super) const CSR_MTVEC: u32 = 0x305;
    /// `time` — the read-only view of the platform timer `rdtime` returns.
    pub(super) const CSR_TIME: u32 = 0xc01;

    fn i_type(opcode: u32, funct3: u32, rd: u32, rs1: u32, imm: i32) -> u32 {
        (((imm as u32) & 0xfff) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
    }

    fn s_type(funct3: u32, rs1: u32, rs2: u32, imm: i32) -> u32 {
        let imm = imm as u32;
        (((imm >> 5) & 0x7f) << 25)
            | (rs2 << 20)
            | (rs1 << 15)
            | (funct3 << 12)
            | ((imm & 0x1f) << 7)
            | 0b0100011
    }

    /// `lui rd, imm20`.
    pub(super) fn lui(rd: u32, imm20: u32) -> u32 {
        ((imm20 & 0xf_ffff) << 12) | (rd << 7) | 0b0110111
    }

    /// `auipc rd, imm20`.
    pub(super) fn auipc(rd: u32, imm20: u32) -> u32 {
        ((imm20 & 0xf_ffff) << 12) | (rd << 7) | 0b0010111
    }

    /// `addi rd, rs1, imm`.
    pub(super) fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0b0010011, 0b000, rd, rs1, imm)
    }

    /// `lbu rd, imm(rs1)`.
    pub(super) fn lbu(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0b0000011, 0b100, rd, rs1, imm)
    }

    /// `lw rd, imm(rs1)`.
    pub(super) fn lw(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0b0000011, 0b010, rd, rs1, imm)
    }

    /// `sb rs2, imm(rs1)`.
    pub(super) fn sb(rs1: u32, rs2: u32, imm: i32) -> u32 {
        s_type(0b000, rs1, rs2, imm)
    }

    /// `sw rs2, imm(rs1)`.
    pub(super) fn sw(rs1: u32, rs2: u32, imm: i32) -> u32 {
        s_type(0b010, rs1, rs2, imm)
    }

    /// `sd rs2, imm(rs1)`.
    pub(super) fn sd(rs1: u32, rs2: u32, imm: i32) -> u32 {
        s_type(0b011, rs1, rs2, imm)
    }

    /// `csrrw rd, csr, rs1`; with `rd = x0` this is `csrw`.
    pub(super) fn csrw(csr: u32, rs1: u32) -> u32 {
        i_type(0b1110011, 0b001, ZERO, rs1, csr as i32)
    }

    /// `csrrs rd, csr, x0` — the `csrr` pseudo-instruction, a pure read.
    pub(super) fn csrr(rd: u32, csr: u32) -> u32 {
        i_type(0b1110011, 0b010, rd, ZERO, csr as i32)
    }

    /// `csrrs rd, csr, rs1`; with `rd = x0` this is `csrs`.
    pub(super) fn csrs(csr: u32, rs1: u32) -> u32 {
        i_type(0b1110011, 0b010, ZERO, rs1, csr as i32)
    }

    /// `wfi` (Volume II).
    pub(super) fn wfi() -> u32 {
        0x1050_0073
    }

    /// `jal x0, imm` — a plain jump. J-type (§2.5).
    pub(super) fn j(imm: i32) -> u32 {
        let imm = imm as u32;
        ((imm >> 20) & 1) << 31
            | ((imm >> 1) & 0x3ff) << 21
            | ((imm >> 11) & 1) << 20
            | ((imm >> 12) & 0xff) << 12
            | 0b1101111
    }

    /// `li rd, value` for a value that fits `lui` + `addi`, as two words.
    ///
    /// Only correct for values below `0x8000_0000`: on RV64 `lui` sign-extends
    /// its result, so anything with bit 31 set would become a negative address.
    /// Every MMIO address on this board is below that, and DRAM addresses are
    /// reached with `auipc` instead.
    pub(super) fn li(rd: u32, value: u32) -> [u32; 2] {
        assert!(value < 0x8000_0000, "use auipc for {value:#x}");
        let hi = (value.wrapping_add(0x800)) >> 12;
        let lo = (value & 0xfff) as i32;
        let lo = if lo >= 0x800 { lo - 0x1000 } else { lo };
        [lui(rd, hi), addi(rd, rd, lo)]
    }
}

/// A program under construction, at a known load address.
struct Program {
    words: Vec<u32>,
    base: u64,
}

impl Program {
    fn new(base: u64) -> Program {
        Program {
            words: Vec::new(),
            base,
        }
    }

    /// Where the next instruction will land.
    fn here(&self) -> u64 {
        self.base + self.words.len() as u64 * 4
    }

    fn push(&mut self, word: u32) -> &mut Program {
        self.words.push(word);
        self
    }

    fn push_all(&mut self, words: impl IntoIterator<Item = u32>) -> &mut Program {
        self.words.extend(words);
        self
    }

    /// `li rd, value`, for an address below 2 GiB.
    fn li(&mut self, rd: u32, value: u32) -> &mut Program {
        self.push_all(asm::li(rd, value))
    }

    /// Load a DRAM address into `rd` with `auipc`, so no 32-bit immediate with
    /// bit 31 set is ever needed.
    fn la(&mut self, rd: u32, target: u64) -> &mut Program {
        let from = self.here();
        let delta = target as i64 - from as i64;
        assert!(
            (-0x8_0000_0000..0x8_0000_0000).contains(&delta),
            "too far for auipc"
        );
        // The `addi` that follows sign-extends its 12-bit immediate, so the
        // high part has to be rounded to compensate — the same correction the
        // `la` pseudo-instruction makes.
        let hi = ((delta + 0x800) >> 12) as u32;
        let lo = (delta & 0xfff) as i32;
        let lo = if lo >= 0x800 { lo - 0x1000 } else { lo };
        self.push(asm::auipc(rd, hi));
        self.push(asm::addi(rd, rd, lo))
    }

    /// Write one byte to the console.
    fn putc(&mut self, byte: u8) -> &mut Program {
        self.li(asm::T1, u32::from(byte));
        self.push(asm::sb(asm::T0, asm::T1, 0))
    }

    /// Stop the machine, successfully, and then spin.
    fn poweroff(&mut self) -> &mut Program {
        self.li(asm::T0, SYSCON as u32);
        self.li(asm::T1, u32::from(super::syscon::CMD_PASS));
        self.push(asm::sw(asm::T0, asm::T1, 0));
        self.push(asm::j(0))
    }

    /// Pad with `nop` until the next instruction lands at `offset`.
    fn pad_to(&mut self, offset: u64) -> &mut Program {
        while self.here() < self.base + offset {
            self.push(asm::addi(asm::ZERO, asm::ZERO, 0));
        }
        assert_eq!(self.here(), self.base + offset, "padded past the target");
        self
    }

    fn bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.words.len() * 4);
        for word in &self.words {
            out.extend_from_slice(&word.to_le_bytes());
        }
        out
    }
}

// ---------------------------------------------------------------------------
// building and running a board
// ---------------------------------------------------------------------------

/// A built machine plus the host ends of its console and its power button.
struct Board {
    machine: Machine,
    console: Arc<CharPort>,
    power: Arc<super::syscon::Signal>,
}

/// Build `riscv-virt` with `firmware` loaded.
///
/// The port names no longer have to differ per test: each build gets its own
/// host objects, so two boards in one binary cannot type at each other however
/// they name their console. `tag` survives only so a failure names the test.
fn board(tag: &str, firmware: &[u8]) -> Board {
    let _ = tag;
    let console_name = String::from("console");
    let power_name = String::from("power");
    let entry = catalog::machine("riscv-virt").expect("this build ships it");
    let options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_media("firmware", firmware)
        .with_media("flash0", &[][..])
        .with_media("flash1", &[][..])
        .with_media("initrd", &[][..])
        .with_media("disk", &[][..])
        .with_param("console", console_name.clone())
        .with_param("power", power_name.clone())
        // Enough for the programs here, and small enough that the cold reset
        // that clears it is not the slowest part of the test.
        .with_param("ram", String::from("8M"));
    let registry = catalog::registry().expect("the catalog agrees with itself");
    let machine = match crate::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("riscv-virt does not build: {e}"),
    };
    Board {
        machine,
        console: ports::open(&options.realize.hosts, &console_name).expect("the UART opened it"),
        power: signals::open(&options.realize.hosts, &power_name).expect("the syscon opened it"),
    }
}

impl Board {
    /// Run until the guest asks the machine to stop, or until `quanta` have
    /// gone by. Returns what it asked for, if it asked.
    fn run(&mut self, quanta: usize) -> Option<Request> {
        for _ in 0..quanta {
            if let Some(request) = self.power.peek() {
                return Some(request);
            }
            self.machine.run_quantum().expect("the machine advances");
        }
        self.power.peek()
    }

    /// Everything the guest has printed.
    fn output(&self) -> String {
        String::from_utf8_lossy(&self.console.drain()).into_owned()
    }

    /// One guest byte, read the way a debugger would.
    fn peek(&self, addr: u64, width: Width) -> u64 {
        self.machine
            .space("mem")
            .expect("the machine has one space")
            .read(addr, width, MemAttrs::DEBUG)
            .unwrap_or_else(|e| panic!("nothing answers at {addr:#x}: {e}"))
    }

    /// The device tree the boot ROM generated, read out of guest memory
    /// exactly as the firmware would find it.
    fn device_tree(&self) -> Vec<u8> {
        let at = BOOT_BASE + DTB_OFFSET;
        let space = self.machine.space("mem").expect("one space");
        let mut header = [0u8; 8];
        space
            .read_bytes(at, &mut header, MemAttrs::DEBUG)
            .expect("the boot ROM answers");
        let total = u32::from_be_bytes([header[4], header[5], header[6], header[7]]) as usize;
        assert!(
            total > 0 && total < 0x1_0000,
            "implausible tree size {total}"
        );
        let mut dtb = alloc::vec![0u8; total];
        space
            .read_bytes(at, &mut dtb, MemAttrs::DEBUG)
            .expect("the boot ROM answers");
        dtb
    }
}

// ---------------------------------------------------------------------------
// the device tree
// ---------------------------------------------------------------------------

#[test]
fn the_boot_rom_hands_over_a_device_tree_that_parses() {
    let b = board("dtb", &[]);
    let dtb = b.device_tree();
    assert_eq!(
        u32::from_be_bytes([dtb[0], dtb[1], dtb[2], dtb[3]]),
        super::fdt::FDT_MAGIC,
        "the blob at the address a1 points at is not a device tree"
    );
    let tree = super::dt::describe(&dtb).expect("it parses");
    for node in [
        "cpu@0",
        "interrupt-controller",
        "memory@80000000",
        "soc",
        "test@100000",
        "clint@2000000",
        "plic@c000000",
        "serial@10000000",
        "virtio_mmio@10001000",
        "virtio_mmio@10002000",
        "chosen",
        "poweroff",
        "reboot",
    ] {
        assert!(tree.contains(node), "no `{node}` in:\n{tree}");
    }
}

#[test]
fn a_ramdisk_is_staged_in_memory_and_pointed_at_by_the_tree() {
    // Both halves of the initrd contract in one test, because either alone is
    // useless: the bytes have to be *in* memory, and `/chosen` has to say
    // where. The two numbers come from different objects in the machine file
    // — a `riscv.loader` puts the image down, the boot ROM describes it — so a
    // test that checked only one would not notice them disagreeing.
    let at = DRAM + 0x10_0000;
    let ramdisk = alloc::vec![0x5au8; 4096];
    let entry = catalog::machine("riscv-virt").expect("shipped");
    let options = catalog::build_options()
        .unwrap()
        .with_media("firmware", &[][..])
        .with_media("flash0", &[][..])
        .with_media("flash1", &[][..])
        .with_media("initrd", ramdisk.as_slice())
        .with_media("disk", &[][..])
        .with_param("console", "console")
        .with_param("power", "power")
        .with_param("ram", "8M")
        // Inside this board's 8 MiB rather than at the shipped default, which
        // assumes the 256M a real kernel wants.
        .with_param("initrd_addr", alloc::format!("{at:#x}"));
    let machine = crate::machine::build(
        entry.name,
        entry.source,
        &catalog::registry().unwrap(),
        &options,
    )
    .expect("a board with a ramdisk still builds");
    let b = Board {
        machine,
        console: ports::open(&options.realize.hosts, "console").expect("the UART opened it"),
        power: signals::open(&options.realize.hosts, "power").expect("the syscon opened it"),
    };

    let dtb = b.device_tree();
    assert_eq!(
        prop_u64(&dtb, "chosen", "linux,initrd-start"),
        Some(at),
        "{}",
        super::dt::describe(&dtb).expect("it parses")
    );
    assert_eq!(
        prop_u64(&dtb, "chosen", "linux,initrd-end"),
        Some(at + ramdisk.len() as u64),
        "the end is one past the last byte"
    );
    assert_eq!(b.peek(at, Width::U8), 0x5a, "the first byte of the ramdisk");
    assert_eq!(
        b.peek(at + ramdisk.len() as u64 - 1, Width::U8),
        0x5a,
        "the last byte of the ramdisk"
    );

    // And with nothing bound, `/chosen` says nothing at all — a kernel that
    // finds the properties present but zero would try to unpack address zero.
    let plain = board("no-initrd", &[]).device_tree();
    assert_eq!(prop_u64(&plain, "chosen", "linux,initrd-start"), None);
    assert_eq!(prop_u64(&plain, "chosen", "linux,initrd-end"), None);
}

#[test]
fn every_address_in_the_tree_came_from_the_memory_map() {
    // The claim `docs/platforms/riscv-virt.md` makes: the tree is produced
    // mechanically from the realized machine. Move a device in the machine
    // file and the node name moves with it, with nothing else edited.
    let b = board("dtb-addr", &[]);
    let tree = super::dt::describe(&b.device_tree()).expect("it parses");
    assert!(tree.contains("serial@10000000"), "{tree}");

    let entry = catalog::machine("riscv-virt").expect("shipped");
    let moved = entry.source.replace(
        "0x10000000 size 0x00000100 = uart",
        "0x10004000 size 0x00000100 = uart",
    );
    assert_ne!(moved, entry.source, "the map statement was not found");
    let options = catalog::build_options()
        .unwrap()
        .with_media("firmware", &[][..])
        .with_media("flash0", &[][..])
        .with_media("flash1", &[][..])
        .with_media("initrd", &[][..])
        .with_media("disk", &[][..])
        .with_param("console", "console")
        .with_param("power", "power")
        .with_param("ram", "8M");
    let machine = crate::machine::build(
        "riscv-virt-moved",
        &moved,
        &catalog::registry().unwrap(),
        &options,
    )
    .expect("a moved UART is still a machine");
    let b2 = Board {
        machine,
        console: ports::open(&options.realize.hosts, "console").expect("the UART opened it"),
        power: signals::open(&options.realize.hosts, "power").expect("the syscon opened it"),
    };
    let tree = super::dt::describe(&b2.device_tree()).expect("it parses");
    assert!(tree.contains("serial@10004000"), "{tree}");
    assert!(!tree.contains("serial@10000000"), "{tree}");
}

#[test]
fn the_nor_banks_appear_as_cfi_flash_nodes() {
    // This is how a UEFI build finds its variable store: EDK2's
    // `FdtNorFlashQemuLib` walks every `cfi-flash` node, skips the bank that
    // overlaps its own firmware volume, and makes the next one
    // `PcdFlashNvStorageVariableBase`. No node, no variable write, and the DXE
    // dispatcher asserts with 47 drivers still waiting on the protocol.
    let b = board("dtb-flash", &[]);
    let tree = super::dt::describe(&b.device_tree()).expect("it parses");
    assert!(tree.contains("flash@20000000"), "{tree}");
    assert!(tree.contains("flash@22000000"), "{tree}");
    // `describe` prints property sizes rather than values, so the compatible
    // string is checked in the blob itself — it is the exact byte sequence
    // EDK2's `FindCompatibleNode` looks for.
    let dtb = b.device_tree();
    assert!(
        dtb.windows(10).any(|w| w == b"cfi-flash\0"),
        "no `cfi-flash` compatible string in the tree"
    );
    // Four bytes, because the bank is two x16 parts side by side. A driver
    // that read 2 here would send each command to one of them.
    assert_eq!(prop_u32(&dtb, "flash@22000000", "bank-width"), Some(4));
}

#[test]
fn the_interrupt_numbers_come_out_of_the_wire_graph() {
    // `wire uart.irq -> plic.irq10` is the only place 10 appears. If the tree
    // says something else, the join between the PLIC's pin table and the
    // device's own net is broken — and a kernel would attach its handler to
    // the wrong line and hang waiting for a keystroke.
    let b = board("dtb-irq", &[]);
    let dtb = b.device_tree();
    assert_eq!(interrupts_of(&dtb, "serial@10000000"), Some(10));
    assert_eq!(interrupts_of(&dtb, "virtio_mmio@10001000"), Some(1));
    assert_eq!(interrupts_of(&dtb, "virtio_mmio@10002000"), Some(2));
}

#[test]
fn the_timebase_is_the_clints_own_clock() {
    let b = board("dtb-time", &[]);
    let dtb = b.device_tree();
    assert_eq!(
        prop_u32(&dtb, "cpus", "timebase-frequency"),
        Some(10_000_000),
        "the tree must report the rate mtime really counts at"
    );
}

#[test]
fn the_tree_is_byte_identical_across_builds() {
    // It lands in guest memory, so a tree that differed run to run would make
    // the machine's state hash differ too (`CLAUDE.md`, determinism).
    let a = board("dtb-det-a", &[]).device_tree();
    let b = board("dtb-det-b", &[]).device_tree();
    assert_eq!(a, b);
}

// ---------------------------------------------------------------------------
// running programs
// ---------------------------------------------------------------------------

#[test]
fn a_bare_metal_program_prints_to_the_uart_and_powers_off() {
    // Milestone one: the console works, from the reset vector, with no
    // firmware and no operating system.
    let mut p = Program::new(DRAM);
    p.li(asm::T0, UART as u32);
    for byte in b"rsemu\n" {
        p.putc(*byte);
    }
    // Prove the boot ROM did its job: stash a0 and a1 where the test can see
    // them.
    p.la(asm::T2, DRAM + 0x1000);
    p.push(asm::sd(asm::T2, asm::A0, 0));
    p.push(asm::sd(asm::T2, asm::A1, 8));
    p.poweroff();

    let mut b = board("hello", &p.bytes());
    assert_eq!(b.run(200), Some(Request::Poweroff), "it never stopped");
    assert_eq!(b.output(), "rsemu\n");
    assert_eq!(b.peek(DRAM + 0x1000, Width::U64), 0, "a0 is the hart id");
    assert_eq!(
        b.peek(DRAM + 0x1008, Width::U64),
        BOOT_BASE + DTB_OFFSET,
        "a1 points at the device tree"
    );
}

#[test]
fn a_timer_interrupt_programmed_through_the_clint_reaches_mtvec() {
    const HANDLER: u64 = 0x100;
    let mut p = Program::new(DRAM);
    p.la(asm::T0, DRAM + HANDLER);
    p.push(asm::csrw(asm::CSR_MTVEC, asm::T0));
    // mtimecmp = 2000 ticks of a 10 MHz crystal: 200 microseconds away.
    p.li(asm::T0, MTIMECMP as u32);
    p.li(asm::T1, 2000);
    p.push(asm::sd(asm::T0, asm::T1, 0));
    // mie.MTIE, then mstatus.MIE. In that order, so an interrupt cannot land
    // before the handler is armed.
    p.li(asm::T0, 1 << 7);
    p.push(asm::csrs(asm::CSR_MIE, asm::T0));
    p.li(asm::T0, 1 << 3);
    p.push(asm::csrs(asm::CSR_MSTATUS, asm::T0));
    p.push(asm::wfi());
    p.push(asm::j(-4));

    p.pad_to(HANDLER);
    p.li(asm::T0, UART as u32);
    p.putc(b'T');
    p.poweroff();

    let mut b = board("timer", &p.bytes());
    assert_eq!(b.run(400), Some(Request::Poweroff), "the timer never fired");
    assert_eq!(b.output(), "T");
}

/// A program that spins publishing `rdtime` to a scratch word.
fn rdtime_loop() -> Program {
    let mut p = Program::new(DRAM);
    p.la(asm::T2, RDTIME_SCRATCH);
    let top = p.here();
    p.push(asm::csrr(asm::T0, asm::CSR_TIME));
    p.push(asm::sd(asm::T2, asm::T0, 0));
    let back = top as i64 - p.here() as i64;
    p.push(asm::j(back as i32));
    p
}

#[test]
fn rdtime_reads_the_clints_counter() {
    // The export seam, end to end on a real board: the CLINT publishes `mtime`
    // as `ExportId::TIMEBASE`, `machines/riscv-virt.machine` says
    // `timer = clint` on the hart, and the hart's `time` CSR reads that cell.
    let mut b = board("rdtime", &rdtime_loop().bytes());
    b.run(200);
    let seen = b.peek(RDTIME_SCRATCH, Width::U64);
    assert!(seen > 0, "`rdtime` still reads zero after 200 quanta");
    // A debug read advances nothing, so `mtime` is the value at the last
    // catch-up: the guest cannot have seen a later one.
    let mtime = b.peek(MTIME, Width::U64);
    assert!(
        seen <= mtime,
        "the guest saw {seen}, ahead of the CLINT's own {mtime}"
    );
}

#[test]
fn a_hart_with_no_timer_named_reads_zero() {
    // The other half of the claim: nothing is found implicitly. Delete the one
    // line that names the source and `rdtime` goes back to reading zero — which
    // is what made the live-lock in `clint.rs`'s history possible, and is why
    // the machine file has to say it rather than the machine guessing.
    let entry = catalog::machine("riscv-virt").expect("shipped");
    let unwired = entry.source.replace("timer  = clint", "");
    assert_ne!(unwired, entry.source, "the `timer` property was not found");
    let options = catalog::build_options()
        .unwrap()
        .with_media("firmware", rdtime_loop().bytes().as_slice())
        .with_media("flash0", &[][..])
        .with_media("flash1", &[][..])
        .with_media("initrd", &[][..])
        .with_media("disk", &[][..])
        .with_param("console", "console")
        .with_param("power", "power")
        .with_param("ram", "8M");
    let machine = crate::machine::build(
        "riscv-virt-untimed",
        &unwired,
        &catalog::registry().unwrap(),
        &options,
    )
    .expect("a board with no timer named is still a board");
    let mut b = Board {
        machine,
        console: ports::open(&options.realize.hosts, "console").expect("the UART opened it"),
        power: signals::open(&options.realize.hosts, "power").expect("the syscon opened it"),
    };
    b.run(200);
    assert_eq!(b.peek(RDTIME_SCRATCH, Width::U64), 0);
    assert!(
        b.peek(MTIME, Width::U64) > 0,
        "the CLINT itself must still be counting"
    );
}

#[test]
fn naming_a_timer_that_publishes_none_says_so() {
    let entry = catalog::machine("riscv-virt").expect("shipped");
    let wrong = entry.source.replace("timer  = clint", "timer  = dram");
    let options = catalog::build_options()
        .unwrap()
        .with_media("firmware", &[][..])
        .with_media("flash0", &[][..])
        .with_media("flash1", &[][..])
        .with_media("initrd", &[][..])
        .with_media("disk", &[][..])
        .with_param("console", "test.riscv.console.mistimed")
        .with_param("power", "test.riscv.power.mistimed")
        .with_param("ram", "8M");
    let e = crate::machine::build(
        "riscv-virt-mistimed",
        &wrong,
        &catalog::registry().unwrap(),
        &options,
    )
    .expect_err("ram publishes no timebase");
    let text = alloc::format!("{e}");
    for want in ["cpu0", "dram", "timebase"] {
        assert!(text.contains(want), "`{want}` missing from {text}");
    }
}

#[test]
fn a_keystroke_crosses_the_uart_the_plic_and_meip() {
    // The whole external-interrupt path in one test: a host byte arrives, the
    // 16550 raises its line, the PLIC gateway forwards it to context 0, the
    // hart takes a machine external interrupt, claims, reads, echoes and
    // completes.
    const HANDLER: u64 = 0x100;
    let mut p = Program::new(DRAM);
    p.la(asm::T0, DRAM + HANDLER);
    p.push(asm::csrw(asm::CSR_MTVEC, asm::T0));
    // PLIC: priority[10] = 1, enabled for context 0, threshold 0.
    p.li(asm::T0, (PLIC + 4 * 10) as u32);
    p.li(asm::T1, 1);
    p.push(asm::sw(asm::T0, asm::T1, 0));
    p.li(asm::T0, (PLIC + 0x2000) as u32);
    p.li(asm::T1, 1 << 10);
    p.push(asm::sw(asm::T0, asm::T1, 0));
    p.li(asm::T0, (PLIC + 0x20_0000) as u32);
    p.push(asm::sw(asm::T0, asm::ZERO, 0));
    // UART: enable the received-data interrupt.
    p.li(asm::T0, UART as u32);
    p.li(asm::T1, 1);
    p.push(asm::sb(asm::T0, asm::T1, 1));
    // mie.MEIE, then mstatus.MIE.
    p.li(asm::T0, 1 << 11);
    p.push(asm::csrs(asm::CSR_MIE, asm::T0));
    p.li(asm::T0, 1 << 3);
    p.push(asm::csrs(asm::CSR_MSTATUS, asm::T0));
    p.push(asm::wfi());
    p.push(asm::j(-4));

    p.pad_to(HANDLER);
    p.li(asm::T0, (PLIC + 0x20_0004) as u32);
    p.push(asm::lw(asm::T1, asm::T0, 0)); // claim
    p.li(asm::T2, UART as u32);
    p.push(asm::lbu(asm::T3, asm::T2, 0)); // read the byte
    p.push(asm::sb(asm::T2, asm::T3, 0)); // echo it
    p.push(asm::sw(asm::T0, asm::T1, 0)); // complete
    p.poweroff();

    let mut b = board("plic-rx", &p.bytes());
    b.console.feed(b"Z");
    assert_eq!(b.run(400), Some(Request::Poweroff), "no interrupt arrived");
    assert_eq!(b.output(), "Z", "the byte did not come back");
}

#[test]
fn a_guest_can_reboot_itself_through_the_system_controller() {
    // `wire test.reset -> cpu0.reset` end to end: the guest writes the reset
    // command, the syscon pulses the hart's reset pin, and the hart starts
    // again at the boot ROM — with the firmware still in DRAM, because
    // resetting one device does not clear memory. So the program prints one
    // character per life, and a long enough run sees several.
    let mut p = Program::new(DRAM);
    p.li(asm::T0, UART as u32);
    p.putc(b'.');
    p.li(asm::T0, SYSCON as u32);
    p.li(asm::T1, u32::from(super::syscon::CMD_RESET));
    p.push(asm::sw(asm::T0, asm::T1, 0));
    p.push(asm::j(0));

    let mut b = board("reboot", &p.bytes());
    for _ in 0..200 {
        b.machine.run_quantum().expect("the machine advances");
    }
    let lives = b.output().matches('.').count();
    assert!(
        lives >= 2,
        "the hart came up {lives} time(s); the reset line never pulsed"
    );
    assert_eq!(b.power.peek(), Some(Request::Reboot), "and it said why");
}

#[test]
fn the_machine_snapshots_and_restores_to_the_same_state_hash() {
    // Every device in this tree has a save/load round trip of its own; this is
    // the one that says they compose (`ROADMAP.md` §4.5).
    let mut p = Program::new(DRAM);
    p.li(asm::T0, UART as u32);
    p.putc(b'x');
    p.poweroff();

    let mut b = board("snapshot", &p.bytes());
    b.run(50);
    let saved = b.machine.save().expect("a machine saves");
    let before = b.machine.state_hash().expect("a machine hashes");
    b.machine.load(&saved).expect("its own snapshot loads");
    assert_eq!(b.machine.state_hash().expect("hashes"), before);
}

#[test]
fn the_virtio_block_device_is_discoverable_and_serves_a_read() {
    // Not a driver — a driver is the guest's job — but every register a driver
    // reads on the way in, checked from the outside.
    use crate::core::space::MemAttrs as Attrs;
    let b = board("virtio", &[]);
    let space = b.machine.space("mem").expect("one space");
    let base = 0x1000_1000u64;
    let read = |off: u64| {
        space
            .read(base + off, Width::U32, Attrs::DEFAULT)
            .expect("the transport answers")
    };
    assert_eq!(read(0x000) as u32, super::virtio::mmio::MAGIC);
    assert_eq!(read(0x004), u64::from(super::virtio::mmio::VERSION));
    assert_eq!(read(0x008), u64::from(super::virtio::DEVICE_ID_BLOCK));
    // The configuration space reports the size the machine file asked for.
    let capacity = space
        .read(base + 0x100, Width::U64, Attrs::DEFAULT)
        .expect("capacity");
    assert_eq!(capacity, 16 * 1024 * 1024 / 512, "16 MiB in sectors");

    let rng = 0x1000_2000u64;
    assert_eq!(
        space.read(rng + 0x008, Width::U32, Attrs::DEFAULT).unwrap(),
        u64::from(super::virtio::DEVICE_ID_ENTROPY)
    );
}

// ---------------------------------------------------------------------------
// firmware fetched at test time
// ---------------------------------------------------------------------------

/// Splice one more image into a machine source: a `riscv.loader` at `addr`,
/// and — when `addr` falls outside DRAM — the RAM to hold it.
///
/// The shipped board has exactly one firmware image because that is what a
/// board *is*. A supervisor-mode payload is a second one, and staging it is a
/// property of the experiment rather than of the machine, so it is spliced in
/// here instead of becoming a media slot every `rsemu run riscv-virt` would
/// have to bind. Two shapes have been wanted:
///
/// * a Linux `Image` at `0x80200000`, which is where OpenSBI's `fw_jump` hands
///   control on, and which is inside DRAM;
/// * a UEFI firmware volume at `0x20000000`, which is not — on a real `virt`
///   board that window is NOR flash, and the closest thing this board can
///   offer is RAM, which reads identically and differs only in that a CFI
///   erase-and-program sequence lands in it as plain data.
///
/// Appended last, deliberately: `Machine::reset` runs devices in declaration
/// order and a cold reset clears RAM, so a loader declared before the region it
/// writes into would have its image erased by the reset that ends realize.
#[cfg(feature = "std")]
fn with_payload(source: &str, index: usize, slot: &str, addr: u64, len: u64) -> String {
    let end = source
        .rfind('}')
        .expect("a machine description ends with a brace");
    let mut out = String::from(&source[..end]);
    if addr < DRAM {
        // Round up to a megabyte so the map statement is readable and a
        // firmware volume that expects its whole aperture finds one.
        let size = len.next_multiple_of(0x10_0000).max(0x10_0000);
        out.push_str(&alloc::format!(
            "\n  object staging{index} \"ram\" {{ size = {size} }}\n  map mem {addr:#x} size \
             {size:#x} = staging{index}\n"
        ));
    }
    out.push_str(&alloc::format!(
        "\n  object payload{index} \"riscv.loader\" {{\n    space = mem\n    image = \
         \"{slot}\"\n    addr  = {addr:#x}\n  }}\n"
    ));
    out.push_str(&source[end..]);
    out
}

/// `\n`, `\r`, `\t` and `\\` in a value that reached us through the
/// environment, where a real newline cannot.
#[cfg(feature = "std")]
fn unescape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut chars = text.chars();
    while let Some(c) = chars.next() {
        if c != '\\' {
            out.push(c);
            continue;
        }
        match chars.next() {
            Some('n') => out.push('\n'),
            Some('r') => out.push('\r'),
            Some('t') => out.push('\t'),
            Some('\\') => out.push('\\'),
            // Anything else is not an escape, so both characters are literal.
            Some(other) => {
                out.push('\\');
                out.push(other);
            }
            None => out.push('\\'),
        }
    }
    out
}

/// Boot whatever `RSEMU_RISCV_FIRMWARE` points at and report what it printed.
///
/// Skipped, cleanly, when the variable is unset — which is every ordinary
/// `cargo test` run. `scripts/fetch-testdata.sh riscv` downloads an OpenSBI
/// release build (BSD-2-Clause, so it may be used and read) and prints the
/// variable to set. Nothing is committed: `CLAUDE.md` forbids vendoring a
/// fixture whose licence has not been checked, and this one is fetched even
/// though its licence is fine, because the rule is about the repository rather
/// than about any one file.
///
/// `RSEMU_RISCV_PAYLOAD` stages further images, as a comma-separated list of
/// `addr:path`. That is how a supervisor-mode guest is booted: `fw_jump.bin` as
/// the firmware, and a Linux `Image` at `0x80200000` — where OpenSBI hands
/// control on — as the payload. An address outside DRAM gets RAM spliced in
/// under it.
///
/// `RSEMU_RISCV_STOP_AT` ends the run as soon as the guest has printed that
/// string, rather than at the quantum budget: firmware that reaches a prompt
/// never stops on its own.
///
/// `RSEMU_RISCV_FLASH0` and `RSEMU_RISCV_FLASH1` bind the board's two NOR
/// banks, which is how a UEFI build is booted: the code image goes in bank 0
/// and the variable store in bank 1. `RSEMU_RISCV_FLASH1_OUT` writes bank 1
/// back out when the run ends, so pointing `FLASH1` at that same file on the
/// next run is a reboot — and a variable written in one run is there in the
/// next, which is the whole reason the flash is a device rather than memory.
///
/// `RSEMU_RISCV_INITRD` binds a ramdisk to the `initrd` media slot, staged at
/// `RSEMU_RISCV_INITRD_ADDR` (default `0x88000000`) and described to the kernel
/// by `/chosen/linux,initrd-start`. `scripts/fetch-testdata.sh initramfs`
/// builds one that boots to a busybox shell.
///
/// `RSEMU_RISCV_DRIVE` backs the `disk` media slot with a host **file** rather
/// than with bytes — `--drive disk=root.qcow2`, in environment-variable form —
/// so the guest's root filesystem can be a sparse qcow2 that stays on disk
/// instead of a copy of it in host memory. `fstool` picks the backend from the
/// file's own contents, so raw, qcow2, DMG and LUKS all work, and the guest's
/// writes go back into that file: a run is a reboot of the previous one.
/// `RSEMU_RISCV_DRIVE_RO` opens it read-only, which the device reports as
/// `VIRTIO_BLK_F_RO` so the guest finds out before it tries. It wins over
/// `RSEMU_RISCV_DISK`, which stays the media-slot path.
///
/// `RSEMU_RISCV_INPUT` types at the guest: one `marker=>text` step per line,
/// where `text` takes `\n`, `\r`, `\t` and `\\`. Each step waits for its marker
/// in the guest's output and then feeds its text to the console. A prompt that
/// echoes what is typed at it is the only proof that the console is
/// bidirectional, and matching on output rather than on elapsed time keeps the
/// run deterministic.
#[cfg(feature = "std")]
#[test]
fn firmware_from_the_environment_reaches_its_console() {
    let Ok(path) = std::env::var("RSEMU_RISCV_FIRMWARE") else {
        eprintln!("skipped: set RSEMU_RISCV_FIRMWARE to a flat binary for 0x80000000");
        return;
    };
    let image = std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
    let ram = std::env::var("RSEMU_RISCV_RAM").unwrap_or_else(|_| String::from("256M"));
    let quanta: usize = std::env::var("RSEMU_RISCV_QUANTA")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2_000_000);
    let spec = std::env::var("RSEMU_RISCV_PAYLOAD").unwrap_or_default();
    let payloads: Vec<(u64, Vec<u8>)> = spec
        .split(',')
        .filter(|s| !s.trim().is_empty())
        .map(|item| {
            let (addr, path) = item
                .split_once(':')
                .unwrap_or_else(|| panic!("`{item}` is not `addr:path`"));
            let addr = u64::from_str_radix(addr.trim().trim_start_matches("0x"), 16)
                .unwrap_or_else(|e| panic!("`{addr}` is not a hexadecimal address: {e}"));
            let bytes =
                std::fs::read(path.trim()).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
            eprintln!("payload {addr:#x}: {path} ({} bytes)", bytes.len());
            (addr, bytes)
        })
        .collect();

    // The two NOR banks. Empty is a board with blank parts on it, which is
    // what the non-UEFI runs want.
    let bank = |var: &str| -> Vec<u8> {
        std::env::var(var).map_or_else(
            |_| Vec::new(),
            |path| {
                let bytes =
                    std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
                eprintln!("{var}: {path} ({} bytes)", bytes.len());
                bytes
            },
        )
    };
    let flash0 = bank("RSEMU_RISCV_FLASH0");
    let flash1 = bank("RSEMU_RISCV_FLASH1");
    // The ramdisk. Empty is a board that boots without one, which is every run
    // that came before this variable existed.
    let initrd = bank("RSEMU_RISCV_INITRD");
    // The virtio disk's contents. Empty is a blank disk of the machine file's
    // `storage` size, which is what every run before this variable had.
    let disk = bank("RSEMU_RISCV_DISK");

    let console_name = String::from("test.riscv.console.firmware");
    let power_name = String::from("test.riscv.power.firmware");
    let entry = catalog::machine("riscv-virt").expect("shipped");
    let mut options = catalog::build_options()
        .expect("catalog")
        .with_media("firmware", image.as_slice())
        .with_media("flash0", flash0.as_slice())
        .with_media("flash1", flash1.as_slice())
        .with_media("initrd", initrd.as_slice())
        .with_media("disk", disk.as_slice())
        .with_param("console", console_name.clone())
        .with_param("power", power_name.clone())
        .with_param("ram", ram)
        .with_param(
            "initrd_addr",
            std::env::var("RSEMU_RISCV_INITRD_ADDR").unwrap_or_else(|_| String::from("0x88000000")),
        )
        .with_param(
            "cmdline",
            std::env::var("RSEMU_RISCV_BOOTARGS")
                .unwrap_or_else(|_| String::from("console=ttyS0 earlycon=sbi")),
        );
    // `RSEMU_RISCV_DRIVE` backs the `disk` media slot with a host *file*
    // instead of with bytes: exactly what `--drive disk=root.qcow2` does, and
    // the reason a guest can root off a qcow2 that stays on disk. It wins over
    // `RSEMU_RISCV_DISK`, which stays the media-slot path (`no_std`, and what
    // wasm runs on).
    #[cfg(feature = "dev-blk")]
    if let Ok(path) = std::env::var("RSEMU_RISCV_DRIVE") {
        let read_only = std::env::var("RSEMU_RISCV_DRIVE_RO").is_ok();
        let mut opts = crate::dev::blk::ImageOptions::new().read_only(read_only);
        // `--drive disk=…,new=<size>` in environment-variable form: make the
        // image instead of opening one, so a first run has something to boot
        // off. `fstool` picks the backend from the extension, so `.qcow2` is a
        // qcow2 and anything else a sparse raw file.
        if let Ok(size) = std::env::var("RSEMU_RISCV_DRIVE_NEW") {
            let (digits, scale) = match size.as_bytes().last() {
                Some(b'K' | b'k') => (&size[..size.len() - 1], 1024),
                Some(b'M' | b'm') => (&size[..size.len() - 1], 1024 * 1024),
                Some(b'G' | b'g') => (&size[..size.len() - 1], 1024 * 1024 * 1024),
                _ => (size.as_str(), 1),
            };
            let bytes: u64 = digits
                .trim()
                .parse()
                .unwrap_or_else(|e| panic!("`{size}` is not a size: {e}"));
            opts = opts.create(bytes * scale);
        }
        let image = crate::dev::blk::Image::open(std::path::Path::new(&path), &opts)
            .unwrap_or_else(|e| panic!("cannot open {path}: {e}"));
        eprintln!(
            "RSEMU_RISCV_DRIVE: {} ({} bytes{})",
            image.describe(),
            crate::dev::ata::Medium::capacity(&image),
            if read_only { ", read-only" } else { "" }
        );
        crate::dev::blk::install(&options.realize.hosts, "disk", alloc::sync::Arc::new(image))
            .expect("nothing else claimed the `disk` media slot");
    }

    let mut source = String::from(entry.source);
    for (i, (addr, bytes)) in payloads.iter().enumerate() {
        let slot = alloc::format!("payload{i}");
        source = with_payload(&source, i, &slot, *addr, bytes.len() as u64);
        options.realize.media.insert(slot, bytes.as_slice());
    }
    let machine = crate::machine::build(
        entry.name,
        &source,
        &catalog::registry().expect("catalog"),
        &options,
    )
    .expect("riscv-virt builds");
    let mut b = Board {
        machine,
        console: ports::open(&options.realize.hosts, &console_name).expect("the UART opened it"),
        power: signals::open(&options.realize.hosts, &power_name).expect("the syscon opened it"),
    };

    // Streamed rather than collected: a kernel boot takes minutes of host time
    // under an interpreter, and a run that prints nothing until it finishes is
    // a run nobody can tell apart from a hang.
    use std::io::Write as _;
    // `RSEMU_RISCV_STOP_AT` ends the run at the first line containing it. A
    // firmware that reaches a prompt does not stop by itself, and a run that
    // then burns its whole quantum budget idling is a run that takes minutes
    // to say what it already said — and, worse, one whose flash never gets
    // written back out.
    let stop_at = std::env::var("RSEMU_RISCV_STOP_AT").unwrap_or_default();
    // A scripted session: one `marker=>text` step per line. Once the guest has
    // printed `marker`, `text` is typed at it. Typing is the only way to show
    // that a console is bidirectional rather than write-only, and keying it
    // off what the guest *said* rather than off elapsed time keeps the run
    // deterministic — no wall clock is consulted anywhere in the loop. The
    // separator is a newline rather than anything punctuation-shaped because
    // the text is usually a shell command, and every candidate separator is
    // something a shell command is allowed to contain.
    let script: Vec<(String, String)> = std::env::var("RSEMU_RISCV_INPUT")
        .unwrap_or_default()
        .split('\n')
        .filter(|s| !s.trim().is_empty())
        .map(|step| {
            let (marker, text) = step
                .split_once("=>")
                .unwrap_or_else(|| panic!("`{step}` is not `marker=>text`"));
            (String::from(marker), unescape(text))
        })
        .collect();
    let mut step = 0usize;
    // Enough tail to span a marker that arrives split across two quanta.
    let window = script
        .iter()
        .map(|(marker, _)| marker.len())
        .chain(core::iter::once(stop_at.len()))
        .max()
        .unwrap_or(0)
        .max(1);
    let mut printed = 0usize;
    let mut seen = String::new();
    eprintln!("--- guest console ---");
    for _ in 0..quanta {
        if b.power.peek().is_some() {
            break;
        }
        b.machine.run_quantum().expect("the machine advances");
        let out = b.output();
        if out.is_empty() {
            continue;
        }
        printed += out.len();
        eprint!("{out}");
        let _ = std::io::stderr().flush();
        seen.push_str(&out);
        if let Some((marker, text)) = script.get(step)
            && seen.contains(marker.as_str())
        {
            eprintln!("\n(typing `{}`)", text.escape_debug());
            let fed = b.console.feed(text.as_bytes());
            assert_eq!(
                fed,
                text.len(),
                "the console took {fed} of {} byte(s)",
                text.len()
            );
            step += 1;
            // So the next step's marker is matched against what the guest says
            // *after* this keystroke, not against the prompt that triggered it.
            seen.clear();
        }
        // Only once the script has run: the marker that ends the run is
        // usually the reply to the last thing typed.
        if !stop_at.is_empty() && step >= script.len() && seen.contains(&stop_at) {
            eprintln!("\n(stopping: the guest printed `{stop_at}`)");
            break;
        }
        if seen.len() > 4 * window {
            seen.drain(..seen.len() - 2 * window);
        }
    }
    let tail = b.output();
    printed += tail.len();
    eprintln!("{tail}\n--------------------- {printed} byte(s)");

    // Write the variable bank back out, if asked. Read with `MemAttrs::DEBUG`
    // through the ordinary address space: the flash honours the debug flag by
    // answering with its contents whatever its command state machine is doing,
    // which is exactly what a snapshot of the array wants and exactly what
    // invariant 5 is for. Nothing here knows the concrete device type.
    if let Ok(out) = std::env::var("RSEMU_RISCV_FLASH1_OUT") {
        let len = flash1.len().max(1);
        let mut bytes = alloc::vec![0u8; len];
        b.machine
            .space("mem")
            .expect("the board has one space")
            .read_bytes(0x2200_0000, &mut bytes, crate::core::space::MemAttrs::DEBUG)
            .expect("the variable bank is mapped");
        std::fs::write(&out, &bytes).unwrap_or_else(|e| panic!("cannot write {out}: {e}"));
        eprintln!("wrote {} byte(s) of flash1 to {out}", bytes.len());
    }

    assert!(
        printed > 0,
        "the firmware printed nothing in {quanta} quanta"
    );
}

// ---------------------------------------------------------------------------
// a very small device tree reader, for the assertions above
// ---------------------------------------------------------------------------

/// The `interrupts` cell of the node called `name`.
fn interrupts_of(dtb: &[u8], name: &str) -> Option<u32> {
    prop_u32(dtb, name, "interrupts")
}

/// The first two cells of property `prop` in node `name`, as one 64-bit value.
fn prop_u64(dtb: &[u8], name: &str, prop: &str) -> Option<u64> {
    let bytes = prop_bytes(dtb, name, prop)?;
    let eight: [u8; 8] = bytes.get(..8)?.try_into().ok()?;
    Some(u64::from_be_bytes(eight))
}

/// The first cell of property `prop` in node `name`.
fn prop_u32(dtb: &[u8], name: &str, prop: &str) -> Option<u32> {
    let bytes = prop_bytes(dtb, name, prop)?;
    let four: [u8; 4] = bytes.get(..4)?.try_into().ok()?;
    Some(u32::from_be_bytes(four))
}

/// The raw value of property `prop` in node `name`.
fn prop_bytes<'a>(dtb: &'a [u8], name: &str, prop: &str) -> Option<&'a [u8]> {
    let word =
        |at: usize| -> u32 { u32::from_be_bytes([dtb[at], dtb[at + 1], dtb[at + 2], dtb[at + 3]]) };
    let off_struct = word(8) as usize;
    let len_struct = word(36) as usize;
    let off_strings = word(12) as usize;
    let name_at = |at: usize| -> &str {
        let end = dtb[at..].iter().position(|b| *b == 0).unwrap_or(0) + at;
        core::str::from_utf8(&dtb[at..end]).unwrap_or("")
    };

    let mut at = off_struct;
    let end = off_struct + len_struct;
    let mut inside = false;
    let mut depth = 0usize;
    while at + 4 <= end {
        let token = word(at);
        at += 4;
        match token {
            1 => {
                let node = name_at(at);
                at += node.len() + 1;
                at = at.next_multiple_of(4);
                depth += 1;
                if node == name {
                    inside = true;
                }
            }
            2 => {
                if inside && depth > 0 {
                    inside = false;
                }
                depth = depth.saturating_sub(1);
            }
            3 => {
                let len = word(at) as usize;
                let name_off = word(at + 4) as usize;
                at += 8;
                if inside && name_at(off_strings + name_off) == prop {
                    return dtb.get(at..at + len);
                }
                at += len;
                at = at.next_multiple_of(4);
            }
            9 => break,
            _ => {}
        }
    }
    None
}
