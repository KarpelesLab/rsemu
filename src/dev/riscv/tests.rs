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

/// Build `riscv-virt` with `firmware` loaded and per-test port names, so two
/// boards in one test binary do not type at each other.
fn board(tag: &str, firmware: &[u8]) -> Board {
    let console_name = alloc::format!("test.riscv.console.{tag}");
    let power_name = alloc::format!("test.riscv.power.{tag}");
    let entry = catalog::machine("riscv-virt").expect("this build ships it");
    let options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_media("firmware", firmware)
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
        console: ports::open(&console_name),
        power: signals::open(&power_name),
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
        .with_param("console", "test.riscv.console.moved")
        .with_param("power", "test.riscv.power.moved")
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
        console: ports::open("test.riscv.console.moved"),
        power: signals::open("test.riscv.power.moved"),
    };
    let tree = super::dt::describe(&b2.device_tree()).expect("it parses");
    assert!(tree.contains("serial@10004000"), "{tree}");
    assert!(!tree.contains("serial@10000000"), "{tree}");
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

/// Boot whatever `RSEMU_RISCV_FIRMWARE` points at and report what it printed.
///
/// Skipped, cleanly, when the variable is unset — which is every ordinary
/// `cargo test` run. `scripts/fetch-testdata.sh riscv` downloads an OpenSBI
/// release build (BSD-2-Clause, so it may be used and read) and prints the
/// variable to set. Nothing is committed: `CLAUDE.md` forbids vendoring a
/// fixture whose licence has not been checked, and this one is fetched even
/// though its licence is fine, because the rule is about the repository rather
/// than about any one file.
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

    let console_name = String::from("test.riscv.console.firmware");
    let power_name = String::from("test.riscv.power.firmware");
    let entry = catalog::machine("riscv-virt").expect("shipped");
    let options = catalog::build_options()
        .expect("catalog")
        .with_media("firmware", image.as_slice())
        .with_param("console", console_name.clone())
        .with_param("power", power_name.clone())
        .with_param("ram", ram)
        .with_param(
            "cmdline",
            std::env::var("RSEMU_RISCV_BOOTARGS")
                .unwrap_or_else(|_| String::from("console=ttyS0 earlycon=sbi")),
        );
    let machine = crate::machine::build(
        entry.name,
        entry.source,
        &catalog::registry().expect("catalog"),
        &options,
    )
    .expect("riscv-virt builds");
    let mut b = Board {
        machine,
        console: ports::open(&console_name),
        power: signals::open(&power_name),
    };

    // Streamed rather than collected: a kernel boot takes minutes of host time
    // under an interpreter, and a run that prints nothing until it finishes is
    // a run nobody can tell apart from a hang.
    use std::io::Write as _;
    let mut printed = 0usize;
    eprintln!("--- guest console ---");
    for _ in 0..quanta {
        if b.power.peek().is_some() {
            break;
        }
        b.machine.run_quantum().expect("the machine advances");
        let out = b.output();
        if !out.is_empty() {
            printed += out.len();
            eprint!("{out}");
            let _ = std::io::stderr().flush();
        }
    }
    let tail = b.output();
    printed += tail.len();
    eprintln!("{tail}\n--------------------- {printed} byte(s)");
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

/// The first cell of property `prop` in node `name`.
fn prop_u32(dtb: &[u8], name: &str, prop: &str) -> Option<u32> {
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
                if inside && name_at(off_strings + name_off) == prop && len >= 4 {
                    return Some(word(at));
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
