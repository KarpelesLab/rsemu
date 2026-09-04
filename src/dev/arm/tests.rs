//! The `arm64-virt` board, end to end.
//!
//! A unit test can say "the GIC claimed interrupt 27". This says something
//! stronger: an AArch64 core **named in a `.machine` file** starts in a boot
//! ROM, is handed a device tree in `x0` that was generated from the machine it
//! is running on, prints through a PL011 that is where that tree says it is,
//! programs the GIC out of the same tree's numbers, takes a *timer* interrupt
//! that leaves the core, crosses the distributor and comes back on `nIRQ`, and
//! then switches the machine off with a PSCI call.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-arm64-virt`.

use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::space::MemAttrs;
use crate::core::value::Width;
use crate::cpu::arm::a64::sysreg::enc;
use crate::host::chardev::{CharPort, ports};
use crate::machine::{Machine, catalog};

use super::loader::{HEADER_LEN, IMAGE_MAGIC};
use super::power::{Request, Signal, signals};

/// Where the board's DRAM starts, with the file's default `ram-base`.
const DRAM: u64 = 0x4000_0000;
/// Where the kernel — here, the test's own program — is entered.
const ENTRY: u64 = DRAM + 0x20_0000;
/// The GIC distributor, as the machine file maps it.
const GICD: u64 = 0x0800_0000;
/// The GIC CPU interface.
const GICC: u64 = 0x0801_0000;
/// The PL011.
const UART: u64 = 0x0900_0000;
/// The virtio-MMIO window the disk is behind.
const BLK: u64 = 0x0a00_0000;
/// The virtio-MMIO window the entropy source is behind.
const RNG: u64 = 0x0a00_1000;

// ---------------------------------------------------------------------------
// A very small A64 assembler
// ---------------------------------------------------------------------------
//
// Encodings from DDI 0487 C4.1, and every one of them is exercised by the
// core's own decoder tests against independently known words. Written as
// functions rather than as a table of hex so the programs below read as
// programs.

const fn movz(rd: u32, imm: u32, shift: u32) -> u32 {
    0xd280_0000 | ((shift / 16) << 21) | ((imm & 0xffff) << 5) | rd
}
const fn movk(rd: u32, imm: u32, shift: u32) -> u32 {
    0xf280_0000 | ((shift / 16) << 21) | ((imm & 0xffff) << 5) | rd
}
/// `MOV Xd, Xm`, which is `ORR Xd, XZR, Xm`.
const fn mov_reg(rd: u32, rm: u32) -> u32 {
    0xaa00_0000 | (rm << 16) | (31 << 5) | rd
}
/// `STR Wt, [Xn, #offset]` — the offset is scaled by four.
const fn str_w(rt: u32, rn: u32, offset: u32) -> u32 {
    0xb900_0000 | ((offset / 4) << 10) | (rn << 5) | rt
}
/// `LDR Wt, [Xn, #offset]`.
const fn ldr_w(rt: u32, rn: u32, offset: u32) -> u32 {
    0xb940_0000 | ((offset / 4) << 10) | (rn << 5) | rt
}
/// `STR Xt, [Xn, #offset]` — the offset is scaled by eight.
const fn str_x(rt: u32, rn: u32, offset: u32) -> u32 {
    0xf900_0000 | ((offset / 8) << 10) | (rn << 5) | rt
}
const fn b(words: i32) -> u32 {
    0x1400_0000 | ((words as u32) & 0x03ff_ffff)
}
/// `CBZ Xt, .` — branch to itself while `Xt` is zero, which is how a program
/// waits for its own interrupt handler.
const fn cbz_self(rt: u32) -> u32 {
    0xb400_0000 | rt
}
fn mrs(reg: u16, rt: u32) -> u32 {
    0xd530_0000 | (u32::from(reg) << 5) | rt
}
fn msr(reg: u16, rt: u32) -> u32 {
    0xd510_0000 | (u32::from(reg) << 5) | rt
}
const ERET: u32 = 0xd69f_03e0;
const ISB: u32 = 0xd503_3fdf;
/// `MSR DAIFClr, #2` — unmask IRQ. `PSTATE.DAIF` comes up with every mask set,
/// so this is the instruction that lets an interrupt in at all.
const DAIFCLR_I: u32 = 0xd503_42ff;
/// `SMC #0`, which on this board is a PSCI call.
const SMC0: u32 = 0xd400_0003;

const VBAR_EL1: u16 = enc(3, 0, 12, 0, 0);
const CNTV_TVAL_EL0: u16 = enc(3, 3, 14, 3, 0);
const CNTV_CTL_EL0: u16 = enc(3, 3, 14, 3, 1);
const CNTFRQ_EL0: u16 = enc(3, 3, 14, 0, 0);

/// Load a 32-bit constant into `Xd` with two moves.
fn load32(rd: u32, value: u32) -> [u32; 2] {
    [
        movz(rd, value & 0xffff, 0),
        movk(rd, (value >> 16) & 0xffff, 16),
    ]
}

/// A program, as words, with an AArch64 `Image` header in front of it.
///
/// The header is not decoration: the machine file's kernel loader is
/// `format = "arm64"` and refuses anything that is not an `Image` that wants
/// to be where the board puts it. So a test program is a kernel, in the only
/// sense the loader cares about — and that is one more thing checked by every
/// test in this file rather than by one of them.
struct Program {
    words: Vec<u32>,
}

impl Program {
    /// A program whose first instruction is at [`Program::CODE`].
    fn new() -> Program {
        Program { words: Vec::new() }
    }

    /// Where the code starts, past the header.
    const CODE: u64 = HEADER_LEN as u64;

    fn push(&mut self, word: u32) -> &mut Program {
        self.words.push(word);
        self
    }

    fn extend(&mut self, words: impl IntoIterator<Item = u32>) -> &mut Program {
        self.words.extend(words);
        self
    }

    /// The offset of the next word, from the start of the image.
    fn here(&self) -> u64 {
        Program::CODE + self.words.len() as u64 * 4
    }

    /// Pad with `UDF`-shaped zeros up to `offset` from the start of the image.
    fn pad_to(&mut self, offset: u64) -> &mut Program {
        assert!(offset >= self.here(), "already past {offset:#x}");
        while self.here() < offset {
            self.words.push(0);
        }
        self
    }

    /// The image: the header, then the words.
    fn image(&self) -> Vec<u8> {
        let mut out = alloc::vec![0u8; HEADER_LEN];
        // `code0` branches over the rest of the header to the first real
        // instruction, which is exactly what a kernel's own `code0` does.
        out[0..4].copy_from_slice(&b((HEADER_LEN / 4) as i32).to_le_bytes());
        // `text_offset = 0`: a relocatable kernel, which is every kernel
        // built in the last decade and what the board defaults to.
        out[0x08..0x10].copy_from_slice(&0u64.to_le_bytes());
        // A non-zero `image_size`, so the loader reads `text_offset` rather
        // than assuming the default — which is the path a real kernel takes.
        out[0x10..0x18].copy_from_slice(&0x10_0000u64.to_le_bytes());
        out[0x18..0x20].copy_from_slice(&0u64.to_le_bytes());
        out[0x38..0x3c].copy_from_slice(&IMAGE_MAGIC.to_le_bytes());
        for word in &self.words {
            out.extend_from_slice(&word.to_le_bytes());
        }
        out
    }
}

// ---------------------------------------------------------------------------
// building and running the board
// ---------------------------------------------------------------------------

/// A built machine plus the host ends of its console and its power button.
struct Board {
    machine: Machine,
    console: Arc<CharPort>,
    power: Arc<Signal>,
}

/// Build `arm64-virt` with `kernel` loaded.
fn board(kernel: &[u8]) -> Board {
    board_named("arm64-virt", kernel)
}

/// The same, for one of the board's variants: `arm64-virt-smp` is this file
/// with a second core.
fn board_named(name: &'static str, kernel: &[u8]) -> Board {
    let entry = catalog::machine(name).expect("this build ships it");
    let options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_media("kernel", kernel)
        .with_media("initrd", &[][..])
        .with_media("disk", &[][..])
        // Enough for the programs here, and small enough that the cold reset
        // that clears it is not the slowest part of the test.
        .with_param("ram", String::from("8M"))
        // A blank disk that costs 64 KiB of host memory rather than the
        // board's default 16 MiB: nothing here reads a sector, and every one
        // of these tests would otherwise allocate and zero the whole platter.
        .with_param("storage", String::from("64K"));
    let registry = catalog::registry().expect("the catalog agrees with itself");
    let machine = match crate::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("{name} does not build: {e}"),
    };
    Board {
        console: ports::open(&options.realize.hosts, "console").expect("the PL011 opened it"),
        power: signals::open(&options.realize.hosts, "power").expect("the controller opened it"),
        machine,
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

    /// One value of the guest's memory space, read the way a debugger would.
    fn peek(&self, addr: u64, width: Width) -> u64 {
        self.machine
            .space("mem")
            .expect("the machine has one space")
            .read(addr, width, MemAttrs::DEBUG)
            .unwrap_or_else(|e| panic!("nothing answers at {addr:#x}: {e}"))
    }

    /// The device tree the boot ROM generated, read out of guest memory
    /// exactly as the kernel would find it.
    fn device_tree(&self) -> Vec<u8> {
        let at = super::boot::DTB_OFFSET;
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

/// A program that does nothing, for a test that only wants the board.
fn spin() -> Vec<u8> {
    let mut p = Program::new();
    p.push(b(0));
    p.image()
}

#[test]
fn the_board_realizes_with_every_object_the_file_names() {
    let m = board(&spin()).machine;
    assert_eq!(m.name(), "arm64-virt");
    for path in ["cpu", "gic", "uart", "pwr", "boot", "dram", "kernel"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
}

#[test]
fn the_boot_rom_hands_over_a_device_tree_that_parses() {
    let b = board(&spin());
    let dtb = b.device_tree();
    let text = super::dt::describe(&dtb).expect("a tree this writer produced");
    for node in [
        "cpus {",
        "cpu@0 {",
        "memory@40000000 {",
        "intc@8000000 {",
        "pl011@9000000 {",
        "psci {",
        "timer {",
        "apb-pclk {",
        "chosen {",
    ] {
        assert!(text.contains(node), "no `{node}` in the tree:\n{text}");
    }
}

#[test]
fn the_trees_addresses_come_out_of_the_map_statements() {
    // The point of generating the tree at all: nothing here is written down
    // twice, so a `map` that moved would move the node with it.
    let b = board(&spin());
    let dtb = b.device_tree();
    // `reg = <0 0x08000000 0 0x1000>, <0 0x08010000 0 0x2000>` on the GIC:
    // two apertures joined into one node from two mappings.
    let needle: Vec<u8> = [0u32, 0x0800_0000, 0, 0x1000, 0, 0x0801_0000, 0, 0x2000]
        .iter()
        .flat_map(|w| w.to_be_bytes())
        .collect();
    assert!(
        dtb.windows(needle.len()).any(|w| w == needle),
        "the GIC's two reg entries are not what the machine file mapped"
    );
}

#[test]
fn the_smp_board_describes_two_processors_and_where_the_second_one_waits() {
    // `arm64-virt-smp` is the same file with a second core, and everything a
    // guest has to be told about that core is in the generated tree.
    let b = board_named("arm64-virt-smp", &spin());
    let dtb = b.device_tree();
    let text = super::dt::describe(&dtb).expect("the generator's own tree parses");
    // Two processors, named by `MPIDR_EL1` affinity 0, each with a word of the
    // release table and the boot method that says so.
    assert!(text.contains("cpu@0 {"), "{text}");
    assert!(text.contains("cpu@1 {"), "{text}");
    assert!(text.contains("cpu-release-addr"), "{text}");
    assert!(text.contains("enable-method"), "{text}");
    // `machines/arm64-virt-smp.machine` puts the table at 0x40001000, so
    // processor 1's word is eight bytes past it.
    let needle = 0x4000_1008u64.to_be_bytes();
    assert!(
        dtb.windows(8).any(|w| w == needle),
        "processor 1's `cpu-release-addr` is not a word past the table"
    );
    // And the page it lands in is reserved, or the kernel's own allocator
    // hands out the memory a parked processor is reading.
    let reservation: Vec<u8> = [0x4000_1000u64, 0x1000]
        .iter()
        .flat_map(|w| w.to_be_bytes())
        .collect();
    assert!(
        dtb.windows(reservation.len()).any(|w| w == reservation),
        "the release table's page is not in the memory reservation block"
    );
}

#[test]
fn the_uarts_interrupt_number_comes_out_of_the_wire() {
    // `wire uart.irq -> gic.spi1` is the only place `1` is written, and the
    // tree says `<0 1 4>` — the binding's own numbering, which subtracts the
    // shared-interrupt base again.
    let b = board(&spin());
    let dtb = b.device_tree();
    let needle: Vec<u8> = [0u32, 1, 4].iter().flat_map(|w| w.to_be_bytes()).collect();
    assert!(
        dtb.windows(needle.len()).any(|w| w == needle),
        "the PL011's `interrupts` is not <0 1 4>"
    );
}

// ---------------------------------------------------------------------------
// virtio
// ---------------------------------------------------------------------------

#[test]
fn the_virtio_devices_are_discoverable_at_the_addresses_the_board_maps() {
    // Not a driver — a driver is the guest's job — but every register a driver
    // reads on the way in, checked from the outside, through the board's own
    // address space. `dev::virtio` is shared with `riscv-virt` and this is what
    // says the sharing reaches an AArch64 board rather than merely compiling
    // for one.
    let b = board(&spin());
    let space = b.machine.space("mem").expect("one space");
    let read = |at: u64| {
        space
            .read(at, Width::U32, MemAttrs::DEFAULT)
            .expect("the transport answers")
    };
    assert_eq!(read(BLK) as u32, crate::dev::virtio::mmio::MAGIC);
    assert_eq!(
        read(BLK + 0x004),
        u64::from(crate::dev::virtio::mmio::VERSION)
    );
    assert_eq!(
        read(BLK + 0x008),
        u64::from(crate::dev::virtio::DEVICE_ID_BLOCK)
    );
    // The configuration space reports the capacity in sectors — `storage`,
    // which `board` sets to 64 KiB so that the platter is not the largest
    // thing these tests allocate.
    assert_eq!(
        space
            .read(BLK + 0x100, Width::U64, MemAttrs::DEFAULT)
            .expect("capacity"),
        64 * 1024 / 512
    );
    assert_eq!(
        read(RNG + 0x008),
        u64::from(crate::dev::virtio::DEVICE_ID_ENTROPY)
    );
}

#[test]
fn the_virtio_nodes_carry_the_gics_numbering_and_not_a_plics() {
    // The whole point of keeping the *generator* per-architecture while
    // sharing the transport: the same `dev::virtio` device that a RISC-V tree
    // describes with a one-cell `interrupts` is described here in three cells,
    // with the shared-peripheral base subtracted again. `wire blk.irq ->
    // gic.spi2` is the only place `2` is written down.
    let b = board(&spin());
    let dtb = b.device_tree();
    let cells = |spi: u32| -> Vec<u8> {
        [0u32, spi, 4]
            .iter()
            .flat_map(|w| w.to_be_bytes())
            .collect()
    };
    assert!(
        dtb.windows(12).any(|w| w == cells(2)),
        "the disk's `interrupts` is not <0 2 4>"
    );
    assert!(
        dtb.windows(12).any(|w| w == cells(3)),
        "the entropy source's `interrupts` is not <0 3 4>"
    );
    // And the node is at the address the `map` statement made, named the way
    // the binding wants it.
    let name = b"virtio_mmio@a000000\0";
    assert!(
        dtb.windows(name.len()).any(|w| w == name),
        "no `virtio_mmio@a000000` node in the generated tree"
    );
    let compatible = b"virtio,mmio\0";
    assert!(
        dtb.windows(compatible.len()).any(|w| w == compatible),
        "the node does not claim `virtio,mmio`"
    );
}

// ---------------------------------------------------------------------------
// the console
// ---------------------------------------------------------------------------

/// Put `x1` at the PL011 and print `text` through `UARTDR`, a byte at a time.
fn print(p: &mut Program, text: &str) {
    p.extend(load32(1, UART as u32));
    for byte in text.bytes() {
        p.push(movz(2, u32::from(byte), 0));
        p.push(str_w(2, 1, 0));
    }
}

#[test]
fn a_program_prints_through_the_pl011() {
    let mut p = Program::new();
    print(&mut p, "hello\n");
    p.push(b(0));
    let mut b = board(&p.image());
    b.run(4);
    assert_eq!(b.output(), "hello\n");
}

// ---------------------------------------------------------------------------
// PSCI
// ---------------------------------------------------------------------------

/// `SYSTEM_OFF`, which is the only way a headless test ends on purpose.
fn poweroff(p: &mut Program) {
    p.extend(load32(
        0,
        super::super::super::cpu::arm::a64::psci::fid::SYSTEM_OFF,
    ));
    p.push(SMC0);
    p.push(b(0));
}

#[test]
fn a_psci_system_off_reaches_the_boards_power_controller() {
    let mut p = Program::new();
    print(&mut p, "bye\n");
    poweroff(&mut p);
    let mut b = board(&p.image());
    assert_eq!(
        b.run(64),
        Some(Request::Poweroff),
        "the machine did not stop; it printed {:?}",
        b.output()
    );
    assert_eq!(b.output(), "bye\n");
}

#[test]
fn psci_version_reports_one_point_zero_to_the_guest() {
    use crate::cpu::arm::a64::psci::fid;
    let mut p = Program::new();
    // x0 = PSCI_VERSION; smc; store the answer in DRAM.
    p.extend(load32(0, fid::VERSION));
    p.push(SMC0);
    p.extend(load32(1, DRAM as u32));
    p.push(movk(1, (DRAM >> 32) as u32, 32));
    p.push(str_x(0, 1, 0));
    poweroff(&mut p);
    let mut b = board(&p.image());
    assert_eq!(b.run(64), Some(Request::Poweroff));
    assert_eq!(
        b.peek(DRAM, Width::U64),
        0x0001_0000,
        "major version in the top half"
    );
}

// ---------------------------------------------------------------------------
// the timer, through the GIC
// ---------------------------------------------------------------------------

/// The interrupt id the EL1 virtual timer arrives on: PPI 11, which is
/// architectural interrupt 27. Written once here and once in the machine
/// file's `wire`, and the device tree derives its own copy from the wire.
const TIMER_INTID: u32 = 27;

#[test]
fn a_timer_interrupt_leaves_the_core_crosses_the_gic_and_comes_back() {
    // The wiring this board exists to prove. The generic timer is inside the
    // core; on `a64-mini` it raises `IRQ` there and nothing else is involved.
    // Here it leaves on `cpu.cntv`, the distributor decides whether to forward
    // it, the CPU interface decides whether it beats the running priority, and
    // only then does `nIRQ` move. A core that also raised it internally would
    // pass this test's *first* assertion and live-lock on the second, because
    // the handler would read `GICC_IAR` and be told 1023.
    let mut p = Program::new();

    // The vector table, first, because the code has to know where it is.
    let vectors = ENTRY + 0x1000;
    p.extend(load32(3, vectors as u32));
    p.push(movk(3, (vectors >> 32) as u32, 32));
    p.push(msr(VBAR_EL1, 3));

    // -- the GIC --------------------------------------------------------
    p.extend(load32(1, GICD as u32));
    // GICD_IPRIORITYR: interrupt 27's byte is the fourth of the word at 0x418.
    p.push(movz(2, 0x8000, 16));
    p.push(str_w(2, 1, 0x418));
    // GICD_ISENABLER0 |= 1 << 27.
    p.extend(load32(2, 1 << TIMER_INTID));
    p.push(str_w(2, 1, 0x100));
    // GICD_CTLR = 1, last, so nothing is forwarded half-configured.
    p.push(movz(2, 1, 0));
    p.push(str_w(2, 1, 0x000));

    p.extend(load32(1, GICC as u32));
    // GICC_PMR = 0xf0: everything stronger than 0xf0 gets through.
    p.push(movz(2, 0xf0, 0));
    p.push(str_w(2, 1, 0x004));
    p.push(movz(2, 1, 0));
    p.push(str_w(2, 1, 0x000));

    // -- the timer ------------------------------------------------------
    //
    // A deadline a few hundred counts away. `CNTFRQ_EL0` is read only to prove
    // the board programmed it; nothing divides by it.
    p.push(mrs(CNTFRQ_EL0, 10));
    p.push(movz(2, 200, 0));
    p.push(msr(CNTV_TVAL_EL0, 2));
    p.push(movz(2, 1, 0));
    p.push(msr(CNTV_CTL_EL0, 2));
    p.push(ISB);

    // -- wait for it ----------------------------------------------------
    p.push(movz(21, 0, 0));
    p.push(DAIFCLR_I);
    p.push(cbz_self(21));

    // The handler ran. Record what it saw and stop.
    p.extend(load32(1, DRAM as u32));
    p.push(movk(1, (DRAM >> 32) as u32, 32));
    p.push(str_x(21, 1, 0));
    p.push(str_x(22, 1, 8));
    p.push(str_x(10, 1, 16));
    poweroff(&mut p);

    // -- the vector table -----------------------------------------------
    //
    // Sixteen slots 128 bytes apart. The one that matters is "current level
    // with SP_ELx, IRQ", which is offset 0x280 — a core that took the
    // exception with `SP_EL0` selected would land at 0x80 instead and run off
    // into zeros.
    p.pad_to(0x1000);
    p.pad_to(0x1280);
    // x22 = what GICC_IAR said. Anything other than 27 means the interrupt
    // reached the core without the distributor knowing about it.
    p.extend(load32(1, GICC as u32));
    p.push(ldr_w(22, 1, 0x00c));
    // Disable the timer before ending the interrupt: it is level-sensitive and
    // would re-pend the moment the handler returned.
    p.push(movz(2, 0, 0));
    p.push(msr(CNTV_CTL_EL0, 2));
    p.push(str_w(22, 1, 0x010));
    p.push(movz(21, 1, 0));
    p.push(ERET);

    let mut b = board(&p.image());
    assert_eq!(
        b.run(400),
        Some(Request::Poweroff),
        "the handler never ran; the console said {:?}",
        b.output()
    );
    assert_eq!(b.peek(DRAM, Width::U64), 1, "the handler set its marker");
    assert_eq!(
        b.peek(DRAM + 8, Width::U64),
        u64::from(TIMER_INTID),
        "GICC_IAR did not name the virtual timer's PPI"
    );
    assert_eq!(
        b.peek(DRAM + 16, Width::U64),
        62_500_000,
        "CNTFRQ_EL0 is not what the machine file said"
    );
}

// ---------------------------------------------------------------------------
// the hand-off
// ---------------------------------------------------------------------------

#[test]
fn the_kernel_is_entered_with_the_tree_in_x0_and_nothing_else_set() {
    // The AArch64 boot convention, and the one thing a kernel cannot discover
    // for itself.
    let mut p = Program::new();
    p.push(mov_reg(19, 0));
    p.extend(load32(1, DRAM as u32));
    p.push(movk(1, (DRAM >> 32) as u32, 32));
    p.push(str_x(19, 1, 0));
    p.push(str_x(1, 1, 8)); // a placeholder, overwritten below
    p.push(str_x(2, 1, 16));
    p.push(str_x(3, 1, 24));
    poweroff(&mut p);

    let mut b = board(&p.image());
    assert_eq!(b.run(64), Some(Request::Poweroff));
    let dtb_at = b.peek(DRAM, Width::U64);
    assert_eq!(
        dtb_at,
        super::boot::DTB_OFFSET,
        "x0 does not point at the generated tree"
    );
    assert_eq!(
        b.peek(dtb_at, Width::U32) & 0xffff_ffff,
        0xedfe_0dd0,
        "and what it points at is not a device tree (the magic, byte-swapped)"
    );
    assert_eq!(b.peek(DRAM + 16, Width::U64), 0, "x2");
    assert_eq!(b.peek(DRAM + 24, Width::U64), 0, "x3");
}

#[test]
fn a_kernel_that_is_not_an_image_is_refused_by_name() {
    // The loader reads the header, so the machine does not build at all — and
    // the message says which format was found and which two are commonly
    // confused with it.
    let entry = catalog::machine("arm64-virt").expect("this build ships it");
    let options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_media("kernel", &alloc::vec![0u8; 1024][..])
        .with_media("initrd", &[][..])
        .with_media("disk", &[][..])
        .with_param("ram", String::from("8M"))
        .with_param("storage", String::from("64K"));
    let registry = catalog::registry().expect("a registry");
    let e = crate::machine::build(entry.name, entry.source, &registry, &options)
        .expect_err("a kernel with no magic must be refused")
        .to_string();
    assert!(e.contains("ARM"), "{e}");
}
