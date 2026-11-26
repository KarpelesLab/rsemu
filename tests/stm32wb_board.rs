//! The `stm32wb55` and `stm32wb35` boards, end to end, with no corpus.
//!
//! `tests/stm32wb_ble.rs` runs ST's HAL and a mailbox client on these boards,
//! but only where someone has fetched and built it. This is what `cargo test`
//! always proves, by playing CPU1's half of the mailbox from the host over the
//! board's own bus — every access an ordinary, non-debug one, so the clock
//! gates, IPCC and PWR see exactly what firmware would do:
//!
//! * `PWR_CR4.C2BOOT` reaches CPU2 through `pwr.c2boot -> cpu2.boot`, and CPU2
//!   finds the reference table at the start of SRAM2a;
//! * its READY event lands on the system queue and occupies IPCC channel 2
//!   (that the line reaches NVIC position 44 is `tests/stm32wb_ble.rs`'s to
//!   prove: the NVIC is the core's, and not on the bus this test drives);
//! * `SHCI_C2_BLE_Init` on channel 2 and `HCI_Reset` on channel 1 are answered
//!   in the formats AN5185 §6.3 and the Core Specification give;
//! * IPCC is gated by RCC: without `AHB3ENR.IPCCEN` it answers nothing;
//! * the CPU2-only top of flash is closed to CPU1, and the WB35 is the smaller
//!   part.
//!
//! The core runs `b .` throughout: [`IDLE`], a two-word vector table and one
//! branch to itself.

#![cfg(all(feature = "machine-stm32wb55", feature = "machine-stm32wb35"))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

const RCC_AHB3ENR: u64 = 0x5800_0050;
const PWR_CR4: u64 = 0x5800_040C;
const IPCC: u64 = 0x5800_0C00;
const C1CR: u64 = IPCC;
const C1MR: u64 = IPCC + 0x04;
const C1SCR: u64 = IPCC + 0x08;
const C1TOC2SR: u64 = IPCC + 0x0C;
const C2TOC1SR: u64 = IPCC + 0x1C;

/// The mailbox, laid out as `tests/fixtures/stm32wb-ble/main.c` does.
const REF_TABLE: u64 = 0x2003_0000;
const DIT: u64 = 0x2003_0100;
const BLE_TABLE: u64 = 0x2003_0140;
const SYS_TABLE: u64 = 0x2003_0150;
const MM_TABLE: u64 = 0x2003_0160;
const EVT_QUEUE: u64 = 0x2003_0180;
const SYS_QUEUE: u64 = 0x2003_0188;
const FREE_QUEUE: u64 = 0x2003_0190;
const BLE_CMD: u64 = 0x2003_0200;
const BLE_CS: u64 = 0x2003_0310;
const ACL: u64 = 0x2003_0320;
const SYS_CMD: u64 = 0x2003_0430;
const SPARE_BLE: u64 = 0x2003_0540;
const SPARE_SYS: u64 = 0x2003_0650;
const POOL: u64 = 0x2003_0800;
const POOL_SIZE: u64 = 8 * 268;

/// SP = the top of the WB35's SRAM1, PC = 0x08000008 | 1, then `b .`.
const IDLE: [u8; 10] = [0x00, 0x80, 0x00, 0x20, 0x09, 0x00, 0x00, 0x08, 0xfe, 0xe7];

fn boot(board: &str) -> Machine {
    let entry = catalog::machine(board).expect("this build ships the board");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", IDLE.to_vec());
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{board} does not realize: {e}"))
}

fn rd(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("read {addr:#x}: {e:?}"))
}

fn try_rd(m: &Machine, addr: u64) -> Option<u64> {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .ok()
}

fn wr(m: &Machine, addr: u64, value: u64) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U32, value, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("write {addr:#x}: {e:?}"));
}

fn wr8(m: &Machine, addr: u64, bytes: &[u8]) {
    let space = m.space("mem").expect("the memory space");
    for (i, b) in bytes.iter().enumerate() {
        space
            .write(addr + i as u64, Width::U8, u64::from(*b), MemAttrs::DEFAULT)
            .unwrap_or_else(|e| panic!("write {:#x}: {e:?}", addr + i as u64));
    }
}

fn rd8(m: &Machine, addr: u64, n: usize) -> Vec<u8> {
    let space = m.space("mem").expect("the memory space");
    (0..n)
        .map(|i| {
            space
                .read(addr + i as u64, Width::U8, MemAttrs::DEFAULT)
                .expect("SRAM2") as u8
        })
        .collect()
}

fn run_ms(m: &mut Machine, ms: u64) {
    m.run_for(GlobalTime::from_nanos(ms * 1_000_000))
        .expect("it runs");
}

fn empty_queue(m: &Machine, head: u64) {
    wr(m, head, head);
    wr(m, head + 4, head);
}

/// Pop the first node of a queue, or `None` when it is empty.
fn pop(m: &Machine, head: u64) -> Option<u64> {
    let first = rd(m, head);
    if first == head {
        return None;
    }
    let next = rd(m, first);
    wr(m, head, next);
    wr(m, next + 4, head);
    Some(first)
}

/// Lay the mailbox out and hand CPU2 its reference table.
fn mailbox(m: &Machine) {
    for q in [EVT_QUEUE, SYS_QUEUE, FREE_QUEUE] {
        empty_queue(m, q);
    }
    for (i, w) in [BLE_CMD, BLE_CS, EVT_QUEUE, ACL].iter().enumerate() {
        wr(m, BLE_TABLE + 4 * i as u64, *w);
    }
    wr(m, SYS_TABLE, SYS_CMD);
    wr(m, SYS_TABLE + 4, SYS_QUEUE);
    for (i, w) in [SPARE_BLE, SPARE_SYS, POOL, POOL_SIZE, FREE_QUEUE, 0, 0]
        .iter()
        .enumerate()
    {
        wr(m, MM_TABLE + 4 * i as u64, *w);
    }
    for i in 0..10 {
        wr(m, REF_TABLE + 4 * i, 0);
    }
    wr(m, REF_TABLE, DIT);
    wr(m, REF_TABLE + 4, BLE_TABLE);
    wr(m, REF_TABLE + 12, SYS_TABLE);
    wr(m, REF_TABLE + 16, MM_TABLE);
}

fn bring_up(board: &str) -> Machine {
    let mut m = boot(board);
    // IPCC's clock (AHB3ENR.IPCCEN, bit 20), keeping the reset-on HSEM/FLASH bits.
    wr(&m, RCC_AHB3ENR, rd(&m, RCC_AHB3ENR) | 1 << 20);
    mailbox(&m);
    // RXOIE and TXFIE; receive channels 1 and 2 unmasked, every TX-free masked.
    wr(&m, C1CR, 1 | 1 << 16);
    wr(&m, C1MR, 0x003F_003F & !0b11);
    wr(&m, PWR_CR4, rd(&m, PWR_CR4) | 1 << 15);
    // CPU2's boot latency is 32000 ticks of HCLK2, 8 ms at the reset 4 MHz.
    run_ms(&mut m, 20);
    m
}

#[test]
fn c2boot_brings_cpu2_up_and_its_ready_event_arrives_on_channel_2() {
    for board in ["stm32wb55", "stm32wb35"] {
        let m = bring_up(board);
        assert_ne!(rd(&m, C2TOC1SR) & 0b10, 0, "{board}: channel 2 occupied");
        let node = pop(&m, SYS_QUEUE).expect("an event on the system queue");
        // type SYSEVT, evtcode 0xFF, plen 3, subevt 0x9200, 0 = wireless stack.
        assert_eq!(
            rd8(&m, node + 8, 6),
            [0x12, 0xFF, 3, 0x00, 0x92, 0x00],
            "{board}"
        );
        assert!(pop(&m, SYS_QUEUE).is_none());
        // The wireless-stack device information table was filled in.
        assert_ne!(rd(&m, DIT + 16), 0, "{board}: WirelessStackVersion");
    }
}

#[test]
fn the_ble_stack_answers_through_the_mailbox() {
    let mut m = bring_up("stm32wb55");
    while let Some(_node) = pop(&m, SYS_QUEUE) {}
    wr(&m, C1SCR, 0b10);

    // SHCI_C2_BLE_Init with an all-zero parameter block ("not enforced").
    let mut cmd = vec![0x10, 0x66, 0xFC, 32];
    cmd.extend_from_slice(&[0u8; 32]);
    wr8(&m, SYS_CMD + 8, &cmd);
    wr(&m, C1SCR, 0b10 << 16);
    run_ms(&mut m, 5);
    assert_eq!(rd(&m, C1TOC2SR) & 0b10, 0, "CPU2 freed channel 2");
    // SYSRSP, Command Complete, plen 4, NumHCI 0xFF, opcode, status 0.
    assert_eq!(
        rd8(&m, SYS_CMD + 8, 7),
        [0x11, 0x0E, 4, 0xFF, 0x66, 0xFC, 0x00]
    );

    // HCI_Reset on channel 1.
    wr8(&m, BLE_CMD + 8, &[0x01, 0x03, 0x0C, 0]);
    wr(&m, C1SCR, 0b01 << 16);
    run_ms(&mut m, 5);
    assert_ne!(rd(&m, C2TOC1SR) & 0b01, 0, "channel 1 occupied");
    let node = pop(&m, EVT_QUEUE).expect("the reply on the event queue");
    // BLEEVT, Command Complete, plen 4, Num_HCI 1, 0x0C03, status 0.
    assert_eq!(rd8(&m, node + 8, 7), [0x04, 0x0E, 4, 1, 0x03, 0x0C, 0x00]);
}

#[test]
fn ipcc_is_gated_by_rcc() {
    let m = boot("stm32wb55");
    wr(&m, C1CR, 1);
    assert_eq!(rd(&m, C1CR), 0, "IPCCEN is clear out of reset");
    wr(&m, RCC_AHB3ENR, rd(&m, RCC_AHB3ENR) | 1 << 20);
    wr(&m, C1CR, 1);
    assert_eq!(rd(&m, C1CR), 1);
}

#[test]
fn the_secure_flash_is_cpu2s_and_the_parts_differ() {
    let m = boot("stm32wb55");
    // FLASH_SIZE in KiB, then the default secure start: page 0xD0 of 1M.
    let size = |m: &Machine| {
        m.space("mem")
            .unwrap()
            .read(0x1FFF_75E0, Width::U16, MemAttrs::DEFAULT)
            .unwrap()
    };
    assert_eq!(size(&m), 1024);
    assert_eq!(rd(&m, 0x5800_4080) & 0xFF, 0xD0, "FLASH_SFR.SFSA");
    assert_eq!(rd(&m, 0x080D_0000), 0, "secure flash reads zero for CPU1");
    assert!(try_rd(&m, 0x4800_0C00).is_some(), "the WB55 has GPIOD");

    let m = boot("stm32wb35");
    assert_eq!(size(&m), 512);
    assert_eq!(rd(&m, 0x5800_4080) & 0xFF, 0x5C, "FLASH_SFR.SFSA");
    assert!(try_rd(&m, 0x4800_0C00).is_none(), "the WB35 has no GPIOD");
    assert!(
        try_rd(&m, 0x2000_8000).is_none(),
        "the WB35's SRAM1 ends at 32 KiB"
    );
}
