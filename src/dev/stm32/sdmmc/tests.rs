//! What the register block must do, what a debugger must not disturb, and what
//! a snapshot must carry.

use super::*;

use alloc::string::ToString;
use core::sync::atomic::{AtomicU32, Ordering as AtomicOrdering};

use crate::core::space::{Region, RegionKind, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
use crate::dev::sd::card::{BLOCK, Identity, IdentityText, Phase};
use crate::dev::sd::{BusMode, SdCard};

/// Where guest RAM sits in the test board's address space.
const RAM_BASE: u64 = 0x2000_0000;
/// How much of it there is.
const RAM_BYTES: u64 = 64 * 1024;

/// A controller, a card in its socket, and an address space with RAM in it.
struct Rig {
    dev: Sdmmc,
    card: Arc<SdCard>,
    space: Arc<AddressSpace>,
}

fn card_of(capacity: u64, high_capacity: bool) -> Arc<SdCard> {
    let id = Identity::new(
        capacity,
        high_capacity,
        false,
        IdentityText {
            manufacturer: 0x03,
            oem: "RE",
            product: "RSEMU",
            revision: 0x10,
            serial: 0x1234_5678,
            year: 2024,
            month: 1,
        },
    )
    .expect("a plausible card");
    Arc::new(SdCard::with_identity(id, BusMode::Sd, 1).expect("it fits"))
}

fn rig() -> Rig {
    rig_with(Some(card_of(8 * 1024 * 1024, true)))
}

fn rig_with(card: Option<Arc<SdCard>>) -> Rig {
    let slot = Arc::new(Slot::new());
    if let Some(card) = card.as_ref() {
        slot.insert(Arc::clone(card)).expect("an empty socket");
    }
    let dev = Sdmmc::with_slot(Arc::clone(&slot), "sd0".to_string());
    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT));
    let ram = Arc::new(crate::core::space::RamStore::new(RAM_BYTES));
    space
        .topology()
        .map(Arc::new(Region::ram("ram", ram)), RAM_BASE)
        .expect("ram maps");
    dev.attach_bus(Arc::clone(&space), RequesterId::ANONYMOUS);
    Rig {
        dev,
        card: card.unwrap_or_else(|| card_of(64 * 1024, false)),
        space,
    }
}

fn ops(dev: &Sdmmc) -> Arc<dyn MemOps> {
    match dev.region("").expect("a register block").kind() {
        RegionKind::Io(o) => Arc::clone(o),
        other => panic!("expected an io region, got {other:?}"),
    }
}

fn poke(dev: &Sdmmc, offset: u64, value: u32) {
    ops(dev)
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a word write is a legal bus cycle");
}

fn peek(dev: &Sdmmc, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    ops(dev)
        .read(offset, &mut buf, MemAttrs::DEFAULT)
        .expect("a word read is a legal bus cycle");
    u32::from_le_bytes(buf)
}

fn peek_debug(dev: &Sdmmc, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    ops(dev)
        .read(offset, &mut buf, MemAttrs::DEBUG)
        .expect("a debugger may read");
    u32::from_le_bytes(buf)
}

/// Send one command through the register block, as a driver does.
fn command(dev: &Sdmmc, index: u32, arg: u32, waitresp: u32) -> u32 {
    poke(dev, R_ICR, ICR_MASK);
    poke(dev, R_ARGR, arg);
    poke(
        dev,
        R_CMDR,
        index | (waitresp << CMD_WAITRESP_SHIFT) | CMD_CPSMEN,
    );
    peek(dev, R_STAR)
}

fn power_on(dev: &Sdmmc) {
    poke(dev, R_POWER, POWER_ON);
    // 400 kHz from a 200 MHz kernel clock, which is what a driver programs for
    // the identification phase.
    poke(dev, R_CLKCR, 250);
}

/// Walk the identification sequence entirely through the register block, and
/// return the address the card published.
///
/// This is the milestone the whole file exists for: no test helper touches the
/// card directly, and the sequence is the one ST's own initialisation performs.
fn bring_up(dev: &Sdmmc) -> u32 {
    power_on(dev);
    let sta = command(dev, 0, 0, WAITRESP_NONE);
    assert_ne!(sta & STA_CMDSENT, 0, "CMD0 has no response to wait for");

    let sta = command(dev, 8, 0x0000_01aa, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0, "CMD8 answered");
    assert_eq!(peek(dev, R_RESP1R), 0x1aa, "the check pattern came back");
    assert_eq!(peek(dev, R_RESPCMDR), 8);

    command(dev, 55, 0, WAITRESP_SHORT);
    // ACMD41's R3 carries no CRC, which is what WAITRESP = 10b is for.
    let sta = command(dev, 41, (1 << 30) | 0x00ff_8000, WAITRESP_SHORT_NOCRC);
    assert_ne!(sta & STA_CMDREND, 0);
    let ocr = peek(dev, R_RESP1R);
    assert_ne!(ocr & (1 << 31), 0, "the card finished powering up");
    assert_ne!(ocr & (1 << 30), 0, "and it is high capacity");
    assert_eq!(peek(dev, R_RESPCMDR), 0x3f, "R3 carries no command index");

    let sta = command(dev, 2, 0, WAITRESP_LONG);
    assert_ne!(sta & STA_CMDREND, 0, "the CID arrived");
    assert_eq!(peek(dev, R_RESPCMDR), 0x3f, "nor does R2");

    let sta = command(dev, 3, 0, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0);
    let rca = peek(dev, R_RESP1R) >> 16;

    command(dev, 7, rca << 16, WAITRESP_SHORT);
    // Four-bit bus, and a 512-byte block length.
    command(dev, 55, rca << 16, WAITRESP_SHORT);
    command(dev, 6, 0b10, WAITRESP_SHORT);
    command(dev, 16, BLOCK as u32, WAITRESP_SHORT);
    poke(dev, R_CLKCR, 4);
    rca
}

/// Arm a data transfer: `len` bytes, in 512-byte blocks, in `dir`.
fn arm_data(dev: &Sdmmc, len: u32, to_host: bool) {
    poke(dev, R_DTIMER, 0x00ff_ffff);
    poke(dev, R_DLENR, len);
    let mut dctrl = (9 << DCTRL_DBLOCKSIZE_SHIFT) | DCTRL_DTEN;
    if to_host {
        dctrl |= DCTRL_DTDIR;
    }
    poke(dev, R_DCTRL, dctrl);
}

fn ram_read(space: &AddressSpace, addr: u64, len: usize) -> alloc::vec::Vec<u8> {
    let mut out = alloc::vec![0u8; len];
    space
        .read_bytes(addr, &mut out, MemAttrs::DEBUG)
        .expect("mapped RAM");
    out
}

fn ram_write(space: &AddressSpace, addr: u64, bytes: &[u8]) {
    space
        .write_bytes(addr, bytes, MemAttrs::DEBUG)
        .expect("mapped RAM");
}

fn pattern(seed: u8) -> alloc::vec::Vec<u8> {
    (0..BLOCK as u32)
        .map(|i| (i as u8).wrapping_mul(3).wrapping_add(seed))
        .collect()
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

#[test]
fn the_registers_a_driver_programs_read_back_what_it_wrote() {
    let rig = rig();
    poke(&rig.dev, R_CLKCR, 0xffff_ffff);
    assert_eq!(
        peek(&rig.dev, R_CLKCR),
        CLKCR_MASK,
        "reserved bits stay zero"
    );
    assert_eq!(rig.dev.clock_divider(), CLKCR_CLKDIV);
    poke(&rig.dev, R_ARGR, 0xdead_beef);
    assert_eq!(peek(&rig.dev, R_ARGR), 0xdead_beef);
    poke(&rig.dev, R_DTIMER, 0x1234_5678);
    assert_eq!(peek(&rig.dev, R_DTIMER), 0x1234_5678);
    poke(&rig.dev, R_DLENR, 0xffff_ffff);
    assert_eq!(peek(&rig.dev, R_DLENR), DLEN_MASK, "DATALENGTH is 25 bits");
    poke(&rig.dev, R_MASKR, 0xffff_ffff);
    assert_eq!(peek(&rig.dev, R_MASKR), MASK_MASK);
    poke(&rig.dev, R_IDMABASE0R, 0x2000_0007);
    assert_eq!(
        peek(&rig.dev, R_IDMABASE0R),
        0x2000_0004,
        "an IDMA base is word aligned"
    );
    // Read-only registers swallow a write rather than faulting the bus.
    poke(&rig.dev, R_RESPCMDR, 0x3f);
    assert_eq!(peek(&rig.dev, R_RESPCMDR), 0);
    // And a reserved word answers zero, which is what this part does.
    assert_eq!(peek(&rig.dev, 0x44), 0);
    assert_eq!(peek(&rig.dev, 0x300), 0);
}

#[test]
fn an_unaligned_or_narrow_access_is_a_bus_fault() {
    let rig = rig();
    let mut byte = [0u8; 1];
    assert!(
        ops(&rig.dev)
            .read(R_STAR, &mut byte, MemAttrs::DEFAULT)
            .is_err()
    );
    let mut word = [0u8; 4];
    assert!(
        ops(&rig.dev)
            .read(R_STAR + 1, &mut word, MemAttrs::DEFAULT)
            .is_err()
    );
}

#[test]
fn a_command_with_the_power_off_times_out() {
    let rig = rig();
    let sta = command(&rig.dev, 0, 0, WAITRESP_SHORT);
    assert_ne!(sta & STA_CTIMEOUT, 0, "there is no clock on the bus");
    assert_eq!(rig.card.phase(), Phase::Idle, "and the card heard nothing");
}

#[test]
fn a_command_into_an_empty_socket_times_out() {
    let rig = rig_with(None);
    power_on(&rig.dev);
    let sta = command(&rig.dev, 8, 0x1aa, WAITRESP_SHORT);
    assert_ne!(sta & STA_CTIMEOUT, 0);
    assert_eq!(sta & STA_CMDREND, 0);
}

#[test]
fn cutting_the_power_resets_the_card() {
    // What this register is for: a driver that cannot get a card to answer
    // power-cycles it and starts the identification sequence again.
    let rig = rig();
    bring_up(&rig.dev);
    assert_eq!(rig.card.phase(), Phase::Transfer);
    poke(&rig.dev, R_POWER, 0);
    assert_eq!(rig.card.phase(), Phase::Idle);
    assert_eq!(rig.card.rca(), 0);
}

// ---------------------------------------------------------------------------
// The command state machine
// ---------------------------------------------------------------------------

#[test]
fn a_guest_walks_the_whole_identification_sequence_through_the_registers() {
    let rig = rig();
    let rca = bring_up(&rig.dev);
    assert_eq!(rca, 1);
    assert_eq!(rig.card.phase(), Phase::Transfer);
    assert_eq!(rig.card.bus_width(), 4, "ACMD6 went through");
}

#[test]
fn a_long_response_lands_in_all_four_response_registers() {
    let rig = rig();
    power_on(&rig.dev);
    command(&rig.dev, 0, 0, WAITRESP_NONE);
    command(&rig.dev, 55, 0, WAITRESP_SHORT);
    command(&rig.dev, 41, (1 << 30) | 0x00ff_8000, WAITRESP_SHORT_NOCRC);
    command(&rig.dev, 2, 0, WAITRESP_LONG);
    let mut bytes = [0u8; 16];
    for i in 0..4 {
        let word = peek(&rig.dev, R_RESP1R + (i as u64) * 4);
        bytes[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    assert_eq!(
        bytes,
        rig.card.identity().cid,
        "RESP1R is bits 127:96 and RESP4R is 31:0, CRC7 and end bit included"
    );
}

#[test]
fn asking_for_the_wrong_response_length_fails_the_way_the_silicon_does() {
    let rig = rig();
    let rca = bring_up(&rig.dev);
    // 48 bits arrive where 136 were expected: the CPSM waits and gives up.
    let sta = command(&rig.dev, 13, rca << 16, WAITRESP_LONG);
    assert_ne!(sta & STA_CTIMEOUT, 0);
    assert_eq!(sta & STA_CMDREND, 0);

    // And the other way: 136 bits where 48 were expected, so what the CPSM
    // sampled as a CRC is not one.
    command(&rig.dev, 7, 0, WAITRESP_SHORT); // deselect, so CMD9 is legal
    let sta = command(&rig.dev, 9, rca << 16, WAITRESP_SHORT);
    assert_ne!(sta & STA_CCRCFAIL, 0);
    assert_eq!(sta & STA_CMDREND, 0);
}

#[test]
fn a_command_with_no_expected_response_reports_cmdsent_whatever_the_card_said() {
    // WAITRESP = 00b does not wait, so a response the card *did* send is
    // simply not heard. A driver getting this wrong is a driver bug, and this
    // is what it looks like on real hardware.
    let rig = rig();
    let rca = bring_up(&rig.dev);
    let before = peek(&rig.dev, R_RESP1R);
    let sta = command(&rig.dev, 13, rca << 16, WAITRESP_NONE);
    assert_ne!(sta & STA_CMDSENT, 0);
    assert_eq!(sta & STA_CMDREND, 0);
    assert_eq!(
        peek(&rig.dev, R_RESP1R),
        before,
        "the previous command's response is still there, unlatched over"
    );
}

// ---------------------------------------------------------------------------
// The FIFO path
// ---------------------------------------------------------------------------

#[test]
fn a_block_reaches_the_guest_through_a_sixteen_word_fifo() {
    let rig = rig();
    let want = pattern(0x11);
    rig.card.write_media(4 * BLOCK, &want).expect("inside");
    let rca = bring_up(&rig.dev);
    let _ = rca;

    arm_data(&rig.dev, BLOCK as u32, true);
    let sta = command(&rig.dev, 17, 4, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0);

    // The FIFO is a real sixteen words deep, so it is full and half full and
    // not empty before the guest has read anything.
    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_RXFIFOF, 0, "sixteen words are waiting");
    assert_ne!(sta & STA_RXFIFOHF, 0);
    assert_eq!(sta & STA_RXFIFOE, 0);
    assert_eq!(
        peek(&rig.dev, R_DCNTR),
        BLOCK as u32 - (FIFO_WORDS as u32) * 4,
        "DCNTR counts what the card has not handed over yet"
    );

    let mut got = alloc::vec::Vec::new();
    for _ in 0..BLOCK / 4 {
        got.extend_from_slice(&peek(&rig.dev, R_FIFOR).to_le_bytes());
    }
    assert_eq!(got, want);
    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_DATAEND, 0);
    assert_ne!(sta & STA_DBCKEND, 0);
    assert_eq!(
        sta & STA_RXOVERR,
        0,
        "the card never ran ahead of the reader"
    );
    assert_eq!(sta & STA_DPSMACT, 0);
    assert_eq!(peek(&rig.dev, R_DCNTR), 0);
}

#[test]
fn a_block_written_through_the_fifo_reads_back_through_it() {
    // The test that proves the model rather than the plumbing: the bytes make
    // the whole round trip through the register block and the card's protocol,
    // and nothing in the middle is peeked at directly.
    let rig = rig();
    bring_up(&rig.dev);
    let want = pattern(0x77);

    arm_data(&rig.dev, BLOCK as u32, false);
    let sta = command(&rig.dev, 24, 7, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0);
    for word in want.chunks(4) {
        assert_ne!(
            peek(&rig.dev, R_STAR) & STA_TXFIFOHE,
            0,
            "there is always room: the card takes each word as it arrives"
        );
        poke(
            &rig.dev,
            R_FIFOR,
            u32::from_le_bytes([word[0], word[1], word[2], word[3]]),
        );
    }
    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_DATAEND, 0);
    assert_eq!(sta & STA_TXUNDERR, 0);

    poke(&rig.dev, R_ICR, ICR_MASK);
    arm_data(&rig.dev, BLOCK as u32, true);
    command(&rig.dev, 17, 7, WAITRESP_SHORT);
    let mut got = alloc::vec::Vec::new();
    for _ in 0..BLOCK / 4 {
        got.extend_from_slice(&peek(&rig.dev, R_FIFOR).to_le_bytes());
    }
    assert_eq!(got, want);
}

#[test]
fn a_multiple_block_read_walks_forward_and_cmd12_stops_it() {
    let rig = rig();
    for block in 0..3u64 {
        rig.card
            .write_media(block * BLOCK, &pattern(block as u8))
            .expect("inside");
    }
    bring_up(&rig.dev);
    arm_data(&rig.dev, 3 * BLOCK as u32, true);
    command(&rig.dev, 18, 0, WAITRESP_SHORT);
    for block in 0..3u8 {
        let mut got = alloc::vec::Vec::new();
        for _ in 0..BLOCK / 4 {
            got.extend_from_slice(&peek(&rig.dev, R_FIFOR).to_le_bytes());
        }
        assert_eq!(got, pattern(block), "block {block}");
    }
    assert_ne!(peek(&rig.dev, R_STAR) & STA_DATAEND, 0);
    // CMDSTOP is the H7's way of saying "this command ends the data path".
    poke(&rig.dev, R_ARGR, 0);
    poke(
        &rig.dev,
        R_CMDR,
        12 | (WAITRESP_SHORT << CMD_WAITRESP_SHIFT) | CMD_STOP | CMD_CPSMEN,
    );
    assert_eq!(rig.card.phase(), Phase::Transfer);
}

#[test]
fn cmdstop_aborts_a_transfer_that_is_still_running() {
    let rig = rig();
    bring_up(&rig.dev);
    arm_data(&rig.dev, 3 * BLOCK as u32, true);
    command(&rig.dev, 18, 0, WAITRESP_SHORT);
    assert_ne!(peek(&rig.dev, R_STAR) & STA_DPSMACT, 0);
    poke(&rig.dev, R_ICR, ICR_MASK);
    poke(&rig.dev, R_ARGR, 0);
    poke(
        &rig.dev,
        R_CMDR,
        12 | (WAITRESP_SHORT << CMD_WAITRESP_SHIFT) | CMD_STOP | CMD_CPSMEN,
    );
    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_DABORT, 0);
    assert_eq!(sta & STA_DPSMACT, 0);
    assert_ne!(sta & STA_RXFIFOE, 0, "and the FIFO went with it");
}

#[test]
fn a_data_path_armed_before_its_command_waits_rather_than_failing() {
    // The older sequence, which plenty of drivers use: set DTEN first, send
    // the read command second. On real silicon the DPSM waits on DAT until the
    // card starts talking, and this model must not mistake that for a timeout.
    let rig = rig();
    let want = pattern(0x81);
    rig.card.write_media(0, &want).expect("inside");
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true);
    let sta = peek(&rig.dev, R_STAR);
    assert_eq!(sta & STA_DTIMEOUT, 0, "nothing has gone wrong yet");
    assert_ne!(sta & STA_DPSMACT, 0, "the data path is armed and waiting");
    assert_eq!(peek(&rig.dev, R_DCNTR), BLOCK as u32);

    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    let mut got = alloc::vec::Vec::new();
    for _ in 0..BLOCK / 4 {
        got.extend_from_slice(&peek(&rig.dev, R_FIFOR).to_le_bytes());
    }
    assert_eq!(got, want);
}

#[test]
fn the_data_path_times_out_when_the_card_stops_talking_mid_transfer() {
    // A card that has started and then stops is a different thing: on the wire
    // that is silence past DTIMER, and the flag exists for it. Reading past the
    // end of a multiple-block transfer is the reachable way to provoke it.
    let rig = rig();
    let blocks = rig.card.identity().blocks() as u32;
    bring_up(&rig.dev);
    arm_data(&rig.dev, 2 * BLOCK as u32, true);
    command(&rig.dev, 18, blocks - 1, WAITRESP_SHORT);
    for _ in 0..BLOCK / 4 {
        let _ = peek(&rig.dev, R_FIFOR);
    }
    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_DTIMEOUT, 0, "the last block was the last block");
    assert_eq!(sta & STA_DATAEND, 0);
}

#[test]
fn a_short_transfer_lands_wholly_inside_the_fifo() {
    // ACMD51's eight bytes and CMD6's sixty-four both fit, so DATAEND is set
    // before the guest reads a word — which is what lets a driver wait for it
    // and then drain, the way ST's own SCR read does.
    let rig = rig();
    let rca = bring_up(&rig.dev);
    arm_data(&rig.dev, 8, true);
    command(&rig.dev, 55, rca << 16, WAITRESP_SHORT);
    command(&rig.dev, 51, 0, WAITRESP_SHORT);
    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_DATAEND, 0);
    let lo = peek(&rig.dev, R_FIFOR).to_le_bytes();
    let hi = peek(&rig.dev, R_FIFOR).to_le_bytes();
    let mut scr = [0u8; 8];
    scr[..4].copy_from_slice(&lo);
    scr[4..].copy_from_slice(&hi);
    assert_eq!(scr, rig.card.identity().scr);
    assert_ne!(peek(&rig.dev, R_STAR) & STA_RXFIFOE, 0);
}

// ---------------------------------------------------------------------------
// The internal DMA
// ---------------------------------------------------------------------------

#[test]
fn the_internal_dma_puts_a_block_in_guest_memory_by_itself() {
    let rig = rig();
    let want = pattern(0x5a);
    rig.card.write_media(2 * BLOCK, &want).expect("inside");
    bring_up(&rig.dev);

    poke(&rig.dev, R_IDMABASE0R, (RAM_BASE + 0x400) as u32);
    poke(&rig.dev, R_IDMACTRLR, IDMA_EN);
    arm_data(&rig.dev, BLOCK as u32, true);
    let sta = command(&rig.dev, 17, 2, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0);

    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_DATAEND, 0);
    assert_eq!(sta & STA_IDMATE, 0);
    assert_eq!(peek(&rig.dev, R_DCNTR), 0);
    assert_ne!(sta & STA_RXFIFOE, 0, "the FIFO is not on this path");
    assert_eq!(ram_read(&rig.space, RAM_BASE + 0x400, want.len()), want);
}

#[test]
fn the_internal_dma_writes_a_block_out_of_guest_memory_and_reads_it_back() {
    let rig = rig();
    let want = pattern(0xc3);
    ram_write(&rig.space, RAM_BASE + 0x800, &want);
    bring_up(&rig.dev);

    poke(&rig.dev, R_IDMABASE0R, (RAM_BASE + 0x800) as u32);
    poke(&rig.dev, R_IDMACTRLR, IDMA_EN);
    arm_data(&rig.dev, BLOCK as u32, false);
    let sta = command(&rig.dev, 24, 11, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0);
    assert_ne!(peek(&rig.dev, R_STAR) & STA_DATAEND, 0);

    poke(&rig.dev, R_ICR, ICR_MASK);
    poke(&rig.dev, R_IDMABASE0R, (RAM_BASE + 0xc00) as u32);
    arm_data(&rig.dev, BLOCK as u32, true);
    command(&rig.dev, 17, 11, WAITRESP_SHORT);
    assert_eq!(ram_read(&rig.space, RAM_BASE + 0xc00, want.len()), want);
}

#[test]
fn the_internal_dma_moves_several_blocks_at_once() {
    let rig = rig();
    for block in 0..4u64 {
        rig.card
            .write_media(block * BLOCK, &pattern(0x20 + block as u8))
            .expect("inside");
    }
    bring_up(&rig.dev);
    poke(&rig.dev, R_IDMABASE0R, RAM_BASE as u32);
    poke(&rig.dev, R_IDMACTRLR, IDMA_EN);
    arm_data(&rig.dev, 4 * BLOCK as u32, true);
    command(&rig.dev, 23, 4, WAITRESP_SHORT);
    command(&rig.dev, 18, 0, WAITRESP_SHORT);
    assert_ne!(peek(&rig.dev, R_STAR) & STA_DATAEND, 0);
    let got = ram_read(&rig.space, RAM_BASE, 4 * BLOCK as usize);
    for block in 0..4usize {
        assert_eq!(
            got[block * 512..(block + 1) * 512],
            pattern(0x20 + block as u8)[..],
            "block {block}"
        );
    }
    assert_eq!(
        rig.card.phase(),
        Phase::Transfer,
        "the CMD23 count ended the transfer without a CMD12"
    );
}

#[test]
fn double_buffer_mode_alternates_and_reports_each_buffer() {
    let rig = rig();
    for block in 0..2u64 {
        rig.card
            .write_media(block * BLOCK, &pattern(0x40 + block as u8))
            .expect("inside");
    }
    bring_up(&rig.dev);
    poke(&rig.dev, R_IDMABASE0R, RAM_BASE as u32);
    poke(&rig.dev, R_IDMABASE1R, (RAM_BASE + 0x1000) as u32);
    // IDMABNDT counts units of eight double words, so 512 bytes is sixteen.
    poke(&rig.dev, R_IDMABSIZER, 16 << IDMABSIZE_SHIFT);
    poke(&rig.dev, R_IDMACTRLR, IDMA_EN | IDMA_BMODE);
    arm_data(&rig.dev, 2 * BLOCK as u32, true);
    command(&rig.dev, 18, 0, WAITRESP_SHORT);
    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_IDMABTC, 0, "a buffer completed");
    assert_ne!(sta & STA_DATAEND, 0);
    assert_eq!(ram_read(&rig.space, RAM_BASE, 512), pattern(0x40));
    assert_eq!(ram_read(&rig.space, RAM_BASE + 0x1000, 512), pattern(0x41));
}

#[test]
fn the_internal_dma_with_no_address_space_reports_a_transfer_error() {
    // A machine file that enables IDMAEN on a controller with no `space =`
    // has made a mistake, and the part already has a flag that says so.
    let slot = Arc::new(Slot::new());
    slot.insert(card_of(8 * 1024 * 1024, true)).expect("empty");
    let dev = Sdmmc::with_slot(slot, "sd0".to_string());
    bring_up(&dev);
    poke(&dev, R_IDMABASE0R, 0x2000_0000);
    poke(&dev, R_IDMACTRLR, IDMA_EN);
    arm_data(&dev, BLOCK as u32, true);
    // Arming is enough: there is nowhere for the bytes to go, and the data
    // path gives up before it has asked the card for any.
    assert_ne!(peek(&dev, R_STAR) & STA_IDMATE, 0);
    assert_eq!(peek(&dev, R_STAR) & STA_DPSMACT, 0);
}

#[test]
fn the_internal_dma_reports_an_address_space_that_refuses() {
    let rig = rig();
    bring_up(&rig.dev);
    // Above the RAM, where the space's unassigned policy is to fault.
    poke(&rig.dev, R_IDMABASE0R, 0x4000_0000);
    poke(&rig.dev, R_IDMACTRLR, IDMA_EN);
    arm_data(&rig.dev, BLOCK as u32, true);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    let sta = peek(&rig.dev, R_STAR);
    assert_ne!(sta & STA_IDMATE, 0);
    assert_eq!(sta & STA_DATAEND, 0);
}

// ---------------------------------------------------------------------------
// The interrupt
// ---------------------------------------------------------------------------

#[derive(Debug, Default)]
struct Counter {
    level: AtomicU32,
    edges: AtomicU32,
}

impl WireSink for Counter {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.level
            .store(u32::from(level.as_bool()), AtomicOrdering::SeqCst);
        self.edges.fetch_add(1, AtomicOrdering::SeqCst);
    }
}

#[test]
fn the_irq_follows_sta_and_mask_and_a_write_to_icr_drops_it() {
    let rig = rig();
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Counter::default());
    let pin: Arc<dyn WireSink> = Arc::clone(&sink) as Arc<dyn WireSink>;
    let wire = Arc::new(Wire::builder().source(id).sink(pin, 0).build());
    rig.dev
        .connect(pin::IRQ, WireSource::new(Arc::clone(&wire), id))
        .expect("the only pin this device has");

    bring_up(&rig.dev);
    poke(&rig.dev, R_ICR, ICR_MASK);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        0,
        "nothing enabled"
    );

    // Enable DATAEND only, then read a block by DMA. The transfer completes
    // inside the write that starts it, so the line is already high when the
    // guest's store retires — which is the whole point of an interrupt-driven
    // driver working under a zero-time model.
    poke(&rig.dev, R_MASKR, STA_DATAEND);
    poke(&rig.dev, R_IDMABASE0R, RAM_BASE as u32);
    poke(&rig.dev, R_IDMACTRLR, IDMA_EN);
    arm_data(&rig.dev, BLOCK as u32, true);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        1,
        "DATAEND is asserted"
    );

    poke(&rig.dev, R_ICR, STA_DATAEND);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        0,
        "and acknowledged"
    );

    // A FIFO *level* is an interrupt source too, and it is the one an
    // interrupt-driven FIFO read uses. It is not clearable — a level is not an
    // event — so the only way down is to drain the FIFO.
    poke(&rig.dev, R_ICR, ICR_MASK);
    poke(&rig.dev, R_IDMACTRLR, 0);
    poke(&rig.dev, R_MASKR, STA_RXFIFOHF);
    arm_data(&rig.dev, BLOCK as u32, true);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        1,
        "the FIFO filled and RXFIFOHFIE is enabled"
    );
    poke(&rig.dev, R_ICR, ICR_MASK);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        1,
        "and ICR cannot clear a level"
    );
    for _ in 0..BLOCK / 4 {
        let _ = peek(&rig.dev, R_FIFOR);
    }
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        0,
        "draining it is what drops the line"
    );

    // A flag that is set but not enabled does not drive the line.
    poke(&rig.dev, R_MASKR, 0);
    poke(&rig.dev, R_ICR, ICR_MASK);
    command(
        &rig.dev,
        13,
        u32::from(rig.card.rca()) << 16,
        WAITRESP_SHORT,
    );
    assert_ne!(peek(&rig.dev, R_STAR) & STA_CMDREND, 0);
    assert_eq!(sink.level.load(AtomicOrdering::SeqCst), 0);
}

#[test]
fn connecting_a_pin_this_device_does_not_have_is_an_error() {
    let rig = rig();
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Arc::new(Wire::builder().source(id).build());
    assert!(rig.dev.connect("dat0", WireSource::new(wire, id)).is_err());
}

// ---------------------------------------------------------------------------
// The debug contract
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_pops_nothing_and_clears_nothing() {
    // The trap this device is most exposed to: a monitor showing the register
    // block would otherwise eat the guest's data one word at a time.
    let rig = rig();
    let want = pattern(0x99);
    rig.card.write_media(0, &want).expect("inside");
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);

    let sta = peek_debug(&rig.dev, R_STAR);
    let dcnt = peek_debug(&rig.dev, R_DCNTR);
    let head = peek_debug(&rig.dev, R_FIFOR);
    for _ in 0..8 {
        assert_eq!(
            peek_debug(&rig.dev, R_FIFOR),
            head,
            "the same word, forever"
        );
    }
    assert_eq!(peek_debug(&rig.dev, R_STAR), sta, "and no flag moved");
    assert_eq!(peek_debug(&rig.dev, R_DCNTR), dcnt);

    // The guest's own read is unaffected: it still gets the whole block, first
    // word first.
    let mut got = alloc::vec::Vec::new();
    for _ in 0..BLOCK / 4 {
        got.extend_from_slice(&peek(&rig.dev, R_FIFOR).to_le_bytes());
    }
    assert_eq!(got, want);
}

#[test]
fn a_debug_write_is_refused_rather_than_obeyed() {
    // There is no harmless version: a write here sends a command, moves a
    // block or clears a status bit.
    let rig = rig();
    assert!(
        ops(&rig.dev)
            .write(R_CMDR, &0u32.to_le_bytes(), MemAttrs::DEBUG)
            .is_err()
    );
    assert!(
        ops(&rig.dev)
            .write(R_POWER, &POWER_ON.to_le_bytes(), MemAttrs::DEBUG)
            .is_err()
    );
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

fn snapshot(dev: &Sdmmc) -> alloc::vec::Vec<u8> {
    let mut shape = MachineShape::new();
    shape
        .add_device("sdmmc", CLASS.name)
        .expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("sdmmc", CLASS.name, CLASS.version)
            .expect("one chunk");
        dev.save(&mut chunk).expect("the controller saves");
    }
    w.to_vec().expect("a snapshot")
}

fn restore(dev: &Sdmmc, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let chunk = reader
        .load("sdmmc", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    dev.load(&mut chunk.reader()).expect("the controller loads");
}

#[test]
fn a_snapshot_carries_a_transfer_in_flight_and_the_words_in_the_fifo() {
    let rig = rig();
    let want = pattern(0x3c);
    rig.card.write_media(0, &want).expect("inside");
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    // Drain a few words, so the FIFO is neither full nor empty and the DPSM is
    // part way through its block.
    for _ in 0..5 {
        let _ = peek(&rig.dev, R_FIFOR);
    }

    let bytes = snapshot(&rig.dev);
    let other = super::tests::rig();
    restore(&other.dev, &bytes);
    assert_eq!(snapshot(&other.dev), bytes, "identical state");
    assert_eq!(peek(&other.dev, R_DCNTR), peek(&rig.dev, R_DCNTR));
    assert_eq!(
        peek(&other.dev, R_STAR) & STA_LATCHED,
        peek(&rig.dev, R_STAR) & STA_LATCHED
    );
    // The restored controller hands back the same next word, which is the one
    // the saved one had not popped yet.
    assert_eq!(peek(&other.dev, R_FIFOR), peek(&rig.dev, R_FIFOR));
}

#[test]
fn a_snapshot_with_an_impossible_fifo_is_refused() {
    let rig = rig();
    let mut bytes = snapshot(&rig.dev);
    // The FIFO length is the first `u64` after the seventeen register words.
    let at = bytes.len() - 8 - 1;
    bytes[at] = 0xff;
    let reader = StateReader::new(&bytes).expect("a snapshot");
    let chunk = reader
        .load("sdmmc", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    assert!(rig.dev.load(&mut chunk.reader()).is_err());
}

#[test]
fn a_reset_clears_the_register_block_and_leaves_the_card_alone() {
    // `Machine::reset` reaches every device once, so the controller must not
    // reach across and reset the card a second time.
    let rig = rig();
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    rig.dev.reset(ResetKind::Cold);
    assert_eq!(peek(&rig.dev, R_POWER), 0);
    assert_eq!(peek(&rig.dev, R_STAR) & STA_LATCHED, 0);
    assert_eq!(peek(&rig.dev, R_DCNTR), 0);
    assert_ne!(peek(&rig.dev, R_STAR) & STA_RXFIFOE, 0);
    assert_eq!(
        rig.card.phase(),
        Phase::SendingData,
        "the card is its own device and resets itself"
    );
}
