//! What the v1 register block must do, where it differs from the H7's, what a
//! debugger must not disturb, what a snapshot must carry — and, the reason this
//! device exists, that a real DMA controller can move a block through it.

use super::*;

use alloc::string::ToString;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU32, Ordering as AtomicOrdering};

use crate::core::space::RegionKind;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
use crate::dev::sd::card::{BLOCK, Identity, IdentityText, Phase};
use crate::dev::sd::{BusMode, SdCard};

/// A controller with a card in its socket.
struct Rig {
    dev: Sdio,
    card: Arc<SdCard>,
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
    let dev = Sdio::with_slot(Arc::clone(&slot), "sd0".to_string());
    Rig {
        dev,
        card: card.unwrap_or_else(|| card_of(64 * 1024, false)),
    }
}

fn ops(dev: &Sdio) -> Arc<dyn MemOps> {
    match dev.region("").expect("a register block").kind() {
        RegionKind::Io(o) => Arc::clone(o),
        other => panic!("expected an io region, got {other:?}"),
    }
}

fn poke(dev: &Sdio, offset: u64, value: u32) {
    ops(dev)
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a word write is a legal bus cycle");
}

fn peek(dev: &Sdio, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    ops(dev)
        .read(offset, &mut buf, MemAttrs::DEFAULT)
        .expect("a word read is a legal bus cycle");
    u32::from_le_bytes(buf)
}

fn peek_debug(dev: &Sdio, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    ops(dev)
        .read(offset, &mut buf, MemAttrs::DEBUG)
        .expect("a debugger may read");
    u32::from_le_bytes(buf)
}

/// Send one command through the register block, as a driver does.
fn command(dev: &Sdio, index: u32, arg: u32, waitresp: u32) -> u32 {
    poke(dev, R_ICR, ICR_MASK);
    poke(dev, R_ARG, arg);
    poke(
        dev,
        R_CMD,
        index | (waitresp << CMD_WAITRESP_SHIFT) | CMD_CPSMEN,
    );
    peek(dev, R_STA)
}

fn power_on(dev: &Sdio) {
    poke(dev, R_POWER, POWER_ON);
    // 400 kHz from a 48 MHz SDIOCLK is a divider of 118, which is what a driver
    // programs for the identification phase.
    poke(dev, R_CLKCR, 118);
}

/// Walk the identification sequence entirely through the register block, and
/// return the address the card published.
///
/// Every command here goes out with the **v1** `WAITRESP` encoding, which is
/// the point: `ACMD41` is `01b` and comes back reporting `CCRCFAIL`, where the
/// same driver on an H7 would write `10b` and read `CMDREND`.
fn bring_up(dev: &Sdio) -> u32 {
    power_on(dev);
    let sta = command(dev, 0, 0, WAITRESP_NONE);
    assert_ne!(sta & STA_CMDSENT, 0, "CMD0 has no response to wait for");

    let sta = command(dev, 8, 0x0000_01aa, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0, "CMD8 answered");
    assert_eq!(peek(dev, R_RESP1), 0x1aa, "the check pattern came back");
    assert_eq!(peek(dev, R_RESPCMD), 8);

    command(dev, 55, 0, WAITRESP_SHORT);
    let sta = command(dev, 41, (1 << 30) | 0x00ff_8000, WAITRESP_SHORT);
    assert_ne!(
        sta & STA_CCRCFAIL,
        0,
        "R3 carries no CRC and this block has no encoding that says so"
    );
    assert_eq!(sta & STA_CMDREND, 0, "so CMDREND is *not* what arrives");
    let ocr = peek(dev, R_RESP1);
    assert_ne!(ocr & (1 << 31), 0, "the card finished powering up");
    assert_ne!(ocr & (1 << 30), 0, "and it is high capacity");
    assert_eq!(peek(dev, R_RESPCMD), 0x3f, "R3 carries no command index");
    // The driver clears it and carries on, which is the whole of the handling.
    poke(dev, R_ICR, STA_CCRCFAIL);

    let sta = command(dev, 2, 0, WAITRESP_LONG);
    assert_ne!(
        sta & STA_CMDREND,
        0,
        "the CID arrived, and R2's CRC7 is real"
    );
    assert_eq!(peek(dev, R_RESPCMD), 0x3f, "nor does R2 carry an index");

    let sta = command(dev, 3, 0, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0);
    let rca = peek(dev, R_RESP1) >> 16;

    command(dev, 7, rca << 16, WAITRESP_SHORT);
    // Four-bit bus, and a 512-byte block length.
    command(dev, 55, rca << 16, WAITRESP_SHORT);
    command(dev, 6, 0b10, WAITRESP_SHORT);
    command(dev, 16, BLOCK as u32, WAITRESP_SHORT);
    poke(dev, R_CLKCR, (0b01 << CLKCR_WIDBUS_SHIFT) | 1);
    rca
}

/// Arm a data transfer: `len` bytes, in 512-byte blocks, in `dir`.
fn arm_data(dev: &Sdio, len: u32, to_host: bool, extra: u32) {
    poke(dev, R_DTIMER, 0x00ff_ffff);
    poke(dev, R_DLEN, len);
    let mut dctrl = (9 << DCTRL_DBLOCKSIZE_SHIFT) | DCTRL_DTEN | extra;
    if to_host {
        dctrl |= DCTRL_DTDIR;
    }
    poke(dev, R_DCTRL, dctrl);
}

fn pattern(seed: u8) -> Vec<u8> {
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
    assert_eq!(peek(&rig.dev, R_CLKCR), CLKCR_MASK, "fifteen bits of CLKCR");
    poke(&rig.dev, R_POWER, 0xffff_ffff);
    assert_eq!(peek(&rig.dev, R_POWER), POWER_PWRCTRL, "two bits of POWER");
    poke(&rig.dev, R_ARG, 0xdead_beef);
    assert_eq!(peek(&rig.dev, R_ARG), 0xdead_beef);
    poke(&rig.dev, R_DTIMER, 0xdead_beef);
    assert_eq!(peek(&rig.dev, R_DTIMER), 0xdead_beef);
    poke(&rig.dev, R_DLEN, 0xffff_ffff);
    assert_eq!(peek(&rig.dev, R_DLEN), DLEN_MASK, "DLEN is 25 bits");
    poke(&rig.dev, R_MASK, 0xffff_ffff);
    assert_eq!(
        peek(&rig.dev, R_MASK),
        0x00ff_ffff,
        "every STA bit is maskable on this block, unlike the H7's"
    );
    // DCTRL without DTEN arms nothing, so the whole field reads back.
    poke(&rig.dev, R_DCTRL, 0xffff_fffe);
    assert_eq!(peek(&rig.dev, R_DCTRL), DCTRL_MASK & !DCTRL_DTEN);
    // Read-only registers swallow a write rather than faulting.
    poke(&rig.dev, R_DCOUNT, 0x1234);
    assert_eq!(peek(&rig.dev, R_DCOUNT), 0);
    poke(&rig.dev, R_FIFOCNT, 0x1234);
    assert_eq!(peek(&rig.dev, R_FIFOCNT), 0);
    // And a reserved word is zero.
    assert_eq!(peek(&rig.dev, 0x40), 0);
    assert_eq!(peek(&rig.dev, 0x200), 0);
}

#[test]
fn an_unaligned_or_narrow_access_is_a_bus_fault() {
    let rig = rig();
    let mut byte = [0u8; 1];
    assert!(ops(&rig.dev).read(0, &mut byte, MemAttrs::DEFAULT).is_err());
    let mut word = [0u8; 4];
    assert!(ops(&rig.dev).read(2, &mut word, MemAttrs::DEFAULT).is_err());
    assert!(
        ops(&rig.dev)
            .write(1, &0u32.to_le_bytes(), MemAttrs::DEFAULT)
            .is_err()
    );
}

#[test]
fn an_empty_fifo_reads_empty_and_half_empty_but_not_half_full() {
    // The thresholds are at eight words on a thirty-two-word FIFO, so they are
    // not symmetrical about the midpoint and getting one of them from the other
    // by subtraction is wrong.
    let rig = rig();
    let sta = peek(&rig.dev, R_STA);
    assert_ne!(sta & STA_TXFIFOE, 0);
    assert_ne!(sta & STA_RXFIFOE, 0);
    assert_ne!(sta & STA_TXFIFOHE, 0);
    assert_eq!(sta & STA_RXFIFOHF, 0);
    assert_eq!(sta & (STA_TXDAVL | STA_RXDAVL), 0);
    assert_eq!(sta & (STA_TXFIFOF | STA_RXFIFOF), 0);
}

// ---------------------------------------------------------------------------
// The command path, and the family difference in it
// ---------------------------------------------------------------------------

#[test]
fn a_command_with_the_h7_bit_layout_is_the_wrong_answer_here() {
    // The incompatibility the whole file exists for, in the form it actually
    // takes: a driver written for the H7 sends ACMD41 with WAITRESP = 10b,
    // which there means "short response, no CRC check" and here means "no
    // response at all". Nothing reports an error; the OCR simply never arrives.
    let rig = rig();
    power_on(&rig.dev);
    command(&rig.dev, 0, 0, WAITRESP_NONE);
    command(&rig.dev, 8, 0x1aa, WAITRESP_SHORT);
    command(&rig.dev, 55, 0, WAITRESP_SHORT);
    let stale = peek(&rig.dev, R_RESP1);
    let sta = command(&rig.dev, 41, (1 << 30) | 0x00ff_8000, WAITRESP_NONE_ALT);
    assert_ne!(sta & STA_CMDSENT, 0, "10b is CMDSENT, not a response");
    assert_eq!(sta & (STA_CMDREND | STA_CCRCFAIL), 0);
    assert_eq!(
        peek(&rig.dev, R_RESP1),
        stale,
        "and RESP1 still holds CMD55's answer, because nothing latched the OCR"
    );
    assert_eq!(
        peek(&rig.dev, R_RESP1) & (1 << 31),
        0,
        "so the driver's `while (!(ocr & BUSY))` loop never terminates"
    );

    // And bit 6 alone — which on an H7 is CMDTRANS with WAITRESP still zero —
    // is a *short response* here.
    let fresh = super::tests::rig();
    power_on(&fresh.dev);
    poke(&fresh.dev, R_ARG, 0x1aa);
    poke(&fresh.dev, R_CMD, 8 | (1 << 6) | CMD_CPSMEN);
    assert_ne!(peek(&fresh.dev, R_STA) & STA_CMDREND, 0);
    assert_eq!(peek(&fresh.dev, R_RESP1), 0x1aa);
}

#[test]
fn the_card_enumerates_with_cmd0_cmd8_acmd41_cmd2_cmd3() {
    let rig = rig();
    let rca = bring_up(&rig.dev);
    assert_ne!(rca, 0, "the card published an address");
    assert_eq!(rig.card.phase(), Phase::Transfer);
    assert_eq!(rig.dev.bus_width(), 4, "WIDBUS = 01b");
}

#[test]
fn a_long_response_lands_in_all_four_response_registers() {
    let rig = rig();
    power_on(&rig.dev);
    command(&rig.dev, 0, 0, WAITRESP_NONE);
    command(&rig.dev, 55, 0, WAITRESP_SHORT);
    command(&rig.dev, 41, (1 << 30) | 0x00ff_8000, WAITRESP_SHORT);
    let sta = command(&rig.dev, 2, 0, WAITRESP_LONG);
    assert_ne!(
        sta & STA_CMDREND,
        0,
        "R2's CRC7 is a real one and it passes"
    );
    let mut bytes = [0u8; 16];
    for i in 0..4 {
        let word = peek(&rig.dev, R_RESP1 + (i as u64) * 4);
        bytes[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    assert_eq!(
        bytes,
        rig.card.identity().cid,
        "RESP1 is bits 127:96 and RESP4 is 31:0, CRC7 and end bit included"
    );
}

#[test]
fn asking_for_the_wrong_response_length_fails_the_way_the_silicon_does() {
    let rig = rig();
    bring_up(&rig.dev);
    let rca = u32::from(rig.card.rca()) << 16;
    // 48 bits where 136 were expected: the CPSM waits and gives up.
    let sta = command(&rig.dev, 13, rca, WAITRESP_LONG);
    assert_ne!(sta & STA_CTIMEOUT, 0);
    assert_eq!(sta & STA_CMDREND, 0);
    // 136 where 48 were expected: whatever it sampled as a CRC was not one.
    command(&rig.dev, 7, 0, WAITRESP_SHORT); // deselect, so CMD9 is legal
    let sta = command(&rig.dev, 9, rca, WAITRESP_SHORT);
    assert_ne!(sta & STA_CCRCFAIL, 0);
    assert_eq!(sta & STA_CMDREND, 0);
}

#[test]
fn a_command_to_an_absent_card_times_out_in_sixty_four_clocks() {
    let rig = rig_with(None);
    power_on(&rig.dev);
    let sta = command(&rig.dev, 8, 0x1aa, WAITRESP_SHORT);
    assert_ne!(sta & STA_CTIMEOUT, 0);
    assert_eq!(sta & STA_CMDREND, 0);
    // And with the supply off, which is the same silence on the wire.
    let unpowered = super::tests::rig();
    let sta = command(&unpowered.dev, 8, 0x1aa, WAITRESP_SHORT);
    assert_ne!(sta & STA_CTIMEOUT, 0);
}

#[test]
fn cutting_the_power_resets_the_card() {
    let rig = rig();
    bring_up(&rig.dev);
    assert_eq!(rig.card.phase(), Phase::Transfer);
    poke(&rig.dev, R_POWER, 0);
    assert_eq!(rig.card.phase(), Phase::Idle);
}

// ---------------------------------------------------------------------------
// The data path
// ---------------------------------------------------------------------------

#[test]
fn a_single_block_read_arrives_through_the_fifo_in_128_words() {
    let rig = rig();
    let want = pattern(0x5a);
    rig.card.write_media(0, &want).expect("inside the card");
    bring_up(&rig.dev);

    arm_data(&rig.dev, BLOCK as u32, true, 0);
    assert_eq!(
        peek(&rig.dev, R_DCOUNT),
        BLOCK as u32,
        "armed, and waiting on DAT for a command that has not gone out"
    );
    assert_eq!(peek(&rig.dev, R_STA) & STA_RXDAVL, 0, "nothing yet");

    let sta = command(&rig.dev, 17, 0, WAITRESP_SHORT);
    assert_ne!(sta & STA_CMDREND, 0);
    assert_ne!(sta & STA_RXACT, 0, "the DPSM is receiving");
    assert_ne!(sta & STA_RXDAVL, 0, "and the FIFO has words in it");
    assert_ne!(sta & STA_RXFIFOF, 0, "thirty-two of them");
    assert_eq!(
        peek(&rig.dev, R_FIFOCNT),
        (BLOCK / 4) as u32,
        "FIFOCNT counts what software still has to read, not what the card owes"
    );

    let mut got = Vec::new();
    let mut words = 0;
    while peek(&rig.dev, R_STA) & STA_RXDAVL != 0 {
        got.extend_from_slice(&peek(&rig.dev, R_FIFO).to_le_bytes());
        words += 1;
        assert!(words <= BLOCK / 4, "RXDAVL never dropped");
    }
    assert_eq!(words, BLOCK / 4, "128 words");
    assert_eq!(got, want);

    let sta = peek(&rig.dev, R_STA);
    assert_ne!(sta & STA_DBCKEND, 0);
    assert_ne!(sta & STA_DATAEND, 0);
    assert_eq!(sta & STA_RXACT, 0, "the DPSM went back to idle");
    assert_eq!(peek(&rig.dev, R_DCOUNT), 0);
    assert_eq!(peek(&rig.dev, R_FIFOCNT), 0);
    assert_eq!(
        peek(&rig.dev, R_DCTRL) & DCTRL_DTEN,
        0,
        "and DTEN went with it, so the next transfer can be armed"
    );
}

#[test]
fn reading_the_fifo_too_slowly_does_not_lose_data_and_raises_no_rxoverr() {
    // The other half of what the issue asked for, and the honest half. The card
    // only hands over a word when there is room for it, so the FIFO cannot
    // overrun and `RXOVERR` is unreachable — the same claim `stm32.sdmmc` makes
    // about the same flag, for the same reason, and it stops being true the day
    // this block gets a clock domain. What *is* testable is that a reader as
    // slow as it likes still gets every byte, in order.
    let rig = rig();
    let want = pattern(0x11);
    rig.card.write_media(0, &want).expect("inside the card");
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true, 0);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);

    let mut got = Vec::new();
    for _ in 0..BLOCK / 4 {
        // Poll a great many times between reads. Nothing advances, which is
        // exactly the property: reading STA is not what moves data.
        for _ in 0..16 {
            let sta = peek(&rig.dev, R_STA);
            assert_eq!(sta & STA_RXOVERR, 0, "the card never runs ahead");
        }
        got.extend_from_slice(&peek(&rig.dev, R_FIFO).to_le_bytes());
    }
    assert_eq!(got, want);
    assert_eq!(peek(&rig.dev, R_STA) & (STA_RXOVERR | STA_DCRCFAIL), 0);
}

#[test]
fn a_block_written_through_the_fifo_reads_back_through_it() {
    let rig = rig();
    bring_up(&rig.dev);
    let want = pattern(0x7e);

    // The write order this block takes: command first, then DCTRL.
    command(&rig.dev, 24, 0, WAITRESP_SHORT);
    arm_data(&rig.dev, BLOCK as u32, false, 0);
    assert_ne!(peek(&rig.dev, R_STA) & STA_TXACT, 0);
    for chunk in want.chunks(4) {
        poke(
            &rig.dev,
            R_FIFO,
            u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
        );
    }
    let sta = peek(&rig.dev, R_STA);
    assert_ne!(sta & STA_DATAEND, 0);
    assert_ne!(sta & STA_DBCKEND, 0);
    assert_eq!(sta & STA_TXUNDERR, 0, "the writer never outran the card");

    let mut back = alloc::vec![0u8; BLOCK as usize];
    rig.card.read_media(0, &mut back).expect("inside the card");
    assert_eq!(back, want);
}

#[test]
fn a_multiple_block_read_walks_forward_and_cmd12_stops_it() {
    let rig = rig();
    let first = pattern(0x20);
    let second = pattern(0x40);
    rig.card.write_media(0, &first).expect("inside");
    rig.card.write_media(BLOCK, &second).expect("inside");
    bring_up(&rig.dev);

    arm_data(&rig.dev, 2 * BLOCK as u32, true, 0);
    command(&rig.dev, 18, 0, WAITRESP_SHORT);
    let mut got = Vec::new();
    for _ in 0..2 * BLOCK / 4 {
        got.extend_from_slice(&peek(&rig.dev, R_FIFO).to_le_bytes());
    }
    assert_eq!(&got[..BLOCK as usize], &first[..]);
    assert_eq!(&got[BLOCK as usize..], &second[..]);
    assert_ne!(peek(&rig.dev, R_STA) & STA_DATAEND, 0);
    command(&rig.dev, 12, 0, WAITRESP_SHORT);
    assert_eq!(rig.card.phase(), Phase::Transfer);
}

#[test]
fn a_transfer_armed_with_no_dtimer_times_out_before_a_byte_moves() {
    // The register a driver forgets. The counter is loaded from DTIMER when the
    // DPSM starts waiting, so zero has expired already.
    let rig = rig();
    rig.card.write_media(0, &pattern(0x99)).expect("inside");
    bring_up(&rig.dev);
    poke(&rig.dev, R_DTIMER, 0);
    poke(&rig.dev, R_DLEN, BLOCK as u32);
    poke(
        &rig.dev,
        R_DCTRL,
        (9 << DCTRL_DBLOCKSIZE_SHIFT) | DCTRL_DTDIR | DCTRL_DTEN,
    );
    let sta = peek(&rig.dev, R_STA);
    assert_ne!(sta & STA_DTIMEOUT, 0);
    assert_eq!(sta & STA_RXACT, 0, "and nothing was armed");
    assert_eq!(peek(&rig.dev, R_DCOUNT), 0);
}

#[test]
fn the_data_path_times_out_when_the_card_stops_talking_mid_transfer() {
    let rig = rig();
    bring_up(&rig.dev);
    // Ask for two blocks with a single-block read command: the card ends after
    // the first, which is silence on DAT for the rest of DLEN.
    arm_data(&rig.dev, 2 * BLOCK as u32, true, 0);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    for _ in 0..BLOCK / 4 {
        let _ = peek(&rig.dev, R_FIFO);
    }
    let sta = peek(&rig.dev, R_STA);
    assert_ne!(sta & STA_DTIMEOUT, 0);
    assert_eq!(sta & STA_DATAEND, 0);
}

#[test]
fn a_stream_transfer_reports_no_block_boundaries() {
    // DTMODE = 1 is stream or multibyte: there are no blocks in it, so DBCKEND
    // has nothing to mark, while DATAEND still lands at DLEN.
    let rig = rig();
    rig.card.write_media(0, &pattern(0x31)).expect("inside");
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true, DCTRL_DTMODE);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    for _ in 0..BLOCK / 4 {
        let _ = peek(&rig.dev, R_FIFO);
    }
    let sta = peek(&rig.dev, R_STA);
    assert_ne!(sta & STA_DATAEND, 0);
    assert_eq!(sta & STA_DBCKEND, 0);
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

/// Wire `port` to a fresh counting sink.
fn watch(dev: &Sdio, port: &str) -> Arc<Counter> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Counter::default());
    let pin: Arc<dyn WireSink> = Arc::clone(&sink) as Arc<dyn WireSink>;
    let wire = Arc::new(Wire::builder().source(id).sink(pin, 0).build());
    dev.connect(port, WireSource::new(wire, id))
        .expect("a pin this device has");
    sink
}

#[test]
fn the_irq_follows_sta_and_mask_and_a_write_to_icr_drops_it() {
    let rig = rig();
    let sink = watch(&rig.dev, pin::IRQ);
    bring_up(&rig.dev);
    poke(&rig.dev, R_ICR, ICR_MASK);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        0,
        "nothing enabled"
    );

    poke(&rig.dev, R_MASK, STA_DATAEND);
    rig.card.write_media(0, &pattern(0x66)).expect("inside");
    arm_data(&rig.dev, BLOCK as u32, true, 0);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        0,
        "the guest has not drained the FIFO, so DATAEND has not landed"
    );
    for _ in 0..BLOCK / 4 {
        let _ = peek(&rig.dev, R_FIFO);
    }
    assert_eq!(sink.level.load(AtomicOrdering::SeqCst), 1, "and now it has");
    poke(&rig.dev, R_ICR, STA_DATAEND);
    assert_eq!(sink.level.load(AtomicOrdering::SeqCst), 0, "acknowledged");
}

#[test]
fn rxdavl_is_an_interrupt_source_on_this_block_and_icr_cannot_clear_it() {
    // Bit 21 is maskable here and does not exist at all on the H7, so this is
    // the family difference in the form a driver meets it: an interrupt-driven
    // FIFO read enables RXDAVLIE and nothing else.
    let rig = rig();
    let sink = watch(&rig.dev, pin::IRQ);
    rig.card.write_media(0, &pattern(0x08)).expect("inside");
    bring_up(&rig.dev);
    poke(&rig.dev, R_ICR, ICR_MASK);
    poke(&rig.dev, R_MASK, STA_RXDAVL);
    arm_data(&rig.dev, BLOCK as u32, true, 0);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    assert_eq!(sink.level.load(AtomicOrdering::SeqCst), 1);
    poke(&rig.dev, R_ICR, ICR_MASK);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        1,
        "a level is not an event, and ICR cannot clear one"
    );
    for _ in 0..BLOCK / 4 {
        let _ = peek(&rig.dev, R_FIFO);
    }
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        0,
        "draining it is what drops the line"
    );
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
// The DMA request line
// ---------------------------------------------------------------------------

#[test]
fn the_dma_request_only_rises_with_dmaen_and_drops_when_the_fifo_empties() {
    let rig = rig();
    let sink = watch(&rig.dev, pin::DMA);
    rig.card.write_media(0, &pattern(0x4d)).expect("inside");
    bring_up(&rig.dev);

    // Without DMAEN the line stays down however full the FIFO gets, which is
    // the bit this issue exists to make load-bearing.
    arm_data(&rig.dev, BLOCK as u32, true, 0);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    assert_ne!(peek(&rig.dev, R_STA) & STA_RXFIFOF, 0);
    assert_eq!(sink.level.load(AtomicOrdering::SeqCst), 0, "DMAEN is clear");
    assert!(!rig.dev.dma_requesting());
    for _ in 0..BLOCK / 4 {
        let _ = peek(&rig.dev, R_FIFO);
    }

    // With it, the line follows the FIFO.
    poke(&rig.dev, R_ICR, ICR_MASK);
    arm_data(&rig.dev, BLOCK as u32, true, DCTRL_DMAEN);
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        0,
        "armed, but the card has not been asked for anything yet"
    );
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    assert_eq!(sink.level.load(AtomicOrdering::SeqCst), 1);
    for _ in 0..BLOCK / 4 {
        let _ = peek(&rig.dev, R_FIFO);
    }
    assert_eq!(
        sink.level.load(AtomicOrdering::SeqCst),
        0,
        "the last word out drops it"
    );
    assert_ne!(peek(&rig.dev, R_STA) & STA_DATAEND, 0);
}

/// A real DMA controller, a real card, and the register block between them.
///
/// This is the claim the issue is about: `st.dma` has no knowledge of this
/// device, reads the FIFO at `CPAR` like any other peripheral register, and the
/// only thing connecting the two is the request line.
#[cfg(feature = "dev-stm32-dma")]
mod with_dma2 {
    use super::*;

    use crate::core::space::{
        AddressSpace, RamStore, Region as MemRegion, RequesterId, UnassignedPolicy,
    };
    use crate::dev::stm32::dma::{Dma, Variant};

    /// Where guest RAM sits in the rig's address space.
    const RAM_BASE: u64 = 0x2000_0000;
    const RAM_LEN: u64 = 0x1_0000;
    /// Where the SDIO register block sits: RM0090 Table 1, APB2 + 0x2c00.
    const SDIO_BASE: u64 = 0x4001_2c00;

    /// A stream's register offsets (RM0090 §10.5).
    const fn s_cr(s: u64) -> u64 {
        0x10 + 0x18 * s
    }
    const fn s_ndtr(s: u64) -> u64 {
        s_cr(s) + 4
    }
    const fn s_par(s: u64) -> u64 {
        s_cr(s) + 8
    }
    const fn s_m0ar(s: u64) -> u64 {
        s_cr(s) + 0x0c
    }

    /// `SxCR`: enable, minc, and word on both ports.
    const CR_EN: u32 = 1 << 0;
    const CR_MINC: u32 = 1 << 10;
    const CR_WORDS: u32 = (0b10 << 11) | (0b10 << 13);
    /// `DIR = 01`, memory to peripheral.
    const CR_M2P: u32 = 0b01 << 6;
    /// `CHSEL = 4`, which is where RM0090 Table 43 puts SDIO. Stored and not
    /// acted on by `st.dma` — the wiring is what selects the stream here — but
    /// written anyway, because a driver writes it and it must read back.
    const CR_CHSEL4: u32 = 4 << 25;

    /// Everything the DMA test needs: the two devices, the space they share,
    /// and the card.
    struct Pair {
        sdio: Sdio,
        card: Arc<SdCard>,
        dma: Dma,
        space: Arc<AddressSpace>,
    }

    /// Build the pair and draw the one wire between them.
    fn pair() -> Pair {
        let card = card_of(8 * 1024 * 1024, true);
        let slot = Arc::new(Slot::new());
        slot.insert(Arc::clone(&card)).expect("an empty socket");
        let sdio = Sdio::with_slot(slot, "sd0".to_string());

        let space =
            Arc::new(AddressSpace::new("dmabus", 32).with_unassigned(UnassignedPolicy::FAULT));
        {
            let mut topo = space.topology();
            topo.map(
                Arc::new(MemRegion::ram("ram", Arc::new(RamStore::new(RAM_LEN)))),
                RAM_BASE,
            )
            .expect("ram maps");
            // The load-bearing line: the peripheral is in the space the DMA
            // controller masters, because the beat is an ordinary bus access at
            // `CPAR` and not a private channel.
            topo.map(sdio.region("").expect("regs"), SDIO_BASE)
                .expect("the register block maps");
        }

        let dma = Dma::with_variant(Variant::Stream);
        dma.attach_bus(&space, RequesterId::ANONYMOUS);

        // `wire sdio.dma -> dma2.req3` — stream 3 is where RM0090 Table 43 puts
        // SDIO's request. Nothing else joins the two devices.
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let req = Device::sink(&dma, "req3", &[id]).expect("the stream face has req3");
        let wire = Arc::new(Wire::builder().source(id).sink(req.sink, req.line).build());
        sdio.connect(pin::DMA, WireSource::new(wire, id))
            .expect("the request output");

        Pair {
            sdio,
            card,
            dma,
            space,
        }
    }

    /// The DMA controller's own register block, as a bus master would see it.
    fn dma_ops(dma: &Dma) -> Arc<dyn MemOps> {
        match Device::region(dma, "").expect("a register block").kind() {
            RegionKind::Io(o) => Arc::clone(o),
            other => panic!("expected an io region, got {other:?}"),
        }
    }

    /// Program one of the DMA controller's registers.
    fn poke_dma(dma: &Dma, offset: u64, value: u32) {
        dma_ops(dma)
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is a legal bus cycle");
    }

    fn ram(space: &AddressSpace, addr: u64, len: usize) -> Vec<u8> {
        let mut out = alloc::vec![0u8; len];
        space
            .read_bytes(addr, &mut out, MemAttrs::DEBUG)
            .expect("mapped RAM");
        out
    }

    #[test]
    fn a_block_read_goes_out_through_dma2_on_the_request_line_alone() {
        let pair = pair();
        let want = pattern(0xc3);
        pair.card.write_media(0, &want).expect("inside the card");
        bring_up(&pair.sdio);

        // Arm stream 3: peripheral to memory, word to word, CPAR at the FIFO.
        let dst = RAM_BASE + 0x100;
        poke_dma(&pair.dma, s_par(3), (SDIO_BASE + R_FIFO) as u32);
        poke_dma(&pair.dma, s_m0ar(3), dst as u32);
        poke_dma(&pair.dma, s_ndtr(3), (BLOCK / 4) as u32);
        poke_dma(&pair.dma, s_cr(3), CR_CHSEL4 | CR_WORDS | CR_MINC | CR_EN);

        assert_eq!(pair.dma.pump(8), 0, "nothing is asking yet");

        arm_data(&pair.sdio, BLOCK as u32, true, DCTRL_DMAEN);
        command(&pair.sdio, 17, 0, WAITRESP_SHORT);
        assert!(pair.sdio.dma_requesting(), "the line is up");

        // One beat is one word, and the controller is given more budget than
        // the transfer needs so that a stream that over-runs would show up.
        assert_eq!(pair.dma.pump(1024), BLOCK / 4, "128 beats and not one more");
        assert_eq!(ram(&pair.space, dst, BLOCK as usize), want);

        let sta = pair.sdio.status();
        assert_ne!(sta & STA_DATAEND, 0, "the SDIO saw its own transfer end");
        assert_ne!(sta & STA_DBCKEND, 0);
        assert_eq!(sta & (STA_RXOVERR | STA_DTIMEOUT), 0);
        assert!(!pair.sdio.dma_requesting(), "and dropped the line");
        assert_eq!(pair.dma.remaining(3), 0);
        assert!(!pair.dma.is_running(3), "the stream disarmed itself");
    }

    #[test]
    fn two_blocks_go_into_the_card_through_dma2_and_read_back_the_same() {
        let pair = pair();
        let want: Vec<u8> = (0..2 * BLOCK as u32)
            .map(|i| (i.wrapping_mul(7).wrapping_add(11)) as u8)
            .collect();
        let src = RAM_BASE + 0x200;
        pair.space
            .write_bytes(src, &want, MemAttrs::DEBUG)
            .expect("mapped RAM");
        bring_up(&pair.sdio);

        poke_dma(&pair.dma, s_par(3), (SDIO_BASE + R_FIFO) as u32);
        poke_dma(&pair.dma, s_m0ar(3), src as u32);
        poke_dma(&pair.dma, s_ndtr(3), (2 * BLOCK / 4) as u32);
        poke_dma(
            &pair.dma,
            s_cr(3),
            CR_CHSEL4 | CR_M2P | CR_WORDS | CR_MINC | CR_EN,
        );

        // CMD25, then the data configuration: the order ST's own driver uses.
        command(&pair.sdio, 25, 0, WAITRESP_SHORT);
        arm_data(&pair.sdio, 2 * BLOCK as u32, false, DCTRL_DMAEN);
        assert!(pair.sdio.dma_requesting());

        assert_eq!(pair.dma.pump(2048), 2 * BLOCK / 4);
        let sta = pair.sdio.status();
        assert_ne!(sta & STA_DATAEND, 0);
        assert_eq!(sta & STA_TXUNDERR, 0);
        command(&pair.sdio, 12, 0, WAITRESP_SHORT);

        let mut back = alloc::vec![0u8; 2 * BLOCK as usize];
        pair.card.read_media(0, &mut back).expect("inside the card");
        assert_eq!(back, want);
    }

    #[test]
    fn a_stream_that_is_armed_but_unasked_moves_nothing() {
        // The defect this issue exists to remove, in test form: a DMAEN bit
        // that was stored and ignored would let the stream run on its own.
        let pair = pair();
        pair.card.write_media(0, &pattern(0x1f)).expect("inside");
        bring_up(&pair.sdio);
        poke_dma(&pair.dma, s_par(3), (SDIO_BASE + R_FIFO) as u32);
        poke_dma(&pair.dma, s_m0ar(3), (RAM_BASE + 0x100) as u32);
        poke_dma(&pair.dma, s_ndtr(3), (BLOCK / 4) as u32);
        poke_dma(&pair.dma, s_cr(3), CR_CHSEL4 | CR_WORDS | CR_MINC | CR_EN);

        // A transfer with a full FIFO and **no** DMAEN.
        arm_data(&pair.sdio, BLOCK as u32, true, 0);
        command(&pair.sdio, 17, 0, WAITRESP_SHORT);
        assert_ne!(pair.sdio.status() & STA_RXFIFOF, 0);
        assert_eq!(pair.dma.pump(1024), 0, "the line was never raised");
        assert_eq!(pair.dma.remaining(3), (BLOCK / 4) as u32);
    }
}

// ---------------------------------------------------------------------------
// The debug contract
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_pops_nothing_and_clears_nothing() {
    let rig = rig();
    let want = pattern(0x99);
    rig.card.write_media(0, &want).expect("inside");
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true, 0);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);

    let sta = peek_debug(&rig.dev, R_STA);
    let dcount = peek_debug(&rig.dev, R_DCOUNT);
    let fifocnt = peek_debug(&rig.dev, R_FIFOCNT);
    let head = peek_debug(&rig.dev, R_FIFO);
    for _ in 0..8 {
        assert_eq!(peek_debug(&rig.dev, R_FIFO), head, "the same word, forever");
    }
    assert_eq!(peek_debug(&rig.dev, R_STA), sta, "and no flag moved");
    assert_eq!(peek_debug(&rig.dev, R_DCOUNT), dcount);
    assert_eq!(peek_debug(&rig.dev, R_FIFOCNT), fifocnt);

    let mut got = Vec::new();
    for _ in 0..BLOCK / 4 {
        got.extend_from_slice(&peek(&rig.dev, R_FIFO).to_le_bytes());
    }
    assert_eq!(got, want);
}

#[test]
fn a_debug_write_is_refused_rather_than_obeyed() {
    let rig = rig();
    assert!(
        ops(&rig.dev)
            .write(R_CMD, &0u32.to_le_bytes(), MemAttrs::DEBUG)
            .is_err()
    );
    assert!(
        ops(&rig.dev)
            .write(R_POWER, &POWER_ON.to_le_bytes(), MemAttrs::DEBUG)
            .is_err()
    );
}

// ---------------------------------------------------------------------------
// Snapshots and reset
// ---------------------------------------------------------------------------

fn snapshot(dev: &Sdio) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("sdio", CLASS.name).expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("sdio", CLASS.name, CLASS.version).expect("a chunk");
        dev.save(&mut chunk).expect("the controller saves");
    }
    w.to_vec().expect("a snapshot")
}

fn restore(dev: &Sdio, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let chunk = reader
        .load("sdio", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    dev.load(&mut chunk.reader()).expect("the controller loads");
}

#[test]
fn a_snapshot_carries_a_transfer_in_flight_and_the_words_in_the_fifo() {
    let rig = rig();
    let want = pattern(0x3c);
    rig.card.write_media(0, &want).expect("inside");
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true, DCTRL_DMAEN);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    // Drain a few words, so the FIFO is neither full nor empty and the DPSM is
    // part way through its block.
    for _ in 0..5 {
        let _ = peek(&rig.dev, R_FIFO);
    }

    let bytes = snapshot(&rig.dev);
    let other = super::tests::rig();
    restore(&other.dev, &bytes);
    assert_eq!(snapshot(&other.dev), bytes, "identical state");
    assert_eq!(peek(&other.dev, R_DCOUNT), peek(&rig.dev, R_DCOUNT));
    assert_eq!(peek(&other.dev, R_FIFOCNT), peek(&rig.dev, R_FIFOCNT));
    assert_eq!(
        peek(&other.dev, R_STA) & STA_LATCHED,
        peek(&rig.dev, R_STA) & STA_LATCHED
    );
    assert_eq!(
        other.dev.dma_requesting(),
        rig.dev.dma_requesting(),
        "and the request line came back up with it"
    );
    // The restored controller hands back the same next word, which is the one
    // the saved one had not popped yet.
    assert_eq!(peek(&other.dev, R_FIFO), peek(&rig.dev, R_FIFO));
}

#[test]
fn a_snapshot_with_an_impossible_fifo_is_refused() {
    let rig = rig();
    let mut bytes = snapshot(&rig.dev);
    // The FIFO length is the first `u64` after the fourteen register words.
    let at = bytes.len() - 8 - 1;
    bytes[at] = 0xff;
    let reader = StateReader::new(&bytes).expect("a snapshot");
    let chunk = reader
        .load("sdio", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    assert!(rig.dev.load(&mut chunk.reader()).is_err());
}

#[test]
fn a_reset_clears_the_register_block_and_leaves_the_card_alone() {
    let rig = rig();
    bring_up(&rig.dev);
    arm_data(&rig.dev, BLOCK as u32, true, DCTRL_DMAEN);
    command(&rig.dev, 17, 0, WAITRESP_SHORT);
    rig.dev.reset(ResetKind::Cold);
    assert_eq!(peek(&rig.dev, R_POWER), 0);
    assert_eq!(peek(&rig.dev, R_STA) & STA_LATCHED, 0);
    assert_eq!(peek(&rig.dev, R_DCOUNT), 0);
    assert_eq!(peek(&rig.dev, R_FIFOCNT), 0);
    assert_ne!(peek(&rig.dev, R_STA) & STA_RXFIFOE, 0);
    assert!(!rig.dev.dma_requesting());
    assert_eq!(
        rig.card.phase(),
        Phase::SendingData,
        "the card is its own device and resets itself"
    );
}
