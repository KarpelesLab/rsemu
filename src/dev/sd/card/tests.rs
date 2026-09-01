//! What a card must refuse, where its state machine must go, and what a
//! snapshot must carry.

use super::*;
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

/// The CID fields every test uses, so a register comparison is against a
/// constant rather than against whatever the last test left behind.
fn text() -> IdentityText<'static> {
    IdentityText {
        manufacturer: 0x03,
        oem: "RE",
        product: "RSEMU",
        revision: 0x10,
        serial: 0x1234_5678,
        year: 2024,
        month: 1,
    }
}

fn card_of(capacity: u64, high_capacity: bool, read_only: bool) -> SdCard {
    let id = Identity::new(capacity, high_capacity, read_only, text()).expect("a plausible card");
    SdCard::with_identity(id, BusMode::Sd, 1).expect("it fits")
}

/// 8 MiB, standard capacity: the smallest card whose `C_SIZE` is exact and
/// whose contents fit comfortably in a test.
fn sdsc() -> SdCard {
    card_of(8 * 1024 * 1024, false, false)
}

/// 8 MiB, high capacity. Smaller than any real SDHC, and deliberately so: the
/// `C_SIZE` encoding is exact at this size, the guest is told the truth, and a
/// test does not have to allocate two gigabytes to exercise block addressing.
fn sdhc() -> SdCard {
    card_of(8 * 1024 * 1024, true, false)
}

fn short(reply: Reply) -> (u8, u32) {
    match reply {
        Reply::Short { index, value, .. } => (index, value),
        other => panic!("expected a short response, got {other:?}"),
    }
}

fn long(reply: Reply) -> [u32; 4] {
    match reply {
        Reply::Long(words) => words,
        other => panic!("expected a long response, got {other:?}"),
    }
}

fn status_state(status: u32) -> u32 {
    (status >> STATE_SHIFT) & 0xf
}

/// Walk the identification sequence a real driver walks, and return the
/// published address.
fn bring_up(card: &SdCard) -> u16 {
    assert_eq!(card.command(cmd::GO_IDLE_STATE, 0), Reply::None);
    let (_, ifcond) = short(card.command(cmd::SEND_IF_COND, 0x0000_01aa));
    assert_eq!(ifcond, 0x1aa, "the card echoes the check pattern");
    let hcs = if card.identity().high_capacity {
        1 << 30
    } else {
        0
    };
    short(card.command(cmd::APP_CMD, 0));
    let (index, ocr) = short(card.command(cmd::A_SD_SEND_OP_COND, hcs | 0x00ff_8000));
    assert_eq!(index, 0x3f, "R3 carries no command index");
    assert_ne!(ocr & (1 << 31), 0, "powered up");
    assert_eq!(card.phase(), Phase::Ready);
    let _cid = long(card.command(cmd::ALL_SEND_CID, 0));
    assert_eq!(card.phase(), Phase::Identification);
    let (_, r6) = short(card.command(cmd::SEND_RELATIVE_ADDR, 0));
    let rca = (r6 >> 16) as u16;
    assert_eq!(card.phase(), Phase::Standby);
    short(card.command(cmd::SELECT_CARD, u32::from(rca) << 16));
    assert_eq!(card.phase(), Phase::Transfer);
    rca
}

fn read_block(card: &SdCard, arg: u32) -> Vec<u8> {
    short(card.command(cmd::READ_SINGLE_BLOCK, arg));
    let mut out = alloc::vec![0u8; BLOCK as usize];
    assert_eq!(card.read_data(&mut out), Data::Moved);
    out
}

// ---------------------------------------------------------------------------
// CRC7
// ---------------------------------------------------------------------------

#[test]
fn crc7_matches_the_two_constants_every_spi_driver_hard_codes() {
    // The only two SD command CRCs in the wild as literals, because SPI mode
    // ignores the CRC afterwards but not for CMD0 and CMD8. Physical Layer
    // §4.5's polynomial has to produce exactly these or every register this
    // file builds is wrong in its last byte.
    assert_eq!((crc7(&[0x40, 0, 0, 0, 0]) << 1) | 1, 0x95, "CMD0");
    assert_eq!((crc7(&[0x48, 0, 0, 0x01, 0xaa]) << 1) | 1, 0x87, "CMD8");
}

#[test]
fn every_register_ends_in_its_own_crc7() {
    let card = sdhc();
    let id = card.identity();
    assert_eq!(id.cid[15], (crc7(&id.cid[..15]) << 1) | 1);
    assert_eq!(id.csd[15], (crc7(&id.csd[..15]) << 1) | 1);
}

// ---------------------------------------------------------------------------
// The registers
// ---------------------------------------------------------------------------

#[test]
fn a_version_2_csd_describes_the_capacity_it_was_built_from() {
    let card = sdhc();
    let csd = card.identity().csd;
    assert_eq!(csd[0] >> 6, 0b01, "CSD_STRUCTURE says version 2.0");
    // §5.3.3: capacity = (C_SIZE + 1) * 512 KiB.
    let c_size = ((u32::from(csd[7]) & 0x3f) << 16) | (u32::from(csd[8]) << 8) | u32::from(csd[9]);
    assert_eq!(
        u64::from(c_size + 1) * HIGH_CAPACITY_UNIT,
        card.identity().capacity
    );
    assert_eq!(csd[5] & 0xf, 9, "READ_BL_LEN is fixed at nine");
}

#[test]
fn a_version_1_csd_describes_the_capacity_it_was_built_from() {
    for size in [8 * 1024 * 1024u64, 64 * 1024 * 1024, 1024 * 1024 * 1024] {
        let card = card_of(size, false, false);
        let csd = card.identity().csd;
        assert_eq!(csd[0] >> 6, 0b00, "CSD_STRUCTURE says version 1.0");
        // §5.3.2: capacity = (C_SIZE + 1) * 2^(C_SIZE_MULT + 2) * 2^READ_BL_LEN.
        let read_bl_len = u32::from(csd[5] & 0xf);
        let c_size = ((u32::from(csd[6]) & 0x03) << 10)
            | (u32::from(csd[7]) << 2)
            | (u32::from(csd[8]) >> 6);
        let c_size_mult = ((u32::from(csd[9]) & 0x03) << 1) | (u32::from(csd[10]) >> 7);
        let capacity = u64::from(c_size + 1) * (1u64 << (c_size_mult + 2)) * (1u64 << read_bl_len);
        assert_eq!(capacity, size, "for a {size}-byte card");
        assert_ne!(csd[6] & 0x80, 0, "READ_BL_PARTIAL is mandatory here");
    }
}

#[test]
fn the_cid_carries_the_text_it_was_given() {
    let card = sdsc();
    let cid = card.identity().cid;
    assert_eq!(cid[0], 0x03, "MID");
    assert_eq!(&cid[1..3], b"RE", "OID");
    assert_eq!(&cid[3..8], b"RSEMU", "PNM");
    assert_eq!(&cid[9..13], &0x1234_5678u32.to_be_bytes(), "PSN");
    // MDT is bits 19:8, eight of year offset from 2000 then four of month.
    let mdt = ((u32::from(cid[13]) & 0x0f) << 8) | u32::from(cid[14]);
    assert_eq!(mdt >> 4, 24, "2024");
    assert_eq!(mdt & 0xf, 1, "January");
}

#[test]
fn a_capacity_the_csd_cannot_express_is_refused_rather_than_rounded() {
    // Rounding here would hand the guest a CSD describing a card of a different
    // size from the one it can address, which is the worst of both.
    let err = Identity::new(8 * 1024 * 1024 + 512, true, false, text());
    assert!(err.is_err(), "not a multiple of the 512 KiB C_SIZE unit");
    let err = Identity::new(3 * 1024 * 1024 * 1024, false, false, text());
    assert!(err.is_err(), "past what a version 1.0 CSD can describe");
    let err = Identity::new(511, false, false, text());
    assert!(err.is_err(), "not a whole number of blocks");
}

// ---------------------------------------------------------------------------
// The initialisation handshake
// ---------------------------------------------------------------------------

#[test]
fn a_card_walks_the_identification_sequence_to_the_transfer_state() {
    let card = sdhc();
    assert_eq!(card.phase(), Phase::Idle);
    let rca = bring_up(&card);
    assert_eq!(rca, 1, "the first published address");
    assert_eq!(card.rca(), rca);
    assert_eq!(card.phase(), Phase::Transfer);
}

#[test]
fn the_cid_a_guest_reads_is_the_register_this_card_holds() {
    let card = sdhc();
    assert_eq!(card.command(cmd::GO_IDLE_STATE, 0), Reply::None);
    short(card.command(cmd::APP_CMD, 0));
    short(card.command(cmd::A_SD_SEND_OP_COND, (1 << 30) | 0x00ff_8000));
    let words = long(card.command(cmd::ALL_SEND_CID, 0));
    let mut bytes = [0u8; 16];
    for (i, word) in words.iter().enumerate() {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    assert_eq!(
        bytes,
        card.identity().cid,
        "R2 is the register, CRC included"
    );
}

#[test]
fn an_inquiry_acmd41_reports_the_ocr_without_starting_initialisation() {
    // §4.2.3: a zero voltage window asks what the card wants. A card that
    // treated it as the real thing would leave `idle` before the host has said
    // it can supply the voltage.
    let card = sdhc();
    short(card.command(cmd::APP_CMD, 0));
    let (_, ocr) = short(card.command(cmd::A_SD_SEND_OP_COND, 0));
    assert_eq!(ocr & 0x00ff_8000, 0x00ff_8000, "the voltage window");
    assert_eq!(ocr & (1 << 31), 0, "not powered up");
    assert_eq!(card.phase(), Phase::Idle);
}

#[test]
fn a_high_capacity_card_refuses_a_host_that_did_not_ask_for_one() {
    // §4.2.3 again: HCS clear means the host cannot address blocks, and the
    // card must go inactive rather than pretend to be standard capacity.
    let card = sdhc();
    short(card.command(cmd::APP_CMD, 0));
    assert_eq!(
        card.command(cmd::A_SD_SEND_OP_COND, 0x00ff_8000),
        Reply::None
    );
    assert_eq!(card.phase(), Phase::Inactive);
    assert_eq!(
        card.command(cmd::GO_IDLE_STATE, 0),
        Reply::None,
        "and stays there until power cycles"
    );
    assert_eq!(card.phase(), Phase::Inactive);
    card.power_cycle();
    assert_eq!(card.phase(), Phase::Idle);
}

#[test]
fn a_standard_capacity_card_accepts_a_host_that_did_not_ask_for_high_capacity() {
    let card = sdsc();
    short(card.command(cmd::APP_CMD, 0));
    let (_, ocr) = short(card.command(cmd::A_SD_SEND_OP_COND, 0x00ff_8000));
    assert_ne!(ocr & (1 << 31), 0);
    assert_eq!(ocr & (1 << 30), 0, "CCS clear: this card counts bytes");
    assert_eq!(card.phase(), Phase::Ready);
}

#[test]
fn cmd8_stays_quiet_at_a_voltage_the_card_cannot_work_at() {
    let card = sdhc();
    assert_eq!(card.command(cmd::SEND_IF_COND, 0x0000_02aa), Reply::None);
}

#[test]
fn a_register_read_addressed_to_another_card_is_answered_by_nobody() {
    let card = sdhc();
    let rca = bring_up(&card);
    // Back to standby: CMD9 is only legal there.
    assert_eq!(card.command(cmd::SELECT_CARD, 0), Reply::None);
    assert_eq!(card.phase(), Phase::Standby);
    assert_eq!(
        card.command(cmd::SEND_CSD, u32::from(rca.wrapping_add(1)) << 16),
        Reply::None
    );
    let words = long(card.command(cmd::SEND_CSD, u32::from(rca) << 16));
    let mut bytes = [0u8; 16];
    for (i, word) in words.iter().enumerate() {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    assert_eq!(bytes, card.identity().csd);
}

// ---------------------------------------------------------------------------
// Addressing
// ---------------------------------------------------------------------------

#[test]
fn a_high_capacity_argument_counts_blocks_and_a_standard_one_counts_bytes() {
    // The bug this test exists to catch reads the right data from the wrong
    // place for the first 512 sectors and then quietly diverges.
    let hc = sdhc();
    hc.write_media(3 * 512, &[0xa5; 512]).expect("inside");
    bring_up(&hc);
    assert_eq!(read_block(&hc, 3), alloc::vec![0xa5; 512], "block three");
    assert_eq!(
        read_block(&hc, 3 * 512),
        alloc::vec![0u8; 512],
        "and 1536 is block 1536, not byte 1536"
    );

    let sc = sdsc();
    sc.write_media(3 * 512, &[0xa5; 512]).expect("inside");
    bring_up(&sc);
    assert_eq!(
        read_block(&sc, 3 * 512),
        alloc::vec![0xa5; 512],
        "byte 1536"
    );
    // Argument three is byte three on this card, which is inside block zero
    // and 509 bytes from its end — a misaligned read the CSD forbids.
    let (_, status) = short(sc.command(cmd::READ_SINGLE_BLOCK, 3));
    assert_ne!(status & ADDRESS_ERROR, 0);
}

#[test]
fn a_read_past_the_end_is_refused_and_starts_no_transfer() {
    let card = sdhc();
    bring_up(&card);
    let blocks = card.identity().blocks() as u32;
    let (_, status) = short(card.command(cmd::READ_SINGLE_BLOCK, blocks));
    assert_ne!(status & OUT_OF_RANGE, 0);
    assert_eq!(card.phase(), Phase::Transfer, "no transfer began");
    let mut out = [0u8; 4];
    assert_eq!(card.read_data(&mut out), Data::Ended);
}

#[test]
fn a_partial_read_may_not_straddle_a_physical_block() {
    // READ_BLK_MISALIGN is clear, so a shorter read is legal but only inside
    // one block (§5.3.2).
    let card = sdsc();
    bring_up(&card);
    short(card.command(cmd::SET_BLOCKLEN, 64));
    short(card.command(cmd::READ_SINGLE_BLOCK, 0));
    assert_eq!(card.phase(), Phase::SendingData);
    card.abort();
    let (_, status) = short(card.command(cmd::READ_SINGLE_BLOCK, 512 - 32));
    assert_ne!(status & ADDRESS_ERROR, 0);
}

#[test]
fn a_high_capacity_card_refuses_any_block_length_but_512() {
    let card = sdhc();
    bring_up(&card);
    let (_, status) = short(card.command(cmd::SET_BLOCKLEN, 64));
    assert_ne!(status & BLOCK_LEN_ERROR, 0);
    let (_, status) = short(card.command(cmd::SET_BLOCKLEN, 512));
    assert_eq!(status & BLOCK_LEN_ERROR, 0);
}

// ---------------------------------------------------------------------------
// Reading and writing
// ---------------------------------------------------------------------------

#[test]
fn a_single_block_read_ends_itself_and_returns_to_transfer() {
    let card = sdhc();
    card.write_media(0, &[0x5a; 512]).expect("inside");
    bring_up(&card);
    short(card.command(cmd::READ_SINGLE_BLOCK, 0));
    assert_eq!(card.phase(), Phase::SendingData);
    let mut out = [0u8; 512];
    assert_eq!(card.read_data(&mut out), Data::Moved);
    assert_eq!(out, [0x5a; 512]);
    assert_eq!(card.phase(), Phase::Transfer, "no CMD12 needed");
    assert_eq!(card.read_data(&mut out), Data::Ended);
}

#[test]
fn a_multiple_block_read_runs_until_cmd12_stops_it() {
    let card = sdhc();
    for block in 0..4u64 {
        card.write_media(block * 512, &[block as u8; 512])
            .expect("inside");
    }
    bring_up(&card);
    short(card.command(cmd::READ_MULTIPLE_BLOCK, 0));
    let mut out = [0u8; 512];
    for block in 0..4u8 {
        assert_eq!(card.read_data(&mut out), Data::Moved);
        assert_eq!(out, [block; 512], "block {block}");
        assert_eq!(card.phase(), Phase::SendingData);
    }
    let (_, status) = short(card.command(cmd::STOP_TRANSMISSION, 0));
    assert_eq!(
        status_state(status),
        Phase::SendingData.code(),
        "the status reports the state the command arrived in"
    );
    assert_eq!(card.phase(), Phase::Transfer);
}

#[test]
fn a_cmd23_count_stops_a_multiple_read_without_a_cmd12() {
    let card = sdhc();
    bring_up(&card);
    short(card.command(cmd::SET_BLOCK_COUNT, 2));
    short(card.command(cmd::READ_MULTIPLE_BLOCK, 0));
    let mut out = [0u8; 512];
    assert_eq!(card.read_data(&mut out), Data::Moved);
    assert_eq!(card.phase(), Phase::SendingData);
    assert_eq!(card.read_data(&mut out), Data::Moved);
    assert_eq!(card.phase(), Phase::Transfer, "the count ran out");
    assert_eq!(card.read_data(&mut out), Data::Ended);
}

#[test]
fn a_block_written_through_the_protocol_reads_back_through_it() {
    // The test that proves the model rather than the plumbing: nothing here
    // touches the array except through commands.
    let card = sdhc();
    bring_up(&card);
    let payload: Vec<u8> = (0..512u32).map(|i| (i * 7) as u8).collect();
    short(card.command(cmd::WRITE_BLOCK, 9));
    assert_eq!(card.phase(), Phase::ReceiveData);
    assert_eq!(card.write_data(&payload), Data::Moved);
    assert_eq!(
        card.phase(),
        Phase::Transfer,
        "programming is instantaneous"
    );
    assert_eq!(read_block(&card, 9), payload);
}

#[test]
fn a_write_arriving_in_pieces_is_programmed_only_once_it_is_whole() {
    let card = sdhc();
    bring_up(&card);
    short(card.command(cmd::WRITE_BLOCK, 0));
    for chunk in 0..7 {
        assert_eq!(card.write_data(&[0xcc; 64]), Data::Moved);
        assert_eq!(
            card.phase(),
            Phase::ReceiveData,
            "still receiving after chunk {chunk}"
        );
        let mut peek = [0u8; 512];
        card.read_media(0, &mut peek).expect("inside");
        assert_eq!(peek, [0u8; 512], "nothing has reached the array yet");
    }
    assert_eq!(card.write_data(&[0xcc; 64]), Data::Moved);
    assert_eq!(card.phase(), Phase::Transfer);
    let mut peek = [0u8; 512];
    card.read_media(0, &mut peek).expect("inside");
    assert_eq!(peek, [0xcc; 512]);
}

#[test]
fn an_aborted_write_drops_the_partial_block_rather_than_programming_it() {
    let card = sdhc();
    bring_up(&card);
    short(card.command(cmd::WRITE_BLOCK, 0));
    assert_eq!(card.write_data(&[0xee; 100]), Data::Moved);
    card.abort();
    assert_eq!(card.phase(), Phase::Transfer);
    let mut peek = [0u8; 512];
    card.read_media(0, &mut peek).expect("inside");
    assert_eq!(peek, [0u8; 512], "a card programs blocks, not bytes");
}

#[test]
fn a_multiple_write_walks_forward_and_cmd12_ends_it() {
    let card = sdhc();
    bring_up(&card);
    short(card.command(cmd::WRITE_MULTIPLE_BLOCK, 2));
    for block in 0..3u8 {
        assert_eq!(card.write_data(&[block + 1; 512]), Data::Moved);
    }
    short(card.command(cmd::STOP_TRANSMISSION, 0));
    for block in 0..3u8 {
        assert_eq!(
            read_block(&card, 2 + u32::from(block)),
            alloc::vec![block + 1; 512]
        );
    }
}

#[test]
fn a_write_protected_card_refuses_the_command_rather_than_the_data() {
    let card = card_of(8 * 1024 * 1024, true, true);
    bring_up(&card);
    let (_, status) = short(card.command(cmd::WRITE_BLOCK, 0));
    assert_ne!(status & WP_VIOLATION, 0);
    assert_eq!(
        card.phase(),
        Phase::Transfer,
        "no receive state was entered"
    );
    assert_eq!(card.write_data(&[0xff; 512]), Data::Ended);
    // And the CSD says so, so a driver can find out before it tries.
    assert_ne!(card.identity().csd[14] & 0x20, 0, "PERM_WRITE_PROTECT");
}

#[test]
fn an_erased_range_reads_as_zero() {
    let card = sdhc();
    card.write_media(0, &[0xff; 2048]).expect("inside");
    bring_up(&card);
    short(card.command(cmd::ERASE_WR_BLK_START, 1));
    short(card.command(cmd::ERASE_WR_BLK_END, 2));
    short(card.command(cmd::ERASE, 0));
    let mut peek = [0u8; 2048];
    card.read_media(0, &mut peek).expect("inside");
    assert_eq!(peek[..512], [0xff; 512], "block 0 is untouched");
    assert_eq!(peek[512..1536], [0u8; 1024], "SCR says erased reads zero");
    assert_eq!(peek[1536..], [0xff; 512], "block 3 is untouched");
}

#[test]
fn an_erase_with_no_range_is_refused() {
    let card = sdhc();
    bring_up(&card);
    let (_, status) = short(card.command(cmd::ERASE, 0));
    assert_ne!(status & ERASE_SEQ_ERROR, 0);
}

// ---------------------------------------------------------------------------
// The state machine's refusals
// ---------------------------------------------------------------------------

#[test]
fn a_data_command_in_standby_is_illegal_rather_than_obeyed() {
    let card = sdhc();
    let rca = bring_up(&card);
    assert_eq!(card.command(cmd::SELECT_CARD, 0), Reply::None);
    assert_eq!(card.phase(), Phase::Standby);
    let (_, status) = short(card.command(cmd::READ_SINGLE_BLOCK, 0));
    assert_ne!(status & ILLEGAL_COMMAND, 0);
    assert_eq!(card.phase(), Phase::Standby, "and nothing started");
    // Reselecting puts it back.
    short(card.command(cmd::SELECT_CARD, u32::from(rca) << 16));
    assert_eq!(card.phase(), Phase::Transfer);
}

#[test]
fn an_error_bit_is_reported_once_and_then_cleared() {
    // §4.10.1's "clear by read" condition: the response to the command that
    // caused the error is where it is reported, and it is gone afterwards. A
    // bit that stayed set would make every later response look like a failure.
    let card = sdhc();
    bring_up(&card);
    let (_, status) = short(card.command(cmd::SET_BLOCKLEN, 3));
    assert_ne!(status & BLOCK_LEN_ERROR, 0, "reported once");
    assert_ne!(status & CARD_ERROR, 0);
    let (_, status) = short(card.command(cmd::SEND_STATUS, u32::from(card.rca()) << 16));
    assert_eq!(status & BLOCK_LEN_ERROR, 0, "and gone");
    assert_eq!(status & CARD_ERROR, 0);
}

#[test]
fn peeking_at_the_status_moves_nothing() {
    // The `MemAttrs::debug` rule one level below a register block: a debugger
    // asking where the card is must not advance it, must not clear anything,
    // and must not disagree with what a CMD13 would have said.
    let card = sdhc();
    bring_up(&card);
    short(card.command(cmd::APP_CMD, u32::from(card.rca()) << 16));
    let first = card.peek_status();
    assert_ne!(first & APP_CMD, 0, "a CMD55 is still outstanding");
    assert_eq!(card.peek_status(), first, "and looking twice says the same");
    // The outstanding CMD55 is still outstanding, so the next command is still
    // an application command.
    short(card.command(cmd::A_SET_BUS_WIDTH, 0b10));
    assert_eq!(card.bus_width(), 4);
}

#[test]
fn cmd0_puts_the_card_back_where_it_started() {
    let card = sdhc();
    bring_up(&card);
    short(card.command(cmd::READ_MULTIPLE_BLOCK, 0));
    assert_eq!(card.command(cmd::GO_IDLE_STATE, 0), Reply::None);
    assert_eq!(card.phase(), Phase::Idle);
    assert_eq!(card.rca(), 0);
    let mut out = [0u8; 4];
    assert_eq!(
        card.read_data(&mut out),
        Data::Ended,
        "and the transfer went"
    );
}

#[test]
fn an_app_cmd_only_applies_to_the_command_immediately_after_it() {
    let card = sdhc();
    bring_up(&card);
    // ACMD6 sets the bus width; CMD6 with no CMD55 in front of it is the
    // switch-function command, which is an entirely different thing.
    short(card.command(cmd::APP_CMD, u32::from(card.rca()) << 16));
    short(card.command(cmd::A_SET_BUS_WIDTH, 0b10));
    assert_eq!(card.bus_width(), 4);
    short(card.command(cmd::SWITCH_FUNC, 0x00ff_ffff));
    assert_eq!(
        card.phase(),
        Phase::SendingData,
        "CMD6 without CMD55 is SWITCH_FUNC and moves data"
    );
}

#[test]
fn a_card_told_to_go_inactive_answers_nothing_at_all() {
    let card = sdhc();
    let rca = bring_up(&card);
    assert_eq!(
        card.command(cmd::GO_INACTIVE_STATE, u32::from(rca) << 16),
        Reply::None
    );
    assert_eq!(card.phase(), Phase::Inactive);
    assert_eq!(
        card.command(cmd::SEND_STATUS, u32::from(rca) << 16),
        Reply::None
    );
}

// ---------------------------------------------------------------------------
// The register data transfers
// ---------------------------------------------------------------------------

#[test]
fn acmd51_sends_the_scr_and_nothing_more() {
    let card = sdhc();
    bring_up(&card);
    short(card.command(cmd::APP_CMD, u32::from(card.rca()) << 16));
    short(card.command(cmd::A_SEND_SCR, 0));
    let mut out = [0u8; 8];
    assert_eq!(card.read_data(&mut out), Data::Moved);
    assert_eq!(out, card.identity().scr);
    assert_eq!(card.phase(), Phase::Transfer);
    assert_eq!(card.read_data(&mut out), Data::Ended);
    // SD_BUS_WIDTHS says one bit and four bits, which is what ACMD6 accepts.
    assert_eq!(out[1] & 0xf, 0b0101);
}

#[test]
fn acmd13_reports_the_bus_width_the_host_selected() {
    let card = sdhc();
    bring_up(&card);
    short(card.command(cmd::APP_CMD, u32::from(card.rca()) << 16));
    short(card.command(cmd::A_SET_BUS_WIDTH, 0b10));
    short(card.command(cmd::APP_CMD, u32::from(card.rca()) << 16));
    short(card.command(cmd::A_SD_STATUS, 0));
    let mut out = [0u8; 64];
    assert_eq!(card.read_data(&mut out), Data::Moved);
    assert_eq!(out[0] >> 6, 0b10, "DAT_BUS_WIDTH says four bits");
}

#[test]
fn cmd6_reports_what_it_supports_and_remembers_what_it_switched_to() {
    let card = sdhc();
    bring_up(&card);
    // Check mode, asking for high speed in group 1.
    short(card.command(cmd::SWITCH_FUNC, 0x00ff_fff1));
    let mut out = [0u8; 64];
    assert_eq!(card.read_data(&mut out), Data::Moved);
    let group1 = (u16::from(out[12]) << 8) | u16::from(out[13]);
    assert_eq!(group1, 0x0003, "default and high speed");
    assert_eq!(out[16] & 0xf, 1, "high speed would be granted");
    assert_eq!(out[17], 1, "data structure version 1");

    // Switch mode. Nothing else in the card changes, but the selection sticks
    // and a later query reports it.
    short(card.command(cmd::SWITCH_FUNC, 0x80ff_fff1));
    assert_eq!(card.read_data(&mut out), Data::Moved);
    short(card.command(cmd::SWITCH_FUNC, 0x00ff_ffff));
    assert_eq!(card.read_data(&mut out), Data::Moved);
    assert_eq!(out[16] & 0xf, 1, "and 0xf means `no change`, not `default`");

    // A function the card does not have comes back as 0xf.
    short(card.command(cmd::SWITCH_FUNC, 0x00ff_fff3));
    assert_eq!(card.read_data(&mut out), Data::Moved);
    assert_eq!(out[16] & 0xf, 0xf);
}

// ---------------------------------------------------------------------------
// SPI mode
// ---------------------------------------------------------------------------

#[test]
fn an_spi_mode_card_is_usable_without_any_bus_addressing() {
    // The claim the module documentation makes, checked: the same model serves
    // a transport that has no CMD2, no CMD3 and no CMD7.
    let id = Identity::new(8 * 1024 * 1024, true, false, text()).expect("a card");
    let card = SdCard::with_identity(id, BusMode::Spi, 1).expect("it fits");
    card.write_media(512, &[0x77; 512]).expect("inside");
    assert_eq!(card.command(cmd::GO_IDLE_STATE, 0), Reply::None);
    short(card.command(cmd::APP_CMD, 0));
    short(card.command(cmd::A_SD_SEND_OP_COND, (1 << 30) | 0x00ff_8000));
    assert_eq!(
        card.phase(),
        Phase::Transfer,
        "an initialised SPI card is simply available"
    );
    assert_eq!(read_block(&card, 1), alloc::vec![0x77; 512]);
    // And CMD13 answers without an address, because SPI has none.
    let (_, status) = short(card.command(cmd::SEND_STATUS, 0));
    assert_eq!(status_state(status), Phase::Transfer.code());
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

fn device(capacity: u64) -> CardDevice {
    let props = Props::new()
        .with("size", Value::Size(capacity))
        .with("high-capacity", Value::Bool(true));
    CardDevice::new(&props).expect("a plausible card")
}

fn snapshot(dev: &CardDevice) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("card", CLASS.name).expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("card", CLASS.name, CLASS.version)
            .expect("one chunk");
        dev.save(&mut chunk).expect("the card saves");
    }
    w.to_vec().expect("a snapshot")
}

fn restore(dev: &CardDevice, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let chunk = reader
        .load("card", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    dev.load(&mut chunk.reader()).expect("the card loads");
}

#[test]
fn a_snapshot_carries_a_read_that_is_half_way_through_a_block() {
    let saved = device(8 * 1024 * 1024);
    saved.card().write_media(0, &[0x11; 1024]).expect("inside");
    bring_up(saved.card());
    short(saved.card().command(cmd::READ_MULTIPLE_BLOCK, 0));
    let mut out = [0u8; 300];
    assert_eq!(saved.card().read_data(&mut out), Data::Moved);

    let bytes = snapshot(&saved);
    let restored = device(8 * 1024 * 1024);
    restore(&restored, &bytes);
    assert_eq!(snapshot(&restored), bytes, "identical state");

    // And the restored card finishes the transfer the saved one had begun,
    // from the byte it had reached rather than from the start of the block.
    let mut rest = [0u8; 212];
    assert_eq!(restored.card().read_data(&mut rest), Data::Moved);
    assert_eq!(rest, [0x11; 212]);
    assert_eq!(restored.card().phase(), Phase::SendingData);
}

#[test]
fn a_snapshot_carries_a_write_block_that_has_not_been_programmed_yet() {
    let saved = device(8 * 1024 * 1024);
    bring_up(saved.card());
    short(saved.card().command(cmd::WRITE_BLOCK, 4));
    assert_eq!(saved.card().write_data(&[0x33; 200]), Data::Moved);

    let bytes = snapshot(&saved);
    let restored = device(8 * 1024 * 1024);
    restore(&restored, &bytes);
    assert_eq!(snapshot(&restored), bytes, "identical state");

    assert_eq!(restored.card().write_data(&[0x33; 312]), Data::Moved);
    let mut peek = [0u8; 512];
    restored.card().read_media(4 * 512, &mut peek).expect("in");
    assert_eq!(peek, [0x33; 512], "the whole block, both halves of it");
}

#[test]
fn a_snapshot_from_a_differently_sized_card_is_refused() {
    let big = device(16 * 1024 * 1024);
    let bytes = snapshot(&big);
    let small = device(8 * 1024 * 1024);
    let reader = StateReader::new(&bytes).expect("a snapshot");
    let chunk = reader
        .load("card", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    assert!(small.load(&mut chunk.reader()).is_err());
}

#[test]
fn a_reset_cycles_the_protocol_and_keeps_the_contents() {
    let dev = device(8 * 1024 * 1024);
    dev.card().write_media(0, &[0x99; 512]).expect("inside");
    bring_up(dev.card());
    dev.reset(ResetKind::Cold);
    assert_eq!(dev.card().phase(), Phase::Idle);
    assert_eq!(dev.card().rca(), 0);
    let mut peek = [0u8; 512];
    dev.card().read_media(0, &mut peek).expect("inside");
    assert_eq!(peek, [0x99; 512], "a card is not volatile");
}

#[test]
fn two_cards_cannot_share_one_slot() {
    let hosts = alloc::sync::Arc::new(crate::core::hosts::HostObjects::new());
    let props = Props::new()
        .with("size", Value::Size(8 * 1024 * 1024))
        .with("slot", Value::Str(String::from("sdx")))
        .with_hosts(alloc::sync::Arc::clone(&hosts));
    let first = CardDevice::new(&props).expect("the socket was empty");
    assert_eq!(first.slot(), "sdx");
    assert!(
        CardDevice::new(&props).is_err(),
        "and a second card has nowhere to go"
    );
    let slot = super::super::slots::get(&hosts, "sdx")
        .expect("no type collision")
        .expect("it was opened");
    assert!(slot.is_occupied());
    assert!(slot.eject().is_some());
    assert!(!slot.is_occupied());
}
