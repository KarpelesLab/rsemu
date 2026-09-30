//! `rcar.sdhi` against a real `sd.card`: the identification sequence a driver
//! walks, single- and multiple-block reads and writes by programmed I/O and by
//! DMA request, the status conventions, and the snapshot.

use super::*;
use crate::dev::sd::card::{BusMode, Identity, IdentityText};

const BLOCK: usize = 512;

fn card() -> Arc<SdCard> {
    let id = Identity::new(
        1024 * 1024,
        true,
        false,
        IdentityText {
            manufacturer: 0x03,
            oem: "RE",
            product: "RSEMU",
            revision: 0x10,
            serial: 1,
            year: 2024,
            month: 1,
        },
    )
    .unwrap();
    let card = Arc::new(SdCard::with_identity(id, BusMode::Sd, 1).unwrap());
    let mut image = alloc::vec![0u8; 4 * BLOCK];
    for (i, b) in image.iter_mut().enumerate() {
        *b = (i / BLOCK) as u8 ^ (i as u8);
    }
    card.load_image(0, &image).unwrap();
    card
}

fn host(with_card: bool) -> (Sdhi, Arc<Slot>) {
    let slot = Arc::new(Slot::new());
    if with_card {
        slot.insert(card()).unwrap();
    }
    let sdhi = Sdhi::with_slot(Arc::clone(&slot), String::from("sd0"));
    (sdhi, slot)
}

fn w16(h: &Sdhi, off: u64, v: u16) {
    h.regs
        .write(off, &v.to_le_bytes(), MemAttrs::DEFAULT)
        .unwrap();
}

fn r16(h: &Sdhi, off: u64) -> u16 {
    let mut b = [0u8; 2];
    h.regs.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
    u16::from_le_bytes(b)
}

fn r32(h: &Sdhi, off: u64) -> u32 {
    let mut b = [0u8; 4];
    h.regs.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
    u32::from_le_bytes(b)
}

/// Clear every flag in both status registers, the way a driver does.
fn ack(h: &Sdhi) {
    w16(h, SD_INFO1, 0);
    w16(h, SD_INFO2, 0);
}

/// Issue `cmd` (the raw `SD_CMD` value) with `arg`, and return the 32-bit
/// response.
fn command(h: &Sdhi, cmd: u16, arg: u32) -> u32 {
    ack(h);
    w16(h, SD_ARG0, arg as u16);
    w16(h, SD_ARG1, (arg >> 16) as u16);
    w16(h, SD_CMD, cmd);
    r32(h, SD_RSP_FIRST)
}

/// The identification sequence, leaving the card selected at 512-byte blocks.
fn bring_up(h: &Sdhi) {
    w16(h, SOFT_RST, 0);
    w16(h, SOFT_RST, 1);
    command(h, 0, 0);
    assert_eq!(
        command(h, 8, 0x1aa) & 0xfff,
        0x1aa,
        "CMD8 echoes the check pattern"
    );
    command(h, 55, 0);
    let ocr = command(h, 0x40 | 41, 0x40ff_8000);
    assert_ne!(ocr & (1 << 31), 0, "powered up");
    command(h, 2, 0);
    assert_eq!(r16(h, SD_INFO1) & INFO1_RSPEND, INFO1_RSPEND);
    let r6 = command(h, 3, 0);
    command(h, 7, r6 & 0xffff_0000);
    w16(h, SD_SIZE, BLOCK as u16);
}

#[test]
fn a_response_lands_in_the_response_registers_and_raises_rspend() {
    let (h, _) = host(true);
    bring_up(&h);
    let status = command(&h, 13, 0x0001_0000);
    assert_eq!((status >> 9) & 0xf, 4, "CMD13 reports the transfer state");
    let info1 = r16(&h, SD_INFO1);
    assert_ne!(info1 & INFO1_RSPEND, 0);
    assert_ne!(
        info1 & INFO1_ACEND,
        0,
        "a command with no data ends at its response"
    );
    assert_ne!(info1 & INFO1_SDCD, 0, "card detect reports the card");
}

#[test]
fn cid_arrives_with_its_crc_shifted_out() {
    let (h, slot) = host(true);
    w16(&h, SOFT_RST, 1);
    command(&h, 0, 0);
    command(&h, 8, 0x1aa);
    command(&h, 55, 0);
    command(&h, 0x40 | 41, 0x40ff_8000);
    command(&h, 2, 0);
    let cid = slot.card().unwrap().identity().cid;
    // SD_RSP76 holds CID[127:104] in its low 24 bits: the manufacturer ID
    // first, then the OEM ID.
    let top = r32(&h, SD_RSP_FIRST + 12);
    assert_eq!(top >> 16 & 0xff, u32::from(cid[0]));
    assert_eq!(top >> 8 & 0xff, u32::from(cid[1]));
}

#[test]
fn an_empty_socket_times_out_and_reads_no_card() {
    let (h, _) = host(false);
    w16(&h, SOFT_RST, 1);
    command(&h, 8, 0x1aa);
    assert_ne!(r16(&h, SD_INFO2) & INFO2_RSPTO, 0);
    assert_eq!(r16(&h, SD_INFO1) & INFO1_SDCD, 0);
}

#[test]
fn a_single_block_reads_through_the_data_port() {
    let (h, _) = host(true);
    bring_up(&h);
    command(&h, 17, 2);
    assert_ne!(r16(&h, SD_INFO2) & INFO2_BRE, 0, "the buffer is full");
    let mut got = Vec::new();
    for _ in 0..BLOCK / 4 {
        got.extend_from_slice(&r32(&h, SD_BUF0).to_le_bytes());
    }
    assert_eq!(got[0], 2);
    assert_eq!(got[5], 2 ^ 5);
    assert_ne!(r16(&h, SD_INFO1) & INFO1_ACEND, 0, "the access has ended");
    assert_eq!(r16(&h, SD_INFO2) & INFO2_BRE, 0);
}

#[test]
fn a_counted_multiple_block_read_stops_itself() {
    let (h, slot) = host(true);
    bring_up(&h);
    w16(&h, SD_STOP, STOP_SEC);
    w16(&h, SD_SECCNT, 2);
    command(&h, 18, 1);
    let mut got = Vec::new();
    for _ in 0..(2 * BLOCK) / 2 {
        got.extend_from_slice(&r16(&h, SD_BUF0).to_le_bytes());
    }
    assert_eq!(got[0], 1);
    assert_eq!(got[BLOCK], 2);
    assert_ne!(r16(&h, SD_INFO1) & INFO1_ACEND, 0);
    // The automatic CMD12 left the card in the transfer state.
    let status = slot.card().unwrap().command(13, 0x0001_0000);
    let Reply::Short { value, .. } = status else {
        panic!("{status:?}")
    };
    assert_eq!((value >> 9) & 0xf, 4);
}

#[test]
fn a_block_write_reaches_the_card() {
    let (h, slot) = host(true);
    bring_up(&h);
    command(&h, 24, 3);
    assert_ne!(r16(&h, SD_INFO2) & INFO2_BWE, 0);
    for i in 0..BLOCK / 2 {
        w16(&h, SD_BUF0, 0xa500 | i as u16);
    }
    assert_ne!(r16(&h, SD_INFO1) & INFO1_ACEND, 0);
    let mut back = [0u8; 2];
    slot.card()
        .unwrap()
        .read_media(3 * BLOCK as u64, &mut back)
        .unwrap();
    assert_eq!(back, [0x00, 0xa5]);
}

#[test]
fn in_dma_mode_the_request_line_follows_the_buffer() {
    let (h, _) = host(true);
    bring_up(&h);
    w16(&h, CC_EXT_MODE, EXT_DMA);
    command(&h, 17, 0);
    assert!(h.regs.outputs().1, "rx request while the buffer holds data");
    for _ in 0..BLOCK / 4 {
        r32(&h, SD_BUF0);
    }
    assert!(!h.regs.outputs().1, "and not once it is drained");
}

#[test]
fn flags_clear_by_writing_zero_and_mask_the_interrupt() {
    let (h, _) = host(true);
    bring_up(&h);
    w16(&h, SD_INFO1_MASK, !INFO1_RSPEND);
    command(&h, 13, 0x0001_0000);
    assert!(h.irq_asserted(), "an unmasked RSPEND interrupts");
    w16(&h, SD_INFO1, !INFO1_RSPEND);
    assert!(!h.irq_asserted(), "writing zero to it clears it");
    assert_ne!(r16(&h, SD_INFO1) & INFO1_ACEND, 0, "and only it");
}

#[test]
fn a_debugger_read_of_the_port_pops_nothing() {
    let (h, _) = host(true);
    bring_up(&h);
    command(&h, 17, 1);
    let mut b = [0u8; 4];
    h.regs.read(SD_BUF0, &mut b, MemAttrs::DEBUG).unwrap();
    h.regs.read(SD_BUF0, &mut b, MemAttrs::DEBUG).unwrap();
    assert_eq!(
        r32(&h, SD_BUF0) & 0xff,
        1,
        "the first byte is still the first"
    );
}

#[test]
fn the_snapshot_round_trips_mid_transfer() {
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    let (h, _) = host(true);
    bring_up(&h);
    command(&h, 17, 2);
    r32(&h, SD_BUF0);
    let save = |dev: &Sdhi| {
        let mut shape = MachineShape::new();
        shape.add_device("sdhi", CLASS_NAME).unwrap();
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer.chunk("sdhi", CLASS_NAME, STATE_VERSION).unwrap();
            dev.save(&mut chunk).unwrap();
        }
        writer.to_vec().unwrap()
    };
    let bytes = save(&h);
    let (other, _) = host(true);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("sdhi", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();
    assert_eq!(save(&other), bytes, "save, load, save is stable");
    assert_eq!(
        r32(&other, SD_BUF0) & 0xff,
        2 ^ 4,
        "the port resumes where it was"
    );
}
