//! The CD-ROM: the packet interface as firmware drives it, and El Torito.
//!
//! Three things live here, and the middle one is the reason for the other two.
//!
//! * **A packet transport.** `PACKET` on the secondary channel, the byte count
//!   limit, the C/D + I/O handshake and a drain loop that takes whatever byte
//!   count the drive announces. Fourteen ports and no SCSI opinions.
//! * **El Torito.** The boot record volume descriptor at logical block 17, the
//!   boot catalog it points at, the validation entry's checksum word and the
//!   default entry's media type — and then either loading an image and jumping
//!   to it, or setting up a diskette emulation and jumping to *that*.
//! * **`INT 13h` for what comes out of it.** A no-emulation boot gets a drive
//!   number in `DL` and reaches the disc through the `INT 13h` extensions with
//!   2048-byte blocks; a diskette emulation gets drive `00h` and reaches a
//!   512-byte-sector image *inside* the disc, four virtual sectors to a
//!   logical block.
//!
//! # Which cases are implemented
//!
//! El Torito 1.0 defines four boot media types in the default entry's byte 1.
//! Two are here:
//!
//! | type | what it is | here |
//! |------|------------|------|
//! | 0 | no emulation | **yes** — the image is loaded at the entry's load segment and entered with `DL` = the CD's drive number |
//! | 1, 2, 3 | 1.2M, 1.44M, 2.88M diskette | **yes** — the image becomes `INT 13h` drive `00h` and the real diskette moves to `01h`, as §"Boot Procedure" says it does |
//! | 4 | hard disk | **no** — declined, and the bootstrap falls through to `INT 18h` rather than loading something it cannot then service |
//!
//! Of the `INT 13h` extensions El Torito adds, **`AH=4Bh` is implemented** —
//! get emulation status without terminating it, which is how a no-emulation
//! loader finds out which drive it came off and where on the disc its image
//! is. `4Ah` (initiate emulation), `4Ch` (initiate and boot), `4Dh` (return
//! boot catalog) and `4Eh` (set hardware configuration) are not: they exist for
//! a loader that wants to *start* an emulation, and nothing this firmware boots
//! does.
//!
//! # Where the CD-ROM is
//!
//! The **secondary channel's master position**, `0x170-0x177` and `0x376`,
//! which is where `machines/pc-at.machine` puts it and where a PC of the period
//! put one. POST probes exactly there, exactly as it probes exactly the primary
//! master for a fixed disk: this firmware is written for one board and says so
//! rather than sweeping four cable positions it knows the contents of.
//!
//! The probe is the **signature**, not the status register. ATA/ATAPI-6 §9.1
//! leaves `0x14` in LBA Mid and `0xEB` in LBA High on a packet device, and a
//! packet device's Status register reads `0x00` at rest (§7.15.6.3 — `DRDY` is
//! not a bit it has), which is indistinguishable from an empty cable. A probe
//! that looked at status would find nothing on a board that has a CD-ROM in it.
//!
//! # Interrupts
//!
//! None. IRQ 15 stays masked and every wait here is a bounded poll, exactly as
//! the fixed-disk and diskette paths are: `nIEN` is set in the device control
//! register at probe time so the drive does not assert `INTRQ` at all.
//!
//! # Sources
//!
//! * **T13, ATA/ATAPI-6 (T13/1410D)** — §7.15 (the register file as a packet
//!   device uses it), §8.21 (`PACKET` and its byte count limit), §9.1 (the
//!   reset signature) and §9.10 (the packet command protocol).
//! * **SFF-8020i** — `READ(10)` and `READ CD-ROM CAPACITY`.
//! * **"El Torito" Bootable CD-ROM Format Specification, Version 1.0**
//!   (Phoenix Technologies and IBM, 25 January 1995) — the boot record volume
//!   descriptor, the boot catalog's validation and initial/default entries, the
//!   boot procedure for each media type, and the `INT 13h AH=4Bh` specification
//!   packet.
//! * **ISO 9660** — only that a volume descriptor begins with a type byte and
//!   the standard identifier `CD001`, which is all El Torito needs of it.
//!
//! **No emulator source and no other firmware's source was consulted**
//! (`CLAUDE.md`, provenance).

use super::{
    EBDA_CD_CAPACITY, EBDA_CD_CDB, EBDA_CD_CYLS, EBDA_CD_DONE, EBDA_CD_DRIVE, EBDA_CD_FLAGS,
    EBDA_CD_GUARD, EBDA_CD_HEADS, EBDA_CD_LBA, EBDA_CD_LEFT, EBDA_CD_SENSE, EBDA_CD_SPEC,
    EBDA_CD_SPT, EBDA_CD_TRIES, EBDA_CD_VLBA, EBDA_HD_FLAGS, EL_TORITO_BUFFER, EMULATED_SECTOR,
    F_AX, F_BX, F_CX, F_DS, F_DX, F_ES, F_SI, Labels, SEGMENT, clear_cf, ds_ebda, load_seg,
};
use crate::fw::asm16::{
    AH, AL, AX, Alu, Asm, BH, BL, BX, CH, CL, CX, Cc, DH, DI, DL, DS, DX, ES, Mem, SI, Shift,
};

/// The secondary channel's command block, where the CD-ROM lives.
const CD_BASE: u16 = 0x0170;
/// Its control block: device control on a write, alternate status on a read.
const CD_CTL: u16 = 0x0376;

/// The Device register value that selects device 0 on that cable.
const CD_SELECT: u8 = 0xa0;

/// `PACKET`, T13/1410D §8.21.
const ATA_PACKET: u8 = 0xa0;

/// `READ(10)`, SFF-8020i.
const SCSI_READ_10: u8 = 0x28;
/// `READ CD-ROM CAPACITY`.
const SCSI_READ_CAPACITY: u8 = 0x25;

/// The El Torito specification packet's length, and its first byte.
const SPEC_PACKET_LEN: u8 = 0x13;

/// Where the boot record volume descriptor is: ISO 9660 puts the descriptor
/// set at logical block 16 and El Torito puts its own at 17.
const BOOT_RECORD_BLOCK: u32 = 17;

/// Emit the CD-ROM transport, `INT 13h`'s CD paths and the El Torito
/// bootstrap.
#[allow(clippy::too_many_lines)]
pub(super) fn emit(a: &mut Asm, l: &Labels) {
    detect(a, l);
    transport(a, l);
    int13(a, l);
    boot(a, l);
}

// ---------------------------------------------------------------------------
// POST: is there one?
// ---------------------------------------------------------------------------

/// The probe, called from POST.
///
/// Leaves [`EBDA_CD_FLAGS`] bit 0 set and [`EBDA_CD_DRIVE`] holding the
/// `INT 13h` drive number if a packet device answered. The number is the one
/// after the fixed disks — `0x80` on a board with no hard disk and `0x81` on
/// one with the single hard disk this firmware detects — which is what El
/// Torito means by "the BIOS assigns the drive number".
fn detect(a: &mut Asm, l: &Labels) {
    a.bind(l.cd_detect);
    a.push(AX);
    a.push(DX);
    a.pushs(DS);
    ds_ebda(a);
    a.movmi8(Mem::abs(EBDA_CD_FLAGS), 0);

    let none = a.label();

    // nIEN: this firmware polls, and IRQ 15 stays masked at the 8259A.
    a.movi(DX, CD_CTL);
    a.movi8(AL, 0x02);
    a.out_dx_al();
    // Select device 0 on the cable. Every device on it sees this write and
    // decides for itself whether it is being addressed.
    a.movi(DX, CD_BASE + 6);
    a.movi8(AL, CD_SELECT);
    a.out_dx_al();
    // The signature, ATA/ATAPI-6 §9.1. A status read would be useless here:
    // 0x00 is both "a packet device at rest" and "nothing on the cable".
    a.movi(DX, CD_BASE + 4);
    a.in_al_dx();
    a.alui8(Alu::CMP, AL, 0x14);
    a.jcc(Cc::NE, none);
    a.movi(DX, CD_BASE + 5);
    a.in_al_dx();
    a.alui8(Alu::CMP, AL, 0xeb);
    a.jcc(Cc::NE, none);

    a.movmi8(Mem::abs(EBDA_CD_FLAGS), 0x01);
    let assign = a.label();
    a.movi8(AL, 0x80);
    a.testi8(Mem::abs(EBDA_HD_FLAGS), 0x01);
    a.jcc(Cc::E, assign);
    a.movi8(AL, 0x81);
    a.bind(assign);
    a.movto8(Mem::abs(EBDA_CD_DRIVE), AL);
    // Collect the power-on unit attention now rather than leaving it for the
    // first thing that wants bytes. The answer is deliberately ignored: a drive
    // with no disc in it is not ready and is still a drive.
    a.call(l.cd_ready);

    a.bind(none);
    a.pops(DS);
    a.pop(DX);
    a.pop(AX);
    a.ret();
}

// ---------------------------------------------------------------------------
// The packet transport
// ---------------------------------------------------------------------------

/// `cd_wait_ready`, `cd_packet`, the drain loop, `cd_read`, `cd_read_sector`
/// and `READ CD-ROM CAPACITY`.
#[allow(clippy::too_many_lines)]
fn transport(a: &mut Asm, l: &Labels) {
    // -- cd_wait_ready -------------------------------------------------------
    //
    // Spin until `BSY` clears. Bounded, so a cable with nothing on it costs
    // 65,536 port reads rather than the machine — the same bound and the same
    // reason as the fixed disk's.
    a.bind(l.cd_wait_ready);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    let r_ok = a.label();
    a.movi(CX, 0);
    a.movi(DX, CD_BASE + 7);
    let r_poll = a.here_label();
    a.in_al_dx();
    a.testi8(AL, 0x80);
    a.jcc(Cc::E, r_ok);
    a.dec(CX);
    a.jcc(Cc::NE, r_poll);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.stc();
    a.ret();
    a.bind(r_ok);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.clc();
    a.ret();

    // -- cd_packet -----------------------------------------------------------
    //
    // Deliver the twelve-byte command descriptor block at [`EBDA_CD_CDB`] with
    // `CX` as the byte count limit. `DS` must be the EBDA, because `REP OUTSW`
    // reads `DS:SI` and that is where the packet is.
    //
    // ATA/ATAPI-6 §9.10, steps 1 to 3: features, the byte count limit, the
    // device, `PACKET`, then wait for `DRQ` with `C/D` set and write six words.
    // There is deliberately no wait on `INTRQ` anywhere: `IDENTIFY PACKET
    // DEVICE` word 0 bits 6:5 report microprocessor DRQ, which is the promise
    // that a host may poll for this phase, and nIEN is set anyway.
    a.bind(l.cd_packet);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    a.push(SI);
    let p_fail = a.label();
    let p_done = a.label();

    a.movi(DX, CD_BASE + 6);
    a.movi8(AL, CD_SELECT);
    a.out_dx_al();
    a.call(l.cd_wait_ready);
    a.jcc(Cc::B, p_fail);

    a.movi(DX, CD_BASE + 1); // Features: no DMA, no overlap
    a.movi8(AL, 0x00);
    a.out_dx_al();
    a.movi(DX, CD_BASE + 4); // Byte Count, low
    a.mov8(AL, CL);
    a.out_dx_al();
    a.movi(DX, CD_BASE + 5); // Byte Count, high
    a.mov8(AL, CH);
    a.out_dx_al();
    a.movi(DX, CD_BASE + 7);
    a.movi8(AL, ATA_PACKET);
    a.out_dx_al();

    a.call(l.cd_wait_ready);
    a.jcc(Cc::B, p_fail);
    a.movi(DX, CD_BASE + 7);
    a.in_al_dx();
    a.testi8(AL, 0x08); // DRQ: the device is asking for the packet
    a.jcc(Cc::E, p_fail);

    a.movi(SI, EBDA_CD_CDB);
    a.movi(CX, 6);
    a.movi(DX, CD_BASE);
    a.rep();
    a.outsw();
    a.clc();
    a.jmp(p_done);
    a.bind(p_fail);
    a.stc();
    a.bind(p_done);
    a.pop(SI);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- the drain -----------------------------------------------------------
    //
    // Take every data block the device offers into `ES:DI` and stop when it
    // stops offering. §9.10 step 4: a block is announced by `DRQ` with `I/O`
    // set and the **actual** byte count in the two Byte Count registers, and
    // completion is `DRQ` clear — with `CHK` set if the command failed, in
    // which case the sense is waiting and this firmware simply reports a
    // failed `INT 13h` rather than asking for it.
    //
    // The guard is not decoration. The loop's termination depends on the
    // device eventually clearing `DRQ`, which is a statement about a device
    // rather than about this code, and firmware that hung on a disc would be
    // worse than firmware that declined one.
    let drain = a.label();
    a.bind(drain);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    let d_ok = a.label();
    let d_fail = a.label();
    let d_out = a.label();
    a.movmi(Mem::abs(EBDA_CD_GUARD), 0x1000);
    let d_next = a.here_label();
    a.decm(Mem::abs(EBDA_CD_GUARD));
    a.jcc(Cc::E, d_fail);
    a.call(l.cd_wait_ready);
    a.jcc(Cc::B, d_fail);
    a.movi(DX, CD_BASE + 7);
    a.in_al_dx();
    a.testi8(AL, 0x01); // CHK
    a.jcc(Cc::NE, d_fail);
    a.testi8(AL, 0x08); // DRQ
    a.jcc(Cc::E, d_ok);
    a.movi(DX, CD_BASE + 4);
    a.in_al_dx();
    a.mov8(CL, AL);
    a.movi(DX, CD_BASE + 5);
    a.in_al_dx();
    a.mov8(CH, AL);
    a.shift(Shift::SHR, CX, 1);
    a.jcc(Cc::E, d_ok);
    a.movi(DX, CD_BASE);
    a.rep();
    a.insw();
    a.jmp(d_next);
    a.bind(d_ok);
    a.clc();
    a.jmp(d_out);
    a.bind(d_fail);
    a.stc();
    a.bind(d_out);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- cd_read -------------------------------------------------------------
    //
    // `READ(10)` of `CX` logical blocks from [`EBDA_CD_LBA`] into `ES:BX`.
    // The byte count limit is a whole block, so the drive hands the blocks over
    // one at a time and the drain copies each straight into the caller's
    // buffer — there is no staging buffer anywhere in this file's `INT 13h`
    // paths, which is what keeps them safe to call once a guest owns memory.
    a.bind(l.cd_read);
    a.push(AX);
    a.push(CX);
    a.push(DI);
    let rd_fail = a.label();
    let rd_out = a.label();
    a.movmi8(Mem::abs(EBDA_CD_CDB), SCSI_READ_10);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 1), 0);
    // The logical block address, big-endian, which is how every SCSI command
    // descriptor block carries one and the opposite of how an x86 holds it.
    a.mov8(AL, Mem::abs(EBDA_CD_LBA + 3));
    a.movto8(Mem::abs(EBDA_CD_CDB + 2), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_LBA + 2));
    a.movto8(Mem::abs(EBDA_CD_CDB + 3), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_LBA + 1));
    a.movto8(Mem::abs(EBDA_CD_CDB + 4), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_LBA));
    a.movto8(Mem::abs(EBDA_CD_CDB + 5), AL);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 6), 0);
    a.movto8(Mem::abs(EBDA_CD_CDB + 7), CH);
    a.movto8(Mem::abs(EBDA_CD_CDB + 8), CL);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 9), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 10), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 11), 0);
    a.movi(CX, super::CD_BLOCK);
    a.call(l.cd_packet);
    a.jcc(Cc::B, rd_fail);
    a.mov(DI, BX);
    a.call(drain);
    a.jcc(Cc::B, rd_fail);
    a.clc();
    a.jmp(rd_out);
    a.bind(rd_fail);
    a.stc();
    a.bind(rd_out);
    a.pop(DI);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- cd_read_sector ------------------------------------------------------
    //
    // One 512-byte *virtual* sector out of the 2048-byte logical block at
    // [`EBDA_CD_LBA`]: `AL` says which quarter, and the 512 bytes land at
    // `ES:BX`.
    //
    // This is diskette emulation's whole transfer, and it is done by setting
    // the byte count limit to 512 and taking the wanted chunk. ATA/ATAPI-6
    // §8.21.5 makes the limit binding — the device transfers the lesser of the
    // limit and what is left — so a 2048-byte block arrives as exactly four
    // 512-byte chunks and the quarter index *is* the chunk index. The
    // alternative was a 2 KiB staging buffer somewhere in a guest's memory,
    // and there is nowhere in a guest's memory that is ours.
    //
    // If the device were to chunk it differently the wanted chunk would not be
    // a whole sector, and this fails rather than copying the wrong bytes.
    a.bind(l.cd_read_sector);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    a.push(SI);
    a.push(DI);
    let rs_fail = a.label();
    let rs_out = a.label();
    let rs_end = a.label();
    a.alui(Alu::AND, AX, 3);
    a.mov(SI, AX); // chunks still to skip
    a.movmi8(Mem::abs(EBDA_CD_CDB), SCSI_READ_10);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 1), 0);
    a.mov8(AL, Mem::abs(EBDA_CD_LBA + 3));
    a.movto8(Mem::abs(EBDA_CD_CDB + 2), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_LBA + 2));
    a.movto8(Mem::abs(EBDA_CD_CDB + 3), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_LBA + 1));
    a.movto8(Mem::abs(EBDA_CD_CDB + 4), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_LBA));
    a.movto8(Mem::abs(EBDA_CD_CDB + 5), AL);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 6), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 7), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 8), 1);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 9), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 10), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 11), 0);
    a.movi(CX, EMULATED_SECTOR);
    a.call(l.cd_packet);
    a.jcc(Cc::B, rs_fail);
    a.mov(DI, BX);
    a.movmi(Mem::abs(EBDA_CD_GUARD), 0x20);
    let rs_chunk = a.here_label();
    let rs_skip = a.label();
    let rs_discard = a.label();
    a.decm(Mem::abs(EBDA_CD_GUARD));
    a.jcc(Cc::E, rs_fail);
    a.call(l.cd_wait_ready);
    a.jcc(Cc::B, rs_fail);
    a.movi(DX, CD_BASE + 7);
    a.in_al_dx();
    a.testi8(AL, 0x01);
    a.jcc(Cc::NE, rs_fail);
    a.testi8(AL, 0x08);
    a.jcc(Cc::E, rs_end);
    a.movi(DX, CD_BASE + 4);
    a.in_al_dx();
    a.mov8(CL, AL);
    a.movi(DX, CD_BASE + 5);
    a.in_al_dx();
    a.mov8(CH, AL);
    a.shift(Shift::SHR, CX, 1);
    a.jcc(Cc::E, rs_fail);
    a.movi(DX, CD_BASE);
    a.alui(Alu::CMP, SI, 0);
    a.jcc(Cc::NE, rs_skip);
    // The wanted chunk. It has to be a whole virtual sector.
    a.alui(Alu::CMP, CX, EMULATED_SECTOR / 2);
    a.jcc(Cc::NE, rs_fail);
    a.rep();
    a.insw();
    // Never match again: what is left of the command still has to be drained,
    // because a device holding DRQ up owns the command block until it is.
    a.movi(SI, 0x00ff);
    a.jmp(rs_chunk);
    a.bind(rs_skip);
    a.dec(SI);
    a.bind(rs_discard);
    a.in_ax_dx();
    a.loop_(rs_discard);
    a.jmp(rs_chunk);
    // The command ended. It moved a sector if and only if `DI` walked one.
    a.bind(rs_end);
    a.mov(AX, DI);
    a.alu(Alu::SUB, AX, BX);
    a.alui(Alu::CMP, AX, EMULATED_SECTOR);
    a.jcc(Cc::NE, rs_fail);
    a.clc();
    a.jmp(rs_out);
    a.bind(rs_fail);
    a.stc();
    a.bind(rs_out);
    a.pop(DI);
    a.pop(SI);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- cd_ready ------------------------------------------------------------
    //
    // `TEST UNIT READY` until the drive says it is, which is the first thing
    // any host says to a packet device and the reason it is here: a drive that
    // has just been powered on owes the host a **unit attention** (SFF-8020i
    // §9.3), and the command that collects it fails. A firmware that went
    // straight to `READ(10)` would be told `CHECK CONDITION` on a disc that is
    // perfectly readable, and would conclude the disc was not bootable.
    //
    // The `REQUEST SENSE` is not there only to be polite. It is what separates
    // "something changed, ask again" from "there is no disc in the drive": a
    // sense key of `UNIT ATTENTION` is worth a retry and `NOT READY` is not,
    // and a loop that retried both would spin four times over an empty tray on
    // every boot.
    a.bind(l.cd_ready);
    a.push(AX);
    a.push(CX);
    a.push(DI);
    a.pushs(ES);
    let ready_ok = a.label();
    let ready_fail = a.label();
    let ready_out = a.label();
    a.movmi8(Mem::abs(EBDA_CD_TRIES), 4);
    let ready_try = a.here_label();
    for at in 0..12u16 {
        a.movmi8(Mem::abs(EBDA_CD_CDB + at), 0);
    }
    a.movi(CX, 8);
    a.call(l.cd_packet);
    a.jcc(Cc::B, ready_fail);
    a.call(l.cd_wait_ready);
    a.jcc(Cc::B, ready_fail);
    a.movi(DX, CD_BASE + 7);
    a.in_al_dx();
    a.testi8(AL, 0x01);
    a.jcc(Cc::E, ready_ok);

    a.movmi8(Mem::abs(EBDA_CD_CDB), 0x03); // REQUEST SENSE
    a.movmi8(Mem::abs(EBDA_CD_CDB + 1), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 2), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 3), 0);
    a.movmi8(Mem::abs(EBDA_CD_CDB + 4), 18);
    for at in 5..12u16 {
        a.movmi8(Mem::abs(EBDA_CD_CDB + at), 0);
    }
    a.movi(CX, 18);
    a.call(l.cd_packet);
    a.jcc(Cc::B, ready_fail);
    a.movrs(AX, DS);
    a.movsr(ES, AX);
    a.movi(DI, EBDA_CD_SENSE);
    a.call(drain);
    a.jcc(Cc::B, ready_fail);
    a.mov8(AL, Mem::abs(EBDA_CD_SENSE + 2));
    a.alui8(Alu::AND, AL, 0x0f);
    a.alui8(Alu::CMP, AL, 0x06); // UNIT ATTENTION
    a.jcc(Cc::NE, ready_fail);
    a.decm8(Mem::abs(EBDA_CD_TRIES));
    a.jcc(Cc::NE, ready_try);
    a.bind(ready_fail);
    a.stc();
    a.jmp(ready_out);
    a.bind(ready_ok);
    a.clc();
    a.bind(ready_out);
    a.pops(ES);
    a.pop(DI);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- read capacity -------------------------------------------------------
    //
    // `READ CD-ROM CAPACITY` into [`EBDA_CD_CAPACITY`], where `AH=48h` finds
    // the disc's size. Eight bytes: the **last** logical block and the block
    // length, both big-endian.
    a.bind(l.cd_capacity);
    a.push(AX);
    a.push(CX);
    a.push(DI);
    a.pushs(ES);
    let cap_fail = a.label();
    let cap_out = a.label();
    for (at, byte) in [
        (0u16, SCSI_READ_CAPACITY),
        (1, 0),
        (2, 0),
        (3, 0),
        (4, 0),
        (5, 0),
        (6, 0),
        (7, 0),
        (8, 0),
        (9, 0),
        (10, 0),
        (11, 0),
    ] {
        a.movmi8(Mem::abs(EBDA_CD_CDB + at), byte);
    }
    a.movi(CX, 8);
    a.call(l.cd_packet);
    a.jcc(Cc::B, cap_fail);
    a.movrs(AX, DS);
    a.movsr(ES, AX);
    a.movi(DI, EBDA_CD_CAPACITY);
    a.call(drain);
    a.jcc(Cc::B, cap_fail);
    a.clc();
    a.jmp(cap_out);
    a.bind(cap_fail);
    a.stc();
    a.bind(cap_out);
    a.pops(ES);
    a.pop(DI);
    a.pop(CX);
    a.pop(AX);
    a.ret();
}

// ---------------------------------------------------------------------------
// INT 13h
// ---------------------------------------------------------------------------

/// The CD drive's `INT 13h` functions, and the emulated diskette's.
#[allow(clippy::too_many_lines)]
fn int13(a: &mut Asm, l: &Labels) {
    // -- the CD-ROM as a drive number ----------------------------------------
    //
    // Reached from `INT 13h`'s dispatch when `DL` is the number POST assigned.
    // `AH` is already loaded and `DS` is already the EBDA.
    a.bind(l.cd_int13);
    let ext_check = a.label();
    let ext_read = a.label();
    let ext_params = a.label();
    let emulation_status = a.label();
    let write_protected = a.label();
    for (function, target) in [
        (0x00u8, l.disk_ok), // reset: nothing to reset
        (0x01, l.disk_ok),   // last status
        (0x04, l.disk_ok),   // verify: nothing to compare against
        (0x41, ext_check),
        (0x42, ext_read),
        (0x43, write_protected), // extended write: a CD-ROM is read-only
        (0x44, l.disk_ok),       // extended verify
        (0x47, l.disk_ok),       // extended seek: there is no head to move
        (0x48, ext_params),
        (0x4b, emulation_status),
    ] {
        a.alui8(Alu::CMP, AH, function);
        a.jcc(Cc::E, target);
    }
    // Everything else, and that deliberately includes AH=02h and AH=08h: a
    // CD-ROM has no cylinder/head/sector geometry, and inventing one would
    // hand a caller an address space that is not the disc's.
    a.jmp(l.disk_fail);

    // AH=43h on a read-only medium. 03h is "write protected", which is a
    // different answer from "no such function" and is the one that is true.
    a.bind(write_protected);
    a.movmi8(Mem::bp(F_AX + 1), 0x03);
    super::set_cf(a);
    a.jmp(l.disk_done);

    // AH=41h. The same fixed-disk access subset the hard disk claims — 42h,
    // 43h, 44h, 47h and 48h — and all five are above. 43h answers "write
    // protected" rather than being absent, which is the distinction the
    // fixed disk's comment was written about: a claimed bit with a missing
    // function behind it is what broke FreeDOS's FDISK.
    a.bind(ext_check);
    a.mov(AX, Mem::bp(F_BX));
    a.alui(Alu::CMP, AX, 0x55aa);
    a.jcc(Cc::NE, l.disk_fail);
    a.movmi(Mem::bp(F_BX), 0xaa55);
    a.movmi(Mem::bp(F_CX), 0x0001);
    a.movmi8(Mem::bp(F_AX + 1), 0x21);
    clear_cf(a);
    a.jmp(l.disk_done);

    // AH=42h, extended read. The disk address packet's block count is in the
    // drive's own sector size, which for a CD-ROM is 2048 bytes — that is what
    // makes this function usable on a disc at all and why El Torito requires
    // the extensions rather than CHS.
    a.bind(ext_read);
    a.pushs(DS);
    a.movsr(DS, Mem::bp(F_DS));
    a.mov(SI, Mem::bp(F_SI));
    a.mov(CX, Mem::si(2));
    a.mov(DI, Mem::si(4));
    a.mov(AX, Mem::si(6));
    a.mov(BX, Mem::si(8));
    a.mov(DX, Mem::si(10));
    a.pops(DS);
    a.movto(Mem::abs(EBDA_CD_LBA), BX);
    a.movto(Mem::abs(EBDA_CD_LBA + 2), DX);
    a.movsr(ES, AX);
    a.mov(BX, DI);
    a.alui(Alu::CMP, CX, 0);
    a.jcc(Cc::E, l.disk_ok);
    a.call(l.cd_read);
    a.jcc(Cc::B, l.disk_fail);
    a.jmp(l.disk_ok);

    // AH=48h, get drive parameters. The same EDD 1.1 table the fixed disk
    // fills, with three differences that are the whole point of it: the
    // information flags say removable, change-line capable and lockable (bits
    // 2, 4 and 5) and do **not** say the CHS geometry is valid, the sector
    // size is 2048 rather than 512, and the total is the disc's own block
    // count out of `READ CD-ROM CAPACITY` rather than a geometry product.
    a.bind(ext_params);
    a.call(l.cd_capacity);
    a.jcc(Cc::B, l.disk_fail);
    a.movsr(ES, Mem::bp(F_DS));
    a.mov(DI, Mem::bp(F_SI));
    a.mov(AX, Mem::di(0).seg(ES));
    a.alui(Alu::CMP, AX, 0x1a);
    a.jcc(Cc::B, l.disk_fail);
    a.movmi(Mem::di(0).seg(ES), 0x1a);
    a.movmi(Mem::di(2).seg(ES), 0x0034);
    a.movmi32(Mem::di(4).seg(ES), 0);
    a.movmi32(Mem::di(8).seg(ES), 0);
    a.movmi32(Mem::di(0x0c).seg(ES), 0);
    // The response is big-endian and the table is little-endian, so the four
    // bytes go back the other way round; the count is the last block plus one.
    a.mov8(AL, Mem::abs(EBDA_CD_CAPACITY + 3));
    a.movto8(Mem::di(0x10).seg(ES), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_CAPACITY + 2));
    a.movto8(Mem::di(0x11).seg(ES), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_CAPACITY + 1));
    a.movto8(Mem::di(0x12).seg(ES), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_CAPACITY));
    a.movto8(Mem::di(0x13).seg(ES), AL);
    a.incm32(Mem::di(0x10).seg(ES));
    a.movmi32(Mem::di(0x14).seg(ES), 0);
    a.movmi(Mem::di(0x18).seg(ES), super::CD_BLOCK);
    a.jmp(l.disk_ok);

    // AH=4Bh, El Torito's "terminate disk emulation". `AL=01h` asks for the
    // status *without* terminating anything, which is the only form here: a
    // loader that has been given a drive number and wants to know where on the
    // disc it came from calls this, and nothing this firmware boots wants the
    // emulation taken away underneath it. The nineteen-byte packet goes to the
    // caller's `DS:SI`.
    a.bind(emulation_status);
    a.mov8(AL, Mem::bp(F_AX));
    a.alui8(Alu::CMP, AL, 0x01);
    a.jcc(Cc::NE, l.disk_fail);
    a.movsr(ES, Mem::bp(F_DS));
    a.mov(DI, Mem::bp(F_SI));
    a.movi(SI, EBDA_CD_SPEC);
    a.movi(CX, u16::from(SPEC_PACKET_LEN));
    a.rep();
    a.movsb();
    a.jmp(l.disk_ok);

    // -- the emulated diskette -----------------------------------------------
    //
    // Reached when El Torito started a diskette emulation and the caller asked
    // for drive 00h. The image is inside the disc, four 512-byte virtual
    // sectors to a logical block.
    a.bind(l.cd_emu_int13);
    let emu_read = a.label();
    let emu_params = a.label();
    let emu_kind = a.label();
    let emu_protected = a.label();
    for (function, target) in [
        (0x00u8, l.disk_ok), // reset
        (0x01, l.disk_ok),   // last status
        (0x02, emu_read),
        (0x03, emu_protected), // write
        (0x04, l.disk_ok),     // verify
        (0x05, emu_protected), // format
        (0x08, emu_params),
        (0x15, emu_kind),
        (0x16, l.disk_ok), // disk change: an image in a disc cannot change
    ] {
        a.alui8(Alu::CMP, AH, function);
        a.jcc(Cc::E, target);
    }
    a.jmp(l.disk_fail);

    // 03h "write protected" again, and for the same reason.
    a.bind(emu_protected);
    a.movmi8(Mem::bp(F_AX + 1), 0x03);
    super::set_cf(a);
    a.jmp(l.disk_done);

    // AH=15h. Type 1 is "diskette, no change line", which is the truth about
    // an image that is part of a read-only disc.
    a.bind(emu_kind);
    a.movmi8(Mem::bp(F_AX + 1), 0x01);
    clear_cf(a);
    a.jmp(l.disk_done);

    // AH=08h, diskette parameters, out of the geometry El Torito's media type
    // implied.
    a.bind(emu_params);
    a.mov(AX, Mem::abs(EBDA_CD_CYLS));
    a.dec(AX);
    a.mov8(CH, AL);
    a.mov8(CL, AH);
    a.shift8(Shift::SHL, CL, 6);
    a.mov8(AL, Mem::abs(EBDA_CD_SPT));
    a.alui8(Alu::AND, AL, 0x3f);
    a.aluto8(Alu::OR, CL, AL);
    a.movto(Mem::bp(F_CX), CX);
    a.mov8(AL, Mem::abs(EBDA_CD_HEADS));
    a.dec(AX);
    a.movto8(Mem::bp(F_DX + 1), AL);
    a.movmi8(Mem::bp(F_DX), 1); // one diskette drive, the emulated one
    a.movmi8(Mem::bp(F_BX), 0x04); // BL: 1.44 MB, which every emulation here is
    a.jmp(l.disk_ok);

    // AH=02h. One virtual sector at a time, because each one is a quarter of a
    // logical block and the block a run of them sits in changes underneath it.
    a.bind(emu_read);
    let emu_loop = a.label();
    a.mov(CX, Mem::bp(F_CX));
    a.mov(DX, Mem::bp(F_DX));
    a.call(l.cd_chs);
    a.movto(Mem::abs(EBDA_CD_VLBA), AX);
    a.mov(AX, Mem::bp(F_AX));
    a.alui(Alu::AND, AX, 0x00ff);
    a.jcc(Cc::E, l.disk_ok);
    a.movto8(Mem::abs(EBDA_CD_LEFT), AL);
    a.movmi8(Mem::abs(EBDA_CD_DONE), 0);
    a.mov(BX, Mem::bp(F_BX));
    a.movsr(ES, Mem::bp(F_ES));
    a.bind(emu_loop);
    a.mov(AX, Mem::abs(EBDA_CD_VLBA));
    a.mov(CX, AX);
    a.alui(Alu::AND, CX, 3);
    a.shift(Shift::SHR, AX, 2);
    a.movmi32(Mem::abs(EBDA_CD_LBA), 0);
    a.movto(Mem::abs(EBDA_CD_LBA), AX);
    a.mov32(AX, Mem::abs(EBDA_CD_SPEC + 4));
    a.alu32(Alu::ADD, AX, Mem::abs(EBDA_CD_LBA));
    a.movto32(Mem::abs(EBDA_CD_LBA), AX);
    a.mov(AX, CX);
    a.call(l.cd_read_sector);
    a.jcc(Cc::B, l.disk_fail);
    a.alui(Alu::ADD, BX, EMULATED_SECTOR);
    a.incm(Mem::abs(EBDA_CD_VLBA));
    a.incm8(Mem::abs(EBDA_CD_DONE));
    a.decm8(Mem::abs(EBDA_CD_LEFT));
    a.jcc(Cc::NE, emu_loop);
    a.mov8(AL, Mem::abs(EBDA_CD_DONE));
    a.movto8(Mem::bp(F_AX), AL);
    a.jmp(l.disk_ok);

    // -- chs_to_virtual ------------------------------------------------------
    //
    // `CX` and `DX` as `INT 13h` packs a cylinder/head/sector triple; out comes
    // the 512-byte virtual sector number, against the *emulated* geometry:
    //
    //   sector = (cylinder x heads + head) x sectors_per_track + (sector - 1)
    //
    // Sixteen-bit arithmetic throughout, which is enough: the largest
    // emulation El Torito defines is a 2.88 MB diskette, 5,760 sectors.
    a.bind(l.cd_chs);
    a.push(BX);
    a.push(CX);
    a.push(DX);
    a.mov8(BL, CL);
    a.alui8(Alu::AND, BL, 0x3f);
    // The head goes into `BH` **before** the first multiply, and that is the
    // whole reason this routine has a spare register in it: `MUL r/m16` writes
    // its high half to `DX`, so the head would be gone by the time it was
    // wanted (SDM Vol 2A, `MUL`).
    a.mov8(BH, DH);
    a.mov8(AL, CH);
    a.shift8(Shift::SHR, CL, 6);
    a.mov8(AH, CL);
    a.mov8(CL, Mem::abs(EBDA_CD_HEADS));
    a.movi8(CH, 0);
    a.mul(CX);
    a.mov8(CL, BH);
    a.movi8(CH, 0);
    a.alu(Alu::ADD, AX, CX);
    a.mov8(CL, Mem::abs(EBDA_CD_SPT));
    a.movi8(CH, 0);
    a.mul(CX);
    a.mov8(CL, BL);
    a.movi8(CH, 0);
    a.alu(Alu::ADD, AX, CX);
    a.dec(AX);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.ret();
}

// ---------------------------------------------------------------------------
// El Torito
// ---------------------------------------------------------------------------

/// The bootstrap, called from `INT 19h` after the diskette and the fixed disk
/// have declined.
///
/// Returns — with everything untouched — if there is no CD-ROM, no boot record,
/// no catalog, a catalog that does not validate, an entry that is not bootable,
/// or a media type this firmware does not implement. `INT 19h` then falls
/// through to `INT 18h`, which is what a board with nothing to boot does.
#[allow(clippy::too_many_lines)]
fn boot(a: &mut Asm, l: &Labels) {
    a.bind(l.cd_boot);
    let no = a.label();
    let no_restore = a.label();
    let match_rom = a.label();
    let banner = a.label();
    let cd001 = a.label();
    let eltorito = a.label();

    a.pushs(DS);
    ds_ebda(a);
    a.testi8(Mem::abs(EBDA_CD_FLAGS), 0x01);
    a.jcc(Cc::E, no);
    // An empty tray, or a disc that has been swapped since POST, is reported
    // here rather than as an unreadable boot record.
    a.call(l.cd_ready);
    a.jcc(Cc::B, no);

    // -- the boot record volume descriptor, at logical block 17 --------------
    a.movmi32(Mem::abs(EBDA_CD_LBA), BOOT_RECORD_BLOCK);
    a.movi(CX, 1);
    a.movi(AX, 0);
    a.movsr(ES, AX);
    a.movi(BX, EL_TORITO_BUFFER);
    a.call(l.cd_read);
    a.jcc(Cc::B, no);
    // Byte 0 is the volume descriptor type and a boot record is type 0; byte 6
    // is the descriptor version, which El Torito fixes at 1.
    a.alui8(Alu::CMP, Mem::abs(EL_TORITO_BUFFER).seg(ES), 0);
    a.jcc(Cc::NE, no);
    a.alui8(Alu::CMP, Mem::abs(EL_TORITO_BUFFER + 6).seg(ES), 1);
    a.jcc(Cc::NE, no);
    // `CD001` is ISO 9660's standard identifier and `EL TORITO SPECIFICATION`
    // is what makes this descriptor El Torito's rather than somebody else's
    // boot record. Both are compared against strings in the ROM, so `DS` moves
    // to this segment for the duration.
    load_seg(a, DS, SEGMENT, AX);
    a.movi_label(SI, cd001);
    a.movi(DI, EL_TORITO_BUFFER + 1);
    a.movi(CX, 5);
    a.call(match_rom);
    a.jcc(Cc::B, no_restore);
    a.movi_label(SI, eltorito);
    a.movi(DI, EL_TORITO_BUFFER + 7);
    a.movi(CX, 23);
    a.call(match_rom);
    a.jcc(Cc::B, no_restore);
    ds_ebda(a);

    // -- the boot catalog ----------------------------------------------------
    a.mov32(AX, Mem::abs(EL_TORITO_BUFFER + 0x47).seg(ES));
    a.movto32(Mem::abs(EBDA_CD_LBA), AX);
    a.movi(CX, 1);
    a.movi(BX, EL_TORITO_BUFFER);
    a.call(l.cd_read);
    a.jcc(Cc::B, no);
    // The validation entry: header ID 1, and the key bytes 55h AAh that end it.
    // The checksum word is not verified, and that is a decision rather than an
    // omission — it is a sixteen-bit sum over a structure whose two key bytes
    // already say "this is a boot catalog", and a disc that got the sum wrong
    // and the keys right is a disc a real BIOS would boot too.
    a.alui8(Alu::CMP, Mem::abs(EL_TORITO_BUFFER).seg(ES), 0x01);
    a.jcc(Cc::NE, no);
    a.alui8(Alu::CMP, Mem::abs(EL_TORITO_BUFFER + 0x1e).seg(ES), 0x55);
    a.jcc(Cc::NE, no);
    a.alui8(Alu::CMP, Mem::abs(EL_TORITO_BUFFER + 0x1f).seg(ES), 0xaa);
    a.jcc(Cc::NE, no);
    // The initial/default entry, thirty-two bytes in. 88h is "bootable"; 00h
    // is a catalog that describes an image nobody is meant to boot.
    a.alui8(Alu::CMP, Mem::abs(EL_TORITO_BUFFER + 0x20).seg(ES), 0x88);
    a.jcc(Cc::NE, no);

    // -- the specification packet --------------------------------------------
    a.movmi8(Mem::abs(EBDA_CD_SPEC), SPEC_PACKET_LEN);
    a.mov8(AL, Mem::abs(EL_TORITO_BUFFER + 0x21).seg(ES));
    a.alui8(Alu::AND, AL, 0x0f);
    a.movto8(Mem::abs(EBDA_CD_SPEC + 1), AL);
    a.mov8(AL, Mem::abs(EBDA_CD_DRIVE));
    a.movto8(Mem::abs(EBDA_CD_SPEC + 2), AL);
    a.movmi8(Mem::abs(EBDA_CD_SPEC + 3), 0); // controller index
    a.mov32(AX, Mem::abs(EL_TORITO_BUFFER + 0x28).seg(ES));
    a.movto32(Mem::abs(EBDA_CD_SPEC + 4), AX);
    a.movto32(Mem::abs(EBDA_CD_LBA), AX);
    a.movmi(Mem::abs(EBDA_CD_SPEC + 8), 0); // device specification: the master
    a.movmi(Mem::abs(EBDA_CD_SPEC + 0x0a), 0); // no user buffer
    a.mov(AX, Mem::abs(EL_TORITO_BUFFER + 0x26).seg(ES));
    a.movto(Mem::abs(EBDA_CD_SPEC + 0x0e), AX);
    // A load segment of zero means 07C0h, which is the address a boot sector
    // has been loaded at since 1981 and which the specification names.
    let have_segment = a.label();
    a.mov(BX, Mem::abs(EL_TORITO_BUFFER + 0x22).seg(ES));
    a.alui(Alu::CMP, BX, 0);
    a.jcc(Cc::NE, have_segment);
    a.movi(BX, 0x07c0);
    a.bind(have_segment);
    a.movto(Mem::abs(EBDA_CD_SPEC + 0x0c), BX);

    // -- dispatch on the media type ------------------------------------------
    let no_emulation = a.label();
    let diskette = a.label();
    let emulate = a.label();
    a.mov8(AL, Mem::abs(EBDA_CD_SPEC + 1));
    a.alui8(Alu::CMP, AL, 0x00);
    a.jcc(Cc::E, no_emulation);
    a.alui8(Alu::CMP, AL, 0x04);
    a.jcc(Cc::AE, no); // hard disk emulation, and anything undefined
    a.jmp(diskette);

    // -- no emulation --------------------------------------------------------
    //
    // The entry's sector count is in 512-byte virtual sectors; the disc hands
    // them over 2048 at a time, so the read is rounded **up** to whole logical
    // blocks and may deliver up to 1,536 bytes past the image. That is what a
    // CD-ROM's geometry costs and what every BIOS that boots one does: a
    // loader that cared would have asked for a whole number of blocks.
    a.bind(no_emulation);
    a.mov(AX, Mem::abs(EBDA_CD_SPEC + 0x0e));
    a.alui(Alu::CMP, AX, 0);
    a.jcc(Cc::E, no);
    a.alui(Alu::ADD, AX, 3);
    a.shift(Shift::SHR, AX, 2);
    a.mov(CX, AX);
    a.mov(BX, Mem::abs(EBDA_CD_SPEC + 0x0c));
    a.movsr(ES, BX);
    a.movi(BX, 0);
    a.call(l.cd_read);
    a.jcc(Cc::B, no);
    a.movi_label(SI, banner);
    a.call(l.puts);
    // The entry point, as a far return: push the segment, push the offset, and
    // let `RETF` do the jump — which is how a segment register and `DL` can
    // both be set on the way out without one of them being the thing the jump
    // is read through.
    a.mov(AX, Mem::abs(EBDA_CD_SPEC + 0x0c));
    a.push(AX);
    a.movi(AX, 0);
    a.push(AX);
    a.mov8(DL, Mem::abs(EBDA_CD_DRIVE));
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.retf();

    // -- diskette emulation ---------------------------------------------------
    //
    // Types 1, 2 and 3 are a 1.2 MB, a 1.44 MB and a 2.88 MB diskette. Two
    // heads and eighty cylinders in every case; only the sectors per track
    // differ, and that is the whole of the difference between the three
    // formats as far as an `INT 13h` caller can tell.
    a.bind(diskette);
    let twelve = a.label();
    let fortyfour = a.label();
    a.movi8(CL, 36); // 2.88 MB
    a.alui8(Alu::CMP, AL, 0x01);
    a.jcc(Cc::E, twelve);
    a.alui8(Alu::CMP, AL, 0x02);
    a.jcc(Cc::E, fortyfour);
    a.jmp(emulate);
    a.bind(twelve);
    a.movi8(CL, 15);
    a.jmp(emulate);
    a.bind(fortyfour);
    a.movi8(CL, 18);

    a.bind(emulate);
    a.movto8(Mem::abs(EBDA_CD_SPT), CL);
    a.movmi8(Mem::abs(EBDA_CD_HEADS), 2);
    a.movmi(Mem::abs(EBDA_CD_CYLS), 80);
    a.alui8(Alu::OR, Mem::abs(EBDA_CD_FLAGS), 0x02);
    // The specification packet's own CHS fields, which are how a loader reads
    // the emulated geometry back: cylinder low, then sectors per track with
    // the cylinder's top two bits above it, then the last head.
    a.movmi8(Mem::abs(EBDA_CD_SPEC + 0x10), 79);
    a.movto8(Mem::abs(EBDA_CD_SPEC + 0x11), CL);
    a.movmi8(Mem::abs(EBDA_CD_SPEC + 0x12), 1);
    // Virtual sector 0 is the emulated diskette's boot sector, and it goes
    // where a diskette's boot sector goes.
    a.movi(AX, 0);
    a.movsr(ES, AX);
    a.movi(BX, 0x7c00);
    a.call(l.cd_read_sector);
    a.jcc(Cc::B, no);
    a.alui(Alu::CMP, Mem::abs(0x7dfe).seg(ES), 0xaa55);
    a.jcc(Cc::NE, no);
    a.movi_label(SI, banner);
    a.call(l.puts);
    a.movi(AX, 0);
    a.push(AX);
    a.movi(AX, 0x7c00);
    a.push(AX);
    // The emulated diskette **is** drive 00h from here on, which is what the
    // specification's boot procedure says: the image becomes A: and the real
    // diskette, if there is one, moves to 01h.
    a.movi8(DL, 0x00);
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.retf();

    // -- the exits -----------------------------------------------------------
    a.bind(no_restore);
    ds_ebda(a);
    a.bind(no);
    a.pops(DS);
    a.ret();

    // -- match_rom -----------------------------------------------------------
    //
    // `CX` bytes at `DS:SI` against `ES:DI`. Carry set on a mismatch, which is
    // how every other routine in this file reports one.
    a.bind(match_rom);
    a.push(AX);
    a.push(CX);
    a.push(SI);
    a.push(DI);
    let m_bad = a.label();
    let m_out = a.label();
    let m_next = a.here_label();
    a.lodsb();
    a.alu8(Alu::CMP, AL, Mem::di(0).seg(ES));
    a.jcc(Cc::NE, m_bad);
    a.inc(DI);
    a.loop_(m_next);
    a.clc();
    a.jmp(m_out);
    a.bind(m_bad);
    a.stc();
    a.bind(m_out);
    a.pop(DI);
    a.pop(SI);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    a.bind(cd001);
    a.db(b"CD001");
    a.bind(eltorito);
    a.db(b"EL TORITO SPECIFICATION");
    a.bind(banner);
    a.db(b"Booting from CD-ROM\r\n\0");
}
