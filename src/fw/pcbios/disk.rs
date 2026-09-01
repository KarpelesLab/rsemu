//! `INT 13h` — disk services, on the primary IDE channel.
//!
//! # Sources
//!
//! * Ralf Brown's Interrupt List, `INT 13h` functions 00h-15h and 41h-42h, for
//!   the ABI: which register carries the drive, where the buffer is, and what
//!   comes back.
//! * **T13/1410D (ATA/ATAPI-6)**: §7.10 for the device/head register and its
//!   LBA bit, §7.15 for the status register's `BSY`, `DRQ` and `ERR`, §8.27 for
//!   `READ SECTOR(S)` and §8.45 for `WRITE SECTOR(S)`, and §7.9 for the device
//!   control register's `nIEN`.
//! * The T13 CHS translation, `LBA = (C x heads + H) x sectors + (S - 1)`, which
//!   is the same equation `src/dev/ata/disk.rs` implements on the other side of
//!   the cable — both from the standard, neither from the other.
//!
//! # Polling, not interrupts
//!
//! Every transfer here spins on the status register with `IRQ14` masked. That
//! is a deliberate simplification and it is honest about a board whose drive
//! model completes a command inside the port write that delivers it: there is
//! nothing for an interrupt to overlap with. It also means `INT 13h` is
//! re-entrant from an interrupt handler, which a firmware that slept on IRQ14
//! would not be.
//!
//! # No diskette
//!
//! `DL < 0x80` answers success for a reset and failure for everything else. The
//! µPD765 path — motor, `SPECIFY`, `RECALIBRATE`, `SEEK`, an 8237 channel-2
//! programming and a `READ DATA` — is real work that is not here yet, and
//! `INT 19h` tries the fixed disk first, so a boot does not depend on it.

use super::{
    EBDA_COMMAND, EBDA_HD_CAPACITY, EBDA_HD_CYLINDERS, EBDA_HD_FLAGS, EBDA_HD_HEADS,
    EBDA_HD_SECTORS, EBDA_LBA_HIGH, EBDA_LBA_LOW, F_AX, F_BX, F_CX, F_DS, F_DX, F_ES, F_SI, Labels,
    clear_cf, ds_ebda, enter, leave, set_cf,
};
use crate::fw::asm16::{
    AH, AL, AX, Alu, Asm, BL, BX, CH, CL, CX, Cc, DI, DL, DS, DX, ES, Mem, SI, Shift,
};

/// The primary channel's command block, and the one register outside it.
const CMD_BASE: u16 = 0x01f0;

/// Emit `INT 13h` and the ATA routines POST shares with it.
#[allow(clippy::too_many_lines)]
pub(super) fn emit(a: &mut Asm, l: &Labels) {
    a.bind(l.int13);
    enter(a);
    a.cld();
    // The whole handler runs with `DS` on the EBDA, where the geometry POST
    // read out of `IDENTIFY DEVICE` lives. The frame is `SS`-relative, so the
    // caller's registers stay reachable either way.
    ds_ebda(a);

    let floppy = a.label();
    let read = a.label();
    let write = a.label();
    let params = a.label();
    let kind = a.label();
    let ext_check = a.label();
    let ext_read = a.label();
    let done = a.label();

    a.mov8(DL, Mem::bp(F_DX));
    a.mov8(AH, Mem::bp(F_AX + 1));
    a.testi8(DL, 0x80);
    a.jcc(Cc::E, floppy);

    // Exactly one fixed disk exists on this board, and it is 0x80.
    a.alui8(Alu::CMP, DL, 0x80);
    a.jcc(Cc::NE, l.disk_fail);
    a.testi8(Mem::abs(EBDA_HD_FLAGS), 0x01);
    a.jcc(Cc::E, l.disk_fail);

    for (function, target) in [
        (0x00u8, l.disk_ok), // reset: the model has no state to reset
        (0x01, l.disk_ok),   // last status, which is only ever "no error"
        (0x02, read),
        (0x03, write),
        (0x04, l.disk_ok), // verify: nothing to compare against
        (0x08, params),
        (0x15, kind),
        (0x41, ext_check),
        (0x42, ext_read),
    ] {
        a.alui8(Alu::CMP, AH, function);
        a.jcc(Cc::E, target);
    }
    a.jmp(l.disk_fail);

    // A diskette drive that answers a reset and nothing else, so `INT 19h`'s
    // fallback path fails cleanly rather than faulting.
    a.bind(floppy);
    a.alui8(Alu::CMP, AH, 0x00);
    a.jcc(Cc::E, l.disk_ok);
    a.jmp(l.disk_fail);

    // AH=02h/03h. The two differ only in the command byte, so they share
    // everything up to it.
    a.bind(read);
    a.movi8(BL, 0x20); // READ SECTOR(S), T13/1410D §8.27
    let transfer = a.label();
    a.jmp(transfer);
    a.bind(write);
    a.movi8(BL, 0x30); // WRITE SECTOR(S), §8.45
    a.bind(transfer);
    a.call(l.chs_to_lba);
    a.jcc(Cc::B, l.disk_fail);
    a.mov(CX, Mem::bp(F_AX));
    a.alui(Alu::AND, CX, 0x00ff);
    a.jcc(Cc::E, l.disk_ok);
    a.mov8(AL, BL);
    a.mov(BX, Mem::bp(F_BX));
    a.movsr(ES, Mem::bp(F_ES));
    a.call(l.ata_read);
    a.jcc(Cc::B, l.disk_fail);
    a.jmp(l.disk_ok);

    // AH=08h, drive parameters. The cylinder count is one *less* than the
    // number of cylinders, and its top two bits ride in CL's top two — the
    // packing every `INT 13h` caller since 1983 unpacks.
    a.bind(params);
    a.mov(AX, Mem::abs(EBDA_HD_CYLINDERS));
    a.dec(AX);
    a.mov8(CH, AL);
    a.mov8(CL, AH);
    a.shift8(Shift::SHL, CL, 6);
    a.mov(AX, Mem::abs(EBDA_HD_SECTORS));
    a.alui8(Alu::AND, AL, 0x3f);
    a.aluto8(Alu::OR, CL, AL);
    a.movto(Mem::bp(F_CX), CX);
    a.mov(AX, Mem::abs(EBDA_HD_HEADS));
    a.dec(AX);
    a.movto8(Mem::bp(F_DX + 1), AL);
    a.movi8(AL, 1);
    a.movto8(Mem::bp(F_DX), AL);
    a.jmp(l.disk_ok);

    // AH=15h, drive type: 3 is "fixed disk", and CX:DX is its sector count.
    a.bind(kind);
    a.mov(AX, Mem::abs(EBDA_HD_CAPACITY + 2));
    a.movto(Mem::bp(F_CX), AX);
    a.mov(AX, Mem::abs(EBDA_HD_CAPACITY));
    a.movto(Mem::bp(F_DX), AX);
    a.movmi8(Mem::bp(F_AX + 1), 3);
    clear_cf(a);
    a.jmp(done);

    // AH=41h, the EDD installation check. BX must arrive as 0x55AA and comes
    // back byte-swapped; CX's bit 0 claims the fixed-disk subset, which is
    // exactly AH=42h and AH=48h — and only 42h is here, so nothing more is
    // claimed.
    a.bind(ext_check);
    a.mov(AX, Mem::bp(F_BX));
    a.alui(Alu::CMP, AX, 0x55aa);
    a.jcc(Cc::NE, l.disk_fail);
    a.movmi(Mem::bp(F_BX), 0xaa55);
    a.movmi(Mem::bp(F_CX), 0x0001);
    a.movmi8(Mem::bp(F_AX + 1), 0x21);
    clear_cf(a);
    a.jmp(done);

    // AH=42h, extended read. DS:SI points at a sixteen-byte disk address
    // packet: a size byte, a reserved byte, a block count, a far buffer
    // pointer, and a 64-bit LBA of which the low 32 bits are all this drive
    // has.
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
    a.movto(Mem::abs(EBDA_LBA_LOW), BX);
    a.movto(Mem::abs(EBDA_LBA_HIGH), DX);
    a.movsr(ES, AX);
    a.mov(BX, DI);
    a.movi8(AL, 0x20);
    a.call(l.ata_read);
    a.jcc(Cc::B, l.disk_fail);
    a.jmp(l.disk_ok);

    // The two exits. `AH` is the status byte every caller reads after the carry
    // flag, so it is written on both paths.
    a.bind(l.disk_ok);
    a.movmi8(Mem::bp(F_AX + 1), 0x00);
    clear_cf(a);
    a.jmp(done);
    a.bind(l.disk_fail);
    a.movmi8(Mem::bp(F_AX + 1), 0x01);
    set_cf(a);
    a.bind(done);
    leave(a);

    // -- chs_to_lba ----------------------------------------------------------
    //
    // The caller's CX and DX hold a cylinder/head/sector triple: CH is the
    // cylinder's low byte, CL's top two bits its high two, CL's low six the
    // one-based sector, and DH the head. The arithmetic is 32-bit because
    // `cylinders x heads` overflows sixteen for any drive worth translating.
    a.bind(l.chs_to_lba);
    a.push(BX);
    a.push(CX);
    a.push(DX);
    let bad = a.label();
    let out = a.label();
    a.mov(CX, Mem::bp(F_CX));
    a.mov8(AL, CH);
    a.mov8(AH, CL);
    a.shift8(Shift::SHR, AH, 6);
    a.mov(BX, AX);
    a.movi32(AX, 0);
    a.mov(AX, BX);
    a.movi32(BX, 0);
    a.mov(BX, Mem::abs(EBDA_HD_HEADS));
    a.mul32(BX);
    a.movi32(BX, 0);
    a.mov8(BL, Mem::bp(F_DX + 1));
    a.alu32(Alu::ADD, AX, BX);
    a.movi32(BX, 0);
    a.mov(BX, Mem::abs(EBDA_HD_SECTORS));
    a.mul32(BX);
    a.movi32(BX, 0);
    a.mov8(BL, Mem::bp(F_CX));
    a.alui8(Alu::AND, BL, 0x3f);
    a.jcc(Cc::E, bad);
    a.dec(BX);
    a.alu32(Alu::ADD, AX, BX);
    a.movto32(Mem::abs(EBDA_LBA_LOW), AX);
    a.clc();
    a.jmp(out);
    a.bind(bad);
    a.stc();
    a.bind(out);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.ret();

    // -- ata_read ------------------------------------------------------------
    //
    // AL is the command byte, CX the sector count, ES:BX the buffer, and the
    // LBA is in the EBDA. Carry comes back set if the drive refused or never
    // answered. `REP INSW` is the transfer: 256 words is one sector, and the
    // instruction exists precisely for this. A write is the same loop with
    // `REP OUTSW`, which reads `DS:SI` — the one place this handler has to
    // leave the EBDA, and it puts it back.
    a.bind(l.ata_read);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    a.push(SI);
    a.push(DI);
    let fail = a.label();
    let finish = a.label();
    let sector = a.label();
    a.movto8(Mem::abs(EBDA_COMMAND), AL);
    a.call(l.ata_wait_ready);
    a.jcc(Cc::B, fail);

    a.movi(DX, CMD_BASE + 6);
    a.mov8(AL, Mem::abs(EBDA_LBA_LOW + 3));
    a.alui8(Alu::AND, AL, 0x0f);
    a.alui8(Alu::OR, AL, 0xe0); // LBA addressing, device 0
    a.out_dx_al();
    a.movi(DX, CMD_BASE + 2);
    a.mov8(AL, CL);
    a.out_dx_al();
    a.movi(DX, CMD_BASE + 3);
    a.mov8(AL, Mem::abs(EBDA_LBA_LOW));
    a.out_dx_al();
    a.movi(DX, CMD_BASE + 4);
    a.mov8(AL, Mem::abs(EBDA_LBA_LOW + 1));
    a.out_dx_al();
    a.movi(DX, CMD_BASE + 5);
    a.mov8(AL, Mem::abs(EBDA_LBA_LOW + 2));
    a.out_dx_al();
    a.movi(DX, CMD_BASE + 7);
    a.mov8(AL, Mem::abs(EBDA_COMMAND));
    a.out_dx_al();

    a.bind(sector);
    a.call(l.ata_wait_drq);
    a.jcc(Cc::B, fail);
    a.movi(DX, CMD_BASE);
    a.push(CX);
    a.movi(CX, 256);
    let do_write = a.label();
    let advance = a.label();
    a.alui8(Alu::CMP, Mem::abs(EBDA_COMMAND), 0x30);
    a.jcc(Cc::E, do_write);
    a.mov(DI, BX);
    a.rep();
    a.insw();
    a.mov(BX, DI);
    a.jmp(advance);
    a.bind(do_write);
    a.pushs(DS);
    a.movrs(AX, ES);
    a.movsr(DS, AX);
    a.mov(SI, BX);
    a.rep();
    a.outsw();
    a.mov(BX, SI);
    a.pops(DS);
    a.bind(advance);
    a.pop(CX);
    a.dec(CX);
    a.jcc(Cc::NE, sector);
    // A write is not finished when the last word has gone in: the drive owns
    // the command block until `BSY` drops (T13/1410D §7.15).
    a.call(l.ata_wait_ready);
    a.jcc(Cc::B, fail);
    a.clc();
    a.jmp(finish);
    a.bind(fail);
    a.stc();
    a.bind(finish);
    a.pop(DI);
    a.pop(SI);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- ata_wait_ready ------------------------------------------------------
    //
    // Spin until `BSY` clears (T13/1410D §7.15). Bounded, so a cable with
    // nothing on it costs 65,536 port reads rather than the machine.
    a.bind(l.ata_wait_ready);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    let r_ok = a.label();
    a.movi(CX, 0);
    a.movi(DX, CMD_BASE + 7);
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

    // -- ata_wait_drq --------------------------------------------------------
    //
    // Spin until the drive is not busy and has data to move, and fail on `ERR`
    // or `DF` rather than transferring whatever the buffer happens to hold.
    a.bind(l.ata_wait_drq);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    let d_ok = a.label();
    let d_err = a.label();
    let d_next = a.label();
    a.movi(CX, 0);
    a.movi(DX, CMD_BASE + 7);
    let d_poll = a.here_label();
    a.in_al_dx();
    a.testi8(AL, 0x80); // BSY
    a.jcc(Cc::NE, d_next);
    a.testi8(AL, 0x21); // ERR | DF
    a.jcc(Cc::NE, d_err);
    a.testi8(AL, 0x08); // DRQ
    a.jcc(Cc::NE, d_ok);
    a.bind(d_next);
    a.dec(CX);
    a.jcc(Cc::NE, d_poll);
    a.bind(d_err);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.stc();
    a.ret();
    a.bind(d_ok);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.clc();
    a.ret();
}
