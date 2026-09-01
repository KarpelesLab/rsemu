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
//! # The diskette
//!
//! `DL < 0x80` goes to the µPD765 through [`diskette`], which has its own
//! source list: the motor and data rate, `SPECIFY`, `SEEK`, `SENSE INTERRUPT
//! STATUS`, an 8237 channel-2 programming and one `READ DATA` per sector. Only
//! unit 0 exists on this board, and a diskette **write** returns carry — the
//! command differs from a read by one opcode bit and one DMA mode nibble, and
//! shipping it untested would be worse than not shipping it.

use super::{
    EBDA_COMMAND, EBDA_FD_COUNT, EBDA_FD_CYLINDER, EBDA_FD_DONE, EBDA_FD_HEAD, EBDA_FD_RESULT,
    EBDA_FD_SECTOR, EBDA_FD_SPT, EBDA_HD_CAPACITY, EBDA_HD_CYLINDERS, EBDA_HD_FLAGS, EBDA_HD_HEADS,
    EBDA_HD_SECTORS, EBDA_LBA_HIGH, EBDA_LBA_LOW, F_AX, F_BX, F_CX, F_DS, F_DX, F_ES, F_SI, Labels,
    clear_cf, ds_ebda, enter, leave, set_cf,
};
use crate::fw::asm16::{
    AH, AL, AX, Alu, Asm, BH, BL, BX, CH, CL, CS, CX, Cc, DH, DI, DL, DS, DX, ES, Mem, SI, Shift,
};

/// The primary channel's command block, and the one register outside it.
const CMD_BASE: u16 = 0x01f0;

/// The diskette adapter's digital output register: drive select, `/RESET`, the
/// DMA and interrupt gate, and the four motor enables.
const FDC_DOR: u16 = 0x03f2;
/// Its main status register, which is the whole of the handshake.
const FDC_MSR: u16 = 0x03f4;
/// Its data register: parameters in, results out.
const FDC_DATA: u16 = 0x03f5;
/// Its configuration control register, which selects the data rate.
const FDC_CCR: u16 = 0x03f7;

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

    // The diskette. Only unit 0 exists on this board, and only reads and the
    // two parameter queries are answered; a write returns carry.
    a.bind(floppy);
    let fd_reset = a.label();
    let fd_read = a.label();
    let fd_params = a.label();
    let fd_kind = a.label();
    a.alui8(Alu::CMP, DL, 0x00);
    a.jcc(Cc::NE, l.disk_fail);
    for (function, target) in [
        (0x00u8, fd_reset),
        (0x01, l.disk_ok),
        (0x02, fd_read),
        (0x04, l.disk_ok),
        (0x08, fd_params),
        (0x15, fd_kind),
    ] {
        a.alui8(Alu::CMP, AH, function);
        a.jcc(Cc::E, target);
    }
    a.jmp(l.disk_fail);

    a.bind(fd_reset);
    a.call(l.fd_start);
    a.jmp(l.disk_ok);

    // AH=08h for a diskette: 80 cylinders, two heads, the CMOS's sectors per
    // track, one drive, and drive type 4 in BL — a 1.44 MB 3.5-inch unit.
    a.bind(fd_params);
    a.call(l.fd_geometry);
    a.movi8(CH, 79);
    a.mov8(CL, Mem::abs(EBDA_FD_SPT));
    a.movto(Mem::bp(F_CX), CX);
    a.movi8(DH, 1);
    a.movi8(DL, 1);
    a.movto(Mem::bp(F_DX), DX);
    a.movmi8(Mem::bp(F_BX), 0x04);
    a.jmp(l.disk_ok);

    // AH=15h: type 1 is "diskette, no change line", which is what this drive
    // reports because nothing here ejects.
    a.bind(fd_kind);
    a.movmi8(Mem::bp(F_AX + 1), 0x01);
    clear_cf(a);
    a.jmp(done);

    // AH=02h for a diskette. The caller's CHS goes to the controller unchanged
    // — a µPD765 addresses by cylinder, head and sector, so unlike the fixed
    // disk there is no translation to get wrong. One `READ DATA` command per
    // sector, because that makes the terminal-count and end-of-track rules
    // somebody else's problem: `EOT` is set to the sector being read.
    a.bind(fd_read);
    let fd_loop = a.label();
    let fd_stop = a.label();
    let fd_finished = a.label();
    let fd_same_track = a.label();
    let fd_seek_again = a.label();
    a.mov(AX, Mem::bp(F_AX));
    a.alui(Alu::AND, AX, 0x00ff);
    a.jcc(Cc::E, l.disk_ok);
    a.movto(Mem::abs(EBDA_FD_COUNT), AX);
    a.movmi8(Mem::abs(EBDA_FD_DONE), 0);
    a.mov(CX, Mem::bp(F_CX));
    a.movto8(Mem::abs(EBDA_FD_CYLINDER), CH);
    a.mov8(AL, CL);
    a.alui8(Alu::AND, AL, 0x3f);
    a.movto8(Mem::abs(EBDA_FD_SECTOR), AL);
    a.mov8(AL, Mem::bp(F_DX + 1));
    a.alui8(Alu::AND, AL, 0x01);
    a.movto8(Mem::abs(EBDA_FD_HEAD), AL);
    a.mov(BX, Mem::bp(F_BX));
    a.movsr(ES, Mem::bp(F_ES));
    a.call(l.fd_geometry);
    a.call(l.fd_start);
    a.call(l.fd_seek);

    a.bind(fd_loop);
    a.call(l.fd_dma);
    a.call(l.fd_read_one);
    a.jcc(Cc::B, fd_stop);
    a.incm8(Mem::abs(EBDA_FD_DONE));
    a.alui(Alu::ADD, BX, 512);
    a.decm(Mem::abs(EBDA_FD_COUNT));
    a.jcc(Cc::E, fd_finished);
    // The next sector, wrapping onto the other head and then the next cylinder.
    a.mov8(AL, Mem::abs(EBDA_FD_SECTOR));
    a.incm8(AL);
    a.alu8(Alu::CMP, AL, Mem::abs(EBDA_FD_SPT));
    a.jcc(Cc::BE, fd_same_track);
    a.movmi8(Mem::abs(EBDA_FD_SECTOR), 1);
    a.mov8(AL, Mem::abs(EBDA_FD_HEAD));
    a.alui8(Alu::XOR, AL, 0x01);
    a.movto8(Mem::abs(EBDA_FD_HEAD), AL);
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::NE, fd_seek_again);
    a.incm8(Mem::abs(EBDA_FD_CYLINDER));
    a.bind(fd_seek_again);
    a.call(l.fd_seek);
    a.jmp(fd_loop);
    a.bind(fd_same_track);
    a.movto8(Mem::abs(EBDA_FD_SECTOR), AL);
    a.jmp(fd_loop);

    // Either way `AL` reports how many sectors actually moved, which is the
    // only way a caller can tell a short read from a complete one.
    a.bind(fd_finished);
    a.mov8(AL, Mem::abs(EBDA_FD_DONE));
    a.movto8(Mem::bp(F_AX), AL);
    a.jmp(l.disk_ok);
    a.bind(fd_stop);
    a.mov8(AL, Mem::abs(EBDA_FD_DONE));
    a.movto8(Mem::bp(F_AX), AL);
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

    diskette(a, l);
}

/// The µPD765 primitives `INT 13h`'s diskette path is built out of.
///
/// # Sources
///
/// * NEC µPD765A data sheet: the main status register's `RQM`/`DIO`/`CB`, the
///   three phases, `SPECIFY`, `RECALIBRATE`, `SEEK`, `SENSE INTERRUPT STATUS`
///   and `READ DATA` with their parameter and result byte orders, and `ST0`'s
///   interrupt code in bits 7:6.
/// * IBM PC/AT Technical Reference, diskette adapter: the digital output
///   register at `0x3f2` (a board latch, not a chip register) and the
///   configuration control register at `0x3f7`.
/// * Intel 8237A data sheet §"Register Description": the mask register at
///   `0x0a`, the byte-pointer flip-flop at `0x0c`, the mode register at `0x0b`,
///   and the address/count pair at `0x04`/`0x05`. The page latch for channel 2
///   is at `0x81`, which is a board fact rather than a chip one.
///
/// # Polling, again
///
/// `IRQ6` is masked, so every wait here spins on the main status register. That
/// is exactly right for this board: `src/dev/pc/fdc.rs` says in its own docs
/// that seeks and transfers complete inside the `out` that starts them, so
/// there is never anything to wait for. A firmware that slept on the interrupt
/// would work too, and would be more code for no behaviour.
#[allow(clippy::too_many_lines)]
fn diskette(a: &mut Asm, l: &Labels) {
    // -- fd_out: send one command byte -------------------------------------
    //
    // The handshake is `RQM` set with `DIO` clear — the controller wants a byte
    // *from* the CPU. A bounded spin, so a controller that never asks costs a
    // moment rather than the machine.
    a.bind(l.fd_out);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    let o_send = a.label();
    let o_out = a.label();
    a.mov8(AH, AL);
    a.movi(CX, 0);
    a.movi(DX, FDC_MSR);
    let o_poll = a.here_label();
    a.in_al_dx();
    a.alui8(Alu::AND, AL, 0xc0);
    a.alui8(Alu::CMP, AL, 0x80);
    a.jcc(Cc::E, o_send);
    a.dec(CX);
    a.jcc(Cc::NE, o_poll);
    a.jmp(o_out);
    a.bind(o_send);
    a.movi(DX, FDC_DATA);
    a.mov8(AL, AH);
    a.out_dx_al();
    a.bind(o_out);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- fd_in: take one result byte, in AL --------------------------------
    a.bind(l.fd_in);
    a.push(CX);
    a.push(DX);
    let i_read = a.label();
    let i_out = a.label();
    a.movi(CX, 0);
    a.movi(DX, FDC_MSR);
    let i_poll = a.here_label();
    a.in_al_dx();
    a.alui8(Alu::AND, AL, 0xc0);
    a.alui8(Alu::CMP, AL, 0xc0);
    a.jcc(Cc::E, i_read);
    a.dec(CX);
    a.jcc(Cc::NE, i_poll);
    a.movi8(AL, 0xff);
    a.jmp(i_out);
    a.bind(i_read);
    a.movi(DX, FDC_DATA);
    a.in_al_dx();
    a.bind(i_out);
    a.pop(DX);
    a.pop(CX);
    a.ret();

    // -- fd_drain: read the whole result phase and throw it away ------------
    //
    // The number of result bytes depends on the command *and* on whether the
    // controller thought it was valid, so counting them is the wrong shape:
    // `RQM | DIO | CB` is the controller saying there is another one.
    a.bind(l.fd_drain);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    let dr_done = a.label();
    a.movi(CX, 0);
    let dr_poll = a.here_label();
    a.movi(DX, FDC_MSR);
    a.in_al_dx();
    a.alui8(Alu::AND, AL, 0xd0);
    a.alui8(Alu::CMP, AL, 0xd0);
    a.jcc(Cc::NE, dr_done);
    a.movi(DX, FDC_DATA);
    a.in_al_dx();
    a.dec(CX);
    a.jcc(Cc::NE, dr_poll);
    a.bind(dr_done);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- fd_start: motor, data rate, and a chip that knows where it is ------
    //
    // The digital output register's bit 2 is `/RESET`, bit 3 gates `DRQ` and
    // `INT` onto the bus, and bit 4 is drive 0's motor. `0x1c` is all three
    // with unit 0 selected. The motor is left running: a diskette's spin-up is
    // not modelled, and a firmware that switched it off between sectors would
    // only be pretending.
    //
    // Four `SENSE INTERRUPT STATUS` commands follow, because a µPD765 coming
    // out of reset reports a ready-state change for each of its four units and
    // will refuse everything else until they are collected.
    a.bind(l.fd_start);
    a.push(AX);
    a.push(CX);
    a.push(DX);
    a.movi(DX, FDC_DOR);
    a.movi8(AL, 0x1c);
    a.out_dx_al();
    a.movi(DX, FDC_CCR);
    a.movi8(AL, 0x00); // 500 kbps, which is every high-density format
    a.out_dx_al();
    a.movi(CX, 4);
    let st_drain = a.here_label();
    a.movi8(AL, 0x08);
    a.call(l.fd_out);
    a.call(l.fd_drain);
    a.dec(CX);
    a.jcc(Cc::NE, st_drain);
    // SPECIFY: a 3 ms step rate and a 240 ms head unload in the first byte, a
    // 16 ms head load and DMA (bit 0 clear) in the second. None of the timings
    // matter to a controller that completes instantly; the DMA bit does.
    a.movi8(AL, 0x03);
    a.call(l.fd_out);
    a.movi8(AL, 0xdf);
    a.call(l.fd_out);
    a.movi8(AL, 0x02);
    a.call(l.fd_out);
    a.pop(DX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- fd_seek ------------------------------------------------------------
    //
    // `SEEK` then `SENSE INTERRUPT STATUS`, which is both how the completion is
    // collected and how the controller's interrupt is acknowledged — issuing it
    // *is* the acknowledgement. The result is discarded: a seek that failed
    // shows up as `ST0`'s not-ready bit on the read that follows, which is
    // checked, and checking twice would only mean two places to get it wrong.
    a.bind(l.fd_seek);
    a.push(AX);
    a.movi8(AL, 0x0f);
    a.call(l.fd_out);
    a.mov8(AL, Mem::abs(EBDA_FD_HEAD));
    a.shift8(Shift::SHL, AL, 2);
    a.call(l.fd_out);
    a.mov8(AL, Mem::abs(EBDA_FD_CYLINDER));
    a.call(l.fd_out);
    a.movi8(AL, 0x08);
    a.call(l.fd_out);
    a.call(l.fd_drain);
    a.pop(AX);
    a.ret();

    // -- fd_dma: one sector, into ES:BX -------------------------------------
    //
    // The 8237 addresses memory physically, so the far pointer is flattened
    // here: `ES * 16 + BX`, whose carry out of sixteen bits is the page latch's
    // business. Mode `0x46` is a single transfer, address increment, no
    // auto-initialise, a *write* transfer (device to memory), channel 2. The
    // count is one less than the length, which is what the chip counts down
    // from.
    a.bind(l.fd_dma);
    a.push(AX);
    a.push(BX);
    a.push(CX);
    a.push(DX);
    a.movrs(AX, ES);
    a.mov(DX, AX);
    a.shift(Shift::SHR, DX, 12);
    a.shift(Shift::SHL, AX, 4);
    a.alu(Alu::ADD, AX, BX);
    a.alui8(Alu::ADC, DL, 0);
    a.mov(CX, AX);
    a.movi8(AL, 0x06); // mask channel 2
    a.out_al(0x0a);
    a.movi8(AL, 0x00); // clear the byte-pointer flip-flop
    a.out_al(0x0c);
    a.movi8(AL, 0x46);
    a.out_al(0x0b);
    a.mov8(AL, CL);
    a.out_al(0x04);
    a.mov8(AL, CH);
    a.out_al(0x04);
    a.mov8(AL, DL);
    a.out_al(0x81); // channel 2's page latch
    a.movi8(AL, 0x00);
    a.out_al(0x0c);
    a.movi8(AL, 0xff); // 512 bytes, less one
    a.out_al(0x05);
    a.movi8(AL, 0x01);
    a.out_al(0x05);
    a.movi8(AL, 0x02); // unmask channel 2
    a.out_al(0x0a);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.pop(AX);
    a.ret();

    // -- fd_read_one --------------------------------------------------------
    //
    // `READ DATA` with `MFM` set and `MT`/`SK` clear, and `EOT` equal to the
    // sector being read, so the command covers exactly one sector and the
    // multi-track and end-of-cylinder rules never come into it. Carry comes
    // back set unless `ST0`'s interrupt code is 00.
    a.bind(l.fd_read_one);
    a.push(AX);
    a.push(CX);
    a.push(DI);
    let r_ok = a.label();
    let r_out = a.label();
    a.movi8(AL, 0x46);
    a.call(l.fd_out);
    a.mov8(AL, Mem::abs(EBDA_FD_HEAD));
    a.shift8(Shift::SHL, AL, 2);
    a.call(l.fd_out);
    a.mov8(AL, Mem::abs(EBDA_FD_CYLINDER));
    a.call(l.fd_out);
    a.mov8(AL, Mem::abs(EBDA_FD_HEAD));
    a.call(l.fd_out);
    a.mov8(AL, Mem::abs(EBDA_FD_SECTOR));
    a.call(l.fd_out);
    a.movi8(AL, 0x02); // N = 2, a 512-byte sector
    a.call(l.fd_out);
    a.mov8(AL, Mem::abs(EBDA_FD_SECTOR)); // EOT
    a.call(l.fd_out);
    a.movi8(AL, 0x1b); // GPL, the standard gap for MFM
    a.call(l.fd_out);
    a.movi8(AL, 0xff); // DTL, ignored when N is non-zero
    a.call(l.fd_out);
    a.movi(DI, EBDA_FD_RESULT);
    a.movi(CX, 7);
    let r_res = a.here_label();
    a.call(l.fd_in);
    a.movto8(Mem::di(0), AL);
    a.inc(DI);
    a.dec(CX);
    a.jcc(Cc::NE, r_res);
    a.mov8(AL, Mem::abs(EBDA_FD_RESULT));
    a.alui8(Alu::AND, AL, 0xc0);
    a.jcc(Cc::E, r_ok);
    a.stc();
    a.jmp(r_out);
    a.bind(r_ok);
    a.clc();
    a.bind(r_out);
    a.pop(DI);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- fd_geometry --------------------------------------------------------
    //
    // Sectors per track, from the CMOS diskette-type byte at 0x10: the high
    // nibble is drive 0. An unrecognised type is treated as a 1.44 MB unit,
    // which is what an emulated machine with an unconfigured CMOS almost
    // always has in it.
    a.bind(l.fd_geometry);
    a.push(AX);
    a.push(BX);
    a.push(SI);
    let g_known = a.label();
    let table = a.label();
    a.movi8(AL, 0x10);
    a.call(l.cmos_read);
    a.shift8(Shift::SHR, AL, 4);
    a.alui8(Alu::CMP, AL, 5);
    a.jcc(Cc::B, g_known);
    a.movi8(AL, 4);
    a.bind(g_known);
    a.mov8(BL, AL);
    a.movi8(BH, 0);
    a.movi_label(SI, table);
    a.alu(Alu::ADD, SI, BX);
    a.mov8(AL, Mem::si(0).seg(CS));
    a.movto8(Mem::abs(EBDA_FD_SPT), AL);
    a.pop(SI);
    a.pop(BX);
    a.pop(AX);
    a.ret();

    // Type 0 is "no drive", which cannot happen on a machine that got here, so
    // it takes the 1.44 MB answer with the rest of the unknowns.
    a.bind(table);
    a.db(&[18, 9, 15, 9, 18]);
}
