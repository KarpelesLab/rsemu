//! Power-on self test: reset, the chipset, the BIOS Data Area, option ROMs and
//! the handover to the bootstrap loader.
//!
//! Every port write here cites the data sheet that defines it. Nothing in this
//! file was derived from another firmware; the ports and the order they are
//! touched in come from the chips' own documentation (`ROADMAP.md` §1).

use super::{
    BDA_ACTIVE_PAGE, BDA_CHAR_HEIGHT, BDA_COLUMNS, BDA_CRTC_PORT, BDA_CURSOR, BDA_CURSOR_SHAPE,
    BDA_EBDA, BDA_EQUIPMENT, BDA_HD_COUNT, BDA_KBBUF, BDA_KBBUF_END, BDA_KBBUF_START, BDA_KBHEAD,
    BDA_KBTAIL, BDA_MEMSIZE, BDA_MODE_SELECT, BDA_PAGE_OFFSET, BDA_PAGE_SIZE, BDA_ROWS,
    BDA_VIDEO_MODE, E820_ENTRY, EBDA_E820, EBDA_E820_COUNT, EBDA_HD_CAPACITY, EBDA_HD_CYLINDERS,
    EBDA_HD_FLAGS, EBDA_HD_HEADS, EBDA_HD_SECTORS, EBDA_SEGMENT, EBDA_SIZE, IDENTIFY_BUFFER,
    KBBUF_BYTES, Labels, POST_STACK, SEGMENT, ds_bda,
};
use crate::fw::asm16::{
    AH, AL, AX, Alu, Asm, BH, BL, BX, CH, CL, CX, Cc, DI, DS, DX, ES, Label, Mem, SI, SP, SS, Shift,
};

/// Install one interrupt vector: `AL` is the number, `BX` the handler's offset.
fn vector(a: &mut Asm, l: &Labels, number: u8, handler: Label) {
    a.movi8(AL, number);
    a.movi_label(BX, handler);
    a.call(l.set_vector);
}

/// Emit POST and the routines only it uses.
#[allow(clippy::too_many_lines)]
pub(super) fn emit(a: &mut Asm, l: &Labels) {
    a.bind(l.post);
    a.cli();
    a.cld();

    // Segments and a stack. `SS:SP` sits immediately below `0000:7C00`, so the
    // stack and the boot sector's landing pad cannot collide.
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, POST_STACK);

    // A20. The 8042 hands the gate over *open* (`docs/platforms/pc-at.md`), so
    // this is belt and braces rather than a requirement — but a firmware that
    // assumed the power-on level would be wrong on a warm reset, where the gate
    // is wherever the last guest left it. Port 0x92 bit 1 is the chipset's fast
    // gate; bit 0 is fast reset and must never be written set.
    a.in_al(0x92);
    a.alui8(Alu::OR, AL, 0x02);
    a.alui8(Alu::AND, AL, 0xfe);
    a.out_al(0x92);

    // Zero the BIOS Data Area. Everything below fills in what it knows; what it
    // does not know must read as zero rather than as whatever DRAM came up as.
    a.movi(DI, 0x400);
    a.movi(CX, 0x100);
    a.movi(AX, 0);
    a.rep();
    a.stosb();

    // The interrupt vector table, every entry pointing at the same stub, so an
    // unimplemented service returns to its caller instead of executing zeroed
    // RAM. The specific handlers overwrite their own entries below.
    a.movi(DI, 0);
    a.movi(CX, 256);
    let fill = a.here_label();
    a.movi_label(AX, l.unimplemented);
    a.stosw();
    a.movi(AX, SEGMENT);
    a.stosw();
    a.dec(CX);
    a.jcc(Cc::NE, fill);

    vector(a, l, 0x08, l.int08);
    vector(a, l, 0x09, l.int09);
    for n in 0x0a..=0x0f {
        vector(a, l, n, l.eoi_master);
    }
    vector(a, l, 0x10, l.int10);
    vector(a, l, 0x11, l.int11);
    vector(a, l, 0x12, l.int12);
    vector(a, l, 0x13, l.int13);
    vector(a, l, 0x15, l.int15);
    vector(a, l, 0x16, l.int16);
    vector(a, l, 0x18, l.int18);
    vector(a, l, 0x19, l.int19);
    vector(a, l, 0x1a, l.int1a);
    // The two user hooks a program is allowed to take over: the ctrl-break
    // handler and the periodic tick. They must exist and do nothing.
    vector(a, l, 0x05, l.iret_stub);
    vector(a, l, 0x1b, l.iret_stub);
    vector(a, l, 0x1c, l.iret_stub);
    for n in 0x70..=0x77 {
        vector(a, l, n, l.eoi_slave);
    }

    // -- the 8259A pair (Intel 8259A data sheet, "Programming the 8259A") ----
    //
    // ICW1 starts the initialisation sequence; ICW2 sets the vector base; ICW3
    // says where the cascade is; ICW4 selects 8086 mode. The bases are the AT's:
    // 0x08 for IRQ0-7 and 0x70 for IRQ8-15.
    a.movi8(AL, 0x11); // ICW1: edge triggered, cascaded, ICW4 to follow
    a.out_al(0x20);
    a.out_al(0xa0);
    a.movi8(AL, 0x08); // ICW2 master: IRQ0 is vector 0x08
    a.out_al(0x21);
    a.movi8(AL, 0x70); // ICW2 slave: IRQ8 is vector 0x70
    a.out_al(0xa1);
    a.movi8(AL, 0x04); // ICW3 master: a slave is attached to IR2
    a.out_al(0x21);
    a.movi8(AL, 0x02); // ICW3 slave: its cascade identity is 2
    a.out_al(0xa1);
    a.movi8(AL, 0x01); // ICW4: 8086/8088 mode, normal EOI
    a.out_al(0x21);
    a.out_al(0xa1);
    // OCW1, the masks. IRQ0 (timer), IRQ1 (keyboard) and IRQ2 (the cascade) are
    // the only lines this firmware services: the disk paths poll rather than
    // wait for an interrupt, so IRQ14 and IRQ6 stay masked and no half-written
    // handler can be reached by an unexpected edge.
    a.movi8(AL, 0xf8);
    a.out_al(0x21);
    a.movi8(AL, 0xff);
    a.out_al(0xa1);

    // -- the 8254 (Intel 8254 data sheet, "Control Word Format") ------------
    //
    // Counter 0, read/write LSB then MSB, mode 3 (square wave), binary. A
    // divisor of zero means 65536, which off the board's 105/88 MHz input is
    // 18.2065 Hz — the tick every PC has counted at since 1981.
    a.movi8(AL, 0x36);
    a.out_al(0x43);
    a.movi8(AL, 0x00);
    a.out_al(0x40);
    a.out_al(0x40);

    // -- the MC146818 (Motorola MC146818 data sheet, §"Registers") ----------
    //
    // Register A: the 32.768 kHz time base and a 1024 Hz periodic rate.
    // Register B: 24-hour mode, BCD data, every interrupt disabled.
    // Registers C and D are read to clear anything the reset left pending.
    a.movi8(AL, 0x0a);
    a.out_al(0x70);
    a.movi8(AL, 0x26);
    a.out_al(0x71);
    a.movi8(AL, 0x0b);
    a.out_al(0x70);
    a.movi8(AL, 0x02);
    a.out_al(0x71);
    a.movi8(AL, 0x0c);
    a.out_al(0x70);
    a.in_al(0x71);
    a.movi8(AL, 0x0d);
    a.out_al(0x70);
    a.in_al(0x71);

    // -- the BIOS Data Area --------------------------------------------------
    ds_bda(a);

    // The equipment word, from CMOS 0x14 — the byte the board's own RTC model
    // publishes, so the machine file stays the single place the answer is
    // written down.
    a.movi8(AL, 0x14);
    a.call(l.cmos_read);
    a.movi8(AH, 0);
    a.movto(Mem::abs(BDA_EQUIPMENT), AX);

    // Base memory, from CMOS 0x15/0x16, less the kilobyte the EBDA takes.
    a.movi8(AL, 0x15);
    a.call(l.cmos_read);
    a.movto8(Mem::abs(BDA_MEMSIZE), AL);
    a.movi8(AL, 0x16);
    a.call(l.cmos_read);
    a.movto8(Mem::abs(BDA_MEMSIZE + 1), AL);
    a.mov(AX, Mem::abs(BDA_MEMSIZE));
    a.dec(AX);
    a.movto(Mem::abs(BDA_MEMSIZE), AX);
    a.movmi(Mem::abs(BDA_EBDA), EBDA_SEGMENT);

    // The keyboard buffer: sixteen words, empty, with head and tail at its
    // start and the wrap-around bounds a program is allowed to move.
    a.movmi(Mem::abs(BDA_KBHEAD), BDA_KBBUF);
    a.movmi(Mem::abs(BDA_KBTAIL), BDA_KBBUF);
    a.movmi(Mem::abs(BDA_KBBUF_START), BDA_KBBUF);
    a.movmi(Mem::abs(BDA_KBBUF_END), BDA_KBBUF + KBBUF_BYTES);

    // The video state. 80x25 colour text on the colour CRTC address.
    a.movmi8(Mem::abs(BDA_VIDEO_MODE), 0x03);
    a.movmi(Mem::abs(BDA_COLUMNS), 80);
    a.movmi(Mem::abs(BDA_PAGE_SIZE), 0x1000);
    a.movmi(Mem::abs(BDA_PAGE_OFFSET), 0);
    a.movmi(Mem::abs(BDA_CURSOR), 0);
    a.movmi(Mem::abs(BDA_CURSOR_SHAPE), 0x0607);
    a.movmi8(Mem::abs(BDA_ACTIVE_PAGE), 0);
    a.movmi(Mem::abs(BDA_CRTC_PORT), 0x03d4);
    a.movmi8(Mem::abs(BDA_MODE_SELECT), 0x29);
    a.movmi8(Mem::abs(BDA_ROWS), 24);
    a.movmi(Mem::abs(BDA_CHAR_HEIGHT), 16);
    a.movmi8(Mem::abs(BDA_HD_COUNT), 0);

    // -- the EBDA, and the E820 map it holds ---------------------------------
    //
    // The map is built here rather than in the `INT 15h` handler because it
    // depends on the CMOS, and because a table in RAM is the thing ACPI and
    // SMBIOS will extend when they arrive.
    a.movi(AX, EBDA_SEGMENT);
    a.movsr(ES, AX);
    a.movmi8(Mem::abs(EBDA_SIZE).seg(ES), 1);

    a.movi(AX, SEGMENT);
    a.movsr(DS, AX);
    a.movi_label(SI, l.e820_template);
    a.movi(DI, EBDA_E820);
    a.movi(CX, E820_ENTRY * 4 / 2);
    a.rep();
    a.movsw();
    ds_bda(a);
    a.movmi(Mem::abs(EBDA_E820_COUNT).seg(ES), 4);

    // Entry 0's length and entry 1's base are both "the top of base memory",
    // which the CMOS has just told us. `mov eax,0` then a 16-bit load leaves
    // the upper half clear, which is the cheapest zero-extension there is.
    a.movi32(AX, 0);
    a.mov(AX, Mem::abs(BDA_MEMSIZE));
    a.shift32(Shift::SHL, AX, 10);
    a.movto32(Mem::abs(EBDA_E820 + 8).seg(ES), AX);
    a.movto32(Mem::abs(EBDA_E820 + E820_ENTRY).seg(ES), AX);

    // Entry 3's length: extended memory, which the CMOS reports in two pieces —
    // kilobytes between 1 MiB and 16 MiB at 0x30/0x31, and 64 KiB blocks above
    // 16 MiB at 0x34/0x35 (MC146818 data sheet plus the AT's CMOS map).
    a.movi8(AL, 0x34);
    a.call(l.cmos_read);
    a.mov8(CL, AL);
    a.movi8(AL, 0x35);
    a.call(l.cmos_read);
    a.mov8(CH, AL);
    a.movi32(DX, 0);
    a.mov(DX, CX);
    a.shift32(Shift::SHL, DX, 16);
    a.movi8(AL, 0x30);
    a.call(l.cmos_read);
    a.mov8(BL, AL);
    a.movi8(AL, 0x31);
    a.call(l.cmos_read);
    a.mov8(BH, AL);
    a.movi32(AX, 0);
    a.mov(AX, BX);
    a.shift32(Shift::SHL, AX, 10);
    a.alu32(Alu::ADD, AX, DX);
    a.movto32(Mem::abs(EBDA_E820 + 3 * E820_ENTRY + 8).seg(ES), AX);

    // -- the PCI configuration window ----------------------------------------
    //
    // Before the option-ROM scan, not after: a PCI expansion ROM is entitled to
    // call `INT 1Ah AH=B1h` to find the function it belongs to (*PCI Firmware
    // Specification* 3.0 §3.2), and a video BIOS built for a PCI card does
    // exactly that. The service has to answer by the time the first ROM is
    // entered.
    a.call(l.pci_detect);

    // -- option ROMs ---------------------------------------------------------
    //
    // PCI Firmware Specification 3.0 §5.2.2 and the ISA convention it inherits:
    // a 2 KiB-aligned `0x55 0xAA`, a length in 512-byte blocks at offset 2, a
    // whole-image checksum of zero, and an entry point at offset 3 reached by a
    // far call. The window is 0xC0000-0xDFFFF, which is where this board puts
    // its video ROM socket.
    a.movi(BX, 0xc000);
    let scan_loop = a.here_label();
    let scan_step = a.label();
    a.movsr(ES, BX);
    a.alui(Alu::CMP, Mem::abs(0).seg(ES), 0xaa55);
    a.jcc(Cc::NE, scan_step);
    a.mov8(AL, Mem::abs(2).seg(ES));
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::E, scan_step);
    a.mov8(CL, AL);
    a.movi8(CH, 0);
    a.shift(Shift::SHL, CX, 9);
    a.movi(SI, 0);
    a.movi8(AL, 0);
    let csum = a.here_label();
    a.alu8(Alu::ADD, AL, Mem::si(0).seg(ES));
    a.inc(SI);
    a.dec(CX);
    a.jcc(Cc::NE, csum);
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::NE, scan_step);

    // Enter it. There is no `CALL FAR imm16:imm16`, so the far return address
    // is pushed by hand and the entry is reached with `RETF` — which is exactly
    // what a far call leaves on the stack, so the ROM's own `RETF` comes back
    // here.
    let back = a.label();
    a.push(BX);
    a.pushs(crate::fw::asm16::CS);
    a.pushi_label(back);
    a.push(BX);
    a.pushi(0x0003);
    a.retf();
    a.bind(back);
    a.pop(BX);
    ds_bda(a);
    a.bind(scan_step);
    a.alui(Alu::ADD, BX, 0x80);
    a.alui(Alu::CMP, BX, 0xe000);
    a.jcc(Cc::NE, scan_loop);

    // -- video, and the banner ----------------------------------------------
    //
    // Through `INT 10h`, not through the hardware: if an option ROM installed
    // its own handler above, that is the one that should set the mode.
    a.movi(AX, 0x0003);
    a.int(0x10);

    let banner = a.label();
    let base_msg = a.label();
    let ext_msg = a.label();
    let end_msg = a.label();
    a.movi_label(SI, banner);
    a.call(l.puts);
    a.mov(AX, Mem::abs(BDA_MEMSIZE));
    a.call(l.put_dec);
    a.movi_label(SI, base_msg);
    a.call(l.puts);
    a.movi8(AL, 0x30);
    a.call(l.cmos_read);
    a.mov8(BL, AL);
    a.movi8(AL, 0x31);
    a.call(l.cmos_read);
    a.mov8(BH, AL);
    a.mov(AX, BX);
    a.call(l.put_dec);
    a.movi_label(SI, ext_msg);
    a.call(l.puts);

    // -- the keyboard controller (Intel 8042 data sheet) --------------------
    //
    // Self test (0xAA, which answers 0x55), then the command byte: bit 0
    // enables the keyboard interrupt, bit 2 is the system flag a warm boot
    // looks at, and bit 6 turns on the set-2-to-set-1 translation that `INT 09h`
    // below assumes. Then the interface is enabled and the keyboard is told to
    // scan.
    a.call(l.kbc_wait_write);
    a.movi8(AL, 0xaa);
    a.out_al(0x64);
    a.call(l.kbc_wait_read);
    a.call(l.kbc_wait_write);
    a.movi8(AL, 0x60);
    a.out_al(0x64);
    a.call(l.kbc_wait_write);
    a.movi8(AL, 0x45);
    a.out_al(0x60);
    a.call(l.kbc_wait_write);
    a.movi8(AL, 0xae);
    a.out_al(0x64);
    a.call(l.kbc_wait_write);
    a.movi8(AL, 0xf4);
    a.out_al(0x60);
    a.call(l.kbc_wait_read);

    // -- the fixed disk ------------------------------------------------------
    detect_hd(a, l);

    a.movi_label(SI, end_msg);
    a.call(l.puts);

    // Hand over. Interrupts on: the tick has to run for `INT 1Ah` to mean
    // anything, and a boot sector that waits on one would otherwise hang.
    a.sti();
    a.int(0x19);
    let wedged = a.here_label();
    a.hlt();
    a.jmp(wedged);

    // -- the routines POST calls --------------------------------------------

    // `set_vector`: AL is the vector number, BX the handler's offset. `DS` must
    // be zero, which it is everywhere this is called from.
    a.bind(l.set_vector);
    a.push(DI);
    a.push(AX);
    a.movi8(AH, 0);
    a.shift(Shift::SHL, AX, 2);
    a.mov(DI, AX);
    a.movto(Mem::di(0), BX);
    a.movmi(Mem::di(2), SEGMENT);
    a.pop(AX);
    a.pop(DI);
    a.ret();

    // `cmos_read`: AL is the CMOS index, and comes back holding its byte.
    // Writing an index below 0x80 leaves NMI enabled, which is what the index
    // register's bit 7 means (MC146818 data sheet).
    a.bind(l.cmos_read);
    a.out_al(0x70);
    a.in_al(0x71);
    a.ret();

    // `kbc_wait_write`: spin until the 8042's input buffer is empty (status bit
    // 1 clear), with a bounded count so a dead controller costs a moment rather
    // than the machine.
    a.bind(l.kbc_wait_write);
    a.push(AX);
    a.push(CX);
    a.movi(CX, 0);
    let ww = a.here_label();
    let ww_done = a.label();
    a.in_al(0x64);
    a.testi8(AL, 0x02);
    a.jcc(Cc::E, ww_done);
    a.dec(CX);
    a.jcc(Cc::NE, ww);
    a.bind(ww_done);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // `kbc_wait_read`: spin until the output buffer is full (status bit 0 set)
    // and take the byte, which comes back in AL. A timeout answers 0xFF.
    a.bind(l.kbc_wait_read);
    a.push(CX);
    a.movi(CX, 0);
    let wr = a.here_label();
    let wr_have = a.label();
    let wr_done = a.label();
    a.in_al(0x64);
    a.testi8(AL, 0x01);
    a.jcc(Cc::NE, wr_have);
    a.dec(CX);
    a.jcc(Cc::NE, wr);
    a.movi8(AL, 0xff);
    a.jmp(wr_done);
    a.bind(wr_have);
    a.in_al(0x60);
    a.bind(wr_done);
    a.pop(CX);
    a.ret();

    // -- the strings and the E820 template ----------------------------------

    a.bind(banner);
    a.db(b"rsemu BIOS, ");
    a.db(&[0]);
    a.bind(base_msg);
    a.db(b"K base, ");
    a.db(&[0]);
    a.bind(ext_msg);
    a.db(b"K extended\r\n");
    a.db(&[0]);
    a.bind(end_msg);
    a.db(b"Booting.\r\n");
    a.db(&[0]);

    e820_template(a, l);
}

/// The `INT 15h AX=E820h` map, as it stands before POST patches the two sizes
/// it cannot know until it has read the CMOS.
///
/// Type 1 is usable memory and type 2 is reserved (ACPI 6.5 §15.2, "Address
/// Range Types"), which is the whole of the vocabulary a legacy map needs.
fn e820_template(a: &mut Asm, l: &Labels) {
    /// One entry: a 64-bit base, a 64-bit length, and a 32-bit type.
    fn entry(a: &mut Asm, base: u64, length: u64, kind: u32) {
        a.db(&base.to_le_bytes());
        a.db(&length.to_le_bytes());
        a.db(&kind.to_le_bytes());
    }

    a.bind(l.e820_template);
    // Conventional memory, up to the EBDA. The length is patched.
    entry(a, 0x0000_0000, 0x0009_fc00, 1);
    // The EBDA. The base is patched to match.
    entry(a, 0x0009_fc00, 0x0000_0400, 2);
    // This ROM's own segment, which is never handed to an operating system.
    entry(a, 0x000f_0000, 0x0001_0000, 2);
    // Extended memory. The length is patched.
    entry(a, 0x0010_0000, 0x0000_0000, 1);
}

/// Find the drive on the primary IDE channel and record what it says about
/// itself.
///
/// T13/1410D §6.17: `IDENTIFY DEVICE` returns 256 words, of which word 1 is the
/// number of logical cylinders, word 3 the logical heads, word 6 the logical
/// sectors per track, word 49 bit 9 whether LBA is supported, and words 60-61
/// the LBA28 capacity.
fn detect_hd(a: &mut Asm, l: &Labels) {
    let none = a.label();
    let done = a.label();

    // The device control register's nIEN bit, so the drive never raises IRQ14:
    // this firmware polls, and an unmasked interrupt with no handler is a way
    // to wedge a machine rather than a feature (T13/1410D §7.9).
    a.movi(DX, 0x03f6);
    a.movi8(AL, 0x02);
    a.out_dx_al();

    // Select device 0. Bit 7 and bit 5 are obsolete-but-set, bit 4 is the
    // device number (T13/1410D §7.10).
    a.movi(DX, 0x01f6);
    a.movi8(AL, 0xa0);
    a.out_dx_al();
    a.call(l.ata_wait_ready);
    a.jcc(Cc::B, none);

    a.movi(DX, 0x01f7);
    a.movi8(AL, 0xec);
    a.out_dx_al();
    a.in_al_dx();
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::E, none);
    a.call(l.ata_wait_drq);
    a.jcc(Cc::B, none);

    a.movi(AX, 0);
    a.movsr(ES, AX);
    a.movi(DI, IDENTIFY_BUFFER);
    a.movi(DX, 0x01f0);
    a.movi(CX, 256);
    a.rep();
    a.insw();

    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movi(AX, EBDA_SEGMENT);
    a.movsr(ES, AX);
    a.movi(SI, IDENTIFY_BUFFER);
    for (word, field) in [
        (1u16, EBDA_HD_CYLINDERS),
        (3, EBDA_HD_HEADS),
        (6, EBDA_HD_SECTORS),
    ] {
        a.mov(AX, Mem::si(i32::from(word) * 2));
        a.movto(Mem::abs(field).seg(ES), AX);
    }
    a.mov(AX, Mem::si(60 * 2));
    a.movto(Mem::abs(EBDA_HD_CAPACITY).seg(ES), AX);
    a.mov(AX, Mem::si(61 * 2));
    a.movto(Mem::abs(EBDA_HD_CAPACITY + 2).seg(ES), AX);
    // Word 49's bit 9 is the LBA-supported bit, which is bit 1 of its high byte.
    a.movi8(AL, 0x01);
    a.mov8(AH, Mem::si(49 * 2 + 1));
    let no_lba = a.label();
    a.testi8(AH, 0x02);
    a.jcc(Cc::E, no_lba);
    a.alui8(Alu::OR, AL, 0x02);
    a.bind(no_lba);
    a.movto8(Mem::abs(EBDA_HD_FLAGS).seg(ES), AL);

    ds_bda(a);
    a.movmi8(Mem::abs(BDA_HD_COUNT), 1);
    a.jmp(done);

    a.bind(none);
    ds_bda(a);
    a.bind(done);
}
