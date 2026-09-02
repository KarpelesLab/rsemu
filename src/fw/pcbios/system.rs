//! The rest of the interrupt table: the tick, the memory map, the equipment
//! word, the clock, the bootstrap, and the stubs everything else lands on.
//!
//! # Sources
//!
//! * Ralf Brown's Interrupt List for `INT 08h`, `INT 11h`, `INT 12h`,
//!   `INT 15h` (`AH=87h`, `AH=88h`, `AX=E801h`, `AX=E820h`), `INT 18h`,
//!   `INT 19h` and `INT 1Ah`.
//! * The **Intel SDM** Volume 3A for what `AH=87h` borrows: §3.4.3 for the
//!   hidden descriptor cache a segment register carries, §3.4.5 for the
//!   descriptor layout the caller fills in, and §9.9.2 for what survives a
//!   switch back to real-address mode.
//! * ACPI 6.5 §15.2 for the `E820` entry layout and its address-range types.
//! * Motorola MC146818 data sheet for the clock registers `INT 1Ah` reads, and
//!   the AT's CMOS map for the century byte at 0x32.
//! * Intel 8259A data sheet for the non-specific end-of-interrupt, `OCW2` with
//!   `EOI` set — 0x20 to the command port.

use super::{
    BDA_EQUIPMENT, BDA_MEMSIZE, BDA_MOTOR, BDA_MOTOR_TIMEOUT, BDA_ROLLOVER, BDA_TICKS, E820_ENTRY,
    EBDA_E820, EBDA_E820_COUNT, EBDA_GDTR, EBDA_SEGMENT, F_AX, F_CX, F_DX, FLAG_CF, Labels,
    POST_STACK, clear_cf, ds_bda, enter, leave, load_seg, set_cf,
};
use crate::fw::asm16::{
    AH, AL, AX, Alu, Asm, BH, BL, BP, BX, CS, CX, Cc, DI, DL, DS, DX, ES, Mem, SI, SP, SS, Shift,
};

// The `INT 15h` frame. That handler alone uses `PUSHAD`, because `E820` is
// specified in terms of `EAX`, `EBX`, `ECX` and `EDX` and a 16-bit `PUSHA`
// would drop the halves the caller cares about. Four bytes per register, so
// every offset differs from the `PUSHA` frame the other handlers use.

/// The saved `ESI` — where `AH=87h`'s descriptor table is pointed at.
const G_ESI: i32 = 4;
/// The saved `EBX` — `E820`'s continuation index.
const G_EBX: i32 = 16;
/// The saved `EDX` — where `'SMAP'` arrives.
const G_EDX: i32 = 20;
/// The saved `ECX` — the caller's buffer size, and the length written back.
const G_ECX: i32 = 24;
/// The saved `EAX` — the function number, and `'SMAP'` on return.
const G_EAX: i32 = 28;
/// The caller's `FLAGS` in the `PUSHAD` frame.
const G_FLAGS: i32 = 40;

/// The signature `E820` requires in `EDX` and answers with in `EAX`.
const SMAP: u32 = 0x534d_4150;

/// How many ticks make a day: 24 x 60 x 60 x 1_193_182 / 65536, rounded, which
/// is the number every PC has rolled its midnight counter over at.
const TICKS_PER_DAY: u32 = 0x0018_00b0;

/// Emit everything that is not video, keyboard or disk.
#[allow(clippy::too_many_lines)]
pub(super) fn emit(a: &mut Asm, l: &Labels) {
    // -- INT 08h, IRQ0 -------------------------------------------------------
    //
    // The tick. No `STI`, and the end-of-interrupt goes out before `INT 1Ch` is
    // called, so a slow user hook delays the next tick rather than blocking
    // every other interrupt on the master.
    a.bind(l.int08);
    a.pushs(DS);
    a.push(AX);
    a.push(DX);
    ds_bda(a);
    let no_roll = a.label();
    let no_motor = a.label();
    a.incm32(Mem::abs(BDA_TICKS));
    a.alui32(Alu::CMP, Mem::abs(BDA_TICKS), TICKS_PER_DAY);
    a.jcc(Cc::B, no_roll);
    a.movmi32(Mem::abs(BDA_TICKS), 0);
    a.incm8(Mem::abs(BDA_ROLLOVER));
    a.bind(no_roll);
    // The diskette motor's countdown. Nothing starts a motor yet, so this only
    // ever sees zero — it is here because the field is part of the BDA's ABI
    // and a program that sets it expects it to run down.
    a.alui8(Alu::CMP, Mem::abs(BDA_MOTOR_TIMEOUT), 0);
    a.jcc(Cc::E, no_motor);
    a.decm8(Mem::abs(BDA_MOTOR_TIMEOUT));
    a.jcc(Cc::NE, no_motor);
    a.movi(DX, 0x03f2);
    a.movi8(AL, 0x0c);
    a.out_dx_al();
    a.movmi8(Mem::abs(BDA_MOTOR), 0);
    a.bind(no_motor);
    a.movi8(AL, 0x20);
    a.out_al(0x20);
    a.pop(DX);
    a.pop(AX);
    a.pops(DS);
    a.int(0x1c);
    a.iret();

    // -- INT 11h and INT 12h -------------------------------------------------
    //
    // Both return a value in AX and nothing else, so neither can use the frame
    // the service handlers share: `POPA` would put the caller's AX back.
    a.bind(l.int11);
    a.pushs(DS);
    ds_bda(a);
    a.mov(AX, Mem::abs(BDA_EQUIPMENT));
    a.pops(DS);
    a.iret();

    a.bind(l.int12);
    a.pushs(DS);
    ds_bda(a);
    a.mov(AX, Mem::abs(BDA_MEMSIZE));
    a.pops(DS);
    a.iret();

    // -- INT 15h -------------------------------------------------------------
    a.bind(l.int15);
    a.sti();
    a.pushs(DS);
    a.pushs(ES);
    a.pushad();
    a.mov(BP, SP);
    a.cld();

    let ext_size = a.label();
    let block_move = a.label();
    let e8xx = a.label();
    let e801 = a.label();
    let e820 = a.label();
    let fail = a.label();
    let g_done = a.label();

    a.mov8(AH, Mem::bp(G_EAX + 1));
    a.alui8(Alu::CMP, AH, 0x88);
    a.jcc(Cc::E, ext_size);
    a.alui8(Alu::CMP, AH, 0x87);
    a.jcc(Cc::E, block_move);
    a.alui8(Alu::CMP, AH, 0xe8);
    a.jcc(Cc::E, e8xx);
    a.jmp(fail);

    a.bind(e8xx);
    a.mov8(AL, Mem::bp(G_EAX));
    a.alui8(Alu::CMP, AL, 0x20);
    a.jcc(Cc::E, e820);
    a.alui8(Alu::CMP, AL, 0x01);
    a.jcc(Cc::E, e801);
    a.jmp(fail);

    // AH=88h: extended memory in kilobytes, straight out of the CMOS pair the
    // POST already trusts.
    a.bind(ext_size);
    a.movi8(AL, 0x30);
    a.call(l.cmos_read);
    a.mov8(BL, AL);
    a.movi8(AL, 0x31);
    a.call(l.cmos_read);
    a.mov8(BH, AL);
    a.movto(Mem::bp(G_EAX), BX);
    a.alui(Alu::AND, Mem::bp(G_FLAGS), !FLAG_CF);
    a.jmp(g_done);

    // AX=E801h: the 1-16 MiB region in kilobytes and everything above 16 MiB in
    // 64 KiB blocks, each answered in two registers because two conventions
    // existed and callers check both.
    a.bind(e801);
    let capped = a.label();
    a.movi8(AL, 0x30);
    a.call(l.cmos_read);
    a.mov8(BL, AL);
    a.movi8(AL, 0x31);
    a.call(l.cmos_read);
    a.mov8(BH, AL);
    a.alui(Alu::CMP, BX, 0x3c00);
    a.jcc(Cc::BE, capped);
    a.movi(BX, 0x3c00);
    a.bind(capped);
    a.movto(Mem::bp(G_EAX), BX);
    a.movto(Mem::bp(G_ECX), BX);
    a.movi8(AL, 0x34);
    a.call(l.cmos_read);
    a.mov8(BL, AL);
    a.movi8(AL, 0x35);
    a.call(l.cmos_read);
    a.mov8(BH, AL);
    a.movto(Mem::bp(G_EBX), BX);
    a.movto(Mem::bp(G_EDX), BX);
    a.alui(Alu::AND, Mem::bp(G_FLAGS), !FLAG_CF);
    a.jmp(g_done);

    // AX=E820h: one entry per call out of the table POST built in the EBDA.
    // ES:DI is the caller's and is left alone; EBX is the continuation index,
    // and comes back zero on the last entry.
    a.bind(e820);
    a.mov32(AX, Mem::bp(G_EDX));
    a.alui32(Alu::CMP, AX, SMAP);
    a.jcc(Cc::NE, fail);
    load_seg(a, DS, EBDA_SEGMENT, AX);
    a.movi32(CX, 0);
    a.mov(CX, Mem::abs(EBDA_E820_COUNT));
    a.mov32(BX, Mem::bp(G_EBX));
    a.alu32(Alu::CMP, BX, CX);
    a.jcc(Cc::AE, fail);

    // The source entry, at table + index x 20. `MUL` writes DX, which is a
    // scratch register here because the caller's is in the frame.
    a.mov(AX, BX);
    a.movi(CX, E820_ENTRY);
    a.mul(CX);
    a.mov(SI, AX);
    a.alui(Alu::ADD, SI, EBDA_E820);
    a.push(DI);
    a.movi(CX, E820_ENTRY / 2);
    a.rep();
    a.movsw();
    a.pop(DI);

    a.movi32(AX, SMAP);
    a.movto32(Mem::bp(G_EAX), AX);
    a.movi32(CX, u32::from(E820_ENTRY));
    a.movto32(Mem::bp(G_ECX), CX);
    let not_last = a.label();
    a.movi32(CX, 0);
    a.mov(CX, Mem::abs(EBDA_E820_COUNT));
    a.mov32(BX, Mem::bp(G_EBX));
    a.alui32(Alu::ADD, BX, 1);
    a.alu32(Alu::CMP, BX, CX);
    a.jcc(Cc::B, not_last);
    a.movi32(BX, 0);
    a.bind(not_last);
    a.movto32(Mem::bp(G_EBX), BX);
    a.alui(Alu::AND, Mem::bp(G_FLAGS), !FLAG_CF);
    a.jmp(g_done);

    // AH=87h, block move. `ES:SI` points at a table of six 8-byte segment
    // descriptors and `CX` holds a *word* count, at most 8000h — one segment's
    // worth, which is the whole reason the interface is shaped this way. Entry
    // 2 describes the source and entry 3 the destination; entries 1, 4 and 5
    // are the caller's to leave blank and a real BIOS's to fill in with the
    // table's own descriptor, its code segment and its stack. **This one does
    // not fill them**, because it never loads a selector that names one, and
    // writing into the caller's table to satisfy a convention nobody reads
    // back would be a side effect rather than a service (RBIL, `INT 15h`
    // `AH=87h`).
    //
    // How the copy is done, and why it is not a full mode switch:
    //
    // The descriptor caches are architectural state: a segment register loaded
    // in protected mode keeps its base and limit when `CR0.PE` goes back to
    // zero, and real-address mode then addresses through them (Intel SDM
    // Vol. 3A §3.4.3 for the hidden part of a segment register, §9.9.2 for
    // what survives the switch back). So `PE` is set just long enough to load
    // `DS` from the source descriptor and `ES` from the destination one, and
    // the transfer itself is an ordinary real-mode `REP MOVSW` reaching
    // wherever those two descriptors point — including above the first
    // megabyte, which is the only reason anybody calls this function.
    //
    // Interrupts are off across the whole of it: while `PE` is set the
    // processor would take an interrupt through a *protected-mode* IDT, and
    // the one loaded is the real-mode vector table. `src/cpu/x86/prot.rs`
    // models the caches this way for exactly this idiom, and real 386 and 486
    // silicon does too.
    //
    // Nothing unreal survives the call: the epilogue's `POP ES` and `POP DS`
    // reload both registers in real mode, which puts their bases back to
    // `selector * 16` and their limits back to 64 KiB. The caller gets its own
    // segments, not this handler's.
    //
    // `CX` is taken at face value. RBIL documents a maximum of 8000h words and
    // this does not enforce it: the descriptors the caller supplied carry the
    // limits, so a caller that asks for more than its own segment holds gets a
    // fault rather than a silently truncated copy, which is the more useful of
    // the two wrong answers.
    //
    // A20 is not re-checked here. POST opens it and nothing this firmware runs
    // closes it, so `AH=03h` — the "A20 failed" status — is a code this
    // handler has no way to produce and does not pretend to.
    a.bind(block_move);
    let bad_descriptor = a.label();
    let flush = a.label();
    a.cli();
    // `ES` is still the caller's — nothing between the handler's prologue and
    // here touches it — but `SI` comes out of the frame rather than being
    // assumed, so a function added to the dispatch above cannot quietly break
    // this one by using it as scratch.
    a.mov(SI, Mem::bp(G_ESI));
    // Neither descriptor being present would fault the moment its selector is
    // loaded, and a fault with `PE` set and the real-mode vector table still
    // installed is not a diagnosable event. Checking the access byte first
    // turns it into a status code (SDM Vol. 3A §3.4.5, the P bit).
    a.testi8(Mem::si(0x15).seg(ES), 0x80);
    a.jcc(Cc::E, bad_descriptor);
    a.testi8(Mem::si(0x1d).seg(ES), 0x80);
    a.jcc(Cc::E, bad_descriptor);

    // The table's linear address, `ES * 16 + SI`, which is what `LGDT` wants.
    a.movrs(AX, ES);
    a.movi32(BX, 0);
    a.mov(BX, AX);
    a.shift32(Shift::SHL, BX, 4);
    a.movi32(AX, 0);
    a.mov(AX, Mem::bp(G_ESI));
    a.alu32(Alu::ADD, BX, AX);
    load_seg(a, DS, EBDA_SEGMENT, AX);
    // Six descriptors, so the limit is 0x2f: the largest selector this handler
    // ever loads is 0x18, and a limit that covered more of the caller's memory
    // than the interface defines would be this firmware's mistake to make.
    a.movmi(Mem::abs(EBDA_GDTR), 6 * 8 - 1);
    a.movto32(Mem::abs(EBDA_GDTR + 2), BX);
    a.lgdt(Mem::abs(EBDA_GDTR));

    // The word count, which the `REP MOVSW` below consumes out of `CX`.
    a.movi32(CX, 0);
    a.mov(CX, Mem::bp(G_ECX));

    a.read_cr0(AX);
    a.alui32(Alu::OR, AX, 1);
    a.write_cr0(AX);
    a.movi(BX, 0x10); // the source descriptor, entry 2
    a.movsr(DS, BX);
    a.movi(BX, 0x18); // the destination descriptor, entry 3
    a.movsr(ES, BX);
    a.read_cr0(AX);
    a.alui32(Alu::AND, AX, !1u32);
    a.write_cr0(AX);
    // A jump to the next instruction, which is what flushes a prefetch queue
    // that was filled under the other mode.
    a.jmp(flush);
    a.bind(flush);
    a.movi(SI, 0);
    a.movi(DI, 0);
    a.cld();
    a.rep();
    a.movsw();
    a.movmi8(Mem::bp(G_EAX + 1), 0x00);
    a.alui(Alu::AND, Mem::bp(G_FLAGS), !FLAG_CF);
    a.jmp(g_done);

    // AH=02h is "exception occurred", which is the nearest thing RBIL has to
    // "your descriptor would have faulted".
    a.bind(bad_descriptor);
    a.movmi8(Mem::bp(G_EAX + 1), 0x02);
    a.alui(Alu::OR, Mem::bp(G_FLAGS), FLAG_CF);
    a.jmp(g_done);

    // Everything else. AH=86h is "unsupported function", which is what a caller
    // that probes for a service expects to see.
    a.bind(fail);
    a.movmi8(Mem::bp(G_EAX + 1), 0x86);
    a.alui(Alu::OR, Mem::bp(G_FLAGS), FLAG_CF);

    a.bind(g_done);
    a.popad();
    a.pops(ES);
    a.pops(DS);
    a.iret();

    // -- INT 1Ah -------------------------------------------------------------
    a.bind(l.int1a);
    enter(a);
    ds_bda(a);
    let get_ticks = a.label();
    let set_ticks = a.label();
    let get_time = a.label();
    let get_date = a.label();
    let t_done = a.label();
    let t_fail = a.label();
    a.mov8(AH, Mem::bp(F_AX + 1));
    for (function, target) in [
        (0x00u8, get_ticks),
        (0x01, set_ticks),
        (0x02, get_time),
        (0x04, get_date),
    ] {
        a.alui8(Alu::CMP, AH, function);
        a.jcc(Cc::E, target);
    }
    a.jmp(t_fail);

    // AH=00h: CX:DX is the tick count and AL the midnight flag, which reading
    // clears — the one BIOS call with a side effect a debugger must not cause,
    // and the reason `MemAttrs::debug` exists on the other side of this fence.
    a.bind(get_ticks);
    a.mov(AX, Mem::abs(BDA_TICKS + 2));
    a.movto(Mem::bp(F_CX), AX);
    a.mov(AX, Mem::abs(BDA_TICKS));
    a.movto(Mem::bp(F_DX), AX);
    a.mov8(AL, Mem::abs(BDA_ROLLOVER));
    a.movto8(Mem::bp(F_AX), AL);
    a.movmi8(Mem::abs(BDA_ROLLOVER), 0);
    a.jmp(t_done);

    a.bind(set_ticks);
    a.mov(AX, Mem::bp(F_CX));
    a.movto(Mem::abs(BDA_TICKS + 2), AX);
    a.mov(AX, Mem::bp(F_DX));
    a.movto(Mem::abs(BDA_TICKS), AX);
    a.movmi8(Mem::abs(BDA_ROLLOVER), 0);
    a.jmp(t_done);

    // AH=02h: the wall clock, in BCD as the MC146818 keeps it. DL is the
    // daylight-saving flag, which this board's RTC has no opinion about.
    a.bind(get_time);
    for (index, slot) in [(0x04u8, F_CX + 1), (0x02, F_CX), (0x00, F_DX + 1)] {
        a.movi8(AL, index);
        a.call(l.cmos_read);
        a.movto8(Mem::bp(slot), AL);
    }
    a.movmi8(Mem::bp(F_DX), 0);
    a.jmp(t_done);

    // AH=04h: the date, century first.
    a.bind(get_date);
    for (index, slot) in [
        (0x32u8, F_CX + 1),
        (0x09, F_CX),
        (0x08, F_DX + 1),
        (0x07, F_DX),
    ] {
        a.movi8(AL, index);
        a.call(l.cmos_read);
        a.movto8(Mem::bp(slot), AL);
    }

    a.bind(t_done);
    clear_cf(a);
    let t_leave = a.label();
    a.jmp(t_leave);
    a.bind(t_fail);
    set_cf(a);
    a.bind(t_leave);
    leave(a);

    // -- INT 19h, the bootstrap ---------------------------------------------
    //
    // The fixed disk first, then the diskette. A boot sector is one sector at
    // cylinder 0, head 0, sector 1, and it is a boot sector because its last
    // two bytes are 0x55 0xAA — the whole of the test, which is why a disk
    // full of anything else is politely declined rather than executed.
    a.bind(l.int19);
    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, POST_STACK);
    a.sti();
    let try_boot = a.label();
    a.movi8(DL, 0x80);
    a.call(try_boot);
    a.movi8(DL, 0x00);
    a.call(try_boot);
    a.int(0x18);

    a.bind(try_boot);
    let no_boot = a.label();
    let booting = a.label();
    a.push(DX);
    a.movi8(AH, 0x00);
    a.int(0x13);
    a.pop(DX);
    a.push(DX);
    a.movi(AX, 0x0201);
    a.movi(CX, 0x0001);
    a.movi8(crate::fw::asm16::DH, 0x00);
    a.movi(BX, 0x7c00);
    a.int(0x13);
    a.pop(DX);
    a.jcc(Cc::B, no_boot);
    a.alui(Alu::CMP, Mem::abs(0x7dfe), 0xaa55);
    a.jcc(Cc::NE, no_boot);
    a.push(DX);
    a.movi_label(SI, booting);
    a.call(l.puts);
    a.pop(DX);
    // DL is the drive the sector came off, which is the one thing a boot sector
    // is entitled to find in a register.
    a.jmpf(0x0000, 0x7c00);
    a.bind(no_boot);
    a.ret();

    // -- INT 18h -------------------------------------------------------------
    //
    // Historically ROM BASIC. There is none, so this says so and stops, with
    // interrupts on so a keystroke still reaches the buffer and a debugger
    // attached to the machine sees a live core rather than a shut-down one.
    //
    // **`STI` is what makes that true.** This is entered by an `INT`
    // instruction, which clears `IF` (SDM Vol 2A, `INT n`), so a park loop that
    // did not turn interrupts back on would `HLT` a processor nothing could
    // wake: no timer tick, no keystroke, no `INT 09h` filling the buffer this
    // comment promises, and a `HLT` with `IF` clear is the one shutdown state
    // an x86 has no way out of but `RESET`. It is placed before the loop rather
    // than inside it because `IF` stays set once set, and after `puts` so the
    // message cannot be interleaved with a handler's own output.
    a.bind(l.int18);
    let no_disk = a.label();
    a.movi_label(SI, no_disk);
    a.call(l.puts);
    a.sti();
    let park = a.here_label();
    a.hlt();
    a.jmp(park);

    // -- the stubs -----------------------------------------------------------
    //
    // `unimplemented` sets the caller's carry flag, which is how every `INT`
    // ABI in this file reports "I do not know that function". `[bp+6]` is the
    // flags word: `[bp+0]` is the pushed BP, then IP and CS.
    a.bind(l.unimplemented);
    a.push(BP);
    a.mov(BP, SP);
    a.alui(Alu::OR, Mem::bp(6), FLAG_CF);
    a.pop(BP);
    a.iret();

    a.bind(l.iret_stub);
    a.iret();

    // A hardware interrupt nothing claims still has to be acknowledged, or the
    // controller's in-service bit stays set and every lower-priority line goes
    // quiet for ever (8259A data sheet, OCW2).
    a.bind(l.eoi_master);
    a.push(AX);
    a.movi8(AL, 0x20);
    a.out_al(0x20);
    a.pop(AX);
    a.iret();

    a.bind(l.eoi_slave);
    a.push(AX);
    a.movi8(AL, 0x20);
    a.out_al(0xa0);
    a.out_al(0x20);
    a.pop(AX);
    a.iret();

    // -- strings -------------------------------------------------------------
    a.bind(booting);
    a.db(b"Booting from 0000:7c00\r\n");
    a.db(&[0]);
    a.bind(no_disk);
    a.db(b"No bootable device.\r\n");
    a.db(&[0]);

    let _ = CS;
}
