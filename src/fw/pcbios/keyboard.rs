//! `INT 09h` and `INT 16h` — the keyboard, from the 8042 to the type-ahead
//! buffer a program reads.
//!
//! `INT 09h` is the hardware half: it takes one byte from port 0x60, tracks the
//! shift state, turns a make code into a character through the tables below and
//! puts the pair in the BDA's ring. `INT 16h` is the software half and only
//! ever touches the ring.
//!
//! # Sources
//!
//! * Intel 8042 data sheet for the two ports and the status bits; the
//!   controller command byte POST writes has translation *on*, so the codes
//!   arriving here are **set 1**.
//! * Ralf Brown's Interrupt List, `INT 16h` functions 00h-02h and 10h-12h, for
//!   what a caller gets back.
//! * The set-1 code assignments and the US layout they map to are the
//!   keyboard's own documented encoding, restated on the OSDev wiki's "PS/2
//!   Keyboard" page.
//!
//! # What it does not decode
//!
//! Extended (`E0`-prefixed) codes are consumed and dropped: the arrow keys, the
//! right-hand modifiers and the grey navigation block produce nothing. Decoding
//! them needs a second table and a prefix state, and every key this firmware's
//! own callers press is in the base set. Caps lock and num lock are not
//! tracked either — releasing either shift key clears both shift bits, which is
//! visible only if a program holds one shift down while releasing the other.

use super::{
    BDA_KBBUF_END, BDA_KBBUF_START, BDA_KBFLAG, BDA_KBHEAD, BDA_KBTAIL, F_AX, F_FLAGS, FLAG_ZF,
    Labels, ds_bda, enter, leave,
};
use crate::fw::asm16::{AH, AL, AX, Alu, Asm, BH, BL, BX, CS, Cc, DI, DL, DS, DX, Mem, SI};

/// How many make codes the tables cover: 0x00 through 0x58, which is every key
/// on the base AT keyboard.
const KEY_TABLE_LEN: usize = 0x59;

/// Write `text` into `table` starting at `at`.
fn put(table: &mut [u8; KEY_TABLE_LEN], at: usize, text: &[u8]) {
    table[at..at + text.len()].copy_from_slice(text);
}

/// Set 1 make code to ASCII, unshifted, US layout. Zero means the key produces
/// no character and is reported by its scan code alone.
fn plain_table() -> [u8; KEY_TABLE_LEN] {
    let mut t = [0u8; KEY_TABLE_LEN];
    t[0x01] = 0x1b; // escape
    put(&mut t, 0x02, b"1234567890-=");
    t[0x0e] = 0x08; // backspace
    t[0x0f] = 0x09; // tab
    put(&mut t, 0x10, b"qwertyuiop[]");
    t[0x1c] = 0x0d; // enter
    put(&mut t, 0x1e, b"asdfghjkl;'`");
    put(&mut t, 0x2b, b"\\zxcvbnm,./");
    t[0x37] = b'*'; // the keypad's asterisk
    t[0x39] = b' ';
    put(&mut t, 0x47, b"789-456+1230.");
    t
}

/// The same, with either shift key down.
fn shift_table() -> [u8; KEY_TABLE_LEN] {
    let mut t = [0u8; KEY_TABLE_LEN];
    t[0x01] = 0x1b;
    put(&mut t, 0x02, b"!@#$%^&*()_+");
    t[0x0e] = 0x08;
    t[0x0f] = 0x09;
    put(&mut t, 0x10, b"QWERTYUIOP{}");
    t[0x1c] = 0x0d;
    put(&mut t, 0x1e, b"ASDFGHJKL:\"~");
    put(&mut t, 0x2b, b"|ZXCVBNM<>?");
    t[0x37] = b'*';
    t[0x39] = b' ';
    put(&mut t, 0x47, b"789-456+1230.");
    t
}

/// Emit `INT 09h`, `INT 16h`, the ring-buffer helper and the two tables.
#[allow(clippy::too_many_lines)]
pub(super) fn emit(a: &mut Asm, l: &Labels) {
    // -- INT 09h, IRQ1 -------------------------------------------------------
    //
    // No `STI`: a keyboard interrupt that could interrupt itself would corrupt
    // the ring, and there is nothing here slow enough to need it.
    a.bind(l.int09);
    a.pushs(DS);
    a.push(AX);
    a.push(BX);
    a.push(DX);
    a.push(SI);
    a.cld();
    ds_bda(a);

    let eoi = a.label();
    let modifier = a.label();
    let mod_up = a.label();
    let shift_key = a.label();
    let ctrl_key = a.label();
    let alt_key = a.label();
    let translate = a.label();

    a.in_al(0x60);
    // The extended prefix, and the key that follows it, are both dropped.
    a.alui8(Alu::CMP, AL, 0xe0);
    a.jcc(Cc::E, eoi);
    a.mov8(AH, AL);
    a.alui8(Alu::AND, AH, 0x7f);
    a.alui8(Alu::CMP, AH, 0x2a);
    a.jcc(Cc::E, shift_key);
    a.alui8(Alu::CMP, AH, 0x36);
    a.jcc(Cc::E, shift_key);
    a.alui8(Alu::CMP, AH, 0x1d);
    a.jcc(Cc::E, ctrl_key);
    a.alui8(Alu::CMP, AH, 0x38);
    a.jcc(Cc::E, alt_key);
    // An ordinary key: only its make code produces anything.
    a.testi8(AL, 0x80);
    a.jcc(Cc::NE, eoi);
    a.jmp(translate);

    a.bind(shift_key);
    a.movi8(BL, 0x03);
    a.jmp(modifier);
    a.bind(ctrl_key);
    a.movi8(BL, 0x04);
    a.jmp(modifier);
    a.bind(alt_key);
    a.movi8(BL, 0x08);

    a.bind(modifier);
    a.testi8(AL, 0x80);
    a.jcc(Cc::NE, mod_up);
    a.aluto8(Alu::OR, Mem::abs(BDA_KBFLAG), BL);
    a.jmp(eoi);
    a.bind(mod_up);
    a.not8(BL);
    a.aluto8(Alu::AND, Mem::abs(BDA_KBFLAG), BL);
    a.jmp(eoi);

    // The make code is in AH. BH keeps it while AX is used to index the table,
    // and DL keeps the shift flags, which the ROM read below would otherwise
    // clobber.
    a.bind(translate);
    let no_shift = a.label();
    let no_ctrl = a.label();
    a.mov8(BH, AH);
    a.mov8(DL, Mem::abs(BDA_KBFLAG));
    a.alui8(Alu::CMP, BH, KEY_TABLE_LEN as u8);
    a.jcc(Cc::AE, eoi);
    a.mov8(AL, BH);
    a.movi8(AH, 0);
    a.movi_label(SI, l.kb_scan_plain);
    a.testi8(DL, 0x03);
    a.jcc(Cc::E, no_shift);
    a.movi_label(SI, l.kb_scan_shift);
    a.bind(no_shift);
    a.alu(Alu::ADD, SI, AX);
    a.mov8(AL, Mem::si(0).seg(CS));
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::E, eoi);
    a.testi8(DL, 0x04);
    a.jcc(Cc::E, no_ctrl);
    // Control folds a letter onto its control code, which is the whole of what
    // the flag does to a character.
    a.alui8(Alu::AND, AL, 0x1f);
    a.bind(no_ctrl);
    a.mov8(AH, BH);
    a.call(l.kb_enqueue);

    // The end-of-interrupt is the last thing, so a slow translation cannot let
    // a second key in behind the first.
    a.bind(eoi);
    a.movi8(AL, 0x20);
    a.out_al(0x20);
    a.pop(SI);
    a.pop(DX);
    a.pop(BX);
    a.pop(AX);
    a.pops(DS);
    a.iret();

    // -- kb_enqueue ----------------------------------------------------------
    //
    // AX is the scan-code/character pair. `DS` must be the BDA. A full ring
    // drops the key, which is what a PC does — and what the beep on a full
    // buffer was for.
    a.bind(l.kb_enqueue);
    a.push(BX);
    a.push(DI);
    let wrap_ok = a.label();
    let full = a.label();
    a.mov(BX, Mem::abs(BDA_KBTAIL));
    a.mov(DI, BX);
    a.alui(Alu::ADD, BX, 2);
    a.alu(Alu::CMP, BX, Mem::abs(BDA_KBBUF_END));
    a.jcc(Cc::B, wrap_ok);
    a.mov(BX, Mem::abs(BDA_KBBUF_START));
    a.bind(wrap_ok);
    a.alu(Alu::CMP, BX, Mem::abs(BDA_KBHEAD));
    a.jcc(Cc::E, full);
    a.movto(Mem::di(0), AX);
    a.movto(Mem::abs(BDA_KBTAIL), BX);
    a.bind(full);
    a.pop(DI);
    a.pop(BX);
    a.ret();

    // -- INT 16h -------------------------------------------------------------
    a.bind(l.int16);
    enter(a);
    ds_bda(a);

    let wait_key = a.label();
    let peek_key = a.label();
    let shift_flags = a.label();
    let done = a.label();
    let no_translate = a.label();

    a.mov8(AH, Mem::bp(F_AX + 1));
    // 10h-12h are the enhanced forms of 00h-02h. They differ only in returning
    // the extended keys this firmware does not decode, so they answer the same.
    a.alui8(Alu::CMP, AH, 0x10);
    a.jcc(Cc::B, no_translate);
    a.alui8(Alu::CMP, AH, 0x12);
    a.jcc(Cc::A, done);
    a.alui8(Alu::SUB, AH, 0x10);
    a.bind(no_translate);
    a.alui8(Alu::CMP, AH, 0x00);
    a.jcc(Cc::E, wait_key);
    a.alui8(Alu::CMP, AH, 0x01);
    a.jcc(Cc::E, peek_key);
    a.alui8(Alu::CMP, AH, 0x02);
    a.jcc(Cc::E, shift_flags);
    a.jmp(done);

    // AH=00h: block until there is a key. `HLT` rather than a spin, so the
    // machine is idle while it waits and the host is not burning a core on it.
    a.bind(wait_key);
    let again = a.here_label();
    let have = a.label();
    let take_wrap = a.label();
    a.cli();
    a.mov(BX, Mem::abs(BDA_KBHEAD));
    a.alu(Alu::CMP, BX, Mem::abs(BDA_KBTAIL));
    a.jcc(Cc::NE, have);
    a.sti();
    a.hlt();
    a.jmp(again);
    a.bind(have);
    a.mov(AX, Mem::bx(0));
    a.alui(Alu::ADD, BX, 2);
    a.alu(Alu::CMP, BX, Mem::abs(BDA_KBBUF_END));
    a.jcc(Cc::B, take_wrap);
    a.mov(BX, Mem::abs(BDA_KBBUF_START));
    a.bind(take_wrap);
    a.movto(Mem::abs(BDA_KBHEAD), BX);
    a.sti();
    a.movto(Mem::bp(F_AX), AX);
    a.jmp(done);

    // AH=01h: the key without taking it, and ZF set when there is none.
    a.bind(peek_key);
    let empty = a.label();
    a.mov(BX, Mem::abs(BDA_KBHEAD));
    a.alu(Alu::CMP, BX, Mem::abs(BDA_KBTAIL));
    a.jcc(Cc::E, empty);
    a.mov(AX, Mem::bx(0));
    a.movto(Mem::bp(F_AX), AX);
    a.alui(Alu::AND, Mem::bp(F_FLAGS), !FLAG_ZF);
    a.jmp(done);
    a.bind(empty);
    a.alui(Alu::OR, Mem::bp(F_FLAGS), FLAG_ZF);
    a.jmp(done);

    // AH=02h: the shift state.
    a.bind(shift_flags);
    a.mov8(AL, Mem::abs(BDA_KBFLAG));
    a.movto8(Mem::bp(F_AX), AL);

    a.bind(done);
    leave(a);

    // -- the tables ----------------------------------------------------------
    a.bind(l.kb_scan_plain);
    a.db(&plain_table());
    a.bind(l.kb_scan_shift);
    a.db(&shift_table());
}
