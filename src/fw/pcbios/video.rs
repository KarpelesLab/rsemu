//! `INT 10h` — video, and the text-page primitives POST prints through.
//!
//! The ABI is Ralf Brown's Interrupt List, `INT 10h` functions 00h-13h. The
//! hardware is the 6845's cursor address registers R14/R15 (MC6845 data sheet,
//! "Register File") reached through the CRTC index port the BDA names at
//! `0040:0063`, and the colour text page at `B800:0000`.
//!
//! A video option ROM that installed its own `INT 10h` during POST replaces all
//! of this; what is here is what the board answers with when the `vgabios`
//! socket is empty, which is the configuration that has to work with nothing
//! external supplied.

use super::{
    BDA_ACTIVE_PAGE, BDA_COLUMNS, BDA_CRTC_PORT, BDA_CURSOR, BDA_CURSOR_SHAPE, BDA_PAGE_OFFSET,
    BDA_PAGE_SIZE, BDA_ROWS, BDA_VIDEO_MODE, F_AX, F_BP, F_BX, F_CX, F_DX, F_ES, Labels, ds_bda,
    enter, leave,
};
use crate::fw::asm16::{
    AH, AL, AX, Alu, Asm, BH, BL, BX, CH, CL, CS, CX, Cc, DH, DI, DL, DS, DX, ES, Mem, SI, Shift,
};

/// The colour text page's segment.
const TEXT_SEGMENT: u16 = 0xb800;
/// A blank cell: a space in light grey on black.
const BLANK_CELL: u16 = 0x0720;
/// The attribute POST and the teletype write with.
const DEFAULT_ATTRIBUTE: u8 = 0x07;

/// Emit `INT 10h` and the primitives it and POST share.
#[allow(clippy::too_many_lines)]
pub(super) fn emit(a: &mut Asm, l: &Labels) {
    // -- INT 10h -------------------------------------------------------------
    a.bind(l.int10);
    enter(a);
    ds_bda(a);

    let set_mode = a.label();
    let set_shape = a.label();
    let set_pos = a.label();
    let get_pos = a.label();
    let scroll_fn = a.label();
    let write_ca = a.label();
    let write_c = a.label();
    let read_ca = a.label();
    let teletype = a.label();
    let get_mode = a.label();
    let write_str = a.label();
    let done = a.label();

    a.mov8(AH, Mem::bp(F_AX + 1));
    for (function, target) in [
        (0x00u8, set_mode),
        (0x01, set_shape),
        (0x02, set_pos),
        (0x03, get_pos),
        (0x06, scroll_fn),
        (0x08, read_ca),
        (0x09, write_ca),
        (0x0a, write_c),
        (0x0e, teletype),
        (0x0f, get_mode),
        (0x13, write_str),
    ] {
        a.alui8(Alu::CMP, AH, function);
        a.jcc(Cc::E, target);
    }
    a.jmp(done);

    // AH=00h, set video mode. `pc.video` is a text-mode CRTC, so a graphics
    // mode is recorded and nothing else happens — the honest answer for a board
    // with no graphics hardware, and the reason `docs/platforms/pc-at.md` says
    // a video BIOS needs a real VGA. Bit 7 of AL is "do not clear", which
    // matters to a program that switches modes to reset the cursor.
    a.bind(set_mode);
    a.mov8(AL, Mem::bp(F_AX));
    a.mov8(AH, AL);
    a.alui8(Alu::AND, AL, 0x7f);
    a.movto8(Mem::abs(BDA_VIDEO_MODE), AL);
    a.movmi(Mem::abs(BDA_COLUMNS), 80);
    a.movmi(Mem::abs(BDA_PAGE_SIZE), 0x1000);
    a.movmi(Mem::abs(BDA_PAGE_OFFSET), 0);
    a.movmi(Mem::abs(BDA_CURSOR), 0);
    a.movmi8(Mem::abs(BDA_ACTIVE_PAGE), 0);
    a.movmi8(Mem::abs(BDA_ROWS), 24);
    a.movmi(Mem::abs(BDA_CURSOR_SHAPE), 0x0607);
    let no_clear = a.label();
    a.testi8(AH, 0x80);
    a.jcc(Cc::NE, no_clear);
    a.call(l.clear_screen);
    a.bind(no_clear);
    a.call(l.set_cursor_hw);
    a.jmp(done);

    // AH=01h, set the cursor's raster lines. Recorded, and handed back by
    // AH=03h; the CRTC's R10/R11 are not written because nothing on this board
    // draws a cursor of a different shape.
    a.bind(set_shape);
    a.mov(AX, Mem::bp(F_CX));
    a.movto(Mem::abs(BDA_CURSOR_SHAPE), AX);
    a.jmp(done);

    // AH=02h, set the cursor position: DH is the row and DL the column.
    a.bind(set_pos);
    a.mov(AX, Mem::bp(F_DX));
    a.movto(Mem::abs(BDA_CURSOR), AX);
    a.call(l.set_cursor_hw);
    a.jmp(done);

    // AH=03h: DX comes back as the position and CX as the shape.
    a.bind(get_pos);
    a.mov(AX, Mem::abs(BDA_CURSOR));
    a.movto(Mem::bp(F_DX), AX);
    a.mov(AX, Mem::abs(BDA_CURSOR_SHAPE));
    a.movto(Mem::bp(F_CX), AX);
    a.jmp(done);

    // AH=06h, scroll up. `AL=0` clears the rectangle CH,CL - DH,DL with the
    // attribute in BH, which is how every program clears a window. A non-zero
    // line count scrolls the *whole screen*, which is what a program that wants
    // a scrolling window will notice — see this module's limitations in
    // `super`.
    a.bind(scroll_fn);
    a.mov8(AL, Mem::bp(F_AX));
    let scroll_lines = a.label();
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::NE, scroll_lines);
    clear_rect(a);
    a.jmp(done);
    a.bind(scroll_lines);
    a.movi8(AH, 0);
    a.mov(CX, AX);
    let scroll_again = a.label();
    a.bind(scroll_again);
    a.push(CX);
    a.call(l.scroll_up);
    a.pop(CX);
    a.dec(CX);
    a.jcc(Cc::NE, scroll_again);
    a.jmp(done);

    // AH=09h, write a character and attribute CX times at the cursor without
    // moving it. AH=0Ah is the same with the attribute left alone, which here
    // means the default one: this board has no per-cell attribute to preserve
    // that a caller could not have written itself.
    a.bind(write_ca);
    a.bind(write_c);
    a.call(l.cell_offset);
    a.mov(CX, Mem::bp(F_CX));
    a.alui(Alu::CMP, CX, 0);
    a.jcc(Cc::E, done);
    a.movi(AX, TEXT_SEGMENT);
    a.movsr(ES, AX);
    a.mov8(AL, Mem::bp(F_AX));
    a.mov8(AH, Mem::bp(F_BX));
    a.rep();
    a.stosw();
    a.jmp(done);

    // AH=08h, read the character and attribute under the cursor: AL is the
    // character and AH the attribute, which is the word the text page holds in
    // that order. `BH`, the page, is ignored for the same reason AH=09h ignores
    // it — this firmware keeps one cursor, in `0040:0050`, and a page it never
    // switches to has no cursor to read from.
    //
    // Not a nicety: FreeDOS's command interpreter calls this hundreds of times
    // during a boot, and an unimplemented function here returns whatever `AX`
    // happened to hold.
    a.bind(read_ca);
    a.call(l.cell_offset);
    a.movi(AX, TEXT_SEGMENT);
    a.movsr(ES, AX);
    a.mov(AX, Mem::di(0).seg(ES));
    a.movto(Mem::bp(F_AX), AX);
    a.jmp(done);

    // AH=0Eh, teletype output: the one function POST and a boot sector both
    // use, and the only one whose absence would be immediately fatal.
    a.bind(teletype);
    a.mov8(AL, Mem::bp(F_AX));
    a.movi8(BL, DEFAULT_ATTRIBUTE);
    a.call(l.putc);
    a.jmp(done);

    // AH=0Fh: AL is the mode, AH the column count, BH the active page.
    a.bind(get_mode);
    a.mov8(AL, Mem::abs(BDA_VIDEO_MODE));
    a.movto8(Mem::bp(F_AX), AL);
    a.mov8(AL, Mem::abs(BDA_COLUMNS));
    a.movto8(Mem::bp(F_AX + 1), AL);
    a.mov8(AL, Mem::abs(BDA_ACTIVE_PAGE));
    a.movto8(Mem::bp(F_BX + 1), AL);
    a.jmp(done);

    // AH=13h, write string: ES:BP points at it, CX is its length, DX the
    // position, and bit 1 of AL says each character carries its own attribute.
    // ES:BP rather than ES:SI is the awkward part — BP is this handler's frame
    // pointer, so the caller's is taken out of the frame.
    a.bind(write_str);
    a.mov(AX, Mem::bp(F_DX));
    a.movto(Mem::abs(BDA_CURSOR), AX);
    a.mov(CX, Mem::bp(F_CX));
    a.alui(Alu::CMP, CX, 0);
    a.jcc(Cc::E, done);
    a.mov(SI, Mem::bp(F_BP));
    a.mov8(BL, Mem::bp(F_BX));
    a.movsr(ES, Mem::bp(F_ES));
    let str_loop = a.here_label();
    a.mov8(AL, Mem::si(0).seg(ES));
    a.inc(SI);
    let no_attr = a.label();
    a.testi8(Mem::bp(F_AX), 0x02);
    a.jcc(Cc::E, no_attr);
    a.mov8(BL, Mem::si(0).seg(ES));
    a.inc(SI);
    a.bind(no_attr);
    a.push(CX);
    a.push(SI);
    a.call(l.putc);
    a.pop(SI);
    a.pop(CX);
    a.dec(CX);
    a.jcc(Cc::NE, str_loop);

    a.bind(done);
    leave(a);

    // -- putc ---------------------------------------------------------------
    //
    // AL is the character and BL the attribute; `DS` must be the BDA. Carriage
    // return, line feed, backspace and bell are acted on, everything else is
    // written at the cursor and the cursor advances, scrolling at the bottom.
    a.bind(l.putc);
    a.push(AX);
    a.push(BX);
    a.push(CX);
    a.push(DX);
    a.push(DI);
    a.pushs(ES);
    a.mov8(BH, AL);
    let p_cr = a.label();
    let p_lf = a.label();
    let p_bs = a.label();
    let p_fin = a.label();
    let p_check = a.label();
    let p_store = a.label();
    a.alui8(Alu::CMP, AL, 0x0d);
    a.jcc(Cc::E, p_cr);
    a.alui8(Alu::CMP, AL, 0x0a);
    a.jcc(Cc::E, p_lf);
    a.alui8(Alu::CMP, AL, 0x08);
    a.jcc(Cc::E, p_bs);
    a.alui8(Alu::CMP, AL, 0x07);
    a.jcc(Cc::E, p_fin);
    a.call(l.cell_offset);
    a.movi(CX, TEXT_SEGMENT);
    a.movsr(ES, CX);
    a.mov8(AL, BH);
    a.mov8(AH, BL);
    a.movto(Mem::di(0).seg(ES), AX);
    a.incm8(DL);
    a.alui8(Alu::CMP, DL, 80);
    a.jcc(Cc::B, p_store);
    a.movi8(DL, 0);
    a.incm8(DH);
    a.bind(p_store);
    a.movto(Mem::abs(BDA_CURSOR), DX);
    a.jmp(p_check);

    a.bind(p_cr);
    a.mov(DX, Mem::abs(BDA_CURSOR));
    a.movi8(DL, 0);
    a.movto(Mem::abs(BDA_CURSOR), DX);
    a.jmp(p_fin);

    a.bind(p_lf);
    a.mov(DX, Mem::abs(BDA_CURSOR));
    a.incm8(DH);
    a.movto(Mem::abs(BDA_CURSOR), DX);
    a.jmp(p_check);

    a.bind(p_bs);
    a.mov(DX, Mem::abs(BDA_CURSOR));
    a.alui8(Alu::CMP, DL, 0);
    a.jcc(Cc::E, p_fin);
    a.decm8(DL);
    a.movto(Mem::abs(BDA_CURSOR), DX);
    a.jmp(p_fin);

    a.bind(p_check);
    a.alui8(Alu::CMP, DH, 25);
    a.jcc(Cc::B, p_fin);
    a.call(l.scroll_up);
    a.mov(DX, Mem::abs(BDA_CURSOR));
    a.movi8(DH, 24);
    a.movto(Mem::abs(BDA_CURSOR), DX);

    a.bind(p_fin);
    a.call(l.set_cursor_hw);
    a.pops(ES);
    a.pop(DI);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.pop(AX);
    a.ret();

    // -- cell_offset --------------------------------------------------------
    //
    // DI comes back as the cursor's byte offset into the text page and DX as
    // the cursor itself. `MUL` writes DX, so the position is reloaded after it
    // rather than kept there.
    a.bind(l.cell_offset);
    a.push(AX);
    a.push(CX);
    a.mov(DX, Mem::abs(BDA_CURSOR));
    a.mov8(AL, DH);
    a.movi8(AH, 0);
    a.movi(CX, 80);
    a.mul(CX);
    a.mov(DX, Mem::abs(BDA_CURSOR));
    a.mov8(CL, DL);
    a.movi8(CH, 0);
    a.alu(Alu::ADD, AX, CX);
    a.shift(Shift::SHL, AX, 1);
    a.mov(DI, AX);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- scroll_up ----------------------------------------------------------
    //
    // The whole page up one row, with the bottom row blanked. Both segments
    // become the text page, so `MOVSW` moves within it.
    a.bind(l.scroll_up);
    a.push(AX);
    a.push(CX);
    a.push(SI);
    a.push(DI);
    a.pushs(DS);
    a.pushs(ES);
    a.movi(AX, TEXT_SEGMENT);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movi(DI, 0);
    a.movi(SI, 160);
    a.movi(CX, 80 * 24);
    a.rep();
    a.movsw();
    a.movi(AX, BLANK_CELL);
    a.movi(CX, 80);
    a.rep();
    a.stosw();
    a.pops(ES);
    a.pops(DS);
    a.pop(DI);
    a.pop(SI);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- clear_screen -------------------------------------------------------
    a.bind(l.clear_screen);
    a.push(AX);
    a.push(CX);
    a.push(DI);
    a.pushs(ES);
    a.movi(AX, TEXT_SEGMENT);
    a.movsr(ES, AX);
    a.movi(DI, 0);
    a.movi(AX, BLANK_CELL);
    a.movi(CX, 80 * 25);
    a.rep();
    a.stosw();
    a.pops(ES);
    a.pop(DI);
    a.pop(CX);
    a.pop(AX);
    a.ret();

    // -- set_cursor_hw ------------------------------------------------------
    //
    // R14 and R15 of the 6845 hold the cursor's address as a 14-bit character
    // offset, high byte first (MC6845 data sheet, "Register File"). The index
    // port comes from the BDA so a monochrome board would work unchanged.
    a.bind(l.set_cursor_hw);
    a.push(AX);
    a.push(BX);
    a.push(CX);
    a.push(DX);
    a.mov(DX, Mem::abs(BDA_CURSOR));
    a.mov8(AL, DH);
    a.movi8(AH, 0);
    a.movi(CX, 80);
    a.mul(CX);
    a.mov(BX, AX);
    a.mov(DX, Mem::abs(BDA_CURSOR));
    a.mov8(CL, DL);
    a.movi8(CH, 0);
    a.alu(Alu::ADD, BX, CX);
    a.mov(DX, Mem::abs(BDA_CRTC_PORT));
    a.movi8(AL, 0x0e);
    a.out_dx_al();
    a.inc(DX);
    a.mov8(AL, BH);
    a.out_dx_al();
    a.dec(DX);
    a.movi8(AL, 0x0f);
    a.out_dx_al();
    a.inc(DX);
    a.mov8(AL, BL);
    a.out_dx_al();
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.pop(AX);
    a.ret();

    // -- puts ---------------------------------------------------------------
    //
    // SI points at a NUL-terminated string in this ROM, so the read carries a
    // CS override: POST's own messages live in the image, not in RAM.
    a.bind(l.puts);
    a.push(AX);
    a.push(BX);
    a.push(SI);
    let s_next = a.here_label();
    let s_done = a.label();
    a.mov8(AL, Mem::si(0).seg(CS));
    a.inc(SI);
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::E, s_done);
    a.movi8(AH, 0x0e);
    a.movi(BX, u16::from(DEFAULT_ATTRIBUTE));
    a.int(0x10);
    a.jmp(s_next);
    a.bind(s_done);
    a.pop(SI);
    a.pop(BX);
    a.pop(AX);
    a.ret();

    // -- put_dec ------------------------------------------------------------
    //
    // AX in decimal, digits pushed then popped so they come out most
    // significant first. Only POST uses it, and only for the memory sizes.
    a.bind(l.put_dec);
    a.push(AX);
    a.push(BX);
    a.push(CX);
    a.push(DX);
    a.movi(CX, 0);
    a.movi(BX, 10);
    let d_div = a.here_label();
    a.movi(DX, 0);
    a.div(BX);
    a.push(DX);
    a.inc(CX);
    a.alui(Alu::CMP, AX, 0);
    a.jcc(Cc::NE, d_div);
    let d_out = a.here_label();
    a.pop(AX);
    a.alui8(Alu::ADD, AL, b'0');
    a.push(CX);
    a.movi8(AH, 0x0e);
    a.movi(BX, u16::from(DEFAULT_ATTRIBUTE));
    a.int(0x10);
    a.pop(CX);
    a.dec(CX);
    a.jcc(Cc::NE, d_out);
    a.pop(DX);
    a.pop(CX);
    a.pop(BX);
    a.pop(AX);
    a.ret();
}

/// `INT 10h AH=06h` with `AL=0`: fill the rectangle CH,CL - DH,DL with blanks
/// in the attribute BH.
///
/// Emitted inline because it is used once and needs the frame `[bp+n]` names.
fn clear_rect(a: &mut Asm) {
    let row_loop = a.label();
    let col_loop = a.label();
    let done = a.label();

    a.movi(AX, TEXT_SEGMENT);
    a.movsr(ES, AX);
    a.mov(CX, Mem::bp(F_CX)); // CH = top row, CL = left column
    a.mov(DX, Mem::bp(F_DX)); // DH = bottom row, DL = right column
    a.mov8(AH, Mem::bp(F_BX + 1)); // the attribute
    a.movi8(AL, b' ');
    a.push(AX);

    a.bind(row_loop);
    a.alu8(Alu::CMP, CH, DH);
    a.jcc(Cc::A, done);
    // DI = (row * 80 + left) * 2, computed with BX as the scratch `MUL` cannot
    // use: `MUL CX` would destroy the column bounds.
    a.push(CX);
    a.push(DX);
    a.mov8(AL, CH);
    a.movi8(AH, 0);
    a.movi(BX, 80);
    a.mul(BX);
    a.pop(DX);
    a.pop(CX);
    a.mov8(BL, CL);
    a.movi8(BH, 0);
    a.alu(Alu::ADD, AX, BX);
    a.shift(Shift::SHL, AX, 1);
    a.mov(DI, AX);
    // How many cells wide, as a count for the inner loop.
    a.mov8(BL, DL);
    a.movi8(BH, 0);
    a.mov8(AL, CL);
    a.movi8(AH, 0);
    a.alu(Alu::SUB, BX, AX);
    a.inc(BX);
    a.pop(AX);
    a.push(AX);
    a.bind(col_loop);
    a.movto(Mem::di(0).seg(ES), AX);
    a.alui(Alu::ADD, DI, 2);
    a.dec(BX);
    a.jcc(Cc::NE, col_loop);
    a.incm8(CH);
    a.jmp(row_loop);

    a.bind(done);
    a.pop(AX);
}
