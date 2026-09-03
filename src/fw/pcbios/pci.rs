//! The PCI BIOS interface — `INT 1Ah AH=B1h` — and the POST probe behind it.
//!
//! This is the real-mode service a DOS-era driver, a 16-bit loader or a PCI
//! option ROM uses to find a function on the bus and read or write its
//! configuration space without knowing how configuration cycles are generated
//! on this board. Everything under it is *mechanism #1*: the `CONFIG_ADDRESS`
//! latch at `0xcf8` and the `CONFIG_DATA` window at `0xcfc`.
//!
//! # Sources
//!
//! * **PCI BIOS Specification** revision 2.1 (PCI SIG, 1994) §4 for the
//!   function numbers, the register-level argument and return conventions, and
//!   the return codes in [`RC_SUCCESSFUL`] and below. Each function's own
//!   section is cited where it is emitted.
//! * **PCI Local Bus Specification** revision 2.1 §3.7.4.1 for configuration
//!   mechanism #1 — the layout of `CONFIG_ADDRESS`, the rule that it is
//!   accessible only as a Dword, and the way the low two bits of the *I/O*
//!   address select bytes inside the addressed Dword — and §6.1 for the header
//!   the scan reads: the Vendor ID at 0, the class code at 09h-0Bh, and Header
//!   Type bit 7, which says whether a device answers on more than function 0.
//! * **Ralf Brown's Interrupt List**, `INT 1A/AH=B1h`, for the ABI as software
//!   in the field actually calls it.
//!
//! No firmware source was read (`ROADMAP.md` §1); the specifications above
//! define an interface, not an implementation of one.
//!
//! # What is here, and what is refused
//!
//! Implemented: `B101h` installation check, `B102h` find by vendor and device,
//! `B103h` find by class code, `B108h`/`B109h`/`B10Ah` read configuration
//! byte/word/dword and `B10Bh`/`B10Ch`/`B10Dh` write them.
//!
//! Refused, each with `AH=81h` (`FUNC_NOT_SUPPORTED`) and carry set, which is
//! what the specification's own return code is for:
//!
//! * **`B106h`, generate special cycle.** Mechanism #1 generates one by
//!   addressing device 1Fh function 7 register 0 and writing `CONFIG_DATA`
//!   (*PCI Local Bus* §3.7.4.1), so the code would be four instructions — and
//!   it would be a lie. rsemu's fabric has no special-cycle path: that write
//!   would be an ordinary configuration write to a function that does not
//!   exist, i.e. a master abort, and no device on any of these boards would
//!   see the cycle. The installation check says the same thing in the other
//!   direction: [`HW_MECHANISM_1`] leaves the special-cycle bits clear, so a
//!   caller that reads `AL` never asks.
//! * **`B10Eh` get IRQ routing options and `B10Fh` set PCI IRQ.** Both are
//!   about the `$PIR` routing table — which `PIRQ` line each slot's `INTA#`
//!   swizzles onto, and which 8259A input each `PIRQ` is routed to. `pc-at`
//!   has a host bridge and a display adapter and no south bridge at all, so
//!   there is no routing to report; fabricating a table would be a claim about
//!   hardware that is not there, which is the same mistake as an ACPI table
//!   describing a device the board does not have.
//! * **The 32-bit BIOS32 service directory** (`_32_`, *PCI BIOS* §4.1), which
//!   is how a protected-mode caller reaches the same functions. It is 32-bit
//!   code, and [`crate::fw::asm16`] emits 16-bit code; a real-mode BIOS that
//!   published a 32-bit entry point it could not assemble would be worse than
//!   one that publishes none, because the structure is found by a search and a
//!   caller that finds it calls it.
//!
//! # Which buses are scanned
//!
//! Bus 0, and [`LAST_BUS`] says so to a caller. That is a fact about these
//! boards rather than a shortcut: a PCI bus beyond 0 exists only behind a
//! PCI-to-PCI bridge, whose primary, secondary and subordinate bus numbers are
//! *assigned by firmware* during POST (*PCI-to-PCI Bridge Architecture
//! Specification* §3.2.5.4). This firmware assigns none, because nothing in
//! the tree models a bridge; when one arrives, the number POST assigns is what
//! [`LAST_BUS`] has to become, and the scan below already loops over buses so
//! that the change is a constant rather than a rewrite.

use super::{
    EBDA_PCI_INDEX, EBDA_PCI_MATCH, EBDA_PCI_MECHANISM, EBDA_PCI_MODE, EBDA_PCI_SAVE,
    EBDA_PCI_STEP, EBDA_SEGMENT, FLAG_CF, G_EAX, G_EBX, G_ECX, G_EDI, G_EDX, G_ESI, G_FLAGS,
    Labels, load_seg,
};
use crate::fw::asm16::{
    AH, AL, AX, Alu, Asm, BH, BL, BP, BX, Cc, DI, DL, DS, DX, ES, Label, Mem, SP, Shift,
};

// ---------------------------------------------------------------------------
// the numbers the interface is defined in terms of
// ---------------------------------------------------------------------------

/// The `AH` value that selects this interface (*PCI BIOS* §4).
pub(super) const PCI_FUNCTION: u8 = 0xb1;

/// The signature `B101h` answers with in `EDX`: `'P'`, `'C'`, `'I'`, `' '`,
/// little-endian, so `DL` holds `'P'` (*PCI BIOS* §4.2).
const PCI_SIGNATURE: u32 = 0x2049_4350;

/// The interface level `B101h` reports in `BX`: major 02h, minor 10h, i.e.
/// revision 2.1 (*PCI BIOS* §4.2).
const VERSION: u16 = 0x0210;

/// The hardware mechanism byte `B101h` answers with in `AL` (*PCI BIOS* §4.2).
///
/// Bits 0-3 are the configuration mechanisms supported and bits 4-7 the
/// special-cycle mechanisms. Bit 0 is mechanism #1; every other bit is clear,
/// which is the honest statement for these boards — there is no mechanism #2
/// decode anywhere in the tree, and no special-cycle path at all.
const HW_MECHANISM_1: u8 = 0x01;

/// The highest bus number the scan reaches, and the one `B101h` reports in
/// `CL`. See the module's own note on why it is a constant.
const LAST_BUS: u8 = 0;

/// `CONFIG_ADDRESS`, the mechanism #1 address latch (*PCI Local Bus* §3.7.4.1).
const CONFIG_ADDRESS: u16 = 0x0cf8;

/// `CONFIG_DATA`, the window the addressed Dword appears in.
const CONFIG_DATA: u16 = 0x0cfc;

/// `CONFIG_ADDRESS` bit 31, the enable that turns a `CONFIG_DATA` reference
/// into a configuration cycle.
const CONFIG_ENABLE: u32 = 0x8000_0000;

/// Configuration register 00h: Vendor ID in the low word, Device ID in the
/// high one (*PCI Local Bus* §6.1). Reads as all ones where no function
/// answers, which is what a master abort gives back.
const REG_ID: u16 = 0x00;

/// Configuration register 08h: Revision ID in the low byte and the 24-bit
/// class code above it, which is why `B103h`'s comparison is a shift.
const REG_CLASS: u16 = 0x08;

/// Configuration register 0Eh, Header Type. Bit 7 set means the device
/// implements more than function 0 (*PCI Local Bus* §6.2.1).
const REG_HEADER_TYPE: u16 = 0x0e;

/// Header Type bit 7, the multi-function bit.
const HEADER_MULTIFUNCTION: u8 = 0x80;

/// How many functions a device may implement, and therefore how far the scan
/// steps to skip one that implements only function 0.
const FUNCTIONS_PER_DEVICE: u8 = 8;

// -- the return codes (*PCI BIOS* §4, "Return Code") -------------------------

/// The call did what was asked.
const RC_SUCCESSFUL: u8 = 0x00;
/// This BIOS does not implement that function.
const RC_FUNC_NOT_SUPPORTED: u8 = 0x81;
/// `FFFFh` is not a vendor identification; it is what an empty slot reads as.
const RC_BAD_VENDOR_ID: u8 = 0x83;
/// The scan finished without an *n*th match.
const RC_DEVICE_NOT_FOUND: u8 = 0x86;
/// The register number was out of range or misaligned for the access width.
const RC_BAD_REGISTER_NUMBER: u8 = 0x87;

// ---------------------------------------------------------------------------
// emission
// ---------------------------------------------------------------------------

/// Emit the POST probe, the `INT 1Ah AH=B1h` handler and the two configuration
/// primitives everything here is built from.
pub(super) fn emit(a: &mut Asm, l: &Labels) {
    detect(a, l);
    handler(a, l);
    select(a, l);
    scan(a, l);
}

/// `pci_detect`: find out at POST whether this board has a mechanism #1
/// configuration window, and record it for `B101h`.
///
/// The test is the one the register's own definition licenses (*PCI Local Bus*
/// §3.7.4.1): `CONFIG_ADDRESS` is a read/write Dword whose bits 30-24 are
/// reserved and read as zero, so writing the enable bit alone and reading back
/// exactly the enable bit alone identifies a live latch. A board with no host
/// bridge has nothing decoding those four bytes and answers with ones, which
/// is not what was written.
///
/// **This is a probe rather than a fact read out of the machine description,
/// and that is deliberate.** [`super::platform`] reads the description for the
/// things a firmware *cannot* find out by asking — how many processors exist,
/// what their APIC IDs are — because a table has to state them before anything
/// is running. Whether a configuration window answers is not one of those: the
/// firmware is running on the board, so it can ask the board, and the answer
/// is then true of the machine as built rather than of the text it was built
/// from.
///
/// Clobbers `AX` and `DX`, preserves `ES` and every other register, and leaves
/// the latch disabled so that a later *byte* reference to `0xcf9` — the
/// chipset reset control register, which on this board passes through the
/// bridge — is not sitting behind an enabled configuration cycle.
fn detect(a: &mut Asm, l: &Labels) {
    a.bind(l.pci_detect);
    a.pushs(ES);
    load_seg(a, ES, EBDA_SEGMENT, AX);
    a.movi(DX, CONFIG_ADDRESS);
    a.movi32(AX, CONFIG_ENABLE);
    a.out_dx_eax();
    a.in_eax_dx();
    a.alui32(Alu::CMP, AX, CONFIG_ENABLE);
    let absent = a.label();
    let recorded = a.label();
    a.jcc(Cc::NE, absent);
    a.movmi8(Mem::abs(EBDA_PCI_MECHANISM).seg(ES), HW_MECHANISM_1);
    a.jmp(recorded);
    a.bind(absent);
    a.movmi8(Mem::abs(EBDA_PCI_MECHANISM).seg(ES), 0);
    a.bind(recorded);
    a.movi32(AX, 0);
    a.movi(DX, CONFIG_ADDRESS);
    a.out_dx_eax();
    a.pops(ES);
    a.ret();
}

/// The `INT 1Ah AH=B1h` handler.
#[allow(clippy::too_many_lines)]
fn handler(a: &mut Asm, l: &Labels) {
    // `PUSHAD`, not `PUSHA`: `B103h` takes a class code in `ECX`, `B10Ah`
    // answers with a Dword in `ECX` and `B101h` answers with a signature in
    // `EDX`. A 16-bit frame would drop exactly the halves this interface is
    // specified in. `super::system` opens `INT 15h` the same way and the two
    // share the `G_*` offsets.
    a.bind(l.int1a_pci);
    a.sti();
    a.pushs(DS);
    a.pushs(ES);
    a.pushad();
    a.mov(BP, SP);
    load_seg(a, DS, EBDA_SEGMENT, AX);

    let present = a.label();
    let find_device = a.label();
    let find_class = a.label();
    let search = a.label();
    let read_byte = a.label();
    let read_word = a.label();
    let read_dword = a.label();
    let write_byte = a.label();
    let write_word = a.label();
    let write_dword = a.label();
    let bad_vendor = a.label();
    let bad_register = a.label();
    let not_found = a.label();
    let unsupported = a.label();
    let fail = a.label();
    let ok = a.label();
    let done = a.label();

    // Nothing here is answerable on a board with no configuration window, and
    // that includes the installation check itself: a caller that gets carry
    // back from `B101h` is being told there is no PCI BIOS, which is the truth.
    a.alui8(Alu::CMP, Mem::abs(EBDA_PCI_MECHANISM), 0);
    a.jcc(Cc::E, unsupported);

    a.mov8(AL, Mem::bp(G_EAX));
    for (function, target) in [
        (0x01u8, present),
        (0x02, find_device),
        (0x03, find_class),
        (0x08, read_byte),
        (0x09, read_word),
        (0x0a, read_dword),
        (0x0b, write_byte),
        (0x0c, write_word),
        (0x0d, write_dword),
    ] {
        a.alui8(Alu::CMP, AL, function);
        a.jcc(Cc::E, target);
    }
    a.jmp(unsupported);

    // -- B101h, PCI BIOS present (*PCI BIOS* §4.2) --------------------------
    //
    // `EDX` is the whole four-character signature; `AL` the hardware mechanism
    // as POST found it; `BX` the interface level; `CL` the last bus number.
    // `AH` is the return code, which the shared `ok` tail writes — so `AL` is
    // written through the frame's low byte rather than as a whole `EAX`.
    a.bind(present);
    a.movi32(AX, PCI_SIGNATURE);
    a.movto32(Mem::bp(G_EDX), AX);
    a.movi(BX, VERSION);
    a.movto(Mem::bp(G_EBX), BX);
    a.movmi8(Mem::bp(G_ECX), LAST_BUS);
    a.mov8(AL, Mem::abs(EBDA_PCI_MECHANISM));
    a.movto8(Mem::bp(G_EAX), AL);
    a.jmp(ok);

    // -- B102h, find PCI device (*PCI BIOS* §4.3) ---------------------------
    //
    // `CX` device, `DX` vendor, `SI` the index — "the *n*th device with this
    // pair", so a board with two identical cards is enumerable. `FFFFh` is not
    // a vendor: it is what an absent function reads as, and answering
    // `DEVICE_NOT_FOUND` for it would be indistinguishable from a real miss,
    // which is why the specification gives it a code of its own.
    a.bind(find_device);
    a.mov(AX, Mem::bp(G_EDX));
    a.alui(Alu::CMP, AX, 0xffff);
    a.jcc(Cc::E, bad_vendor);
    a.movi32(AX, 0);
    a.mov(AX, Mem::bp(G_ECX));
    a.shift32(Shift::SHL, AX, 16);
    a.mov(AX, Mem::bp(G_EDX));
    a.movto32(Mem::abs(EBDA_PCI_MATCH), AX);
    a.movmi8(Mem::abs(EBDA_PCI_MODE), MODE_ID);
    a.jmp(search);

    // -- B103h, find PCI class code (*PCI BIOS* §4.4) -----------------------
    //
    // `ECX` carries base class, sub-class and programming interface in its low
    // three bytes — the same order they sit in configuration space at 09h-0Bh,
    // which is why the scan compares the Dword at 08h shifted down by the
    // Revision ID byte rather than reading three bytes.
    a.bind(find_class);
    a.mov32(AX, Mem::bp(G_ECX));
    a.alui32(Alu::AND, AX, 0x00ff_ffff);
    a.movto32(Mem::abs(EBDA_PCI_MATCH), AX);
    a.movmi8(Mem::abs(EBDA_PCI_MODE), MODE_CLASS);

    a.bind(search);
    a.mov(AX, Mem::bp(G_ESI));
    a.movto(Mem::abs(EBDA_PCI_INDEX), AX);
    a.call(l.pci_scan);
    a.jcc(Cc::B, not_found);
    // `BH` is the bus and `BL` the device in bits 7-3 with the function in
    // bits 2-0 — the same byte `CONFIG_ADDRESS` carries in bits 15-8, which is
    // why the scan's cursor and this return value are the same register.
    a.movto(Mem::bp(G_EBX), BX);
    a.jmp(ok);

    // -- B108h-B10Dh, read and write configuration space (*PCI BIOS* §4.6-7)-
    //
    // `BH` bus, `BL` device and function, `DI` register number, and the value
    // in `CL`, `CX` or `ECX`. Six functions, three widths, one shape.
    a.bind(read_byte);
    access(a, l, bad_register, 0);
    a.in_al_dx();
    a.movto8(Mem::bp(G_ECX), AL);
    a.jmp(ok);

    a.bind(read_word);
    access(a, l, bad_register, 1);
    a.in_ax_dx();
    a.movto(Mem::bp(G_ECX), AX);
    a.jmp(ok);

    a.bind(read_dword);
    access(a, l, bad_register, 3);
    a.in_eax_dx();
    a.movto32(Mem::bp(G_ECX), AX);
    a.jmp(ok);

    // The value is loaded *after* the selection, because building
    // `CONFIG_ADDRESS` is what `EAX` is used for.
    a.bind(write_byte);
    access(a, l, bad_register, 0);
    a.mov8(AL, Mem::bp(G_ECX));
    a.out_dx_al();
    a.jmp(ok);

    a.bind(write_word);
    access(a, l, bad_register, 1);
    a.mov(AX, Mem::bp(G_ECX));
    a.out_dx_ax();
    a.jmp(ok);

    a.bind(write_dword);
    access(a, l, bad_register, 3);
    a.mov32(AX, Mem::bp(G_ECX));
    a.out_dx_eax();
    a.jmp(ok);

    // -- the two tails ------------------------------------------------------
    a.bind(ok);
    a.movmi8(Mem::bp(G_EAX + 1), RC_SUCCESSFUL);
    a.alui(Alu::AND, Mem::bp(G_FLAGS), !FLAG_CF);
    a.jmp(done);

    a.bind(not_found);
    a.movmi8(Mem::bp(G_EAX + 1), RC_DEVICE_NOT_FOUND);
    a.jmp(fail);
    a.bind(bad_vendor);
    a.movmi8(Mem::bp(G_EAX + 1), RC_BAD_VENDOR_ID);
    a.jmp(fail);
    a.bind(bad_register);
    a.movmi8(Mem::bp(G_EAX + 1), RC_BAD_REGISTER_NUMBER);
    a.jmp(fail);
    // `B106h`, `B10Eh`, `B10Fh` and anything else land here; the module's own
    // documentation says which are refused deliberately and why.
    a.bind(unsupported);
    a.movmi8(Mem::bp(G_EAX + 1), RC_FUNC_NOT_SUPPORTED);
    a.bind(fail);
    a.alui(Alu::OR, Mem::bp(G_FLAGS), FLAG_CF);

    a.bind(done);
    a.popad();
    a.pops(ES);
    a.pops(DS);
    a.iret();
}

/// The prologue every configuration read and write shares: check the register
/// number, then point the latch at it.
///
/// `align` is the mask the register number must be clear in — 0 for a byte, 1
/// for a word, 3 for a Dword. *PCI BIOS* §4.6 gives `BAD_REGISTER_NUMBER` for
/// exactly this, and the check is worth making rather than letting the
/// hardware alias: mechanism #1 would happily answer a misaligned word out of
/// the wrong two bytes of the addressed Dword, which is a wrong answer rather
/// than an error.
///
/// The register number arrives in `DI`, and the specification's range is
/// 0-255; a caller with anything in the high byte is refused rather than
/// silently masked, because a program computing a register number that
/// overflowed is not asking for register `n & 0xff`.
fn access(a: &mut Asm, l: &Labels, bad_register: Label, align: u8) {
    a.alui8(Alu::CMP, Mem::bp(G_EDI + 1), 0);
    a.jcc(Cc::NE, bad_register);
    if align != 0 {
        a.testi8(Mem::bp(G_EDI), align);
        a.jcc(Cc::NE, bad_register);
    }
    a.mov(BX, Mem::bp(G_EBX));
    a.mov(DI, Mem::bp(G_EDI));
    a.call(l.pci_select);
}

/// `pci_select`: point `CONFIG_ADDRESS` at one register of one function, and
/// answer with the I/O port its bytes appear at.
///
/// Entry: `BH` bus, `BL` device and function, `DI` register number.
/// Exit: `DX` is the port to reference, which is `CONFIG_DATA` plus the low two
/// bits of the register number — *PCI Local Bus* §3.7.4.1's rule that
/// `CONFIG_ADDRESS[7:2]` names the Dword and the I/O address's own low bits
/// select bytes within it. Clobbers `EAX` and `DX`; `BX` and `DI` survive,
/// which is what lets the scan hold its cursor in them across a call.
///
/// The address is built high half first because `DI` has no 8-bit alias: `mov
/// ax, di` writes the low half of `EAX` and leaves the bus number, already
/// shifted up, alone. The register's low two bits are masked off here rather
/// than by the caller, so a Dword-only latch never sees them.
fn select(a: &mut Asm, l: &Labels) {
    a.bind(l.pci_select);
    a.movi32(AX, 0);
    a.mov8(AL, BH);
    a.shift32(Shift::SHL, AX, 16);
    a.mov(AX, DI);
    a.alui8(Alu::AND, AL, 0xfc);
    a.mov8(AH, BL);
    a.alui32(Alu::OR, AX, CONFIG_ENABLE);
    a.movi(DX, CONFIG_ADDRESS);
    a.out_dx_eax();

    a.mov(AX, DI);
    a.alui8(Alu::AND, AL, 0x03);
    a.movi(DX, CONFIG_DATA);
    a.aluto8(Alu::ADD, DL, AL);
    a.ret();
}

/// What [`super::EBDA_PCI_MODE`] holds for a vendor-and-device search.
const MODE_ID: u8 = 0;
/// What it holds for a class-code search.
const MODE_CLASS: u8 = 1;

/// `pci_scan`: walk the bus for the *n*th function matching what the caller
/// left in the EBDA, and answer with its bus and device/function byte.
///
/// Entry: `DS` is the EBDA, [`super::EBDA_PCI_MATCH`] is the value to compare,
/// [`super::EBDA_PCI_MODE`] says which comparison, and
/// [`super::EBDA_PCI_INDEX`] is how many further matches to skip.
/// Exit: carry clear and `BX` holding bus and device/function, or carry set.
/// Clobbers `EAX`, `DX` and `DI`.
///
/// The walk is the one *PCI Local Bus* §6.2.1 describes and not a sweep of all
/// 256 function numbers: a device whose function 0 does not answer is absent
/// entirely, and a device whose Header Type bit 7 is clear implements only
/// function 0. Both facts are checked at function 0 and turn into the step
/// [`super::EBDA_PCI_STEP`] holds, so an empty bus costs 32 configuration
/// reads rather than 256 — and, more importantly, a single-function device is
/// never reported eight times by hardware that aliases its functions.
fn scan(a: &mut Asm, l: &Labels) {
    a.bind(l.pci_scan);
    a.movi(BX, 0);

    let function = a.here_label();
    let have = a.label();
    let single = a.label();
    let compare = a.label();
    let advance = a.label();
    let found = a.label();
    let exhausted = a.label();

    a.movmi8(Mem::abs(EBDA_PCI_STEP), 1);
    a.movi(DI, REG_ID);
    a.call(l.pci_read_dword);
    // Function 0 is where a device is present or absent and where it says how
    // many functions it has; every other function number is simply read.
    a.testi8(BL, FUNCTIONS_PER_DEVICE - 1);
    a.jcc(Cc::NE, have);
    a.movmi8(Mem::abs(EBDA_PCI_STEP), FUNCTIONS_PER_DEVICE);
    a.alui32(Alu::CMP, AX, 0xffff_ffff);
    a.jcc(Cc::E, advance);
    a.movto32(Mem::abs(EBDA_PCI_SAVE), AX);
    a.movi(DI, REG_HEADER_TYPE);
    a.call(l.pci_select);
    a.in_al_dx();
    a.testi8(AL, HEADER_MULTIFUNCTION);
    a.jcc(Cc::E, single);
    a.movmi8(Mem::abs(EBDA_PCI_STEP), 1);
    a.bind(single);
    a.mov32(AX, Mem::abs(EBDA_PCI_SAVE));

    a.bind(have);
    a.alui32(Alu::CMP, AX, 0xffff_ffff);
    a.jcc(Cc::E, advance);
    a.alui8(Alu::CMP, Mem::abs(EBDA_PCI_MODE), MODE_ID);
    a.jcc(Cc::E, compare);
    a.movi(DI, REG_CLASS);
    a.call(l.pci_read_dword);
    a.shift32(Shift::SHR, AX, 8);

    a.bind(compare);
    a.alu32(Alu::CMP, AX, Mem::abs(EBDA_PCI_MATCH));
    a.jcc(Cc::NE, advance);
    a.alui(Alu::CMP, Mem::abs(EBDA_PCI_INDEX), 0);
    a.jcc(Cc::E, found);
    a.decm(Mem::abs(EBDA_PCI_INDEX));

    // The step carries out of `BL` exactly when the bus is finished, which is
    // also what leaves `BL` at zero for the next one.
    a.bind(advance);
    a.alu8(Alu::ADD, BL, Mem::abs(EBDA_PCI_STEP));
    a.jcc(Cc::AE, function);
    a.alui8(Alu::CMP, BH, LAST_BUS);
    a.jcc(Cc::AE, exhausted);
    a.alui8(Alu::ADD, BH, 1);
    a.jmp(function);

    a.bind(exhausted);
    a.stc();
    a.ret();
    a.bind(found);
    a.clc();
    a.ret();

    // `pci_read_dword`: the whole of a Dword configuration read, which is the
    // only width the scan needs.
    a.bind(l.pci_read_dword);
    a.call(l.pci_select);
    a.in_eax_dx();
    a.ret();
}
