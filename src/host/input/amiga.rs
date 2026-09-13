//! A person's keys and pointer, as an Amiga's keyboard and mouse receive them.
//!
//! Two [`InputSink`]s, the Amiga's counterparts of [`KeyboardSink`] and
//! [`MouseSink`]: [`AmigaKeyboardSink`] turns keysyms into raw keycodes for an
//! `amiga.keyboard`, and [`AmigaMouseSink`] turns absolute pointer positions
//! into counts for an `amiga.mouse`. Both deliver straight into the device's
//! host object, downstream of the frontend's own `input:` channel — the event
//! was recorded when the frontend posted it, so posting again on the device's
//! channel would record it twice.
//!
//! [`KeyboardSink`]: super::KeyboardSink
//! [`MouseSink`]: super::MouseSink
//!
//! # The keymap
//!
//! Raw keycodes are positions, not characters: "raw keycodes provide positional
//! information only, the legend which is printed on top of the keys changes from
//! country to country" (*Amiga Hardware Reference Manual*, 3rd edition, chapter
//! 8, "The Keyboard"). [`rawkey`] maps each keysym to the key that carries that
//! legend on the **USA keyboard**, from Appendix G's matrix table (pp. 362-363),
//! which prints every key's legend beside its code, and chapter 8's list of
//! codes `$40`-`$67`. A shifted character — `!`, `A`, `{` — is the key with
//! that legend and a shift, synthesised when the client has not said it is
//! holding one, exactly as [`KeyMap`](super::KeyMap) does for an AT keyboard.
//!
//! Host keys the Amiga has no key for, and what happens to them:
//!
//! | host key | Amiga |
//! | --- | --- |
//! | `Control_R` | Ctrl (`$63`): an Amiga has one Control key |
//! | `Super_L`/`Super_R`, `Meta_L`/`Meta_R` | Left and Right Amiga (`$66`, `$67`), which sit where a PC's Windows keys do |
//! | `Help` | Help (`$5F`) |
//! | `Insert` | Help (`$5F`). A PC keyboard has no Help key and the Amiga's is the right-hand key above the cursor keys, beside Del; `Insert` is the only key of that cluster left over once `Delete` is Del |
//! | the keypad with Num Lock off (`KP_Home`, `KP_Up`, …) | the keypad key in that position |
//! | `Home`, `End`, `Page_Up`, `Page_Down`, `F11`, `F12`, `Print`, `Scroll_Lock`, `Pause`, `Num_Lock`, `Menu`, and anything else | **dropped**: no code is sent. A guest given a key nobody pressed is worse off than one given nothing |
//!
//! The international keys `$2B` and `$30` ("cut out of" Return and the left
//! Shift on non-US keyboards) and the keypad's `(` and `)` (`$5A`, `$5B`) have
//! no X11 keysym a PC keyboard produces, so nothing reaches them.

#[cfg(any(feature = "dev-amiga-keyboard", feature = "dev-amiga-mouse"))]
use alloc::sync::Arc;

use super::Keysym;
#[cfg(any(feature = "dev-amiga-keyboard", feature = "dev-amiga-mouse"))]
use super::{InputEvent, InputSink};
#[cfg(any(feature = "dev-amiga-keyboard", feature = "dev-amiga-mouse"))]
use crate::core::sync::Mutex;

/// An Amiga key: its raw code, and whether the legend asked for is the shifted
/// one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawKey {
    /// The raw keycode, `$00`-`$67`.
    pub code: u8,
    /// Whether the character needs shift held.
    pub shifted: bool,
}

impl RawKey {
    const fn plain(code: u8) -> Option<RawKey> {
        Some(RawKey {
            code,
            shifted: false,
        })
    }

    const fn shift(code: u8) -> Option<RawKey> {
        Some(RawKey {
            code,
            shifted: true,
        })
    }
}

/// Left Shift's raw code.
pub const LEFT_SHIFT: u8 = 0x60;

/// Where a keysym is on a USA Amiga keyboard, or `None` for a key it lacks.
///
/// A `match`, like [`set2`](super::set2), so each line can be checked against
/// the matrix table.
#[must_use]
#[allow(clippy::too_many_lines)]
pub const fn rawkey(keysym: Keysym) -> Option<RawKey> {
    match keysym.0 {
        // -- row 1 of the matrix: the number row ------------------------------
        0x60 => RawKey::plain(0x00), // `
        0x7e => RawKey::shift(0x00), // ~
        0x31 => RawKey::plain(0x01),
        0x21 => RawKey::shift(0x01), // !
        0x32 => RawKey::plain(0x02),
        0x40 => RawKey::shift(0x02), // @
        0x33 => RawKey::plain(0x03),
        0x23 => RawKey::shift(0x03), // #
        0x34 => RawKey::plain(0x04),
        0x24 => RawKey::shift(0x04), // $
        0x35 => RawKey::plain(0x05),
        0x25 => RawKey::shift(0x05), // %
        0x36 => RawKey::plain(0x06),
        0x5e => RawKey::shift(0x06), // ^
        0x37 => RawKey::plain(0x07),
        0x26 => RawKey::shift(0x07), // &
        0x38 => RawKey::plain(0x08),
        0x2a => RawKey::shift(0x08), // *
        0x39 => RawKey::plain(0x09),
        0x28 => RawKey::shift(0x09), // (
        0x30 => RawKey::plain(0x0a),
        0x29 => RawKey::shift(0x0a), // )
        0x2d => RawKey::plain(0x0b), // -
        0x5f => RawKey::shift(0x0b), // _
        0x3d => RawKey::plain(0x0c), // =
        0x2b => RawKey::shift(0x0c), // +
        0x5c => RawKey::plain(0x0d), // \
        0x7c => RawKey::shift(0x0d), // |
        // -- row 2: Q to ] ----------------------------------------------------
        0x71 => RawKey::plain(0x10),
        0x77 => RawKey::plain(0x11),
        0x65 => RawKey::plain(0x12),
        0x72 => RawKey::plain(0x13),
        0x74 => RawKey::plain(0x14),
        0x79 => RawKey::plain(0x15),
        0x75 => RawKey::plain(0x16),
        0x69 => RawKey::plain(0x17),
        0x6f => RawKey::plain(0x18),
        0x70 => RawKey::plain(0x19),
        0x5b => RawKey::plain(0x1a), // [
        0x7b => RawKey::shift(0x1a), // {
        0x5d => RawKey::plain(0x1b), // ]
        0x7d => RawKey::shift(0x1b), // }
        // -- row 3: A to ' ----------------------------------------------------
        0x61 => RawKey::plain(0x20),
        0x73 => RawKey::plain(0x21),
        0x64 => RawKey::plain(0x22),
        0x66 => RawKey::plain(0x23),
        0x67 => RawKey::plain(0x24),
        0x68 => RawKey::plain(0x25),
        0x6a => RawKey::plain(0x26),
        0x6b => RawKey::plain(0x27),
        0x6c => RawKey::plain(0x28),
        0x3b => RawKey::plain(0x29), // ;
        0x3a => RawKey::shift(0x29), // :
        0x27 => RawKey::plain(0x2a), // '
        0x22 => RawKey::shift(0x2a), // "
        // -- row 4: Z to / ----------------------------------------------------
        0x7a => RawKey::plain(0x31),
        0x78 => RawKey::plain(0x32),
        0x63 => RawKey::plain(0x33),
        0x76 => RawKey::plain(0x34),
        0x62 => RawKey::plain(0x35),
        0x6e => RawKey::plain(0x36),
        0x6d => RawKey::plain(0x37),
        0x2c => RawKey::plain(0x38), // ,
        0x3c => RawKey::shift(0x38), // <
        0x2e => RawKey::plain(0x39), // .
        0x3e => RawKey::shift(0x39), // >
        0x2f => RawKey::plain(0x3a), // /
        0x3f => RawKey::shift(0x3a), // ?
        // -- capitals: the letter's key, shifted -----------------------------
        #[allow(clippy::cast_possible_truncation)]
        c @ 0x41..=0x5a => match rawkey(Keysym(c + 0x20)) {
            Some(k) => RawKey::shift(k.code),
            None => None,
        },
        // -- $40-$5F: "codes common to all keyboards" ------------------------
        0x20 => RawKey::plain(0x40),   // space
        0xff08 => RawKey::plain(0x41), // BackSpace
        0xff09 => RawKey::plain(0x42), // Tab
        0xff8d => RawKey::plain(0x43), // KP_Enter
        0xff0d => RawKey::plain(0x44), // Return
        0xff1b => RawKey::plain(0x45), // Escape
        0xffff => RawKey::plain(0x46), // Delete
        0xffad => RawKey::plain(0x4a), // KP_Subtract
        0xff52 => RawKey::plain(0x4c), // Up
        0xff54 => RawKey::plain(0x4d), // Down
        0xff53 => RawKey::plain(0x4e), // Right
        0xff51 => RawKey::plain(0x4f), // Left
        #[allow(clippy::cast_possible_truncation)]
        f @ 0xffbe..=0xffc7 => RawKey::plain(0x50 + (f - 0xffbe) as u8), // F1-F10
        0xffaf => RawKey::plain(0x5c), // KP_Divide
        0xffaa => RawKey::plain(0x5d), // KP_Multiply
        0xffab => RawKey::plain(0x5e), // KP_Add
        0xff6a => RawKey::plain(0x5f), // Help
        0xff63 => RawKey::plain(0x5f), // Insert, standing in for Help
        // -- the keypad's digits, Num Lock on and off ------------------------
        0xffb0 | 0xff9e => RawKey::plain(0x0f), // KP_0, KP_Insert
        0xffb1 | 0xff9c => RawKey::plain(0x1d), // KP_1, KP_End
        0xffb2 | 0xff99 => RawKey::plain(0x1e), // KP_2, KP_Down
        0xffb3 | 0xff9b => RawKey::plain(0x1f), // KP_3, KP_Page_Down
        0xffb4 | 0xff96 => RawKey::plain(0x2d), // KP_4, KP_Left
        0xffb5 | 0xff9d => RawKey::plain(0x2e), // KP_5, KP_Begin
        0xffb6 | 0xff98 => RawKey::plain(0x2f), // KP_6, KP_Right
        0xffb7 | 0xff95 => RawKey::plain(0x3d), // KP_7, KP_Home
        0xffb8 | 0xff97 => RawKey::plain(0x3e), // KP_8, KP_Up
        0xffb9 | 0xff9a => RawKey::plain(0x3f), // KP_9, KP_Page_Up
        0xffae | 0xff9f => RawKey::plain(0x3c), // KP_Decimal, KP_Delete
        // -- $60-$67: the qualifiers -----------------------------------------
        0xffe1 => RawKey::plain(0x60),          // Shift_L
        0xffe2 => RawKey::plain(0x61),          // Shift_R
        0xffe5 => RawKey::plain(0x62),          // Caps_Lock
        0xffe3 | 0xffe4 => RawKey::plain(0x63), // Control_L, Control_R
        0xffe9 => RawKey::plain(0x64),          // Alt_L
        0xffea | 0xfe03 => RawKey::plain(0x65), // Alt_R, ISO_Level3_Shift
        0xffeb | 0xffe7 => RawKey::plain(0x66), // Super_L, Meta_L
        0xffec | 0xffe8 => RawKey::plain(0x67), // Super_R, Meta_R
        _ => None,
    }
}

/// Keysyms into raw key movements, with shift synthesised where a client left
/// it out.
#[cfg(feature = "dev-amiga-keyboard")]
#[derive(Debug, Clone, Default)]
pub struct AmigaKeyMap {
    shift_held: bool,
}

#[cfg(feature = "dev-amiga-keyboard")]
impl AmigaKeyMap {
    /// A map with nothing held.
    #[must_use]
    pub const fn new() -> AmigaKeyMap {
        AmigaKeyMap { shift_held: false }
    }

    /// The movements — raw code, bit 7 set for a release — one transition
    /// produces, in order. Empty for a key the Amiga does not have.
    #[must_use]
    pub fn encode(&mut self, keysym: Keysym, down: bool) -> alloc::vec::Vec<u8> {
        use crate::dev::amiga::keyboard::code::KEY_UP;
        let mut out = alloc::vec::Vec::new();
        let Some(key) = rawkey(keysym) else {
            return out;
        };
        if keysym.is_shift() {
            self.shift_held = down;
        }
        let synth = key.shifted && !self.shift_held;
        if down {
            if synth {
                out.push(LEFT_SHIFT);
            }
            out.push(key.code);
        } else {
            out.push(key.code | KEY_UP);
            if synth {
                out.push(LEFT_SHIFT | KEY_UP);
            }
        }
        out
    }
}

/// An [`InputSink`] that types on an `amiga.keyboard`.
#[cfg(feature = "dev-amiga-keyboard")]
#[derive(Debug)]
pub struct AmigaKeyboardSink {
    keyboard: Arc<crate::dev::amiga::keyboard::Keyboard>,
    map: Mutex<AmigaKeyMap>,
}

#[cfg(feature = "dev-amiga-keyboard")]
impl AmigaKeyboardSink {
    /// Type on `keyboard`.
    #[must_use]
    pub fn new(keyboard: Arc<crate::dev::amiga::keyboard::Keyboard>) -> AmigaKeyboardSink {
        AmigaKeyboardSink {
            keyboard,
            map: Mutex::new(AmigaKeyMap::new()),
        }
    }

    /// Type on the first Amiga keyboard this build opened, if it has one.
    #[must_use]
    pub fn open(hosts: &crate::core::hosts::HostObjects) -> Option<AmigaKeyboardSink> {
        use crate::dev::amiga::keyboard::keys;
        let name = keys::names(hosts).into_iter().next()?;
        let keyboard = keys::get(hosts, &name).ok().flatten()?;
        Some(AmigaKeyboardSink::new(keyboard))
    }
}

#[cfg(feature = "dev-amiga-keyboard")]
impl InputSink for AmigaKeyboardSink {
    fn deliver(&self, event: InputEvent) {
        let InputEvent::Key { keysym, down } = event else {
            return;
        };
        // Translate under the map's lock, type with it released: the keyboard
        // takes its own and drives wires.
        let moves = self.map.lock().encode(keysym, down);
        for m in moves {
            self.keyboard.key(m);
        }
    }
}

/// Framebuffer pixels per mouse count.
///
/// Denise's picture is laid out in high-resolution pixels across and two rows
/// per line down (`host::display::amiga`), so a low-resolution pixel — the unit
/// a pointer is positioned in — is two framebuffer pixels each way. One count a
/// low-resolution pixel is a host convention, not a hardware fact: how far the
/// guest's pointer moves per count is the guest's own business (its mouse speed
/// and acceleration), and a relative mouse driven from an absolute protocol
/// cannot track the host cursor exactly whatever the ratio.
#[cfg(feature = "dev-amiga-mouse")]
pub const PIXELS_PER_COUNT: i64 = 2;

/// An [`InputSink`] that moves an `amiga.mouse`.
///
/// Converts as [`MouseSink`](super::MouseSink) does — keeps the last position
/// and sends the difference, the first event only establishing where the
/// pointer is — and carries the part of a delta smaller than one count so slow
/// movement is not lost.
#[cfg(feature = "dev-amiga-mouse")]
#[derive(Debug)]
pub struct AmigaMouseSink {
    mouse: Arc<crate::dev::amiga::mouse::Mouse>,
    /// Where the pointer was last reported, in framebuffer pixels, and the
    /// buttons then held.
    at: Mutex<Option<(i64, i64, u8)>>,
}

#[cfg(feature = "dev-amiga-mouse")]
impl AmigaMouseSink {
    /// Move `mouse`.
    #[must_use]
    pub fn new(mouse: Arc<crate::dev::amiga::mouse::Mouse>) -> AmigaMouseSink {
        AmigaMouseSink {
            mouse,
            at: Mutex::new(None),
        }
    }

    /// Move the first Amiga mouse this build opened, if it has one.
    #[must_use]
    pub fn open(hosts: &crate::core::hosts::HostObjects) -> Option<AmigaMouseSink> {
        use crate::dev::amiga::mouse::mice;
        let name = mice::names(hosts).into_iter().next()?;
        let mouse = mice::get(hosts, &name).ok().flatten()?;
        Some(AmigaMouseSink::new(mouse))
    }
}

#[cfg(feature = "dev-amiga-mouse")]
impl InputSink for AmigaMouseSink {
    fn deliver(&self, event: InputEvent) {
        let InputEvent::Pointer { x, y, buttons } = event else {
            return;
        };
        let (x, y) = (i64::from(x), i64::from(y));
        let buttons = buttons & 0b111;
        let (dx, dy, moved_buttons) = {
            let mut at = self.at.lock();
            let (px, py, pb) = at.unwrap_or((x, y, 0));
            let dx = (x - px) / PIXELS_PER_COUNT;
            let dy = (y - py) / PIXELS_PER_COUNT;
            // Advance by what was sent, so the remainder is still owed.
            *at = Some((
                px + dx * PIXELS_PER_COUNT,
                py + dy * PIXELS_PER_COUNT,
                buttons,
            ));
            (dx, dy, pb != buttons)
        };
        if dx == 0 && dy == 0 && !moved_buttons {
            return;
        }
        #[allow(clippy::cast_possible_truncation)]
        self.mouse.report(
            dx.clamp(-32_767, 32_767) as i32,
            dy.clamp(-32_767, 32_767) as i32,
            buttons,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_matrix_table_legends_land_on_their_codes() {
        // Spot checks against Appendix G's table, one per row and column group.
        let code = |c: u8| rawkey(Keysym::from_ascii(c)).map(|k| k.code);
        assert_eq!(code(b'`'), Some(0x00));
        assert_eq!(code(b'1'), Some(0x01));
        assert_eq!(code(b'0'), Some(0x0a));
        assert_eq!(code(b'\\'), Some(0x0d));
        assert_eq!(code(b'q'), Some(0x10));
        assert_eq!(code(b']'), Some(0x1b));
        assert_eq!(code(b'a'), Some(0x20));
        assert_eq!(code(b'\''), Some(0x2a));
        assert_eq!(code(b'z'), Some(0x31));
        assert_eq!(code(b'b'), Some(0x35), "chapter 8's own example: B is $35");
        assert_eq!(code(b'/'), Some(0x3a));
        assert_eq!(code(b' '), Some(0x40));
        assert_eq!(rawkey(Keysym::F1).map(|k| k.code), Some(0x50));
        assert_eq!(rawkey(Keysym(0xffc7)).map(|k| k.code), Some(0x59), "F10");
        assert_eq!(rawkey(Keysym::RETURN).map(|k| k.code), Some(0x44));
        assert_eq!(rawkey(Keysym::DELETE).map(|k| k.code), Some(0x46));
        assert_eq!(rawkey(Keysym::LEFT).map(|k| k.code), Some(0x4f));
        assert_eq!(rawkey(Keysym::CONTROL_R).map(|k| k.code), Some(0x63));
        assert_eq!(code(b'!'), Some(0x01));
        assert_eq!(rawkey(Keysym::from_ascii(b'B')), RawKey::shift(0x35));
    }

    #[cfg(feature = "dev-amiga-keyboard")]
    #[test]
    fn keys_the_amiga_lacks_send_nothing() {
        let mut map = AmigaKeyMap::new();
        for k in [Keysym::HOME, Keysym::END, Keysym::PAGE_UP, Keysym::F12] {
            assert!(map.encode(k, true).is_empty(), "{k}");
        }
    }

    #[cfg(feature = "dev-amiga-keyboard")]
    #[test]
    fn a_shifted_character_brings_its_own_shift_unless_one_is_held() {
        use crate::dev::amiga::keyboard::code::KEY_UP;
        let mut map = AmigaKeyMap::new();
        assert_eq!(map.encode(Keysym::from_ascii(b'!'), true), [0x60, 0x01]);
        assert_eq!(
            map.encode(Keysym::from_ascii(b'!'), false),
            [0x01 | KEY_UP, 0x60 | KEY_UP]
        );
        assert_eq!(map.encode(Keysym::SHIFT_L, true), [0x60]);
        assert_eq!(map.encode(Keysym::from_ascii(b'!'), true), [0x01]);
    }
}
