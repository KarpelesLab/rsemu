//! A native window onto a machine's display, through `minifb`.
//!
//! **The one GUI dependency in the tree, and opt-in.** The dependency policy
//! (`CLAUDE.md`) keeps every GUI toolkit out, and [`super::display`] explains
//! why a hand-written X11, Wayland, Win32 and Cocoa backend each is its own
//! piece of protocol work. `minifb` is the exception the owner allowed, behind
//! the `window` feature: one crate — five in its macOS tree, six on Windows —
//! that opens a window, takes a `u32` pixel buffer and reports keys and the
//! mouse, which is the whole of what a scanout needs. A default build, and
//! every `no_std` and wasm build, never sees it. [`super::vnc`] remains the
//! dependency-free way to look at a screen.
//!
//! # The loop
//!
//! [`WindowSession`] is [`VncSession`](super::vnc::VncSession)'s loop with a
//! window where the socket was, and every argument that module makes about
//! determinism holds here unchanged: a slice of virtual time per turn, input
//! *posted* to the machine's recorder (or delivered straight to the sinks when
//! nothing is recording), and the wait decided by the scheduler's rate
//! controller, never by this module's own timing.
//!
//! ```text
//!   capture the scanout ─► present it ─► collect keys and the pointer
//!        ▲                                         │ post / deliver
//!        └── wait (rate controller) ◄── run_until(now + slice)
//! ```
//!
//! # Input
//!
//! Keys become X11 keysyms (the [`Keysym`] the rest of `host::input` speaks),
//! and the pointer becomes absolute [`InputEvent::Pointer`]s in framebuffer
//! pixels — a left button held on a touchscreen machine is a finger on the
//! glass. Only changes are sent: a pointer that has not moved and whose
//! buttons have not changed produces nothing.

use std::sync::Arc;
use std::time::Duration;

use minifb::{Key, KeyRepeat, MouseButton, MouseMode, Scale, ScaleMode, Window, WindowOptions};

use crate::core::clock::GlobalTime;
use crate::core::record::Channel;
use crate::core::sched::{Pace, RateControl};
use crate::host::clock::MonotonicClock;
use crate::host::display::{PixelFormat, Scanout, Surface};
use crate::host::input::{self, Feed, InputEvent, InputSink, Keysym};
use crate::machine::Machine;

/// How much virtual time one turn advances: one frame at 60 Hz, as
/// [`super::vnc`]'s `SLICE` and for the same reasons.
pub const SLICE: GlobalTime = GlobalTime::from_nanos(16_666_667);

/// The longest single wait, so the window keeps answering its event queue
/// while the machine is ahead of the wall.
const MAX_WAIT: Duration = Duration::from_millis(50);

/// How far behind the wall a machine may fall before the debt is forgiven.
const MAX_CATCHUP_NANOS: u64 = 250_000_000;

/// Why a window could not be opened.
#[derive(Debug)]
pub struct WindowError(String);

impl core::fmt::Display for WindowError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for WindowError {}

/// The most pixels a side this window will hold.
///
/// A scanout reports whatever geometry its device computed, and on a machine
/// with a programmable display that is whatever the guest wrote into a mode
/// register. The adapters in this tree bound their own, but this is the last
/// place before a host allocation and a window-system call, and it should not
/// depend on every device model, present and future, having remembered to.
/// 16 384 a side is far past any display this tree models and inside what a
/// window system that counts pixels in sixteen bits can be asked for.
pub const MAX_SIDE: u32 = 16_384;

/// How many `0RGB` words a `width` x `height` picture is, or why this window
/// will not hold one.
///
/// The product is taken in `usize` after both factors are bounded: as
/// `(width * height) as usize` it was a `u32` multiply, which panics in a
/// debug build and wraps in a release one for a geometry of 65 536 squared —
/// leaving a buffer of the wrong size behind a window of the right one.
fn words_for(width: u32, height: u32) -> Result<usize, WindowError> {
    if width > MAX_SIDE || height > MAX_SIDE {
        return Err(WindowError(format!(
            "the display reports a {width}x{height} picture, and a window holds at most \
             {MAX_SIDE} pixels a side"
        )));
    }
    Ok(width as usize * height as usize)
}

/// A machine, a window, and the loop between them.
pub struct WindowSession {
    window: Window,
    scanout: Box<dyn Scanout>,
    surface: Surface,
    /// The surface repacked as `0RGB` words, which is what `minifb` draws.
    words: Vec<u32>,
    feed: Arc<Feed>,
    channel: Channel,
    captured: u64,
    /// The pointer state last sent, so only changes cross the seam.
    pointer: Option<(u32, u32, u8)>,
    /// What the window was opened as, to put back once it stops saying why it
    /// is not showing the guest's picture.
    title: String,
    /// The geometry currently being refused ([`MAX_SIDE`]), if any.
    refused: Option<(u32, u32)>,
}

impl core::fmt::Debug for WindowSession {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("WindowSession")
            .field("scanout", &self.scanout)
            .field("channel", &self.channel.to_string())
            .finish_non_exhaustive()
    }
}

impl WindowSession {
    /// Open a window titled `title` onto `scanout`, drawn `scale` times its
    /// native size (1, 2 or 4; anything else is 1).
    ///
    /// # Errors
    ///
    /// The platform refused the window — no display server, say.
    pub fn open(
        title: &str,
        scanout: Box<dyn Scanout>,
        scale: u32,
    ) -> Result<WindowSession, WindowError> {
        let info = scanout.info();
        // Before the window exists, so a refusal leaves nothing to tear down.
        let words = words_for(info.width, info.height)?;
        let options = WindowOptions {
            resize: true,
            scale: match scale {
                2 => Scale::X2,
                4 => Scale::X4,
                _ => Scale::X1,
            },
            scale_mode: ScaleMode::AspectRatioStretch,
            ..WindowOptions::default()
        };
        // `{e:?}`, not `{e}`: minifb's Display drops the reason it carries, so
        // an XOpenIM failure under an unusable XMODIFIERS read only as "Failed
        // to create window".
        let window = Window::new(title, info.width as usize, info.height as usize, options)
            .map_err(|e| WindowError(format!("cannot open a window: {e:?}")))?;
        Ok(WindowSession {
            window,
            surface: Surface::new(PixelFormat::BGRA8888, info.width, info.height),
            words: vec![0; words],
            scanout,
            feed: Arc::new(Feed::new()),
            channel: input::channel(input::DEFAULT_STREAM),
            captured: u64::MAX,
            pointer: None,
            title: title.to_string(),
            refused: None,
        })
    }

    /// Send input to `sink` as well as to whatever is already attached.
    #[must_use]
    pub fn with_sink(self, sink: Arc<dyn InputSink>) -> WindowSession {
        self.feed.attach(sink);
        self
    }

    /// Register this session's channel with `recorder`, as
    /// [`VncSession::attach`](super::vnc::VncSession::attach) does.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if the recorder is already sealed.
    pub fn attach(&self, recorder: &crate::core::record::Recorder) -> crate::Result<()> {
        recorder.register(self.channel.clone(), input::sink(&self.feed))
    }

    /// The screen this session was handed, for a screenshot at the end.
    #[must_use]
    pub fn scanout(&self) -> &dyn Scanout {
        self.scanout.as_ref()
    }

    /// Whether the window is still open.
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.window.is_open()
    }

    /// Run until the window closes or `keep_going` says stop.
    ///
    /// # Errors
    ///
    /// Whatever the machine refuses, or a recorder that does not know this
    /// session's channel.
    pub fn run(
        &mut self,
        machine: &mut Machine,
        mut keep_going: impl FnMut(&mut Machine) -> bool,
    ) -> crate::Result<()> {
        self.install(machine);
        while self.window.is_open() {
            self.present();
            for event in self.collect() {
                match machine.recorder() {
                    Some(recorder) => {
                        // `false` is a replay declining live input, which is
                        // the recorder's rule, not an error.
                        let _ = recorder.post(&self.channel, &event.encode())?;
                    }
                    None => self.feed.deliver(event),
                }
            }
            let deadline = machine.now().saturating_add(SLICE);
            machine.run_until(deadline)?;
            let pace = machine.scheduler_mut().pace()?;
            if let Pace::Wait { nanos } = pace {
                std::thread::sleep(Duration::from_nanos(nanos).min(MAX_WAIT));
            }
            if !keep_going(machine) {
                break;
            }
        }
        Ok(())
    }

    /// A host clock and real-time pacing, as a live VNC session sets up.
    fn install(&self, machine: &mut Machine) {
        let clock = MonotonicClock::new();
        let now_host = {
            use crate::core::sched::HostClock;
            clock.monotonic_nanos()
        };
        let now = machine.now();
        machine.set_host_clock(Box::new(clock));
        machine.scheduler_mut().rate_controller_mut().set_control(
            RateControl::Realtime {
                max_catchup_nanos: MAX_CATCHUP_NANOS,
            },
            now_host,
            now,
        );
    }

    /// Refill the surface if the device drew since, and put it on screen.
    /// Pumps the window's event queue either way.
    ///
    /// A guest that programs a picture larger than a window holds
    /// ([`MAX_SIDE`]) is not a reason to end the run — a mode set can pass
    /// through nonsense on its way somewhere sensible. The window keeps its
    /// last picture and says why in its title, and nothing of the refused
    /// size is allocated: the check is before the surface is reshaped or the
    /// device is asked to paint.
    fn present(&mut self) {
        let info = self.scanout.info();
        let words = match words_for(info.width, info.height) {
            Ok(words) => words,
            Err(e) => {
                if self.refused != Some((info.width, info.height)) {
                    self.refused = Some((info.width, info.height));
                    self.window.set_title(&format!("{} — {e}", self.title));
                }
                // The event queue still has to be answered.
                self.window.update();
                return;
            }
        };
        // Back to something a window holds — which may be the very geometry
        // the surface still has, so this is not inside the reshape below.
        if self.refused.take().is_some() {
            self.window.set_title(&self.title);
        }
        if info.width != self.surface.width() || info.height != self.surface.height() {
            self.surface
                .reshape(PixelFormat::BGRA8888, info.width, info.height);
            self.words = vec![0; words];
            self.captured = u64::MAX;
        }
        let counter = self.scanout.frame_counter();
        if counter != self.captured {
            self.captured = self.scanout.capture(&mut self.surface);
            // BGRA in memory is a little-endian 0xAARRGGBB word; minifb reads
            // the low 24 bits as RGB and ignores the top byte.
            let (pixels, _) = self.surface.pixels().as_chunks::<4>();
            for (word, px) in self.words.iter_mut().zip(pixels) {
                *word = u32::from_le_bytes([px[0], px[1], px[2], 0]);
            }
        }
        let (w, h) = (
            self.surface.width() as usize,
            self.surface.height() as usize,
        );
        // A failed present is a window going away; `is_open` reports it on
        // the next turn, which is where the loop ends.
        let _ = self.window.update_with_buffer(&self.words, w, h);
    }

    /// The keys and pointer changes since the last turn, as input events.
    fn collect(&mut self) -> Vec<InputEvent> {
        let mut events = Vec::new();
        for key in self.window.get_keys_pressed(KeyRepeat::No) {
            if let Some(keysym) = keysym(key) {
                events.push(InputEvent::Key { keysym, down: true });
            }
        }
        for key in self.window.get_keys_released() {
            if let Some(keysym) = keysym(key) {
                events.push(InputEvent::Key {
                    keysym,
                    down: false,
                });
            }
        }
        if let Some((x, y)) = self.window.get_mouse_pos(MouseMode::Discard) {
            let mut buttons = 0u8;
            for (bit, button) in [
                (0, MouseButton::Left),
                (1, MouseButton::Middle),
                (2, MouseButton::Right),
            ] {
                if self.window.get_mouse_down(button) {
                    buttons |= 1 << bit;
                }
            }
            let now = (x.max(0.0) as u32, y.max(0.0) as u32, buttons);
            if self.pointer != Some(now) {
                self.pointer = Some(now);
                events.push(InputEvent::Pointer {
                    x: now.0,
                    y: now.1,
                    buttons: now.2,
                });
            }
        }
        events
    }
}

/// The X11 keysym for a `minifb` key, for the keys a guest is likely to
/// want. Letters are the lower-case keysyms; shift is its own key, as on
/// the wire.
fn keysym(key: Key) -> Option<Keysym> {
    use Key as K;
    let letters = [
        K::A,
        K::B,
        K::C,
        K::D,
        K::E,
        K::F,
        K::G,
        K::H,
        K::I,
        K::J,
        K::K,
        K::L,
        K::M,
        K::N,
        K::O,
        K::P,
        K::Q,
        K::R,
        K::S,
        K::T,
        K::U,
        K::V,
        K::W,
        K::X,
        K::Y,
        K::Z,
    ];
    if let Some(i) = letters.iter().position(|&k| k == key) {
        return Some(Keysym(u32::from(b'a') + i as u32));
    }
    let digits = [
        K::Key0,
        K::Key1,
        K::Key2,
        K::Key3,
        K::Key4,
        K::Key5,
        K::Key6,
        K::Key7,
        K::Key8,
        K::Key9,
    ];
    if let Some(i) = digits.iter().position(|&k| k == key) {
        return Some(Keysym(u32::from(b'0') + i as u32));
    }
    // F1 to F12 are consecutive keysyms (0xffbe..=0xffc9), and they are not
    // optional: the Amiga map has ten of them, the AT keyboard map all
    // twelve, and the navigation unit's setup key is F1. `Keysym` names the
    // two ends of the run.
    let function = [
        K::F1,
        K::F2,
        K::F3,
        K::F4,
        K::F5,
        K::F6,
        K::F7,
        K::F8,
        K::F9,
        K::F10,
        K::F11,
        K::F12,
    ];
    if let Some(i) = function.iter().position(|&k| k == key) {
        return Some(Keysym(Keysym::F1.0 + i as u32));
    }
    Some(match key {
        K::Delete => Keysym::DELETE,
        K::Space => Keysym(0x20),
        K::Enter => Keysym::RETURN,
        K::Escape => Keysym::ESCAPE,
        K::Backspace => Keysym::BACKSPACE,
        K::Tab => Keysym::TAB,
        K::Left => Keysym::LEFT,
        K::Right => Keysym::RIGHT,
        K::Up => Keysym::UP,
        K::Down => Keysym::DOWN,
        K::Home => Keysym::HOME,
        K::End => Keysym::END,
        K::PageUp => Keysym::PAGE_UP,
        K::PageDown => Keysym::PAGE_DOWN,
        K::Insert => Keysym::INSERT,
        K::LeftShift => Keysym::SHIFT_L,
        K::RightShift => Keysym::SHIFT_R,
        K::LeftCtrl => Keysym::CONTROL_L,
        K::RightCtrl => Keysym::CONTROL_R,
        K::LeftAlt => Keysym::ALT_L,
        K::RightAlt => Keysym::ALT_R,
        K::CapsLock => Keysym::CAPS_LOCK,
        K::Comma => Keysym(u32::from(b',')),
        K::Period => Keysym(u32::from(b'.')),
        K::Slash => Keysym(u32::from(b'/')),
        K::Semicolon => Keysym(u32::from(b';')),
        K::Apostrophe => Keysym(u32::from(b'\'')),
        K::Minus => Keysym(u32::from(b'-')),
        K::Equal => Keysym(u32::from(b'=')),
        K::LeftBracket => Keysym(u32::from(b'[')),
        K::RightBracket => Keysym(u32::from(b']')),
        K::Backslash => Keysym(u32::from(b'\\')),
        K::Backquote => Keysym(u32::from(b'`')),
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A geometry no window holds is refused with its numbers in the message,
    /// and one that fits is sized without a 32-bit multiply.
    #[test]
    fn an_absurd_geometry_is_refused_rather_than_multiplied() {
        assert_eq!(words_for(720, 400).ok(), Some(288_000));
        assert_eq!(words_for(0, 0).ok(), Some(0));
        assert_eq!(
            words_for(MAX_SIDE, MAX_SIDE).ok(),
            Some(MAX_SIDE as usize * MAX_SIDE as usize)
        );
        // 65 536 squared is zero as a `u32` product: the old sizing made an
        // empty buffer for it in release and panicked in debug.
        for (w, h) in [(65_536, 65_536), (MAX_SIDE + 1, 1), (1, u32::MAX)] {
            let e = words_for(w, h).expect_err("too large");
            let text = e.to_string();
            assert!(text.contains(&format!("{w}x{h}")), "{text}");
            assert!(text.contains("16384"), "{text}");
        }
    }

    #[test]
    fn keys_map_to_x11_keysyms() {
        assert_eq!(keysym(Key::A), Some(Keysym(0x61)));
        assert_eq!(keysym(Key::Z), Some(Keysym(0x7a)));
        assert_eq!(keysym(Key::Key7), Some(Keysym(0x37)));
        assert_eq!(keysym(Key::Enter), Some(Keysym::RETURN));
        assert_eq!(keysym(Key::LeftShift), Some(Keysym::SHIFT_L));
        assert_eq!(keysym(Key::Pause), None, "unmapped keys send nothing");
    }

    /// The keys a VNC client can already send and this window could not: the
    /// guest keyboards in `host::input` map all of them, so dropping them here
    /// made the native window the one frontend a firmware setup screen or an
    /// Amiga function key could not be reached from.
    #[test]
    fn the_function_keys_and_delete_reach_the_guest() {
        assert_eq!(keysym(Key::F1), Some(Keysym::F1));
        assert_eq!(keysym(Key::F6), Some(Keysym(0xffc3)));
        assert_eq!(keysym(Key::F10), Some(Keysym(0xffc7)));
        assert_eq!(keysym(Key::F12), Some(Keysym::F12));
        assert_eq!(keysym(Key::Delete), Some(Keysym::DELETE));
    }
}
