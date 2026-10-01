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
        let window = Window::new(title, info.width as usize, info.height as usize, options)
            .map_err(|e| WindowError(format!("cannot open a window: {e}")))?;
        Ok(WindowSession {
            window,
            surface: Surface::new(PixelFormat::BGRA8888, info.width, info.height),
            words: vec![0; (info.width * info.height) as usize],
            scanout,
            feed: Arc::new(Feed::new()),
            channel: input::channel(input::DEFAULT_STREAM),
            captured: u64::MAX,
            pointer: None,
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
    fn present(&mut self) {
        let info = self.scanout.info();
        if info.width != self.surface.width() || info.height != self.surface.height() {
            self.surface
                .reshape(PixelFormat::BGRA8888, info.width, info.height);
            self.words = vec![0; (info.width * info.height) as usize];
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
    Some(match key {
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

    #[test]
    fn keys_map_to_x11_keysyms() {
        assert_eq!(keysym(Key::A), Some(Keysym(0x61)));
        assert_eq!(keysym(Key::Z), Some(Keysym(0x7a)));
        assert_eq!(keysym(Key::Key7), Some(Keysym(0x37)));
        assert_eq!(keysym(Key::Enter), Some(Keysym::RETURN));
        assert_eq!(keysym(Key::LeftShift), Some(Keysym::SHIFT_L));
        assert_eq!(keysym(Key::F12), None, "unmapped keys send nothing");
    }
}
