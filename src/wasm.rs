//! WebAssembly entry points.
//!
//! Plain C-ABI exports and embedder-supplied imports, following purecrypto's
//! browser convention — deliberately **no** `wasm-bindgen`: the dependency
//! policy forbids it (`ROADMAP.md` §0), and the boundary is small enough that
//! it does not need a binding generator.
//!
//! The host supplies the JS glue: it instantiates the module, reads exported
//! memory directly, and provides the imports rsemu needs (a clock, entropy,
//! and — once the JIT lands — module compilation). See `ROADMAP.md` §11, and
//! `web/` for the page that drives everything below.
//!
//! # Build
//!
//! ```sh
//! # the boundary alone, which is what CI builds every commit
//! cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
//!     --no-default-features --features wasm --release
//!
//! # the demo, which adds the machines the page offers
//! cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
//!     --no-default-features --features demo --release
//! ```
//!
//! # The ABI
//!
//! Three rules, and everything below follows from them.
//!
//! 1. **Nothing crosses as a pointer the embedder made.** JavaScript writes
//!    into a buffer rsemu owns ([`rsemu_input_reserve`]) and reads out of one
//!    rsemu owns ([`rsemu_output_ptr`], [`rsemu_frame_ptr`]). So there is no
//!    `from_raw_parts` on caller-supplied addresses anywhere in this file, and
//!    a page that gets a length wrong corrupts its own picture rather than the
//!    heap.
//! 2. **Machines are named by index**, from [`rsemu_machine_count`] and
//!    [`rsemu_machine_name`] — a build is a feature set, so the catalog is
//!    build-specific and the page has to ask anyway. That also means no string
//!    ever crosses *into* the module.
//! 3. **One machine at a time**, in a module-wide slot. The browser runs one
//!    console in one tab; a second instance is a second module.
//!
//! Every call that can fail returns `0` for failure and leaves a message in
//! [`rsemu_error`]. Every call that returns bytes returns their length and
//! leaves them at [`rsemu_output_ptr`], valid until the next call that writes
//! there.
//!
//! Two buffers sit outside that rule because they are read many times per
//! second and copying them through `output` would be silly: the picture
//! ([`rsemu_frame_ptr`]) and the sound ([`rsemu_audio_ptr`]). Both are still
//! rsemu's own memory — rule 1 holds — and both are read at an address the
//! module hands out, never one JavaScript made.
//!
//! # What runs where
//!
//! The non-threaded configuration is a supported target, not a fallback
//! (`ROADMAP.md` §11.3): [`rsemu_run_frame`] advances virtual time by exactly
//! one video frame and returns, so a page can drive it from
//! `requestAnimationFrame` and stay responsive without `SharedArrayBuffer`,
//! `Atomics.wait`, or a worker. Nothing here reads a host clock — the frame
//! period is computed from the machine's own oscillator forest.
//!
//! Sound rides the same call. [`rsemu_run_frame`] drains the machine's audio
//! device into a queue the page reads through [`rsemu_audio_ptr`], resampled
//! from the console's own crystal-derived rate to whatever the page's
//! `AudioContext` runs at. **The pull happens whether or not the page is
//! listening and never changes how far the machine advances**, which is what
//! keeps [`rsemu_state_hash`] independent of the audio path — see
//! [`crate::host::audio`] for the whole argument.
//!
//! # `unsafe` in this module
//!
//! This is the **C ABI boundary**, one of the six subsystems `ROADMAP.md` §0
//! sanctions to opt back in. Two things here need it: `#[unsafe(no_mangle)]`,
//! which edition 2024 classifies as an unsafe attribute because duplicate
//! exported symbols are the linker's problem rather than the compiler's; and
//! the private `leaked` helper, which rebuilds a `&'static str` from a
//! pointer/length pair. The allow is module-scoped rather than crate-wide, and
//! every genuine `unsafe` block below carries its own `// SAFETY:` argument.
#![allow(unsafe_code)]

use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::sync::{Global, LockRank};

/// Length in bytes of the string [`rsemu_version_ptr`] points at.
///
/// The pair is the minimal ABI for returning a string without an allocator
/// dance on the JS side: the host reads `len` bytes of exported memory from
/// `ptr`. Both values are stable for the life of the module.
///
/// # Safety
///
/// This function is safe; the pointer it pairs with is into a leaked static
/// allocation that outlives every caller.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_version_len() -> usize {
    version_static().len()
}

/// Pointer to the UTF-8 build-info string, `rsemu_version_len()` bytes long.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_version_ptr() -> *const u8 {
    version_static().as_ptr()
}

/// A trivial round-trip export, so the host glue can prove the module is live
/// before any real functionality exists.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_echo(value: u32) -> u32 {
    value
}

/// The build-info string, allocated once and leaked.
///
/// Leaking is correct here rather than lazy: the value is needed for the whole
/// lifetime of the module, and a wasm module's memory dies with the page.
fn version_static() -> &'static str {
    use core::sync::atomic::{AtomicPtr, Ordering};

    static CACHE: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());
    static LEN: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

    let cached = CACHE.load(Ordering::Acquire);
    if !cached.is_null() {
        let len = LEN.load(Ordering::Acquire);
        return leaked(cached, len);
    }

    let info: String = crate::build_info();
    let leaked_str: &'static str = String::leak(info);
    LEN.store(leaked_str.len(), Ordering::Release);
    CACHE.store(leaked_str.as_ptr().cast_mut(), Ordering::Release);
    leaked_str
}

/// Rebuild the `&'static str` we leaked earlier.
///
/// Kept in one place so the single `unsafe` block has one safety argument
/// rather than one per call site.
fn leaked(ptr: *const u8, len: usize) -> &'static str {
    // SAFETY: `ptr`/`len` come from a `String::leak` in `version_static`, so
    // they describe a live, immutable, well-formed UTF-8 allocation that is
    // never freed and never written again. The only writer publishes with
    // Release before any reader can observe a non-null pointer with Acquire.
    unsafe {
        let bytes = core::slice::from_raw_parts(ptr, len);
        core::str::from_utf8_unchecked(bytes)
    }
}

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

/// Everything the module owns between calls.
///
/// One slot, taken at [`LockRank::MACHINE`] — the outermost rank, so anything
/// a machine does underneath (scheduler, bus, device, wire) is a strictly
/// increasing acquisition and the debug lock-order check stays satisfied.
struct State {
    machine: Option<crate::machine::Machine>,
    /// The picture, in the format a canvas wants. Its address is stable while
    /// the geometry is, which is what lets JS keep one `Uint8ClampedArray`.
    frame: crate::host::display::Surface,
    /// Where the picture comes from, if this machine has one.
    scanout: Option<alloc::boxed::Box<dyn crate::host::display::Scanout>>,
    /// The character port the machine opened, if it opened one.
    console: Option<alloc::sync::Arc<crate::host::chardev::CharPort>>,
    /// The pad port the machine's controllers read, if it has any.
    #[cfg(feature = "dev-nes-io")]
    pad: Option<alloc::sync::Arc<crate::dev::nes::input::Pad>>,
    /// The sound, resampled to whatever the page's `AudioContext` runs at, if
    /// this machine makes any.
    audio: Option<crate::host::audio::AudioStream>,
    /// The rate the page last asked for. Kept across boots, because an
    /// `AudioContext`'s rate is a property of the browser rather than of the
    /// machine and the page should not have to say it twice.
    audio_rate: u32,
    /// Bytes JavaScript hands in: ROM images, typed characters, save states.
    input: Vec<u8>,
    /// Bytes JavaScript reads back: console output, save states, messages.
    output: Vec<u8>,
    /// Why the last call that returned `0` did.
    error: String,
    /// Controller state per port, as most-recently set by the embedder.
    buttons: [u32; 2],
    /// The frame serial the embedder has already been shown.
    shown: u64,
}

impl State {
    const fn new() -> State {
        State {
            machine: None,
            frame: crate::host::display::Surface::empty(),
            scanout: None,
            console: None,
            #[cfg(feature = "dev-nes-io")]
            pad: None,
            audio: None,
            audio_rate: DEFAULT_AUDIO_RATE,
            input: Vec::new(),
            output: Vec::new(),
            error: String::new(),
            buttons: [0; 2],
            shown: u64::MAX,
        }
    }
}

/// The module's one machine slot.
///
/// [`Global`] rather than `Mutex`: this is a `static`, so it is reachable from
/// every thread in the process — one Web Worker per hart on a threaded wasm
/// build, and the test harness's threads in the unit tests below (`core::sync`).
static STATE: Global<State> = Global::with_rank(LockRank::MACHINE, State::new());

/// Run `f` against the module state.
fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    let mut state = STATE.lock();
    f(&mut state)
}

/// Record a failure and answer `0`, the ABI's "no".
fn fail(state: &mut State, message: impl ToString) -> u32 {
    state.error = message.to_string();
    0
}

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------

/// Resize the input buffer to `len` bytes and return its address.
///
/// JavaScript writes a ROM image, typed characters or a save state here and
/// then calls whichever function consumes it. The address changes whenever the
/// buffer grows, so call this immediately before writing and do not cache it.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_input_reserve(len: usize) -> *mut u8 {
    with_state(|state| {
        state.input.clear();
        state.input.resize(len, 0);
        state.input.as_mut_ptr()
    })
}

/// Address of the output buffer: whatever the last call that returned a length
/// left there.
///
/// Valid until the next call that writes output, which is any of
/// [`rsemu_machine_name`], [`rsemu_machine_media`], [`rsemu_console_read`],
/// [`rsemu_save`] and [`rsemu_error`].
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_output_ptr() -> *const u8 {
    with_state(|state| state.output.as_ptr())
}

/// Copy the message left by the last failing call into the output buffer,
/// returning its length in bytes. `0` means there is nothing to report.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_error() -> usize {
    with_state(|state| {
        state.output.clear();
        state.output.extend_from_slice(state.error.as_bytes());
        state.output.len()
    })
}

// ---------------------------------------------------------------------------
// The catalog
// ---------------------------------------------------------------------------

/// How many machines this build can run.
///
/// A machine is a feature set (`ROADMAP.md` §3), so this is a fact about the
/// `.wasm` the page fetched, not about rsemu. Zero is a correct answer for a
/// module built with `--features wasm` alone.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_count() -> u32 {
    crate::machine::catalog::machines().len() as u32
}

/// Copy machine `index`'s name into the output buffer, returning its length.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_name(index: u32) -> usize {
    with_state(|state| {
        state.output.clear();
        if let Some(entry) = catalog_entry(index) {
            state.output.extend_from_slice(entry.name.as_bytes());
        }
        state.output.len()
    })
}

/// Copy machine `index`'s summary into the output buffer, returning its length.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_summary(index: u32) -> usize {
    with_state(|state| {
        state.output.clear();
        if let Some(entry) = catalog_entry(index) {
            state.output.extend_from_slice(entry.summary.as_bytes());
        }
        state.output.len()
    })
}

/// Copy the name of the media slot machine `index` loads an image into — the
/// NES's `cart`, the Apple 1's `rom` — returning its length.
///
/// `0` means the machine needs no image, which is how a page knows whether to
/// insist on a file before offering to boot.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_media(index: u32) -> usize {
    with_state(|state| {
        state.output.clear();
        if let Some(slot) = catalog_entry(index).and_then(|e| e.media.first()) {
            state.output.extend_from_slice(slot.as_bytes());
        }
        state.output.len()
    })
}

/// One catalog entry by index.
fn catalog_entry(index: u32) -> Option<&'static crate::machine::catalog::CatalogEntry> {
    crate::machine::catalog::machines()
        .get(index as usize)
        .copied()
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/// Build machine `index`, binding the first `image_len` bytes of the input
/// buffer to its media slot. Returns `1` on success, `0` with [`rsemu_error`].
///
/// An `image_len` of `0` binds nothing, which is what an Apple 1 wants: it
/// falls back to rsemu's own monitor ROM, exactly as `rsemu run apple1` does.
///
/// Any previous machine is dropped first, so booting twice is how the page
/// changes cartridges.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_boot(index: u32, image_len: usize) -> u32 {
    with_state(|state| {
        let Some(entry) = catalog_entry(index) else {
            return fail(state, "no machine with that index in this build");
        };

        // Drop the old machine before building the new one: its devices hold
        // character ports and clock domains, and two machines fighting over a
        // process-wide port table would be a confusing way to find that out.
        state.machine = None;
        state.scanout = None;
        state.console = None;
        state.shown = u64::MAX;
        state.audio = None;
        #[cfg(feature = "dev-nes-ppu")]
        crate::host::display::nes::capture::clear();
        #[cfg(feature = "dev-nes-apu")]
        crate::host::audio::nes::capture::clear();

        let registry = match crate::machine::catalog::registry() {
            Ok(r) => r,
            Err(e) => return fail(state, e),
        };
        let mut options = match crate::machine::catalog::build_options() {
            Ok(o) => o,
            Err(e) => return fail(state, e),
        };

        let image: Vec<u8> = state.input.get(..image_len).unwrap_or(&[]).to_vec();
        if image_len > 0 {
            let Some(slot) = entry.media.first() else {
                return fail(state, "this machine takes no media");
            };
            options.realize.media.insert(*slot, image.as_slice());
        } else if entry.media.contains(&"rom") {
            // The same courtesy the CLI extends: a machine that wants a `rom`
            // and was given none gets rsemu's own monitor, so the Apple 1 boots
            // with nothing uploaded and no ROM of unclear provenance.
            #[cfg(feature = "dev-apple1")]
            options
                .realize
                .media
                .insert("rom", &crate::dev::apple1::RSMON[..]);
        }

        #[cfg(feature = "dev-nes-ppu")]
        if let Err(e) = crate::host::display::nes::capture::install(&mut options) {
            return fail(state, e);
        }

        // The APU's output ring has to survive one `rsemu_run_frame`, which is
        // as long as anything here ever goes without draining it. Sizing it is
        // the *only* thing this interception changes about the machine, and it
        // is not guest-visible — see `host::audio::nes::capture`.
        #[cfg(feature = "dev-nes-apu")]
        if let Err(e) = crate::host::audio::nes::capture::install(
            &mut options,
            crate::dev::apu::DEFAULT_SAMPLE_BUFFER,
        ) {
            return fail(state, e);
        }

        let machine = match crate::machine::build(entry.name, entry.source, &registry, &options) {
            Ok(m) => m,
            Err(e) => return fail(state, e),
        };

        #[cfg(feature = "dev-nes-ppu")]
        if let Some(scanout) = crate::host::display::nes::capture::take() {
            state.frame = crate::host::display::Surface::for_scanout(&scanout);
            state.scanout = Some(alloc::boxed::Box::new(scanout));
        }

        // Sound goes out as interleaved `f32` in [-1, 1], which is what
        // WebAudio's `AudioBuffer` holds, so the page copies rather than
        // converts.
        #[cfg(feature = "dev-nes-apu")]
        if let Some(source) = crate::host::audio::nes::capture::take() {
            state.audio = Some(crate::host::audio::AudioStream::new(
                alloc::boxed::Box::new(source),
                state.audio_rate,
                crate::host::audio::SampleFormat::F32,
            ));
        }

        // Whatever character port the machine's devices opened is the console.
        // One is unambiguous; a machine with several is not this ABI's problem
        // until one exists.
        let names = crate::host::chardev::ports::names();
        if let Some(port) = names
            .first()
            .and_then(|n| crate::host::chardev::ports::get(n))
        {
            // Anything a previous machine left queued is not this machine's.
            let _discarded = port.drain();
            state.console = Some(port);
        }

        // And whatever pad port its controllers read is where buttons go. The
        // same name-based seam as the console (`dev::nes::input::pads`), for
        // the same reason: a machine file can hand a device a name and nothing
        // else.
        #[cfg(feature = "dev-nes-io")]
        {
            let pads = crate::dev::nes::input::pads::names();
            state.pad = pads
                .first()
                .and_then(|n| crate::dev::nes::input::pads::get(n));
            if let Some(pad) = state.pad.as_ref() {
                // Nothing is held at power-on, whatever the last machine left.
                pad.set(0, 0);
                pad.set(1, 0);
            }
        }
        state.buttons = [0; 2];

        state.machine = Some(machine);
        state.error.clear();
        1
    })
}

/// Drop the running machine, releasing everything it holds.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_shutdown() {
    with_state(|state| {
        state.machine = None;
        state.scanout = None;
        state.console = None;
        state.audio = None;
        #[cfg(feature = "dev-nes-io")]
        {
            state.pad = None;
        }
        state.shown = u64::MAX;
    });
}

/// Whether a machine is loaded.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_is_running() -> u32 {
    with_state(|state| u32::from(state.machine.is_some()))
}

/// Reset the machine as the console's reset button would. `0` if none is
/// loaded.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_reset() -> u32 {
    with_state(|state| {
        let Some(machine) = state.machine.as_mut() else {
            return fail(state, "no machine is loaded");
        };
        machine.reset(crate::core::device::ResetKind::Warm);
        // Whatever was queued belongs to the run that just ended. Playing it
        // after a reset would be a second or two of the previous game.
        if let Some(audio) = state.audio.as_mut() {
            let queued = audio.buffer().frames();
            audio.consume(queued);
        }
        1
    })
}

// ---------------------------------------------------------------------------
// Running
// ---------------------------------------------------------------------------

/// How long a frame is when the machine has no display to ask: 60 Hz, which is
/// how often a page will call anyway.
const DEFAULT_FRAME_NS: u64 = 16_666_667;

/// Advance the machine by exactly one video frame of **virtual** time and
/// capture the picture.
///
/// Returns `1` if the frame buffer now holds a picture the embedder has not
/// seen, `0` if it does not (no machine, no display, or a frame that had not
/// finished — a page redraws only when this says so).
///
/// One frame per call is what keeps the non-threaded browser configuration
/// honest: the page stays responsive because the module returns, not because
/// anything yields.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_run_frame() -> u32 {
    with_state(|state| {
        let period = state
            .scanout
            .as_ref()
            .map(|s| s.frame_period_ns())
            .filter(|ns| *ns > 0)
            .unwrap_or(DEFAULT_FRAME_NS);

        let Some(machine) = state.machine.as_mut() else {
            return fail(state, "no machine is loaded");
        };
        if let Err(e) = machine.run_for(crate::core::clock::GlobalTime::from_nanos(period)) {
            return fail(state, e);
        }

        // Immediately after the machine ran and before anything else: the
        // device's ring is sized for exactly one of these, and this is the
        // cadence the page would run at whether or not it were listening — the
        // audio path must never be what decides how far the machine advances.
        if let Some(audio) = state.audio.as_mut() {
            audio.pull();
        }

        let Some(scanout) = state.scanout.as_ref() else {
            return 0;
        };
        let serial = scanout.capture(&mut state.frame);
        if serial == state.shown {
            return 0;
        }
        state.shown = serial;
        1
    })
}

/// Advance by `frames` video frames, returning how many produced a new
/// picture. For fast-forwarding, and for a page catching up after a stall.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_run_frames(frames: u32) -> u32 {
    let mut drawn = 0;
    for _ in 0..frames {
        drawn += rsemu_run_frame();
    }
    drawn
}

/// How much virtual time one [`rsemu_run_frame`] advances, in nanoseconds.
///
/// A page paces itself with this rather than assuming 60 Hz: an NTSC NES frame
/// is 16 639 356 ns and a PAL one is not, and both are exact ratios of the
/// machine's own crystal rather than round numbers.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_frame_period_ns() -> u64 {
    with_state(|state| {
        state
            .scanout
            .as_ref()
            .map(|s| s.frame_period_ns())
            .filter(|ns| *ns > 0)
            .unwrap_or(DEFAULT_FRAME_NS)
    })
}

/// Virtual nanoseconds the machine has run for.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_now_ns() -> u64 {
    with_state(|state| state.machine.as_ref().map_or(0, |m| m.now().as_nanos()))
}

/// The machine's state hash — the same number `rsemu run` prints, and the
/// thing that makes a browser session comparable with a native one
/// (`ROADMAP.md` §11.6). `0` if there is no machine or it cannot be hashed.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_state_hash() -> u64 {
    with_state(|state| {
        state
            .machine
            .as_ref()
            .and_then(|m| m.state_hash().ok())
            .unwrap_or(0)
    })
}

// ---------------------------------------------------------------------------
// The picture
// ---------------------------------------------------------------------------

/// Whether this machine produces a picture at all.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_has_video() -> u32 {
    with_state(|state| u32::from(state.scanout.is_some()))
}

/// The frame buffer's address: `width × height` pixels, four bytes each, in
/// `R`, `G`, `B`, `A` order — exactly what `ImageData` wants.
///
/// Stable while the geometry is, so a page may keep one view over it; call it
/// again after [`rsemu_boot`].
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_frame_ptr() -> *const u8 {
    with_state(|state| state.frame.as_ptr())
}

/// The frame buffer's length in bytes.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_frame_len() -> usize {
    with_state(|state| state.frame.len() as usize)
}

/// The picture's width in pixels.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_frame_width() -> u32 {
    with_state(|state| state.frame.width())
}

/// The picture's height in pixels.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_frame_height() -> u32 {
    with_state(|state| state.frame.height())
}

/// Which frame the buffer holds, as counted by the display device since reset.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_frame_serial() -> u64 {
    with_state(|state| state.frame.serial())
}

// ---------------------------------------------------------------------------
// The sound
// ---------------------------------------------------------------------------

/// The rate assumed until the page says otherwise.
///
/// 48 000 is what an `AudioContext` reports on most desktops; 44 100 is common
/// enough that a page really should call [`rsemu_audio_set_rate`] with its
/// context's own `sampleRate` rather than trusting this.
const DEFAULT_AUDIO_RATE: u32 = 48_000;

/// Whether this machine makes a sound at all.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_has_audio() -> u32 {
    with_state(|state| u32::from(state.audio.is_some()))
}

/// Tell rsemu what rate the page's `AudioContext` runs at, in hertz.
///
/// Returns `1`, or `0` for a rate outside 8 000–384 000. Anything already
/// queued is discarded: it was converted for the old rate and playing it at the
/// new one would be a chirp.
///
/// The rate is remembered across [`rsemu_boot`], because it is a property of
/// the browser rather than of the machine.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_audio_set_rate(hz: u32) -> u32 {
    with_state(|state| {
        if !(8_000..=384_000).contains(&hz) {
            return fail(state, "an audio rate outside 8000..=384000 Hz");
        }
        state.audio_rate = hz;
        if let Some(audio) = state.audio.as_mut() {
            audio.set_rate(hz);
        }
        1
    })
}

/// The rate the queued frames are at, in hertz.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_audio_rate() -> u32 {
    with_state(|state| state.audio_rate)
}

/// How many channels one queued frame holds. `1` is mono, which every machine
/// in this build is.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_audio_channels() -> u32 {
    with_state(|state| {
        state
            .audio
            .as_ref()
            .map_or(0, |a| u32::from(a.buffer().channels()))
    })
}

/// How many frames are waiting to be played.
///
/// A *frame* is one sample per channel. [`rsemu_run_frame`] appends to this
/// queue, so a page reads it after every advance.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_audio_frames() -> usize {
    with_state(|state| {
        state
            .audio
            .as_ref()
            .map_or(0, |a| a.buffer().frames() as usize)
    })
}

/// The address of the queued frames: `rsemu_audio_frames() ×
/// rsemu_audio_channels()` little-endian `f32` samples in `[-1.0, 1.0]`,
/// interleaved — exactly what a WebAudio `AudioBuffer` holds.
///
/// **Read it immediately before copying**, and again after every call: unlike
/// the frame buffer, this queue grows, and a wasm memory that grows detaches
/// every view JavaScript is holding.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_audio_ptr() -> *const u8 {
    with_state(|state| {
        state
            .audio
            .as_ref()
            .map_or(core::ptr::null(), |a| a.buffer().as_ptr())
    })
}

/// Drop the oldest `frames` frames, returning how many were actually dropped.
///
/// The page calls this once it has copied them into an `AudioBuffer`. Nothing
/// drops them on its own: a queue that emptied itself between the read and the
/// copy would hand the page a view over freed bytes.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_audio_consume(frames: usize) -> usize {
    with_state(|state| {
        state
            .audio
            .as_mut()
            .map_or(0, |a| a.consume(frames as u64) as usize)
    })
}

/// How many frames have been lost because nobody kept up — at the device's ring
/// or at this queue.
///
/// A **diagnostic**: audio the host never collected is not machine state, no
/// guest can observe it, and it does not enter [`rsemu_state_hash`]. A page can
/// show it to explain a crackle.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_audio_dropped() -> u64 {
    with_state(|state| state.audio.as_ref().map_or(0, |a| a.dropped()))
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

/// Set the buttons held on controller `port` (0 or 1).
///
/// The mask is the shift register's own output order — `0x80` A, `0x40` B,
/// `0x20` Select, `0x10` Start, `0x08` Up, `0x04` Down, `0x02` Left, `0x01`
/// Right — because that is the order the hardware shifts out and therefore the
/// order the host seam speaks
/// ([NESdev, "Standard controller"](https://www.nesdev.org/wiki/Standard_controller),
/// and [`dev::nes::input::buttons`](crate::dev::nes::input::buttons), which is
/// where the constants live).
///
/// The state is a **level, not an event**: the console samples it whenever the
/// guest strobes `$4016`, so a button stays held until the embedder clears it.
/// That is also what makes the seam replayable.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_set_buttons(port: u32, mask: u32) {
    with_state(|state| {
        if let Some(slot) = state.buttons.get_mut(port as usize) {
            *slot = mask & 0xff;
        }
        #[cfg(feature = "dev-nes-io")]
        if let Some(pad) = state.pad.as_ref() {
            pad.set(port as usize, (mask & 0xff) as u8);
        }
    });
}

/// What [`rsemu_set_buttons`] last recorded for `port`, so a page can show it.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_buttons(port: u32) -> u32 {
    with_state(|state| state.buttons.get(port as usize).copied().unwrap_or(0))
}

/// Give the machine's console the first `len` bytes of the input buffer, as if
/// they had been typed. Returns how many were accepted — a full queue takes
/// fewer, which is the back pressure a real terminal has.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_console_write(len: usize) -> usize {
    with_state(|state| {
        let Some(port) = state.console.clone() else {
            return 0;
        };
        let bytes = state.input.get(..len).unwrap_or(&[]);
        port.feed(bytes)
    })
}

/// Copy everything the machine's console has produced into the output buffer,
/// returning its length. `0` means it said nothing since the last call.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_console_read() -> usize {
    with_state(|state| {
        state.output.clear();
        if let Some(port) = state.console.clone() {
            let bytes = port.drain();
            state.output.extend_from_slice(&bytes);
        }
        state.output.len()
    })
}

/// Whether this machine has a console to type at.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_has_console() -> u32 {
    with_state(|state| u32::from(state.console.is_some()))
}

// ---------------------------------------------------------------------------
// Save states
// ---------------------------------------------------------------------------

/// Snapshot the machine into the output buffer, returning its length. `0` with
/// [`rsemu_error`] if there is nothing to save or the save failed.
///
/// This is §11.7's "take a save state, all client-side with nothing uploaded":
/// the bytes never leave the page unless the page writes them somewhere.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_save() -> usize {
    with_state(|state| {
        state.output.clear();
        let Some(machine) = state.machine.as_ref() else {
            fail(state, "no machine is loaded");
            return 0;
        };
        match machine.save() {
            Ok(bytes) => {
                state.output = bytes;
                state.output.len()
            }
            Err(e) => {
                fail(state, e);
                0
            }
        }
    })
}

/// Restore the machine from the first `len` bytes of the input buffer.
/// Returns `1`, or `0` with [`rsemu_error`].
///
/// The snapshot must come from an identically configured machine — the same
/// description and the same cartridge — which is exactly the rule a native
/// save state follows.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_load(len: usize) -> u32 {
    with_state(|state| {
        let bytes: Vec<u8> = state.input.get(..len).unwrap_or(&[]).to_vec();
        let Some(machine) = state.machine.as_mut() else {
            return fail(state, "no machine is loaded");
        };
        match machine.load(&bytes) {
            Ok(()) => {
                state.shown = u64::MAX;
                // Same argument as `rsemu_reset`: the queue holds audio from
                // before the restore.
                if let Some(audio) = state.audio.as_mut() {
                    let queued = audio.buffer().frames();
                    audio.consume(queued);
                }
                1
            }
            Err(e) => fail(state, e),
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn echo_round_trips() {
        assert_eq!(rsemu_echo(0xdead_beef), 0xdead_beef);
    }

    #[test]
    fn version_pointer_and_length_describe_the_same_string() {
        let len = rsemu_version_len();
        assert!(len > 0);
        // Calling twice must return the identical cached allocation, or the
        // pointer handed to the host could dangle across calls.
        assert_eq!(rsemu_version_ptr(), rsemu_version_ptr());
        assert_eq!(rsemu_version_len(), len);
    }

    /// The catalog is readable through the ABI without any machine loaded, and
    /// every index it reports has a name.
    #[test]
    fn the_catalog_is_readable_by_index() {
        let _serialised = crate::host::display::PROCESS_WIDE.lock();
        let count = rsemu_machine_count();
        for index in 0..count {
            assert!(rsemu_machine_name(index) > 0, "machine {index} has no name");
            assert!(rsemu_machine_summary(index) > 0);
        }
        // One past the end is an empty answer, not a panic.
        assert_eq!(rsemu_machine_name(count), 0);
    }

    /// Every call that needs a machine says so rather than misbehaving.
    #[test]
    fn calls_without_a_machine_report_rather_than_panic() {
        let _serialised = crate::host::display::PROCESS_WIDE.lock();
        rsemu_shutdown();
        assert_eq!(rsemu_is_running(), 0);
        assert_eq!(rsemu_run_frame(), 0);
        assert_eq!(rsemu_reset(), 0);
        assert_eq!(rsemu_save(), 0);
        assert_eq!(rsemu_state_hash(), 0);
        assert!(rsemu_error() > 0, "a failure must leave a message");
    }

    /// The input buffer is rsemu's, and reserving it is what makes the address
    /// meaningful. Nothing here dereferences a caller's pointer.
    #[test]
    fn the_input_buffer_is_resizable() {
        let _serialised = crate::host::display::PROCESS_WIDE.lock();
        let a = rsemu_input_reserve(16);
        assert!(!a.is_null());
        let b = rsemu_input_reserve(1 << 16);
        assert!(!b.is_null());
        // Writing zero bytes into it is legal and consumes nothing.
        assert_eq!(rsemu_console_write(0), 0);
    }

    /// A whole machine through the ABI: boot, run frames, get a picture, take
    /// a save state and put it back.
    #[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
    #[test]
    fn a_nes_boots_and_draws_through_the_abi() {
        // The module's state is one slot for the whole process, and so is the
        // scanout capture table underneath it; see `display::PROCESS_WIDE`.
        let _serialised = crate::host::display::PROCESS_WIDE.lock();

        /// The same minimal NROM the display tests use: `JMP $C000` forever.
        static MINIMAL_NROM: &[u8] = &{
            let mut image = [0u8; 16 + 16384 + 8192];
            image[0] = b'N';
            image[1] = b'E';
            image[2] = b'S';
            image[3] = 0x1a;
            image[4] = 1;
            image[5] = 1;
            image[16 + 0x3ffc] = 0x00;
            image[16 + 0x3ffd] = 0xc0;
            image[16] = 0x4c;
            image[17] = 0x00;
            image[18] = 0xc0;
            image
        };

        let index = (0..rsemu_machine_count())
            .find(|i| {
                rsemu_machine_name(*i);
                with_state(|state| state.output == b"nes-ntsc")
            })
            .expect("machine-nes is on, so the catalog has one");

        // The embedder's half of the ABI: reserve, write, boot.
        rsemu_input_reserve(MINIMAL_NROM.len());
        with_state(|state| state.input.copy_from_slice(MINIMAL_NROM));
        if rsemu_boot(index, MINIMAL_NROM.len()) != 1 {
            let len = rsemu_error();
            let message = with_state(|state| String::from_utf8_lossy(&state.output).into_owned());
            panic!("boot failed ({len} bytes): {message}");
        }
        assert_eq!(rsemu_is_running(), 1);
        assert_eq!(rsemu_has_video(), 1);
        assert_eq!(rsemu_frame_width(), 256);
        assert_eq!(rsemu_frame_height(), 240);
        assert_eq!(rsemu_frame_len(), 256 * 240 * 4);

        let mut drawn = 0;
        for _ in 0..4 {
            drawn += rsemu_run_frame();
        }
        assert!(drawn > 0, "four frames produced no picture");
        assert!(rsemu_now_ns() > 0);
        let hash = rsemu_state_hash();
        assert_ne!(hash, 0);

        // A save state, and the machine moving on from it.
        let len = rsemu_save();
        assert!(len > 0);
        let snapshot = with_state(|state| state.output.clone());
        rsemu_run_frames(2);
        assert_ne!(rsemu_state_hash(), hash, "time did not pass");

        // Buttons reach the console's controller port, not just the module's
        // own record of them: the guest strobes $4016 and reads them back.
        #[cfg(feature = "dev-nes-io")]
        {
            use crate::dev::nes::input::{buttons, pads};

            rsemu_set_buttons(0, u32::from(buttons::A | buttons::START));
            let pad = pads::names()
                .first()
                .and_then(|n| pads::get(n))
                .expect("the machine opened a pad port");
            assert_eq!(pad.get(0), buttons::A | buttons::START);
            rsemu_set_buttons(0, 0);
            assert_eq!(pad.get(0), buttons::NONE);
        }

        rsemu_input_reserve(snapshot.len());
        with_state(|state| state.input.copy_from_slice(&snapshot));
        assert_eq!(rsemu_load(snapshot.len()), 1, "load failed");
        assert_eq!(rsemu_state_hash(), hash, "the snapshot did not restore");

        rsemu_shutdown();
    }

    /// Sound through the ABI: a machine that boots produces frames, the page
    /// drains them, and doing so leaves the machine bit-identical.
    ///
    /// The hash comparison is the load-bearing half. It is run against a
    /// machine driven exactly as [`rsemu_run_frames`] drives it, because that
    /// is the cadence a page uses; if pulling audio could ever move
    /// architectural state, this is where it would show.
    #[cfg(all(feature = "machine-nes", feature = "dev-nes-apu"))]
    #[test]
    fn a_nes_plays_through_the_abi() {
        let _serialised = crate::host::display::PROCESS_WIDE.lock();

        /// `LDA #$0F / STA $4015 / LDA #$9F / STA $4000 / LDA #$08 / STA $4003`
        /// and then a tight loop: both pulse channels at full volume, so there
        /// is something to hear.
        static NOISY_NROM: &[u8] = &{
            let mut image = [0u8; 16 + 16384 + 8192];
            image[0] = b'N';
            image[1] = b'E';
            image[2] = b'S';
            image[3] = 0x1a;
            image[4] = 1;
            image[5] = 1;
            image[16 + 0x3ffc] = 0x00;
            image[16 + 0x3ffd] = 0xc0;
            let program: [u8; 18] = [
                0xa9, 0x0f, 0x8d, 0x15, 0x40, 0xa9, 0x9f, 0x8d, 0x00, 0x40, 0xa9, 0x08, 0x8d, 0x03,
                0x40, 0x4c, 0x0f, 0xc0,
            ];
            let mut i = 0;
            while i < program.len() {
                image[16 + i] = program[i];
                i += 1;
            }
            image
        };

        fn boot_noisy() -> u32 {
            let index = (0..rsemu_machine_count())
                .find(|i| {
                    rsemu_machine_name(*i);
                    with_state(|state| state.output == b"nes-ntsc")
                })
                .expect("machine-nes is on, so the catalog has one");
            rsemu_input_reserve(NOISY_NROM.len());
            with_state(|state| state.input.copy_from_slice(NOISY_NROM));
            assert_eq!(rsemu_boot(index, NOISY_NROM.len()), 1, "boot failed");
            index
        }

        boot_noisy();
        assert_eq!(rsemu_has_audio(), 1);
        assert_eq!(rsemu_audio_channels(), 1);
        assert_eq!(rsemu_audio_rate(), 48_000);

        // A page announces its own context's rate; anything absurd is refused
        // rather than silently accepted.
        assert_eq!(rsemu_audio_set_rate(44_100), 1);
        assert_eq!(rsemu_audio_rate(), 44_100);
        assert_eq!(rsemu_audio_set_rate(3), 0);
        assert_eq!(rsemu_audio_rate(), 44_100);

        // Thirty frames is half a second, so about 22 050 frames of audio.
        rsemu_run_frames(30);
        let queued = rsemu_audio_frames();
        assert!(queued > 20_000, "half a second gave {queued} frames");
        assert!(!rsemu_audio_ptr().is_null());
        assert_eq!(rsemu_audio_dropped(), 0, "the ring is sized for one frame");

        // The page copies and then says so; nothing drops on its own.
        assert_eq!(rsemu_audio_consume(1000), 1000);
        assert_eq!(rsemu_audio_frames(), queued - 1000);
        assert_eq!(rsemu_audio_consume(usize::MAX), queued - 1000);
        assert_eq!(rsemu_audio_frames(), 0);

        let listened = rsemu_state_hash();
        assert_ne!(listened, 0);

        // The same run again, with nobody reading the queue at all.
        boot_noisy();
        rsemu_run_frames(30);
        assert_eq!(
            rsemu_state_hash(),
            listened,
            "the state hash depends on whether the page was listening"
        );

        rsemu_shutdown();
        assert_eq!(rsemu_has_audio(), 0);
        assert_eq!(rsemu_audio_frames(), 0);
        assert!(rsemu_audio_ptr().is_null());
    }
}
