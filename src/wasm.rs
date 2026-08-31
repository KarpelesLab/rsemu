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
//! # What runs where
//!
//! The non-threaded configuration is a supported target, not a fallback
//! (`ROADMAP.md` §11.3): [`rsemu_run_frame`] advances virtual time by exactly
//! one video frame and returns, so a page can drive it from
//! `requestAnimationFrame` and stay responsive without `SharedArrayBuffer`,
//! `Atomics.wait`, or a worker. Nothing here reads a host clock — the frame
//! period is computed from the machine's own oscillator forest — so a session
//! in a browser and the same session under a native debugger produce the same
//! state hash (§11.6).
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

use crate::core::sync::{LockRank, Mutex};

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
            input: Vec::new(),
            output: Vec::new(),
            error: String::new(),
            buttons: [0; 2],
            shown: u64::MAX,
        }
    }
}

static STATE: Mutex<State> = Mutex::with_rank(LockRank::MACHINE, State::new());

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
        #[cfg(feature = "dev-nes-ppu")]
        crate::host::display::nes::capture::clear();

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

        let machine = match crate::machine::build(entry.name, entry.source, &registry, &options) {
            Ok(m) => m,
            Err(e) => return fail(state, e),
        };

        #[cfg(feature = "dev-nes-ppu")]
        if let Some(scanout) = crate::host::display::nes::capture::take() {
            state.frame = crate::host::display::Surface::for_scanout(&scanout);
            state.scanout = Some(alloc::boxed::Box::new(scanout));
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
// Input
// ---------------------------------------------------------------------------

/// Set the buttons held on controller `port` (0 or 1).
///
/// The mask is the NES's own bit order as `$4016` shifts it out: bit 0 A,
/// 1 B, 2 Select, 3 Start, 4 Up, 5 Down, 6 Left, 7 Right
/// ([NESdev, "Standard controller"](https://www.nesdev.org/wiki/Standard_controller)).
///
/// **The state is recorded and not yet delivered**, because the controller
/// ports do not exist as a device yet — `machines/nes-ntsc.machine` names that
/// gap in its own TODO, `$4016`/`$4017` read open bus, and software sees no
/// buttons held. Keeping the export honest and wired from the page means the
/// day the device lands, one line here changes and the demo has input; the
/// alternative was a page that silently does not send keystrokes anywhere.
/// The Apple 1's keyboard is a different path and works today — see
/// [`rsemu_console_write`].
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_set_buttons(port: u32, mask: u32) {
    with_state(|state| {
        if let Some(slot) = state.buttons.get_mut(port as usize) {
            *slot = mask;
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

        rsemu_input_reserve(snapshot.len());
        with_state(|state| state.input.copy_from_slice(&snapshot));
        assert_eq!(rsemu_load(snapshot.len()), 1, "load failed");
        assert_eq!(rsemu_state_hash(), hash, "the snapshot did not restore");

        rsemu_shutdown();
    }
}
