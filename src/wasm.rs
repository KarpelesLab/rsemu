//! WebAssembly entry points.
//!
//! Plain C-ABI exports and embedder-supplied imports, following purecrypto's
//! browser convention — deliberately **no** `wasm-bindgen`: the dependency
//! policy forbids it (`ROADMAP.md` §0), and the boundary is small enough that
//! it does not need a binding generator.
//!
//! The host supplies the JS glue: it instantiates the module, reads exported
//! memory directly, and provides the imports rsemu needs. Three of those exist
//! today and they are the **WebAssembly JIT's** — `rsemu.jit_compile`,
//! `jit_enter`, `jit_release`, at the bottom of this file, with `web/src/jit.js`
//! as the reference implementation; a clock and entropy will arrive the same
//! way. See `ROADMAP.md` §11 and `docs/techniques/wasm-jit.md`, and `web/` for
//! the page that drives everything below.
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
//!    ever crosses *into* the module. **Built-in images are named by index
//!    too**, per machine ([`rsemu_machine_builtin_count`],
//!    [`rsemu_machine_builtin_name`]), which is how the browser gets what
//!    `rsemu run beneater-6502 --monitor wozmon` gets on a command line the
//!    page does not have. **Media slots are named by index too**
//!    ([`rsemu_machine_media_count`], [`rsemu_machine_media_name`]), which is
//!    what lets [`rsemu_stage_media`] fill a second bay — a diskette in a PC
//!    that is already booting rsemu's own BIOS — without a slot name crossing
//!    in.
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
//! # Two ways in for a person's hands
//!
//! [`rsemu_console_write`] delivers **characters** to whatever character port
//! the machine opened; [`rsemu_key`] delivers **key transitions** to the AT
//! keyboard `pc.kbc` opens, which carries set-2 scan codes rather than text.
//! No machine in this build has both — an Apple 1's port is characters, a
//! PC/AT's is a keyboard — but nothing forbids one, and a PC with a serial
//! console would. So they are two questions ([`rsemu_has_console`],
//! [`rsemu_has_keyboard`]) and a page should ask both rather than inferring one
//! from the other: a terminal pane in front of a keyboard would show an empty
//! screen and send `0x41` for `A` meaning the `9` key.
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
//! This is the **C ABI boundary**, one of the seven subsystems `ROADMAP.md` §0
//! sanctions to opt back in. Three things here need it: `#[unsafe(no_mangle)]`,
//! which edition 2024 classifies as an unsafe attribute because duplicate
//! exported symbols are the linker's problem rather than the compiler's; the
//! private `leaked` helper, which rebuilds a `&'static str` from a
//! pointer/length pair; and the **wasm JIT's activation** at the bottom of the
//! file, which turns a token a generated module handed back into the
//! `&mut dyn Env` and `&mut [u8]` it names. `Activation`'s doc comment is that
//! third one's whole argument, and it is the review CLAUDE.md asks for: no
//! eighth site, because turning a foreign embedder's `i32` back into something
//! typed is what this site has always been. The allow is module-scoped rather
//! than crate-wide, and every genuine `unsafe` block below carries its own
//! `// SAFETY:` argument.
#![allow(unsafe_code)]

use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::sync::{Global, LockRank};

/// Whichever console's controllers a machine has, behind one host mask.
///
/// `host::input`'s, not this module's: [`rsemu_set_buttons`] and a VNC client
/// press the same eight named bits, and translating them onto a console's pins
/// is one job with one answer. It lived here first, which is why the doc
/// comment about the Game Boy's reversed matrix and the Master System's Pause
/// switch is over there now — with the code that does it.
#[cfg(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms"))]
use crate::host::input::Pads;

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
    /// The AT keyboard `pc.kbc` opened, if this machine has one.
    ///
    /// Not the console and never confused with it: the far end of this port
    /// carries set-2 scan codes, so what crosses it is a *key transition*
    /// rather than a character. [`rsemu_key`] is how a page presses one.
    keyboard: Option<crate::host::input::KeyboardSink>,
    /// The host objects the live machine was built against: its character
    /// ports, its pads, and the capture tables the interceptions filled in.
    ///
    /// Kept because the module hands them out after the build — the console and
    /// the pad below are found by name in here — and because dropping it with
    /// the machine is what makes a second `rsemu_boot` a genuinely fresh set of
    /// ports rather than the last machine's.
    hosts: Option<alloc::sync::Arc<crate::core::hosts::HostObjects>>,
    /// The pad port the machine's controllers read, if it has any.
    #[cfg(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms"))]
    pad: Option<Pads>,
    /// The sound, resampled to whatever the page's `AudioContext` runs at, if
    /// this machine makes any.
    audio: Option<crate::host::audio::AudioStream>,
    /// The rate the page last asked for. Kept across boots, because an
    /// `AudioContext`'s rate is a property of the browser rather than of the
    /// machine and the page should not have to say it twice.
    audio_rate: u32,
    /// Bytes JavaScript hands in: ROM images, typed characters, save states.
    input: Vec<u8>,
    /// Media slots staged for the next boot, in the order they were staged.
    ///
    /// The uploaded image [`rsemu_boot`] binds goes into one slot, and a board
    /// with a built-in firmware has no other way to be handed a *disk*. This is
    /// the second bay: [`rsemu_stage_media`] fills it by slot index, and the
    /// next boot binds every entry alongside whatever the boot itself chose.
    /// Slot names are resolved when a slot is staged, so nothing here is an
    /// index into a machine that is no longer the one being booted.
    staged: Vec<(&'static str, Vec<u8>)>,
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
            keyboard: None,
            hosts: None,
            #[cfg(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms"))]
            pad: None,
            audio: None,
            audio_rate: DEFAULT_AUDIO_RATE,
            input: Vec::new(),
            staged: Vec::new(),
            output: Vec::new(),
            error: String::new(),
            buttons: [0; 2],
            shown: u64::MAX,
        }
    }

    /// Take `scanout` as this machine's display, and shape the frame buffer for
    /// it.
    ///
    /// **`RGBA8888` whatever the adapter would prefer.** `Surface::for_scanout`
    /// asks the device family — the NES's PPU says RGBA, an RGB panel says
    /// RGB888 to avoid a padding byte — but this buffer's format is part of the
    /// ABI ([`rsemu_frame_ptr`]: four bytes a pixel, which is what `ImageData`
    /// holds), so it is fixed here and every adapter converts on capture.
    #[cfg(any(
        feature = "dev-nes-ppu",
        feature = "dev-lcdc",
        feature = "dev-gb",
        feature = "dev-pc-video",
        feature = "dev-sms"
    ))]
    fn attach_scanout(&mut self, scanout: alloc::boxed::Box<dyn crate::host::display::Scanout>) {
        let info = scanout.info();
        self.frame = crate::host::display::Surface::new(
            crate::host::display::PixelFormat::RGBA8888,
            info.width,
            info.height,
        );
        self.scanout = Some(scanout);
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

/// How many media slots machine `index` declares.
///
/// [`rsemu_machine_media`] answers the *first* of them, which is the one a page
/// fills to boot a cartridge. This is the whole list, because a PC has five —
/// `bios`, `vgabios`, `floppy`, `hd0`, `hd1` — and a page that can only reach
/// the first can hand it a firmware or a disk but never both.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_media_count(index: u32) -> u32 {
    catalog_entry(index).map_or(0, |e| e.media.len() as u32)
}

/// Copy the name of machine `index`'s media slot `slot` into the output buffer,
/// returning its length. `0` for a slot this machine does not have.
///
/// The same names `rsemu run … --media floppy=disk.img` takes, so a browser
/// session and a command line are describable in the same words.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_media_name(index: u32, slot: u32) -> usize {
    with_state(|state| {
        state.output.clear();
        if let Some(name) = catalog_entry(index).and_then(|e| e.media.get(slot as usize)) {
            state.output.extend_from_slice(name.as_bytes());
        }
        state.output.len()
    })
}

/// Stage the first `len` bytes of the input buffer into machine `index`'s media
/// slot `slot`, for the next boot. Returns `1`, or `0` with [`rsemu_error`].
///
/// This is the second bay, and it is what [`rsemu_boot`] on its own cannot be:
/// that call binds one uploaded image to one slot, so a board whose firmware
/// this module *carries* had no way to also be handed a disk. Stage a floppy
/// here and then [`rsemu_boot_builtin`] the PC's BIOS, and the machine comes up
/// on rsemu's own firmware with the visitor's diskette in the drive.
///
/// Rule 2 of the ABI still holds — no string crosses in. The slot is an index
/// into [`rsemu_machine_media_count`], and its *name* is resolved here, so a
/// staged image belongs to a named slot from this moment rather than to a
/// position in whatever machine is booted later. A staged slot the machine
/// being booted does not have is refused at boot rather than ignored.
///
/// Staging survives a boot, so `Reboot` reboots the same media. Drop it with
/// [`rsemu_clear_media`], which is also what a page does when it changes
/// machines.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_stage_media(index: u32, slot: u32, len: usize) -> u32 {
    with_state(|state| {
        let Some(name) = catalog_entry(index)
            .and_then(|e| e.media.get(slot as usize))
            .copied()
        else {
            return fail(state, "no media slot with that index on that machine");
        };
        let bytes = state.input.get(..len).unwrap_or(&[]).to_vec();
        // Staging the same slot twice replaces it: a page that lets someone
        // change their mind about which floppy is in the drive should not have
        // to say so in a second call.
        state.staged.retain(|(s, _)| *s != name);
        state.staged.push((name, bytes));
        state.error.clear();
        1
    })
}

/// Forget everything [`rsemu_stage_media`] staged.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_clear_media() {
    with_state(|state| state.staged.clear());
}

/// How many built-in images machine `index` carries.
///
/// An image rsemu ships for that machine's own media slot: RSMON, the Woz
/// Monitor, a board's demonstration firmware. **A machine with at least one
/// boots with nothing uploaded**, which is the question a page really wants
/// answered — [`rsemu_machine_media`] says a slot exists, not that the visitor
/// has to fill it.
///
/// `0` for a machine whose image is the user's to supply, which is every
/// cartridge and every BIOS (`ROADMAP.md` §1).
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_builtin_count(index: u32) -> u32 {
    catalog_entry(index).map_or(0, |e| builtins(e).len() as u32)
}

/// Copy the name of machine `index`'s built-in image `builtin` into the output
/// buffer, returning its length.
///
/// The same names the CLI takes — `rsmon`, `wozmon` — so a browser session and
/// a `rsemu run` are describable in the same words.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_builtin_name(index: u32, builtin: u32) -> usize {
    with_state(|state| {
        state.output.clear();
        if let Some(image) = builtin_image(index, builtin) {
            state.output.extend_from_slice(image.name().as_bytes());
        }
        state.output.len()
    })
}

/// Copy the one-line description of machine `index`'s built-in image `builtin`
/// into the output buffer, returning its length.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_builtin_summary(index: u32, builtin: u32) -> usize {
    with_state(|state| {
        state.output.clear();
        if let Some(image) = builtin_image(index, builtin) {
            state.output.extend_from_slice(image.summary().as_bytes());
        }
        state.output.len()
    })
}

/// Copy the media slot machine `index`'s built-in image `builtin` fills into
/// the output buffer, returning its length.
///
/// `rom` for a monitor, `firmware` for a board's demonstration program. A page
/// shows it so that "boots with no upload" does not have to be taken on trust.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_builtin_slot(index: u32, builtin: u32) -> usize {
    with_state(|state| {
        state.output.clear();
        if let Some(image) = builtin_image(index, builtin) {
            state.output.extend_from_slice(image.slot().as_bytes());
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

/// One image this build can boot a machine on with nothing uploaded.
///
/// Two kinds, because rsemu ships two kinds. A monitor is a couple of hundred
/// bytes the catalog carries verbatim as a `&'static [u8]`. The legacy PC BIOS
/// is **assembled for the board it is about to run in** — its MP, ACPI and
/// SMBIOS tables describe *that* machine's processors and chips
/// ([`crate::fw::pcbios`]) — so it does not exist until someone asks for it and
/// cannot be a static slice.
///
/// The CLI (`builtin_bios` in `src/bin/rsemu.rs`) and the C ABI
/// (`builtin_media` in `crate::ffi`) each have the same fork for the same
/// reason. This is the browser's, and it is here rather than in
/// `machine::catalog` because a generated image has no `'static` bytes for
/// [`crate::machine::catalog::BuiltinImage`] to hold.
#[derive(Debug, Clone, Copy)]
enum Builtin {
    /// An image the catalog carries as bytes.
    Static(&'static crate::machine::catalog::BuiltinImage),
    /// rsemu's own legacy PC BIOS, assembled for this machine description.
    #[cfg(all(feature = "fw-pcbios", feature = "machine-pc-at"))]
    PcBios,
}

impl Builtin {
    /// How a host names it — the same names `rsemu run … --monitor` takes.
    fn name(self) -> &'static str {
        match self {
            Builtin::Static(image) => image.name,
            #[cfg(all(feature = "fw-pcbios", feature = "machine-pc-at"))]
            Builtin::PcBios => "rsemu-bios",
        }
    }

    /// One line about what it is, for a picker.
    fn summary(self) -> &'static str {
        match self {
            Builtin::Static(image) => image.summary,
            #[cfg(all(feature = "fw-pcbios", feature = "machine-pc-at"))]
            Builtin::PcBios => {
                "rsemu's own legacy PC BIOS: POST, the MP/ACPI/SMBIOS tables, and a boot attempt"
            }
        }
    }

    /// The media slot it fills.
    fn slot(self) -> &'static str {
        match self {
            Builtin::Static(image) => image.slot,
            #[cfg(all(feature = "fw-pcbios", feature = "machine-pc-at"))]
            Builtin::PcBios => "bios",
        }
    }

    /// The bytes, assembling them if this is the kind that is generated.
    ///
    /// `entry` is the description the firmware is being built *for*: a copy of
    /// `pc-at` with a second processor in it gets a table with two processors
    /// in it. A description this firmware cannot read falls back to the shipped
    /// board's, exactly as the CLI does — a table generator is never the reason
    /// a machine will not start.
    fn bytes(
        self,
        entry: &'static crate::machine::catalog::CatalogEntry,
    ) -> alloc::borrow::Cow<'static, [u8]> {
        let _ = entry;
        match self {
            Builtin::Static(image) => alloc::borrow::Cow::Borrowed(image.bytes),
            #[cfg(all(feature = "fw-pcbios", feature = "machine-pc-at"))]
            Builtin::PcBios => alloc::borrow::Cow::Owned(
                crate::fw::pcbios::image_for_machine(entry.name, entry.source)
                    .unwrap_or_else(|_| crate::fw::pcbios::image()),
            ),
        }
    }
}

/// The images this build can boot `entry` on, in the order a picker shows them.
///
/// A `Vec` rather than a slice because one of them is generated. This is a cold
/// path — a page asks once, at load — so the allocation is not worth designing
/// around.
// `mut` only in a build that has the one generated image, exactly as
// `machine::catalog::machines` is only `mut` in a build that has a machine.
#[allow(unused_mut)]
fn builtins(entry: &'static crate::machine::catalog::CatalogEntry) -> Vec<Builtin> {
    let mut out: Vec<Builtin> = crate::machine::catalog::builtins(entry.name)
        .iter()
        .map(Builtin::Static)
        .collect();
    // The one board whose firmware rsemu ships. `ROADMAP.md` phase 6a: every
    // other legacy PC BIOS anyone could reach for is GPL, and running one is
    // fine while shipping one is not.
    #[cfg(all(feature = "fw-pcbios", feature = "machine-pc-at"))]
    if entry.name == "pc-at" {
        out.push(Builtin::PcBios);
    }
    out
}

/// One built-in image, by machine index and image index.
fn builtin_image(index: u32, builtin: u32) -> Option<Builtin> {
    builtins(catalog_entry(index)?)
        .get(builtin as usize)
        .copied()
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/// The character port `pc.kbc` opens, which carries scan codes rather than
/// text and is therefore never this module's console.
const KEYBOARD_PORT: &str = "keyboard";

/// Media slots whose unbound state is "empty", not "missing".
///
/// A PC's second IDE bay, its diskette drive and its video option-ROM socket
/// are all ordinarily empty; a `riscv-virt`'s NOR banks are blank on a board
/// straight from the factory. `machine::realize` refuses an unbound slot, so
/// "empty" has to be said explicitly — with no bytes. The same list, and the
/// same argument, as `src/bin/rsemu.rs`'s.
///
/// `nvme0` is here for the reason `hd0` is: a controller with no bytes bound is
/// a drive whose capacity comes from the board's `disk` parameter and whose
/// contents are zeroes, which is a blank disk rather than a missing one. It
/// arrived on the CLI's list when `q35-linux` grew a namespace and was missed
/// here, so the two lists had quietly parted; `q35-uefi` grew one too, and a
/// `demo,machine-q35-uefi` module would have refused to assemble the board over
/// a bay it had been given no way to name.
///
/// `df0` is an Amiga's internal drive: empty is the insert-disk screen. `ext`
/// is the A500 board's extended-ROM window, which only AROS fills: no bytes is
/// no ROM, and the board is then exactly the machine without the window.
/// `cdrom` is a PC's CD-ROM drive, where empty is an open tray: the drive is
/// still on the cable and still answers, with `MEDIUM NOT PRESENT`.
const EMPTY_BAYS: &[&str] = &[
    "flash0", "flash1", "initrd", "disk", "hd0", "hd1", "floppy", "vgabios", "nvme0", "df0", "ext",
    "cdrom",
];

/// Where the media image a boot binds comes from.
///
/// Not part of the ABI — the two exported entry points below each name one, so
/// that "boot with an uploaded cartridge" and "boot with the Woz Monitor" share
/// every line after this choice.
#[derive(Debug, Clone, Copy)]
enum Media {
    /// The first `n` bytes of the input buffer, in the machine's first slot.
    Uploaded(usize),
    /// This machine's built-in image `n`, in whichever slot it belongs to.
    Builtin(u32),
    /// Whatever this machine boots with when nobody says: its first built-in
    /// image, or nothing at all.
    Default,
}

/// Build machine `index`, binding the first `image_len` bytes of the input
/// buffer to its media slot. Returns `1` on success, `0` with [`rsemu_error`].
///
/// An `image_len` of `0` binds this machine's **default built-in image** if it
/// has one — RSMON on an Apple 1, exactly as `rsemu run apple1` does — and
/// nothing if it has none. [`rsemu_boot_builtin`] picks a different one.
///
/// Any previous machine is dropped first, so booting twice is how the page
/// changes cartridges.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_boot(index: u32, image_len: usize) -> u32 {
    if image_len > 0 {
        boot_with(index, Media::Uploaded(image_len))
    } else {
        boot_with(index, Media::Default)
    }
}

/// Build machine `index` with its built-in image `builtin` bound, uploading
/// nothing. Returns `1` on success, `0` with [`rsemu_error`].
///
/// This is `--monitor wozmon` for a page that has no command line: the images
/// are compiled into the module, so a visitor is typing at a 1976 monitor one
/// click after the module loads. [`rsemu_machine_builtin_count`] says how many
/// a machine has and [`rsemu_machine_builtin_name`] what they are called.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_boot_builtin(index: u32, builtin: u32) -> u32 {
    boot_with(index, Media::Builtin(builtin))
}

/// The whole of a boot, whichever way the image was chosen.
fn boot_with(index: u32, media: Media) -> u32 {
    with_state(|state| {
        let Some(entry) = catalog_entry(index) else {
            return fail(state, "no machine with that index in this build");
        };

        // Drop the old machine and its host objects before building the new
        // one: the new build gets a table of its own, so nothing the last
        // machine opened can be mistaken for this one's.
        state.machine = None;
        state.scanout = None;
        state.console = None;
        state.keyboard = None;
        state.shown = u64::MAX;
        state.audio = None;
        state.hosts = None;

        // The image has to outlive the build whichever way it was chosen: the
        // uploaded one is copied out of the input buffer, the generated one is
        // assembled here, and a static one already lives forever. One `Cow`
        // covers all three, and it is bound **before** `options` so that it
        // outlives the map that borrows it.
        let binding: Option<(&'static str, alloc::borrow::Cow<'static, [u8]>)> = match media {
            Media::Uploaded(len) => {
                let Some(slot) = entry.media.first() else {
                    return fail(state, "this machine takes no media");
                };
                let bytes = state.input.get(..len).unwrap_or(&[]).to_vec();
                Some((*slot, alloc::borrow::Cow::Owned(bytes)))
            }
            Media::Builtin(which) => {
                let Some(image) = builtins(entry).get(which as usize).copied() else {
                    return fail(state, "this machine has no built-in image with that index");
                };
                Some((image.slot(), image.bytes(entry)))
            }
            // The same courtesy the CLI extends: a machine that ships an image
            // and was given none boots on it, so an Apple 1 comes up with
            // nothing uploaded and no ROM of unclear provenance.
            Media::Default => builtins(entry)
                .first()
                .copied()
                .map(|i| (i.slot(), i.bytes(entry))),
        };

        // The second bay and every one after it, taken out of the module state
        // so that a rejection below can still write an error message into it.
        // A staged slot this machine does not declare is a page bug rather than
        // something to bind silently, and one that collides with the image this
        // boot already chose is ambiguous — either would end as a diskette
        // quietly not in the drive, which is the failure this call exists to
        // remove.
        let staged = state.staged.clone();
        for (slot, _) in &staged {
            if !entry.media.contains(slot) {
                return fail(state, "a staged media slot is not on this machine");
            }
            if binding.as_ref().is_some_and(|(s, _)| s == slot) {
                return fail(state, "a staged media slot is the one this boot binds");
            }
        }

        let registry = match crate::machine::catalog::registry() {
            Ok(r) => r,
            Err(e) => return fail(state, e),
        };
        let mut options = match crate::machine::catalog::build_options() {
            Ok(o) => o,
            Err(e) => return fail(state, e),
        };
        if let Some((slot, bytes)) = &binding {
            options.realize.media.insert(*slot, bytes.as_ref());
        }
        for (slot, bytes) in &staged {
            options.realize.media.insert(*slot, bytes.as_slice());
        }
        // An unbound slot is an error by design (`machine::realize`), so the
        // empty bays have to be bound as empty. Same list and same argument as
        // the CLI's: a PC with no diskette, no option ROM in the video socket
        // and one empty IDE bay is an ordinary PC, and a board that refused to
        // assemble without a floppy would be describing no machine anyone
        // owned. Only slots this machine actually declares, and never the one
        // the image above went into.
        for slot in EMPTY_BAYS {
            if entry.media.contains(slot)
                && binding.as_ref().is_none_or(|(s, _)| s != slot)
                && !staged.iter().any(|(s, _)| s == slot)
            {
                options.realize.media.insert(*slot, &[][..]);
            }
        }

        // One arm per display family this build has, exactly like the
        // registration lists in `machine::catalog`: a family that is not named
        // here has no picture in a browser, and that is visible by reading the
        // code rather than by booting the machine and seeing black.
        #[cfg(feature = "dev-nes-ppu")]
        if let Err(e) = crate::host::display::nes::capture::install(&mut options) {
            return fail(state, e);
        }
        #[cfg(feature = "dev-pc-video")]
        if let Err(e) = crate::host::display::pc::capture::install(&mut options) {
            return fail(state, e);
        }
        #[cfg(feature = "dev-lcdc")]
        if let Err(e) = crate::host::display::lcd::capture::install(&mut options) {
            return fail(state, e);
        }
        #[cfg(feature = "dev-gb")]
        if let Err(e) = crate::host::display::gb::capture::install(&mut options) {
            return fail(state, e);
        }
        #[cfg(feature = "dev-sms")]
        if let Err(e) = crate::host::display::sms::capture::install(&mut options) {
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
        // The two console chips have a fixed ring instead of a sized one — a
        // quarter of a second on a Game Boy, a third on a Master System — so
        // there is nothing to size and the interception's one effect is to turn
        // recording on. A frame is sixty times inside the shallower of those.
        #[cfg(feature = "dev-gb")]
        if let Err(e) = crate::host::audio::gb::capture::install(&mut options) {
            return fail(state, e);
        }
        #[cfg(feature = "dev-sms")]
        if let Err(e) = crate::host::audio::sms::capture::install(&mut options) {
            return fail(state, e);
        }
        // An Amiga's is fixed too — a third of a second, and a PAL frame is
        // sixteen times inside it.
        #[cfg(feature = "dev-amiga-paula")]
        if let Err(e) = crate::host::audio::amiga::capture::install(&mut options) {
            return fail(state, e);
        }

        let machine = match crate::machine::build(entry.name, entry.source, &registry, &options) {
            Ok(m) => m,
            Err(e) => return fail(state, e),
        };

        let hosts = alloc::sync::Arc::clone(&options.realize.hosts);

        #[cfg(feature = "dev-nes-ppu")]
        if let Some(scanout) = crate::host::display::nes::capture::take(&hosts) {
            state.attach_scanout(alloc::boxed::Box::new(scanout));
        }
        // The PC's adapter reshapes itself when the guest sets a mode, so the
        // surface this attaches is only the geometry the card powers up in —
        // `rsemu_frame_width` answers whatever the last capture produced, and
        // the page resizes its canvas from that.
        #[cfg(feature = "dev-pc-video")]
        if state.scanout.is_none()
            && let Some(scanout) = crate::host::display::pc::capture::take_clocked(&hosts, &machine)
        {
            state.attach_scanout(alloc::boxed::Box::new(scanout));
        }
        // The panel boards' engine, taken after the build because its frame
        // period is read out of the realized machine's clock forest rather than
        // written into a machine file twice.
        #[cfg(feature = "dev-lcdc")]
        if state.scanout.is_none()
            && let Some(scanout) = crate::host::display::lcd::capture::take(&hosts, &machine)
        {
            state.attach_scanout(alloc::boxed::Box::new(scanout));
        }
        #[cfg(feature = "dev-gb")]
        if state.scanout.is_none()
            && let Some(scanout) = crate::host::display::gb::capture::take(&hosts)
        {
            state.attach_scanout(alloc::boxed::Box::new(scanout));
        }
        #[cfg(feature = "dev-sms")]
        if state.scanout.is_none()
            && let Some(scanout) = crate::host::display::sms::capture::take_vdp(&hosts)
        {
            state.attach_scanout(alloc::boxed::Box::new(scanout));
        }

        // Sound goes out as interleaved `f32` in [-1, 1], which is what
        // WebAudio's `AudioBuffer` holds, so the page copies rather than
        // converts.
        #[cfg(feature = "dev-nes-apu")]
        if let Some(source) = crate::host::audio::nes::capture::take(&hosts) {
            state.audio = Some(crate::host::audio::AudioStream::new(
                alloc::boxed::Box::new(source),
                state.audio_rate,
                crate::host::audio::SampleFormat::F32,
            ));
        }
        // Both of these are taken *after* the build and with the machine in
        // hand, because neither chip knows its own sample rate: it is its clock
        // domain's frequency over a constant, and a Master System's is not even
        // the same between regions.
        #[cfg(feature = "dev-gb")]
        if state.audio.is_none()
            && let Some(source) = crate::host::audio::gb::capture::take(&hosts, &machine)
        {
            state.audio = Some(crate::host::audio::AudioStream::new(
                alloc::boxed::Box::new(source),
                state.audio_rate,
                crate::host::audio::SampleFormat::F32,
            ));
        }
        #[cfg(feature = "dev-sms")]
        if state.audio.is_none()
            && let Some(source) = crate::host::audio::sms::capture::take(&hosts, &machine)
        {
            state.audio = Some(crate::host::audio::AudioStream::new(
                alloc::boxed::Box::new(source),
                state.audio_rate,
                crate::host::audio::SampleFormat::F32,
            ));
        }
        // And Paula, whose rate is its colour clock over a constant — the same
        // reason, and the first stereo source with a rational rate.
        #[cfg(feature = "dev-amiga-paula")]
        if state.audio.is_none()
            && let Some(source) = crate::host::audio::amiga::capture::take(&hosts, &machine)
        {
            state.audio = Some(crate::host::audio::AudioStream::new(
                alloc::boxed::Box::new(source),
                state.audio_rate,
                crate::host::audio::SampleFormat::F32,
            ));
        }

        // Whatever character port the machine's devices opened is the console.
        // One is unambiguous; a machine with several is not this ABI's problem
        // until one exists.
        //
        // Except `keyboard`, which is not a console: it is what `pc.kbc` opens,
        // and **every byte on it is a raw AT scan code in set 2** rather than a
        // character (`dev::pc::kbc`). A page that put it behind a terminal pane
        // would show an empty screen and send `0x41` for `A` meaning the `9`
        // key. `src/bin/rsemu.rs` draws the same line by the same name for the
        // same reason, and the PC's output goes to its video adapter anyway.
        let names = crate::host::chardev::ports::names(&hosts);
        if let Some(port) = names
            .iter()
            .find(|n| n.as_str() != KEYBOARD_PORT)
            .and_then(|n| crate::host::chardev::ports::get(&hosts, n).ok().flatten())
        {
            state.console = Some(port);
        }

        // And that same excluded port is the *keyboard*, which is a different
        // seam rather than a worse console. `host::input::KeyboardSink` turns
        // an X11 keysym into the set-2 make and break codes an AT keyboard
        // would have put on the wire, which is exactly what the 8042 on the
        // other end is expecting — the identical path the VNC front end uses,
        // so a browser and a VNC client type at a PC the same way.
        state.keyboard = crate::host::chardev::ports::get(&hosts, KEYBOARD_PORT)
            .ok()
            .flatten()
            .map(crate::host::input::KeyboardSink::new);

        // And whatever pad port its controllers read is where buttons go. The
        // same name-based seam as the console, for the same reason: a machine
        // file can hand a device a name and nothing else. Which console's pad
        // it is decides how a host mask reaches its pins — see [`Pads`].
        #[cfg(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms"))]
        {
            state.pad = Pads::take(&hosts);
        }
        state.hosts = Some(hosts);
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
        state.keyboard = None;
        state.audio = None;
        state.hosts = None;
        #[cfg(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms"))]
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
/// The mask is `0x80` A, `0x40` B, `0x20` Select, `0x10` Start, `0x08` Up,
/// `0x04` Down, `0x02` Left, `0x01` Right. That is the NES shift register's own
/// output order
/// ([NESdev, "Standard controller"](https://www.nesdev.org/wiki/Standard_controller),
/// and [`dev::nes::input::buttons`](crate::dev::nes::input::buttons), which is
/// where the constants live), and it is **the ABI's order for every machine**,
/// not only that one: a page presses "A" and each console receives whatever A
/// is wired to on its own pins. The module's own `Pads` is the translation, and
/// it says what each console does with a button it has not got — a Game Boy has
/// one pad, a Master System has two and no Select, and its Start is the Pause
/// switch on the console rather than a line on the pad.
///
/// The state is a **level, not an event**: the console samples it whenever the
/// guest strobes its port, so a button stays held until the embedder clears it.
/// That is also what makes the seam replayable.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_set_buttons(port: u32, mask: u32) {
    with_state(|state| {
        if let Some(slot) = state.buttons.get_mut(port as usize) {
            *slot = mask & 0xff;
        }
        #[cfg(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms"))]
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

/// Whether this machine has controllers to press at all.
///
/// The companion of [`rsemu_has_video`] and [`rsemu_has_console`], and it is
/// **not** implied by either: a board with a display panel and no game pad is
/// an ordinary machine, and a page that drew a d-pad for it would be inventing
/// hardware. It also decides whether the arrow keys belong to the guest.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_has_pad() -> u32 {
    with_state(|state| {
        #[cfg(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms"))]
        {
            u32::from(state.pad.is_some())
        }
        // A build with no controller device has no pad on any machine, and the
        // export still exists so a page never has to feature-detect.
        #[cfg(not(any(feature = "dev-nes-io", feature = "dev-gb", feature = "dev-sms")))]
        {
            let _ = state;
            0
        }
    })
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

/// Press or release one key on the machine's AT keyboard, naming it by **X11
/// keysym** the way RFB does. Returns `1` if this keyboard has that key.
///
/// The other half of [`rsemu_console_write`], and deliberately not the same
/// call: a console takes *characters* and an AT keyboard takes *transitions*,
/// because the make and break codes are what the hardware puts on the wire and
/// a guest that watches for a key coming up can tell the difference (`ROADMAP.md`
/// §8). So a page sends `down`, then `up`, and the 8042 sees what a person's
/// hands would have produced.
///
/// Printable ASCII keysyms **are** their own character codes, so `A` is `0x41`
/// and a page needs no table for the letters; the named keys are X11's
/// (`Return` `0xff0d`, `BackSpace` `0xff08`, `Escape` `0xff1b`, the arrows
/// `0xff51`-`0xff54`, `F1` `0xffbe`). A key this keyboard does not have puts
/// **no bytes at all** on the wire and answers `0` — a guest handed a scan code
/// for a key nobody pressed is worse off than one handed nothing.
///
/// Shift is synthesised for a character that needs it when the page has not
/// said a shift key is down, exactly as it is for a VNC client, so a page may
/// send `!` without sending `Shift_L` first.
///
/// A page that stops receiving key events — a window losing focus — must send
/// the `up` for whatever it last sent down. Nothing here can know, and a stuck
/// key is a real key stuck.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_key(keysym: u32, down: u32) -> u32 {
    with_state(|state| {
        let Some(keyboard) = state.keyboard.as_ref() else {
            return 0;
        };
        // `set2` is the same table the sink consults, asked here so the answer
        // distinguishes "this keyboard has no such key" from "there is no
        // keyboard" — the page shows the two differently.
        let sym = crate::host::input::Keysym(keysym);
        if crate::host::input::set2(sym).is_none() {
            return 0;
        }
        crate::host::input::InputSink::deliver(
            keyboard,
            crate::host::input::InputEvent::Key {
                keysym: sym,
                down: down != 0,
            },
        );
        1
    })
}

/// Whether this machine has a keyboard [`rsemu_key`] can press.
///
/// Distinct from [`rsemu_has_console`], and on a PC the two are opposites: the
/// PC/AT has a keyboard and no console, and an Apple 1 has a console and no
/// keyboard, because on the Apple 1 the port carries characters.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_has_keyboard() -> u32 {
    with_state(|state| u32::from(state.keyboard.is_some()))
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

// ---------------------------------------------------------------------------
// The wasm JIT's browser embedder
// ---------------------------------------------------------------------------
//
// `ROADMAP.md` §11.4 and `docs/techniques/wasm-jit.md`. `jit::wasm` lowers an
// IR block to a complete `WebAssembly.Module` and has, in `jit::wasm::exec`, a
// reference executor that runs one anywhere — correctly, and more slowly than
// interpreting the IR would be, because it is a wasm interpreter running wasm
// generated from IR. The point of the backend is the *other* implementation of
// `jit::wasm::embed::Embedder`: a host engine that compiles a module once and
// runs it many times. This is that implementation, and it is the only place in
// the crate that can be, because instantiating a module is something only an
// embedder can do and rsemu's embedder is the page.
//
// Three imports out, four exports back, and `docs/techniques/wasm-jit.md` has
// the table. The JavaScript is in `web/src/jit.js` and contains no semantics:
// every observable thing a generated block does routes through `Thunks` in
// `jit::wasm::rt`, which is a transcription of `ir::interp`. That is deliberate
// and it is what makes the determinism claim checkable — there is one
// implementation of the IR's meaning and a browser does not get its own.

/// The activation a generated module was entered with, and the one `unsafe`
/// question in this subsystem.
///
/// # What is handed to JavaScript
///
/// A **token**, not a pointer. `docs/techniques/wasm-jit.md` specified `ctx`
/// as "a real pointer" and this is the one place the built thing differs from
/// the written one, deliberately: a pointer handed to an embedder is a pointer
/// the embedder can hand back wrong, and dereferencing a number JavaScript
/// chose is unsound however carefully the JavaScript is written. A token is
/// **checked** — it names a slot in this thread's activation stack and it
/// carries a sequence number, so a value that is stale, forged, or simply
/// wrong finds no activation and the import answers
/// [`status::ERROR`](crate::jit::wasm::abi::status::ERROR), which the engine
/// turns into an `Err` and the dispatcher into an interpreted block. Nothing a
/// hostile or broken page can put in that argument is unsound.
///
/// The pointers are *inside*: `env` is the address of a `&mut dyn Env` living
/// on [`Browser::enter`]'s own stack frame, and `frame`/`len` describe the
/// engine's temporary-frame buffer. Neither ever leaves this module.
///
/// # What makes the round trip sound
///
/// One invariant, and it is structural rather than hoped for:
///
/// > An activation is reachable **exactly for the dynamic extent of the
/// > `jit_enter` import call that pushed it**, during which the `&mut dyn Env`
/// > and the `&mut [u8]` it describes are alive, unaliased and untouched by
/// > the frame that owns them.
///
/// Every clause is upheld here rather than by the embedder:
///
/// * *Reachable exactly for that extent* — [`Browser::enter`] pushes before
///   the call and an [`Activation`]-popping guard drops after it, on every
///   path out including a panic.
/// * *Alive* — both are locals of that same frame, which is blocked in the
///   import call for the whole time.
/// * *Unaliased and untouched* — `enter` derives a raw pointer from each and
///   then does not name the reference again, so the `&mut` the export
///   reconstitutes is the only live one. A reborrow through a raw pointer
///   derived from the original is exactly the shape this is allowed in.
/// * *This thread's* — the stack is a `thread_local!`, so an activation is
///   never visible to a hart running on another worker. That is also why it is
///   not a `core::sync::Global`: a lock would be taken on every guest memory
///   access made from compiled code, and two harts on two workers would
///   contend for a table neither can see the other's half of.
///
/// Re-entrancy is bounded by construction: a generated module never calls
/// `jit_enter`, so an activation is never nested inside its own, and the four
/// imports create their `&mut` inside one export call and drop it before
/// returning. At most one exists at any instant.
#[cfg(feature = "jit-wasm")]
#[derive(Clone, Copy, Debug)]
struct Activation {
    /// The token this activation answers to. Never zero.
    ctx: u32,
    /// `*mut &mut dyn Env` as an integer: a thin pointer to the fat one.
    ///
    /// Thin because `ctx` has to cross a wasm `i32` in the general case and
    /// because a fat pointer in a `static` would need a lifetime it does not
    /// have. Pointing at the reference rather than at the `Env` keeps the
    /// vtable with it and keeps this type free of generics.
    env: usize,
    /// The temporary frame's address in this module's linear memory.
    frame: usize,
    /// How many bytes of frame there are.
    len: usize,
}

// The activation stack, per thread. Depth is one in practice: a generated
// module never calls `jit_enter`, so the only way to nest one is a host that
// re-enters rsemu from inside an import, which nothing does.
#[cfg(feature = "jit-wasm")]
std::thread_local! {
    static ACTIVE: core::cell::RefCell<Vec<Activation>> =
        const { core::cell::RefCell::new(Vec::new()) };
}

/// The next token. Process-wide, so a token from one thread never names
/// another thread's activation even by accident.
///
/// This and the three items after it are the *minting* half of the protocol,
/// and nothing on a native host mints a token: a block is entered by
/// `jit::wasm::exec` there, which needs no activation at all. So they are
/// gated on the browser target **or `test`** — the tests are what exercise
/// them everywhere else, and they are the reason the invariant is checkable
/// on a runner that has no browser. The *checking* half below
/// ([`jit_dispatch`] and the four exports) is not gated, because a token that
/// names nothing has to be refused wherever somebody calls in with one.
#[cfg(all(
    feature = "jit-wasm",
    any(test, all(target_arch = "wasm32", target_os = "unknown"))
))]
static NEXT_CTX: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(1);

/// Make `env` and `mem` reachable by token for the duration of `enter`.
///
/// The whole of the pointer discipline [`Activation`] documents, in one place
/// so that the browser's `jit_enter` and the round-trip test below cannot
/// implement it differently. `enter` is handed the token and the frame's wasm
/// offset and is expected to do nothing but call into the module; everything
/// it calls back reaches [`jit_dispatch`], which checks the token.
#[cfg(all(
    feature = "jit-wasm",
    any(test, all(target_arch = "wasm32", target_os = "unknown"))
))]
fn with_activation<R>(
    mem: &mut [u8],
    env: &mut dyn crate::jit::wasm::Env,
    enter: impl FnOnce(u32, u32) -> R,
) -> R {
    // Both references become raw addresses here and are not named again until
    // `enter` returns — the clause of `Activation`'s invariant this function
    // is responsible for, and the reason the reborrow in `jit_dispatch` is the
    // only live `&mut` to either.
    let mut env_ref: &mut dyn crate::jit::wasm::Env = env;
    let env_at = (&raw mut env_ref) as usize;
    let frame = mem.as_mut_ptr() as usize;
    let len = mem.len();
    let ctx = push_activation(env_at, frame, len);
    let _pop = Pop(ctx);
    // The frame's wasm offset *is* its address: a generated module imports
    // rsemu's own exported memory, which is the arrangement that keeps guest
    // RAM addressable by byte offset and the frame legal in a
    // `SharedArrayBuffer` (CLAUDE.md, "Targets").
    enter(ctx, frame as u32)
}

/// Pushes `env` and `frame` onto this thread's stack and returns its token.
#[cfg(all(
    feature = "jit-wasm",
    any(test, all(target_arch = "wasm32", target_os = "unknown"))
))]
fn push_activation(env: usize, frame: usize, len: usize) -> u32 {
    // Zero is "no activation", so skip it on the wrap. The counter wrapping at
    // all needs 2^32 block entries in one process, and even then the worst a
    // collision could do is name a *different live* activation — memory-safe,
    // because every activation in the stack is valid by the invariant above.
    let mut ctx = NEXT_CTX.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    if ctx == 0 {
        ctx = NEXT_CTX.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
    ACTIVE.with_borrow_mut(|stack| {
        stack.push(Activation {
            ctx,
            env,
            frame,
            len,
        });
    });
    ctx
}

/// Pops the activation `ctx` names, whatever happened inside the call.
#[cfg(all(
    feature = "jit-wasm",
    any(test, all(target_arch = "wasm32", target_os = "unknown"))
))]
struct Pop(u32);

#[cfg(all(
    feature = "jit-wasm",
    any(test, all(target_arch = "wasm32", target_os = "unknown"))
))]
impl Drop for Pop {
    fn drop(&mut self) {
        ACTIVE.with_borrow_mut(|stack| {
            if let Some(at) = stack.iter().rposition(|a| a.ctx == self.0) {
                stack.remove(at);
            }
        });
    }
}

/// Route one import call back into the [`Env`] that `ctx` names.
///
/// `args[0]` is `ctx` itself, because that is the first parameter generated
/// code passes to every import (`jit::wasm::abi`) and `Thunks::call` reads its
/// arguments at the positions the generated call site put them.
#[cfg(feature = "jit-wasm")]
fn jit_dispatch(ctx: u32, func: u32, args: &[i64]) -> i64 {
    let Some(active) =
        ACTIVE.with_borrow(|stack| stack.iter().rev().find(|a| a.ctx == ctx).copied())
    else {
        // A token naming no activation: a stale `ctx`, a forged one, or an
        // import called after `jit_enter` returned. Not a panic and not a
        // dereference — the block reports an error and the dispatcher
        // interprets it.
        return crate::jit::wasm::abi::status::ERROR;
    };
    // SAFETY: `active.env` is the address of a `&mut dyn Env` local to the
    // `Browser::enter` frame that pushed this activation, and `active.frame`
    // /`active.len` describe the `&mut [u8]` that frame was given. That frame
    // is blocked inside the `jit_enter` import call for the whole time this
    // activation is reachable (`Pop` removes it on every path out), so both
    // are alive; it derived raw pointers from each and never names the
    // references again, so these reborrows are the only live `&mut` to either;
    // and the stack is thread-local, so no other thread can reach them. The
    // two borrows do not overlap each other: the frame buffer is the engine's
    // scratch memory and is not reachable from the `Env` except through the
    // `mem` argument it is being passed as.
    let (env, mem) = unsafe {
        (
            &mut *(active.env as *mut &mut dyn crate::jit::wasm::Env),
            core::slice::from_raw_parts_mut(active.frame as *mut u8, active.len),
        )
    };
    env.call(func, args, mem)
}

// The three imports a page supplies, and the whole of what the host does.
//
// `ROADMAP.md` §11.5's convention: module `rsemu`, no bundled JS runtime, the
// embedder hands them in at instantiation. `web/src/jit.js` is the reference
// implementation, and it has no semantics in it at all:
//
//   jit_compile(ptr, len) -> handle   `new WebAssembly.Module(bytes)`,
//                                     instantiated; 0 if the engine refused
//   jit_enter(handle, ctx, frame)     call that instance's `b` export
//   jit_release(handle)               drop the instance, on eviction
//
// Declared only for `wasm32-unknown-unknown` because there is nothing to bind
// them to anywhere else: WASI preview 1 and preview 2 have no interface for
// compiling or instantiating a module, so a `wasm32-wasip1` build has no host
// JIT and runs the reference executor. `docs/techniques/wasm-jit.md`, "What
// WASI would need", is the long form.
#[cfg(all(feature = "jit-wasm", target_arch = "wasm32", target_os = "unknown"))]
#[link(wasm_import_module = "rsemu")]
unsafe extern "C" {
    fn jit_compile(ptr: *const u8, len: usize) -> u32;
    fn jit_enter(handle: u32, ctx: u32, frame: u32) -> i64;
    fn jit_release(handle: u32);
}

/// The embedder that hands modules to the page's `WebAssembly` engine.
#[cfg(all(feature = "jit-wasm", target_arch = "wasm32", target_os = "unknown"))]
#[derive(Debug)]
struct Browser;

#[cfg(all(feature = "jit-wasm", target_arch = "wasm32", target_os = "unknown"))]
impl crate::jit::wasm::Embedder for Browser {
    fn compile(&self, module: &[u8]) -> u32 {
        // SAFETY: the import is handed a pointer into this module's own linear
        // memory and a length that matches it. `web/src/jit.js` reads exactly
        // those bytes, synchronously, into a `WebAssembly.Module` and does not
        // keep the pointer; `module` outlives the call because it is the
        // engine's own resident module vector.
        unsafe { jit_compile(module.as_ptr(), module.len()) }
    }

    fn enter(
        &self,
        handle: u32,
        mem: &mut [u8],
        env: &mut dyn crate::jit::wasm::Env,
    ) -> Option<i64> {
        let status = with_activation(mem, env, |ctx, frame| {
            // SAFETY: `handle` came from this embedder's own `compile` and
            // names a live instance until `release`; `ctx` names the
            // activation `with_activation` pushed, which it removes before it
            // returns. The import does nothing but call that instance's `b`
            // export, whose only way back into Rust is the four exports below,
            // each of which checks its token before it reconstitutes anything.
            unsafe { jit_enter(handle, ctx, frame) }
        });
        // Negative is the host saying *this did not run* — a handle it has
        // forgotten. Every [`status`] is non-negative, so the encoding is
        // free, and the alternative for the page was throwing into a wasm
        // frame, which traps and takes the emulator down rather than costing
        // it one interpreted block.
        (status >= 0).then_some(status)
    }

    fn release(&self, handle: u32) {
        // SAFETY: `handle` is one this embedder's `compile` returned and which
        // has not been released; the import drops the instance and returns.
        unsafe { jit_release(handle) }
    }
}

/// The installed browser embedder, as a `&'static`.
#[cfg(all(feature = "jit-wasm", target_arch = "wasm32", target_os = "unknown"))]
static BROWSER: Browser = Browser;

/// Route the wasm JIT's generated modules through the page's engine.
///
/// Answers 1 if this build can do that and has now been told to, 0 otherwise —
/// which is every build that is not `wasm32-unknown-unknown` with `jit-wasm`,
/// including the demo the site ships. **Call it before booting**: an engine
/// reads the installed embedder once, when it is built, so a machine already
/// running keeps the reference executor.
///
/// Explicit rather than automatic because a page that calls this must also
/// have supplied the three `rsemu.jit_*` imports, and a module that installed
/// itself would fail at the first compiled block on a page that had not.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_jit_enable() -> u32 {
    #[cfg(all(feature = "jit-wasm", target_arch = "wasm32", target_os = "unknown"))]
    {
        crate::jit::wasm::install(&BROWSER);
        return 1;
    }
    #[cfg(not(all(feature = "jit-wasm", target_arch = "wasm32", target_os = "unknown")))]
    0
}

/// `GET_SLOT`: `(ctx, slot) -> value`.
#[cfg(feature = "jit-wasm")]
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_jit_slot(ctx: u32, slot: u32) -> i64 {
    jit_dispatch(
        ctx,
        crate::jit::wasm::abi::func::SLOT,
        &[i64::from(ctx), i64::from(slot)],
    )
}

/// A guest load: `(ctx, memop, addr, at) -> answer`, the value in the frame.
#[cfg(feature = "jit-wasm")]
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_jit_load(ctx: u32, memop: u32, addr: i64, at: u32) -> i32 {
    jit_dispatch(
        ctx,
        crate::jit::wasm::abi::func::LOAD,
        &[i64::from(ctx), i64::from(memop), addr, i64::from(at)],
    ) as i32
}

/// A guest store: `(ctx, memop, addr, value, at) -> answer`.
#[cfg(feature = "jit-wasm")]
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_jit_store(ctx: u32, memop: u32, addr: i64, value: i64, at: u32) -> i32 {
    jit_dispatch(
        ctx,
        crate::jit::wasm::abi::func::STORE,
        &[i64::from(ctx), i64::from(memop), addr, value, i64::from(at)],
    ) as i32
}

/// A charge or a boundary: `(ctx, kind, arg) -> answer`.
#[cfg(feature = "jit-wasm")]
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_jit_note(ctx: u32, kind: u32, arg: i64) -> i32 {
    jit_dispatch(
        ctx,
        crate::jit::wasm::abi::func::NOTE,
        &[i64::from(ctx), i64::from(kind), arg],
    ) as i32
}

// ---------------------------------------------------------------------------
// The guest the embedder is measured and compared on
// ---------------------------------------------------------------------------
//
// Two questions need one guest, and neither can be answered from inside the
// crate:
//
//   1. **Determinism.** `tests/riscv_virt_engines.rs` asserts one state hash
//      across `interp`, `jit`, `jit-host` and `jit-wasm` — but only ever with
//      `jit::wasm::exec` behind the last one, because a native test has no
//      embedder. A browser run is a *fourth* execution of the same blocks and
//      has to hash the same. `web/check.mjs` is where that is asserted, and
//      this is what it calls.
//   2. **Speed.** The whole reason §11.4 wants this backend is the multiplier
//      a real engine gives over interpreting, and the only place to take it is
//      in a browser.
//
// Both want the *same* guest run the *same* span under two engines, which is
// why this is one function taking an engine rather than a benchmark and a test
// that could drift apart. It is also callable from Rust, so the native numbers
// in `docs/techniques/wasm-jit.md` are taken on this workload and not on a
// different one.

/// The guest: a seven-instruction RV64I loop with its scratch word on the page
/// after its code.
///
/// Lifted from `cpu::riscv::engine`'s `FAR_LOOP`, and the separation matters:
/// a loop that stores *into its own page* invalidates its block every pass, so
/// nothing is ever chained, every pass re-lifts, and what gets measured is the
/// lifter rather than the backend. `lui x7, 1` moves the scratch word to
/// 0x1050 and the loop starts behaving like code — one block, compiled once,
/// entered once per pass, which is exactly the shape an embedder's
/// instantiation cost has to be amortised over.
///
/// Every instruction is in the lifted subset, so `jit-wasm` lowers the block
/// to a module rather than refusing it — [`rsemu_jit_guest_stat`] answers with
/// the counts that say so, because a benchmark of a backend that quietly
/// refused everything would report the interpreter twice.
#[cfg(all(feature = "jit-wasm", feature = "cpu-riscv-lift"))]
const GUEST: [u32; 7] = [
    0x0000_13b7, // lui   x7, 1        ; x7 = 0x1000, the next page
    0x0000_0293, // addi  x5, x0, 0
    0x0010_0313, // addi  x6, x0, 1
    0x0062_82b3, // add   x5, x5, x6   ; the loop starts here
    0x0453_b823, // sd    x5, 80(x7)
    0x0503_be03, // ld    x28, 80(x7)
    0xff5f_f06f, // jal   x0, -12
];

/// How much RAM the guest gets: two pages, code in the first, scratch in the
/// second.
#[cfg(all(feature = "jit-wasm", feature = "cpu-riscv-lift"))]
const GUEST_RAM: u64 = 8192;

/// What the last [`rsemu_jit_guest_run`] did, by the column indices
/// [`rsemu_jit_guest_stat`] documents.
///
/// Atomics rather than a `Global<Stats>` because a `Global` wants a `const`
/// initialiser and `Stats` grows a field whenever a backend learns to count
/// something new; a fixed array of counters does not have to be rewritten each
/// time.
#[cfg(all(feature = "jit-wasm", feature = "cpu-riscv-lift"))]
static GUEST_STATS: [core::sync::atomic::AtomicU64; 8] =
    [const { core::sync::atomic::AtomicU64::new(0) }; 8];

/// Run the fixed RV64I guest for `quanta` budgets of `budget` ticks under
/// `engine`, and
/// answer with a hash of everything a guest can see afterwards.
///
/// `engine` is [`crate::cpu::riscv::Engine`]'s discriminant order: 0 `interp`,
/// 1 `jit`, 2 `jit-host`, 3 `jit-wasm`. An engine this build does not have
/// falls back to the portable backend, which is `Jit::new`'s own rule and not
/// this function's — a machine file is portable and a measurement is never
/// silently of something else.
///
/// The hash is FNV-1a over the thirty-two integer registers, the program
/// counter, the cycle counter and `minstret`: the same columns
/// `cpu::riscv::engine`'s `agree_built` compares one at a time, folded into one
/// number because the thing on the other end of this is JavaScript. It is a
/// comparison device and not a persisted format, so it is not
/// `core::state`'s.
#[cfg(all(feature = "jit-wasm", feature = "cpu-riscv-lift"))]
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_jit_guest_run(engine: u32, quanta: u32, budget: u64) -> u64 {
    use alloc::sync::Arc;

    use crate::core::space::{AddressSpace, Perms, RamStore, Region};
    use crate::cpu::riscv::{Config, Engine, Hart};

    let engine = match engine {
        1 => Engine::Jit,
        2 => Engine::JitHost,
        3 => Engine::JitWasm,
        _ => Engine::Interp,
    };

    let ram = Arc::new(RamStore::new(GUEST_RAM));
    for (i, word) in GUEST.iter().enumerate() {
        for (j, byte) in word.to_le_bytes().iter().enumerate() {
            ram.write_u8((i * 4 + j) as u64, *byte)
                .expect("the program fits in the first page");
        }
    }
    let space = AddressSpace::new("mem", 64);
    space
        .topology()
        .map_with_perms(Region::ram("ram", ram), 0, Perms::RWX)
        .expect("nothing else is mapped");
    let hart = Hart::new(Config::rv64gc().with_reset_vector(0)).with_engine(engine);
    hart.attach_space(Arc::new(space));

    for _ in 0..quanta {
        hart.run_budget(budget);
    }

    let s = hart.jit_stats().unwrap_or_default();
    for (cell, value) in GUEST_STATS.iter().zip([
        s.blocks,
        s.compiled,
        s.translated,
        s.wasm_instantiated,
        s.wasm_embedded,
        s.retired,
        s.interpreted,
        // The one column every engine fills, because it is architectural and
        // not a statistic: `interp` has no `Jit` at all, so a rate computed
        // from `retired` would divide by zero for the very engine the others
        // are measured against.
        hart.instret(),
    ]) {
        cell.store(value, core::sync::atomic::Ordering::Relaxed);
    }

    // FNV-1a over the architectural columns, in a fixed order.
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut fold = |v: u64| {
        for byte in v.to_le_bytes() {
            h ^= u64::from(byte);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for n in 0..32 {
        fold(hart.x(n));
    }
    fold(hart.pc());
    fold(hart.cycles());
    fold(hart.instret());
    h
}

/// What the last [`rsemu_jit_guest_run`] counted, by column.
///
/// The gate `web/check.mjs` needs and a hash cannot give: a run whose
/// `compiled` is zero produced the same hash as the interpreter because it
/// *was* the interpreter, and a benchmark of that would be a benchmark of
/// nothing. Column 4 is the one this work is about — blocks entered in a
/// module the page's own engine compiled.
///
/// 0 blocks, 1 compiled, 2 translated, 3 instantiated, 4 embedded,
/// 5 retired, 6 interpreted, 7 `minstret`. Anything else is 0.
#[cfg(all(feature = "jit-wasm", feature = "cpu-riscv-lift"))]
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_jit_guest_stat(column: u32) -> u64 {
    GUEST_STATS
        .get(column as usize)
        .map_or(0, |c| c.load(core::sync::atomic::Ordering::Relaxed))
}

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // The wasm JIT's import round trip
    // -----------------------------------------------------------------------
    //
    // The browser half of this is `unsafe` — a token minted here, handed to a
    // wasm module, passed back through four exports, and turned into a `&mut
    // dyn Env` and a `&mut [u8]`. The *engine* behind it is the only part that
    // needs a browser, so the round trip is checked here on whatever host is
    // running the suite: the closure below stands in for a generated module
    // and calls the exports exactly as its imports do.
    //
    // What that leaves for `web/check.mjs` is the one thing it cannot do: a
    // real `WebAssembly.Module`, compiled by a real engine, making the same
    // calls.

    #[cfg(feature = "jit-wasm")]
    #[derive(Debug, Default)]
    struct Recording {
        calls: Vec<(u32, Vec<i64>)>,
        frame_len: usize,
    }

    #[cfg(feature = "jit-wasm")]
    impl crate::jit::wasm::Env for Recording {
        fn call(&mut self, func: u32, args: &[i64], mem: &mut [u8]) -> i64 {
            self.calls.push((func, args.to_vec()));
            self.frame_len = mem.len();
            // Write where a load's value goes, so the caller can prove this is
            // the *engine's* frame and not a copy of it.
            mem[..8].copy_from_slice(&0xfeed_face_u64.to_le_bytes());
            0
        }
    }

    #[cfg(feature = "jit-wasm")]
    #[test]
    fn an_import_reaches_the_env_its_token_names() {
        use crate::jit::wasm::abi::func;

        let mut frame = alloc::vec![0u8; 64];
        let mut env = Recording::default();
        let ctx_seen = with_activation(&mut frame, &mut env, |ctx, frame_at| {
            // Standing in for a generated module: the four imports, with the
            // argument positions `jit::wasm::compile` emits.
            assert_ne!(ctx, 0, "zero is reserved for `no activation`");
            assert_ne!(frame_at, 0, "the frame has an address in linear memory");
            rsemu_jit_slot(ctx, 3);
            rsemu_jit_load(ctx, 1, 0x2000, 4);
            rsemu_jit_store(ctx, 2, 0x2008, 0x55, 5);
            rsemu_jit_note(ctx, 0, 9);
            ctx
        });

        assert_eq!(env.calls.len(), 4, "every import reached the env");
        assert_eq!(env.frame_len, 64, "and saw the engine's own frame");
        let want = [
            (func::SLOT, alloc::vec![i64::from(ctx_seen), 3]),
            (func::LOAD, alloc::vec![i64::from(ctx_seen), 1, 0x2000, 4]),
            (
                func::STORE,
                alloc::vec![i64::from(ctx_seen), 2, 0x2008, 0x55, 5],
            ),
            (func::NOTE, alloc::vec![i64::from(ctx_seen), 0, 9]),
        ];
        assert_eq!(env.calls, want, "import index and argument positions");
        // The frame the exports wrote into is the caller's buffer, which is
        // the whole reason `$frame` crosses as an address rather than a copy.
        assert_eq!(
            u64::from_le_bytes(frame[..8].try_into().expect("eight bytes")),
            0xfeed_face,
        );
    }

    #[cfg(feature = "jit-wasm")]
    #[test]
    fn a_token_naming_no_activation_is_refused_rather_than_dereferenced() {
        use crate::jit::wasm::abi::status;

        // A stale token: one that named an activation which has since been
        // popped. This is the case a *pointer* would have made unsound, and it
        // is why `docs/techniques/wasm-jit.md`'s "ctx becomes a real pointer"
        // is the one thing the built embedder does differently.
        let mut frame = alloc::vec![0u8; 64];
        let mut env = Recording::default();
        let stale = with_activation(&mut frame, &mut env, |ctx, _| ctx);
        assert_eq!(i64::from(rsemu_jit_load(stale, 0, 0, 0)), status::ERROR);
        assert_eq!(i64::from(rsemu_jit_note(stale, 0, 0)), status::ERROR);
        assert_eq!(rsemu_jit_slot(stale, 0), status::ERROR);

        // And a token nothing ever minted, which is what a hostile page has.
        assert_eq!(rsemu_jit_slot(0, 0), status::ERROR);
        assert_eq!(rsemu_jit_slot(u32::MAX, 0), status::ERROR);
        assert_eq!(env.calls.len(), 0, "none of it reached an env");
    }

    #[test]
    fn enabling_the_host_jit_answers_for_this_build() {
        // 1 only where there is something to enable: `wasm32-unknown-unknown`
        // with `jit-wasm`. Everywhere else — the demo the site ships, every
        // native build, both WASI targets — the answer is 0 and the reference
        // executor stays, which is what `docs/techniques/wasm-jit.md`'s "What
        // WASI would need" says and what a page has to be able to ask.
        let want = u32::from(cfg!(all(
            feature = "jit-wasm",
            target_arch = "wasm32",
            target_os = "unknown"
        )));
        assert_eq!(rsemu_jit_enable(), want);
    }

    #[cfg(all(feature = "jit-wasm", feature = "cpu-riscv-lift"))]
    #[test]
    fn the_measured_guest_hashes_the_same_under_every_engine_this_build_has() {
        // The native half of what `web/check.mjs` asserts in a browser. There
        // the fourth engine is V8 running a module it compiled; here it is
        // `jit::wasm::exec` running the same module. Same blocks, same hash —
        // and the counts below are what says the wasm backend was reached at
        // all, because a hash that matched because nothing compiled would be
        // the interpreter agreeing with itself.
        let interp = rsemu_jit_guest_run(0, 16, 1000);
        let jit = rsemu_jit_guest_run(1, 16, 1000);
        let wasm = rsemu_jit_guest_run(3, 16, 1000);
        assert_eq!(interp, jit, "the portable backend");
        assert_eq!(interp, wasm, "the wasm backend");
        assert!(rsemu_jit_guest_stat(1) > 0, "blocks ran compiled");
        assert_eq!(
            rsemu_jit_guest_stat(4),
            0,
            "and on a host with no embedder, none of them in a host module"
        );
    }

    /// Serialises the tests that boot a machine through the ABI.
    ///
    /// This one is not going away with the rest of them. The ABI genuinely has
    /// **one machine per module instance** — [`STATE`] is that slot, and
    /// `rsemu_boot` is documented as replacing whatever was there — so two
    /// libtest threads booting at once is not a table collision to be designed
    /// out, it is two callers using the ABI wrongly. Serialising is what a page
    /// does for free by being single-threaded.
    ///
    /// [`LockRank::UNCHECKED`] because it is deliberately held across `STATE`,
    /// which sits at [`LockRank::MACHINE`]; any checked rank would forbid that.
    static ONE_MACHINE: Global<()> = Global::with_rank(LockRank::UNCHECKED, ());

    #[test]
    fn echo_round_trips() {
        assert_eq!(rsemu_echo(0xdead_beef), 0xdead_beef);
    }

    #[test]
    fn version_pointer_and_length_describe_the_same_string() {
        let _one = ONE_MACHINE.lock();
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
        let _one = ONE_MACHINE.lock();
        let count = rsemu_machine_count();
        for index in 0..count {
            assert!(rsemu_machine_name(index) > 0, "machine {index} has no name");
            assert!(rsemu_machine_summary(index) > 0);
        }
        // One past the end is an empty answer, not a panic.
        assert_eq!(rsemu_machine_name(count), 0);
    }

    /// Whatever the output buffer holds, as a string.
    fn out() -> String {
        with_state(|state| String::from_utf8_lossy(&state.output).into_owned())
    }

    /// The catalog index of the machine called `name` in this build.
    ///
    /// Only the per-machine transcript tests name a machine, so this is dead
    /// code in a build with none of them -- and `--no-default-features
    /// --features wasm` is a real sweep configuration that compiles the ABI
    /// with an empty catalog.
    #[cfg(any(
        feature = "machine-beneater",
        feature = "machine-gameboy",
        feature = "machine-pc-at",
        feature = "machine-spi-panel",
        feature = "machine-sms"
    ))]
    fn machine_index(name: &str) -> u32 {
        (0..rsemu_machine_count())
            .find(|i| {
                rsemu_machine_name(*i);
                out() == name
            })
            .unwrap_or_else(|| panic!("this build has no `{name}` in its catalog"))
    }

    /// The index of `machine`'s built-in image called `image`.
    ///
    /// Only the `machine-beneater` tests name a machine and an image, so this
    /// is dead code in a build without it -- and `--no-default-features
    /// --features wasm` is a real sweep configuration that compiles the ABI
    /// with an empty catalog.
    #[cfg(feature = "machine-beneater")]
    fn builtin_index(machine: u32, image: &str) -> u32 {
        (0..rsemu_machine_builtin_count(machine))
            .find(|b| {
                rsemu_machine_builtin_name(machine, *b);
                out() == image
            })
            .unwrap_or_else(|| panic!("no built-in image called `{image}`"))
    }

    /// Every built-in image is describable, and one past the end is empty
    /// rather than a panic — the same contract the machine catalog has.
    #[test]
    fn built_in_images_are_readable_by_index() {
        let _one = ONE_MACHINE.lock();
        for machine in 0..rsemu_machine_count() {
            let count = rsemu_machine_builtin_count(machine);
            for image in 0..count {
                assert!(rsemu_machine_builtin_name(machine, image) > 0);
                assert!(rsemu_machine_builtin_summary(machine, image) > 0);
                // A built-in image that named no slot could not be bound to
                // anything, which would make it undiscoverable rather than
                // merely undocumented.
                assert!(rsemu_machine_builtin_slot(machine, image) > 0);
                let slot = out();
                rsemu_machine_media(machine);
                let media = out();
                assert!(
                    !media.is_empty() && (slot == media || catalog_slots(machine).contains(&slot)),
                    "built-in image {image} fills `{slot}`, which is not one of {media:?}"
                );
            }
            assert_eq!(rsemu_machine_builtin_name(machine, count), 0);
        }
        assert_eq!(rsemu_machine_builtin_count(rsemu_machine_count()), 0);
    }

    /// Every media slot a machine declares, for the assertion above.
    fn catalog_slots(index: u32) -> Vec<String> {
        catalog_entry(index)
            .map(|e| e.media.iter().map(|s| String::from(*s)).collect())
            .unwrap_or_default()
    }

    /// Type `text` at the machine's console and run for `frames` frames.
    ///
    /// Only the `machine-beneater` transcript tests drive a console this way.
    #[cfg(feature = "machine-beneater")]
    fn exchange(text: &str, frames: u32) -> String {
        if !text.is_empty() {
            let bytes = text.as_bytes();
            rsemu_input_reserve(bytes.len());
            with_state(|state| state.input.copy_from_slice(bytes));
            assert_eq!(rsemu_console_write(bytes.len()), bytes.len());
        }
        let mut seen = String::new();
        for _ in 0..frames {
            rsemu_run_frame();
            if rsemu_console_read() > 0 {
                with_state(|state| {
                    for byte in &state.output {
                        // A 1976 console ends a line with a bare carriage
                        // return and has no lower case; `web/src/session.js`
                        // does exactly this on the page's side.
                        match byte & 0x7f {
                            0x0d => seen.push('\n'),
                            c @ 0x20..=0x7e => seen.push(c as char),
                            _ => {}
                        }
                    }
                });
            }
        }
        seen
    }

    /// **The Woz Monitor of 1976, in a browser, with nothing uploaded.**
    ///
    /// Everything a visitor does: pick the machine, pick the monitor, press
    /// boot, type. The expected bytes are not rsemu's — the dump is what the
    /// *Apple-1 Operation Manual*'s own listing holds at `$FF00`, fetched by
    /// Woz's code through this board's bus, and `dev::wdc::tests` asserts the
    /// same transcript one layer down.
    #[cfg(feature = "machine-beneater")]
    #[test]
    fn wozmon_boots_and_answers_through_the_abi() {
        let _one = ONE_MACHINE.lock();
        let machine = machine_index("beneater-6502");
        let wozmon = builtin_index(machine, "wozmon");

        // No `rsemu_input_reserve`, no file, no bytes from the page at all.
        assert_eq!(rsemu_boot_builtin(machine, wozmon), 1, "{}", {
            rsemu_error();
            out()
        });
        assert_eq!(rsemu_has_console(), 1);
        assert_eq!(rsemu_has_video(), 0, "this board drives a serial line");
        assert_eq!(rsemu_has_pad(), 0, "and it has no controllers to draw");

        // Wozmon greets with a backslash and a carriage return, and then waits.
        let banner = exchange("", 30);
        assert_eq!(banner, "\\\n", "got {banner:?}");

        // `AAAA.BBBB` examines a range, eight bytes to a line.
        let dump = exchange("FF00.FF0F\r", 60);
        assert!(
            dump.contains("FF00: D8 58 A0 7F A9 1F 8D 03")
                && dump.contains("FF08: 50 A9 0B 8D 02 50 EA C9"),
            "got {dump:?}"
        );

        // `AAAA: xx yy` deposits, which is the other half of the monitor.
        let deposit = exchange("0300: AA BB CC\r", 60);
        assert!(deposit.contains("0300: 00"), "got {deposit:?}");
        let readback = exchange("0300.0302\r", 60);
        assert!(readback.contains("0300: AA BB CC"), "got {readback:?}");

        std::println!("--- rsemu_boot_builtin(beneater-6502, wozmon) ---");
        std::println!("{banner}{dump}{deposit}{readback}");
        rsemu_shutdown();
    }

    /// The same board with nothing chosen at all: `rsemu_boot(index, 0)` is the
    /// machine's default image, which is rsemu's own monitor.
    #[cfg(feature = "machine-beneater")]
    #[test]
    fn a_default_boot_takes_the_first_built_in_image() {
        let _one = ONE_MACHINE.lock();
        let machine = machine_index("beneater-6502");
        assert_eq!(builtin_index(machine, "rsmon"), 0, "rsmon is the default");
        assert_eq!(rsemu_boot(machine, 0), 1);
        let banner = exchange("", 40);
        assert!(banner.starts_with("RSMON"), "got {banner:?}");

        // And an index nobody offers is an error with a message, not a panic
        // and not a silent boot on the wrong image.
        assert_eq!(rsemu_boot_builtin(machine, 99), 0);
        assert!(rsemu_error() > 0);
        rsemu_shutdown();
    }

    /// A machine whose picture does not come from a NES PPU still reaches the
    /// canvas: `spi-panel` boots its own firmware and paints a gradient.
    ///
    /// The reason this test is here rather than in `host::display`: the frame
    /// buffer's format is part of the ABI, and this adapter would rather hand
    /// out `RGB888`. A page building `ImageData` over three-byte pixels gets a
    /// sheared picture, and nothing else would notice.
    #[cfg(all(feature = "machine-spi-panel", feature = "dev-lcdc"))]
    #[test]
    fn a_panel_board_draws_through_the_abi() {
        let _one = ONE_MACHINE.lock();
        let machine = machine_index("spi-panel");
        assert_eq!(rsemu_boot_builtin(machine, 0), 1, "{}", {
            rsemu_error();
            out()
        });
        assert_eq!(rsemu_has_video(), 1);
        // A picture is not a game console: this board has no controller port,
        // and a page that drew a d-pad for it would be inventing hardware.
        assert_eq!(rsemu_has_pad(), 0);
        let (width, height) = (rsemu_frame_width(), rsemu_frame_height());
        assert!(width > 0 && height > 0);
        assert_eq!(
            rsemu_frame_len(),
            (width as usize) * (height as usize) * 4,
            "the ABI promises four bytes a pixel whatever the adapter prefers"
        );

        // The demo has a whole SPI configuration sequence to get through before
        // it paints, so this is generous on purpose.
        let mut drawn = 0;
        for _ in 0..240 {
            drawn += rsemu_run_frame();
        }
        assert!(drawn > 0, "the panel never produced a frame");

        let colours = with_state(|state| {
            let mut seen = alloc::collections::BTreeSet::new();
            for pixel in state.frame.pixels().as_chunks::<4>().0 {
                assert_eq!(pixel[3], 0xff, "a pixel the canvas would draw see-through");
                seen.insert((pixel[0], pixel[1], pixel[2]));
            }
            seen.len()
        });
        assert!(colours > 1, "the panel drew one flat colour");
        rsemu_shutdown();
    }

    /// The PC/AT posts on rsemu's own BIOS with nothing bound from outside.
    ///
    /// The only built-in image in this module that is *generated* rather than
    /// carried: `fw::pcbios` assembles it for this board, because its MP and
    /// ACPI tables describe this board's processors. Two things are being
    /// claimed. First, that the boot succeeds at all — a PC declares five media
    /// slots and `machine::realize` refuses an unbound one, so the empty bays
    /// have to be bound as empty or `vgacard` will not assemble. Second, that
    /// the firmware ran: the VGA is in text mode 3 at 720x400, three rows carry
    /// ink and nothing below them does, and the nine-by-sixteen block of pixels
    /// that is the `B` of `BIOS` on the banner is the same block as the `B` of
    /// `Booting.` on the line below. No font table is consulted; glyph identity
    /// is what a screen full of words has and a test pattern does not.
    #[cfg(all(feature = "machine-pc-at", feature = "fw-pcbios"))]
    #[test]
    fn a_pc_at_posts_on_its_own_bios_through_the_abi() {
        let _one = ONE_MACHINE.lock();
        let machine = machine_index("pc-at");
        assert_eq!(rsemu_boot_builtin(machine, 0), 1, "{}", {
            rsemu_error();
            out()
        });
        assert_eq!(rsemu_has_video(), 1);
        // `pc.kbc` opens a character port, and every byte on it is a scan code
        // rather than text — so this machine has a keyboard and not a console,
        // and a page that put a terminal in front of it would be lying.
        assert_eq!(rsemu_has_console(), 0);
        assert_eq!(rsemu_has_pad(), 0);
        assert_eq!((rsemu_frame_width(), rsemu_frame_height()), (720, 400));
        // The CRTC's own rate, not the 60 Hz a machine with no display gets.
        assert_ne!(rsemu_frame_period_ns(), DEFAULT_FRAME_NS);

        for _ in 0..240 {
            rsemu_run_frame();
        }

        // One 9x16 text cell's ink, as a bit string.
        let cell = |cx: usize, cy: usize| -> String {
            with_state(|state| {
                let px = state.frame.pixels();
                let w = state.frame.width() as usize;
                let mut bits = String::new();
                for y in 0..16 {
                    for x in 0..9 {
                        let i = ((cy * 16 + y) * w + cx * 9 + x) * 4;
                        let lit = px[i] | px[i + 1] | px[i + 2] != 0;
                        bits.push(if lit { '1' } else { '0' });
                    }
                }
                bits
            })
        };
        let row_has_ink = |cy: usize| (0..80).any(|cx| cell(cx, cy).contains('1'));

        assert!(
            row_has_ink(0) && row_has_ink(1) && row_has_ink(2),
            "the BIOS printed fewer than its three lines"
        );
        // Row 3 is the cursor, and it blinks, so it is deliberately not asserted.
        assert!(
            (4..25).all(|cy| !row_has_ink(cy)),
            "something below the POST output — it scrolled, or it is not a POST screen"
        );
        // "rsemu BIOS, …" over "Booting." over "No bootable device."
        assert_eq!(
            cell(6, 0),
            cell(0, 1),
            "the B of BIOS is not the B of Booting."
        );
        assert_eq!(cell(1, 1), cell(2, 1), "the two o's of Booting. differ");
        assert_eq!(cell(1, 2), cell(1, 1), "the o of No is not that o");
        assert_ne!(cell(0, 2), cell(0, 1), "N and B are the same glyph");
        rsemu_shutdown();
    }

    /// The index of `machine`'s media slot called `name`.
    ///
    /// Only the staging tests name a slot, and `--no-default-features
    /// --features wasm` compiles this module with an empty catalog, so this is
    /// dead code in a build without the PC.
    #[cfg(all(feature = "machine-pc-at", feature = "fw-pcbios"))]
    fn machine_media_index(machine: u32, name: &str) -> u32 {
        (0..rsemu_machine_media_count(machine))
            .find(|s| {
                rsemu_machine_media_name(machine, *s);
                out() == name
            })
            .unwrap_or_else(|| panic!("this machine has no `{name}` slot"))
    }

    /// One 9x16 VGA text cell's ink, as a bit string.
    ///
    /// Glyph *identity* rather than a font table: two cells that carry the same
    /// letter have the same bits, and a test pattern has no such pairs. The
    /// same trick `web/check.mjs` plays on the picture from the other side of
    /// the ABI.
    #[cfg(all(feature = "machine-pc-at", feature = "fw-pcbios"))]
    fn text_cell(cx: usize, cy: usize) -> String {
        with_state(|state| {
            let px = state.frame.pixels();
            let w = state.frame.width() as usize;
            let mut bits = String::new();
            for y in 0..16 {
                for x in 0..9 {
                    let i = ((cy * 16 + y) * w + cx * 9 + x) * 4;
                    let lit = px[i] | px[i + 1] | px[i + 2] != 0;
                    bits.push(if lit { '1' } else { '0' });
                }
            }
            bits
        })
    }

    /// A 1.44 MB diskette whose boot sector prints one `B` and stops.
    ///
    /// Hand-encoded rather than assembled, because `src/fw/asm16` is the
    /// *firmware's* assembler and this is the guest — a fixture built with the
    /// code under test would agree with itself. Nine bytes of real mode: an
    /// `INT 10h` teletype call, a halt, and a jump to itself. `B` because the
    /// assertion below is glyph identity against the `B` the BIOS printed on
    /// the line above, and no font table is consulted at either end.
    #[cfg(all(feature = "machine-pc-at", feature = "fw-pcbios"))]
    fn bootable_diskette() -> Vec<u8> {
        let mut image = alloc::vec![0u8; 1_474_560];
        image[..11].copy_from_slice(&[
            0xb4, 0x0e, // mov ah, 0x0e   -- teletype output
            0xb0, 0x42, // mov al, 'B'
            0xb7, 0x00, // mov bh, 0      -- page zero
            0xcd, 0x10, // int 0x10
            0xf4, // hlt
            0xeb, 0xfe, // jmp $
        ]);
        image[510..512].copy_from_slice(&[0x55, 0xaa]);
        image
    }

    /// The second bay: rsemu's own BIOS **and** a diskette the visitor brought,
    /// bound to two slots by one boot.
    ///
    /// This is the whole point of [`rsemu_stage_media`]. [`rsemu_boot`] binds
    /// one image to one slot, so before this existed a page could hand `pc-at`
    /// either a firmware or a disk and never both — and since the firmware is
    /// the one rsemu ships, "both" is the only interesting case.
    ///
    /// The assertion is the *negative* of the last one in
    /// [`a_pc_at_posts_on_its_own_bios_through_the_abi`]. That test asserts the
    /// third line's first glyph is not the `B` of `Booting.`, because with
    /// every drive empty the BIOS prints `No bootable device.` there. Here it
    /// is that `B`, because `INT 19h` found a `0x55 0xAA` on the diskette,
    /// loaded the sector at `0000:7c00`, jumped to it, and the sector's own
    /// `INT 10h` printed one.
    #[cfg(all(feature = "machine-pc-at", feature = "fw-pcbios"))]
    #[test]
    fn a_staged_diskette_boots_under_the_bios_this_module_carries() {
        let _one = ONE_MACHINE.lock();
        let machine = machine_index("pc-at");
        let floppy = machine_media_index(machine, "floppy");

        let image = bootable_diskette();
        rsemu_input_reserve(image.len());
        with_state(|state| state.input.copy_from_slice(&image));
        assert_eq!(rsemu_stage_media(machine, floppy, image.len()), 1, "{}", {
            rsemu_error();
            out()
        });

        // The BIOS still comes from the module, in its own slot, with nothing
        // uploaded for it — `rsemu_boot_builtin`'s contract, unchanged.
        assert_eq!(rsemu_boot_builtin(machine, 0), 1, "{}", {
            rsemu_error();
            out()
        });
        for _ in 0..240 {
            rsemu_run_frame();
        }
        assert_eq!(
            text_cell(0, 2),
            text_cell(0, 1),
            "the third line does not start with the B of Booting. — the \
             diskette was not booted"
        );
        rsemu_clear_media();
        rsemu_shutdown();
    }

    /// A staged slot is checked against the machine that is actually booting.
    ///
    /// Ignoring one silently would be exactly the failure this call exists to
    /// remove: a page that staged a diskette and got a machine with no drive
    /// would see a correct-looking boot and an empty one.
    #[cfg(all(feature = "machine-pc-at", feature = "fw-pcbios"))]
    #[test]
    fn a_staged_slot_that_does_not_fit_the_boot_is_refused() {
        let _one = ONE_MACHINE.lock();
        let machine = machine_index("pc-at");

        // A slot index this machine has not got.
        assert_eq!(rsemu_stage_media(machine, 99, 0), 0);
        assert!(rsemu_error() > 0);

        // And the slot the boot is about to bind itself: either image would end
        // as a drive quietly not holding what the page put in it.
        let bios = machine_media_index(machine, "bios");
        rsemu_input_reserve(4);
        assert_eq!(rsemu_stage_media(machine, bios, 4), 1);
        assert_eq!(rsemu_boot_builtin(machine, 0), 0);
        assert!(rsemu_error() > 0);

        rsemu_clear_media();
        assert_eq!(rsemu_boot_builtin(machine, 0), 1, "{}", {
            rsemu_error();
            out()
        });
        rsemu_shutdown();
    }

    /// A PC takes keys; a monitor takes characters. They are not one seam.
    ///
    /// `pc.kbc`'s port carries set-2 scan codes, so [`rsemu_key`] reaches it and
    /// [`rsemu_console_write`] has nowhere to go — which is why this board
    /// answers `0` to [`rsemu_has_console`] and `1` to [`rsemu_has_keyboard`],
    /// and why the page shows a picture rather than a terminal pane.
    #[cfg(all(feature = "machine-pc-at", feature = "fw-pcbios"))]
    #[test]
    fn a_pc_takes_keys_rather_than_characters() {
        let _one = ONE_MACHINE.lock();
        let machine = machine_index("pc-at");
        assert_eq!(rsemu_boot_builtin(machine, 0), 1);
        assert_eq!(rsemu_has_keyboard(), 1);
        assert_eq!(rsemu_has_console(), 0, "a scan-code port is not a console");

        // A key this keyboard has, and one it has not. The second puts no bytes
        // on the wire at all rather than guessing at a scan code.
        assert_eq!(rsemu_key(u32::from(b'a'), 1), 1);
        assert_eq!(rsemu_key(u32::from(b'a'), 0), 1);
        assert_eq!(rsemu_key(0xdead_beef, 1), 0);

        // And those transitions really crossed into the machine: the same run
        // with nothing typed at it reaches a different state.
        for _ in 0..8 {
            rsemu_run_frame();
        }
        let typed = rsemu_state_hash();

        assert_eq!(rsemu_boot_builtin(machine, 0), 1);
        for _ in 0..8 {
            rsemu_run_frame();
        }
        assert_ne!(typed, rsemu_state_hash(), "the key press changed nothing");
        rsemu_shutdown();
    }

    /// A machine with no keyboard says so rather than accepting a key.
    #[cfg(feature = "machine-beneater")]
    #[test]
    fn a_board_with_a_character_console_has_no_keyboard() {
        let _one = ONE_MACHINE.lock();
        let machine = machine_index("beneater-6502");
        assert_eq!(rsemu_boot_builtin(machine, 0), 1);
        assert_eq!(rsemu_has_console(), 1);
        assert_eq!(rsemu_has_keyboard(), 0);
        assert_eq!(rsemu_key(u32::from(b'a'), 1), 0);
        rsemu_shutdown();
    }

    /// Every call that needs a machine says so rather than misbehaving.
    #[test]
    fn calls_without_a_machine_report_rather_than_panic() {
        let _one = ONE_MACHINE.lock();
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
        let _one = ONE_MACHINE.lock();
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
        let _one = ONE_MACHINE.lock();

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
        assert_eq!(rsemu_has_pad(), u32::from(cfg!(feature = "dev-nes-io")));
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
            let hosts = with_state(|state| state.hosts.clone()).expect("a machine is booted");
            let pad = pads::names(&hosts)
                .first()
                .and_then(|n| pads::get(&hosts, n).ok().flatten())
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

    /// The Game Boy through the ABI: a picture, and a button that lands on the
    /// console's own pins rather than only in the module's record of it.
    ///
    /// The translation is the point. [`rsemu_set_buttons`] speaks one order for
    /// every machine and no two of these consoles agree on one, so "A" has to
    /// arrive as the DMG's bit 4 and not as `$80` — and the only way to know is
    /// to read it back off the pad the machine actually opened.
    #[cfg(feature = "machine-gameboy")]
    #[test]
    fn a_game_boy_draws_and_takes_a_button_through_the_abi() {
        let _one = ONE_MACHINE.lock();

        // `LD A,$91 / LDH ($40),A` — the LCD on with the background enabled,
        // then a park. Enough for the controller to complete frames; the
        // picture a program draws is `host::display::gb`'s own test.
        let image = crate::dev::gb::cart::synthetic_image(
            2,
            0x00,
            0x00,
            &[0x3e, 0x91, 0xe0, 0x40, 0x18, 0xfe],
        );
        let index = machine_index("gameboy");
        rsemu_input_reserve(image.len());
        with_state(|state| state.input.copy_from_slice(&image));
        assert_eq!(rsemu_boot(index, image.len()), 1, "the Game Boy boots");

        assert_eq!(rsemu_has_video(), 1, "the DMG has a picture now");
        assert_eq!(rsemu_frame_width(), 160);
        assert_eq!(rsemu_frame_height(), 144);
        assert_eq!(rsemu_frame_len(), 160 * 144 * 4, "four bytes a pixel");
        assert_eq!(rsemu_frame_period_ns(), 16_742_706);
        assert!(rsemu_run_frames(4) > 0, "four frames produced no picture");

        assert_eq!(rsemu_has_pad(), 1, "and eight buttons to press");
        {
            use crate::dev::gb::joypad::{Button, DEFAULT_PAD_PORT, pads};

            let hosts = with_state(|state| state.hosts.clone()).expect("a machine is booted");
            let pad = pads::get(&hosts, DEFAULT_PAD_PORT)
                .ok()
                .flatten()
                .expect("the machine opened its pad port");
            // `0x80 | 0x08` is A and Up in the ABI's order; on a DMG that is
            // bit 4 and bit 2.
            rsemu_set_buttons(0, 0x88);
            assert_eq!(
                pad.buttons(),
                (1 << Button::A.bit()) | (1 << Button::Up.bit()),
                "A and Up did not arrive in the joypad's own bit order"
            );
            // Port 1 is nobody: a Game Boy has one matrix.
            rsemu_set_buttons(1, 0xff);
            assert_eq!(
                pad.buttons(),
                (1 << Button::A.bit()) | (1 << Button::Up.bit()),
                "a second controller pressed buttons on the only one"
            );
            rsemu_set_buttons(0, 0);
            assert_eq!(pad.buttons(), 0);
        }

        rsemu_shutdown();
    }

    /// The Master System through the ABI, and the two things it does that no
    /// other machine here does: a second controller port, and a Start that is
    /// the console's Pause switch rather than a line on the pad.
    #[cfg(feature = "machine-sms")]
    #[test]
    fn a_master_system_draws_and_takes_two_pads_through_the_abi() {
        let _one = ONE_MACHINE.lock();

        // `DI / LD A,$40 / OUT ($BF),A / LD A,$81 / OUT ($BF),A / JR $` —
        // register 1 with the display enabled, then a park.
        let mut image = alloc::vec![0xffu8; 0x4000 * 2];
        image[..11].copy_from_slice(&[
            0xf3, 0x3e, 0x40, 0xd3, 0xbf, 0x3e, 0x81, 0xd3, 0xbf, 0x18, 0xfe,
        ]);
        let index = machine_index("sms-ntsc");
        rsemu_input_reserve(image.len());
        with_state(|state| state.input.copy_from_slice(&image));
        assert_eq!(rsemu_boot(index, image.len()), 1, "the Master System boots");

        assert_eq!(rsemu_has_video(), 1);
        assert_eq!(rsemu_frame_width(), 256);
        assert_eq!(rsemu_frame_height(), 192, "mode 4's default height");
        assert!(rsemu_run_frames(4) > 0, "four frames produced no picture");

        assert_eq!(rsemu_has_pad(), 1);
        {
            use crate::dev::sms::io::{Button, DEFAULT_PAD_PORT, pads};

            let hosts = with_state(|state| state.hosts.clone()).expect("a machine is booted");
            let pads = pads::get(&hosts, DEFAULT_PAD_PORT)
                .ok()
                .flatten()
                .expect("the machine opened its pad ports");
            // A and Left in the ABI's order become button 1 and Left.
            rsemu_set_buttons(0, 0x82);
            assert_eq!(
                pads.buttons(0),
                (1 << Button::One.bit()) | (1 << Button::Left.bit())
            );
            // And port 1 is a real second pad, which is what a two-player
            // machine means.
            rsemu_set_buttons(1, 0x40);
            assert_eq!(pads.buttons(1), 1 << Button::Two.bit());
            assert_eq!(
                pads.buttons(0),
                (1 << Button::One.bit()) | (1 << Button::Left.bit()),
                "port 1 wrote over port 0"
            );
            rsemu_set_buttons(0, 0);
            rsemu_set_buttons(1, 0);
            assert_eq!((pads.buttons(0), pads.buttons(1)), (0, 0));
        }

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
        let _one = ONE_MACHINE.lock();

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
