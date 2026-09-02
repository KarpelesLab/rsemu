//! The C ABI itself: every type, constant and entry point C can see.
//!
//! **This file is the ABI.** `include/rsemu.h` is generated from it by
//! [`header`](super::header) and a test compares the two, so a signature that
//! changes here and nowhere else fails `cargo test` rather than a caller's
//! program. Nothing exported to C may live in another file — a test asserts
//! that too, because "generated from one file" is only a guarantee while the
//! one file is the whole of it.
//!
//! Read [the module docs](super) first for the conventions; this file assumes
//! them.

use alloc::string::String;
use alloc::vec::Vec;
use core::ffi::c_char;

use crate::core::device::ResetKind;
use crate::core::space::MemAttrs;
use crate::ffi::common::{
    Body, Config, Running, cstr, cstr_opt, guard, guard_with, insert, lookup, out_write, remove,
    set_error, slice, slice_mut, with_config, with_machine,
};
use crate::machine::Machine;

// ---------------------------------------------------------------------------
// Types and constants
// ---------------------------------------------------------------------------

/// A live object owned by rsemu: a configuration, or a machine.
///
/// Deliberately an integer and not a pointer. `0` is never a valid handle, ids
/// are never reused, and no value a caller invents is ever dereferenced — see
/// the module docs for why this ABI departs from the siblings here.
pub type RsemuHandle = u64;

/// The value of a handle that names nothing.
pub const RSEMU_INVALID_HANDLE: RsemuHandle = 0;

/// The revision of this ABI. Bumped whenever a signature or a numeric value
/// changes meaning; compare it against [`rsemu_abi_version`] at startup to
/// catch a header that does not match the library it is linked against.
pub const RSEMU_ABI_VERSION: u32 = 1;

/// Power-on reset: every register returns to its documented reset value.
pub const RSEMU_RESET_COLD: i32 = 0;

/// A reset-line pulse: battery-backed and always-on state survives.
pub const RSEMU_RESET_WARM: i32 = 1;

/// A bus-level reset, affecting only the devices on that bus.
pub const RSEMU_RESET_BUS: i32 = 2;

/// The result of a C ABI call. `0` is success; every failure is negative.
///
/// The numeric values are part of the ABI and do not change. The first three
/// match `purecrypto` and `kataan` exactly; the rest are one code per
/// [`Error`](crate::Error) variant, plus one per
/// [`BusError`](crate::core::BusError) variant so that a caller can retry a
/// busy access without matching on a message.
///
/// The code says *which kind* of failure; [`rsemu_last_error`] says which one,
/// in the words rsemu would have printed to a terminal.
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RsemuStatus {
    /// The call succeeded.
    Ok = 0,
    /// A pointer argument was NULL where a value was required.
    NullPointer = -1,
    /// The output buffer was too small; `*out_len` holds the length needed.
    BufferTooSmall = -2,
    /// An argument was out of range, or a string was not UTF-8.
    InvalidInput = -3,
    /// The handle names nothing, or names the other kind of object. A double
    /// free and a use-after-free both arrive here.
    BadHandle = -4,
    /// A machine description could not be parsed, resolved or validated. The
    /// message carries `file:line:col` and a caret.
    Config = -5,
    /// The description names a device class this build does not contain —
    /// usually a Cargo feature that is off rather than a typo.
    UnknownClass = -6,
    /// A property was missing, of the wrong type, or out of range.
    Property = -7,
    /// A snapshot could not be written or restored.
    State = -8,
    /// A translation block was malformed. Always an rsemu bug.
    Ir = -9,
    /// The operation is not implemented in this build yet.
    Unimplemented = -10,
    /// Nothing is mapped at that guest address.
    BusUnassigned = -11,
    /// The access width or alignment is not permitted there.
    BusBadAccess = -12,
    /// Something is mapped there and does not permit this access.
    BusProtected = -13,
    /// The target was busy; the access may be retried.
    BusRetry = -14,
    /// A panic was caught at the boundary. A machine that returns this is
    /// poisoned: only [`rsemu_last_error`] and [`rsemu_free`] still work on it.
    Panic = -100,
}

impl RsemuStatus {
    /// The code for `error`, losing nothing a code can carry.
    ///
    /// Deliberately without a catch-all arm. [`Error`](crate::Error) and
    /// [`BusError`](crate::core::BusError) are `#[non_exhaustive]` to the
    /// world and exhaustive in here, so a variant added later stops the build
    /// on this line — which is where the question "what does C call this?"
    /// belongs. A `_` arm would answer it silently and wrongly.
    pub(super) fn of(error: &crate::Error) -> RsemuStatus {
        use crate::core::BusError;
        use crate::core::Error;
        match error {
            Error::Config { .. } => RsemuStatus::Config,
            Error::UnknownClass(_) => RsemuStatus::UnknownClass,
            Error::Property(_) => RsemuStatus::Property,
            Error::State(_) => RsemuStatus::State,
            Error::Ir(_) => RsemuStatus::Ir,
            Error::Unimplemented(_) => RsemuStatus::Unimplemented,
            Error::Bus(BusError::Unassigned) => RsemuStatus::BusUnassigned,
            Error::Bus(BusError::BadAccess) => RsemuStatus::BusBadAccess,
            Error::Bus(BusError::Protected) => RsemuStatus::BusProtected,
            Error::Bus(BusError::Retry) => RsemuStatus::BusRetry,
        }
    }

    /// The sentence `rsemu_strerror` hands back, NUL-terminated so it can be
    /// returned as a `const char *` without an allocation.
    fn text(self) -> &'static str {
        match self {
            RsemuStatus::Ok => "ok\0",
            RsemuStatus::NullPointer => "null pointer argument\0",
            RsemuStatus::BufferTooSmall => "output buffer too small\0",
            RsemuStatus::InvalidInput => "invalid argument\0",
            RsemuStatus::BadHandle => "no such handle, or the wrong kind of handle\0",
            RsemuStatus::Config => "machine description error\0",
            RsemuStatus::UnknownClass => "unknown device class (is its feature enabled?)\0",
            RsemuStatus::Property => "device property error\0",
            RsemuStatus::State => "snapshot error\0",
            RsemuStatus::Ir => "malformed IR\0",
            RsemuStatus::Unimplemented => "not implemented in this build\0",
            RsemuStatus::BusUnassigned => "nothing mapped at this guest address\0",
            RsemuStatus::BusBadAccess => "access width or alignment not permitted\0",
            RsemuStatus::BusProtected => "the mapping does not permit this access\0",
            RsemuStatus::BusRetry => "target busy, retry\0",
            RsemuStatus::Panic => "a panic was caught at the C ABI boundary\0",
        }
    }

    /// Every code, so `rsemu_strerror` can answer without a `transmute` and a
    /// test can assert the header lists exactly these.
    pub(super) const ALL: [RsemuStatus; 16] = [
        RsemuStatus::Ok,
        RsemuStatus::NullPointer,
        RsemuStatus::BufferTooSmall,
        RsemuStatus::InvalidInput,
        RsemuStatus::BadHandle,
        RsemuStatus::Config,
        RsemuStatus::UnknownClass,
        RsemuStatus::Property,
        RsemuStatus::State,
        RsemuStatus::Ir,
        RsemuStatus::Unimplemented,
        RsemuStatus::BusUnassigned,
        RsemuStatus::BusBadAccess,
        RsemuStatus::BusProtected,
        RsemuStatus::BusRetry,
        RsemuStatus::Panic,
    ];
}

// ---------------------------------------------------------------------------
// Build information
// ---------------------------------------------------------------------------

/// Returns the ABI revision this library implements.
///
/// Compare it with `RSEMU_ABI_VERSION` from the header. A mismatch means the
/// header and the library came from different builds, and nothing below this
/// line can be trusted.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_abi_version() -> u32 {
    RSEMU_ABI_VERSION
}

/// Returns a NUL-terminated description of `status`, in static storage.
///
/// The one string rsemu owns that a caller may hold on to. It must not be
/// freed, and it stays valid for the life of the process. An unrecognised code
/// answers "unknown status" rather than NULL, so a caller can print it
/// unconditionally.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_strerror(status: i32) -> *const c_char {
    let text = RsemuStatus::ALL
        .iter()
        .find(|s| **s as i32 == status)
        .map_or("unknown status\0", |s| s.text());
    text.as_ptr().cast::<c_char>()
}

/// Writes a one-line description of how this library was configured.
///
/// A machine is a feature set, so "which rsemu is this?" has a build-specific
/// answer and this is it: the version and the features that were enabled.
///
/// # Safety
///
/// `out_len` must point at a writable `size_t` holding the capacity of `out`,
/// and `out` must be writable for that many bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_build_info(out: *mut c_char, out_len: *mut usize) -> RsemuStatus {
    guard(|| {
        let info = crate::build_info();
        // SAFETY: forwarded verbatim to the shared helper, whose contract is
        // the one this function documents; the caller upholds it.
        unsafe { out_write(info.as_bytes(), out.cast::<u8>(), out_len) }
    })
}

// ---------------------------------------------------------------------------
// The catalog
// ---------------------------------------------------------------------------

/// Returns how many machines this build ships ready to run.
///
/// Zero is a correct answer for a library built with `--features ffi` alone:
/// a machine is a feature set. A description passed to
/// [`rsemu_config_machine`] does not have to be one of these.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_catalog_count() -> u32 {
    guard_with(0, || crate::machine::catalog::machines().len() as u32)
}

/// Writes the name of catalog machine `index` — what `rsemu run <name>` takes.
///
/// # Safety
///
/// As `rsemu_build_info`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_catalog_name(
    index: u32,
    out: *mut c_char,
    out_len: *mut usize,
) -> RsemuStatus {
    guard(|| {
        let Some(entry) = catalog_entry(index) else {
            return RsemuStatus::InvalidInput;
        };
        // SAFETY: forwarded verbatim to the shared helper, whose contract is
        // the one this function documents; the caller upholds it.
        unsafe { out_write(entry.name.as_bytes(), out.cast::<u8>(), out_len) }
    })
}

/// Writes the name of media slot `slot` of catalog machine `index`.
///
/// Slots are numbered from zero and an out-of-range `slot` is
/// `RSEMU_INVALID_INPUT`, so counting them is a loop rather than another call.
/// These are the names [`rsemu_config_media`] binds, and they are what
/// `--cart`, `--rom`, `--bios` and `--disk` spell on the command line.
///
/// # Safety
///
/// As `rsemu_build_info`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_catalog_media(
    index: u32,
    slot: u32,
    out: *mut c_char,
    out_len: *mut usize,
) -> RsemuStatus {
    guard(|| {
        let Some(entry) = catalog_entry(index) else {
            return RsemuStatus::InvalidInput;
        };
        let Some(name) = entry.media.get(slot as usize) else {
            return RsemuStatus::InvalidInput;
        };
        // SAFETY: forwarded verbatim to the shared helper, whose contract is
        // the one this function documents; the caller upholds it.
        unsafe { out_write(name.as_bytes(), out.cast::<u8>(), out_len) }
    })
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Creates an empty machine configuration and returns its handle.
///
/// `RSEMU_INVALID_HANDLE` on failure, which at this point can only mean the
/// allocator refused. Free it with [`rsemu_free`] once
/// [`rsemu_machine_new`] has built the machine; the machine does not borrow it.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_config_new() -> RsemuHandle {
    guard_with(RSEMU_INVALID_HANDLE, || {
        insert(Body::Config(Config::default()))
    })
}

/// Says which machine to build.
///
/// `name` is a catalog name when `source` is NULL — `rsemu_catalog_name` lists
/// them — and otherwise the name diagnostics should use for the description,
/// the way a filename appears in `file:line:col`.
///
/// `source` is the text of a `.machine` description, as `(pointer, length)`
/// and not NUL-terminated. It is copied, so the caller may free it as soon as
/// this returns. rsemu never opens a file on the caller's behalf: an embedder
/// that wants `include` resolution or a search path owns that policy, and a C
/// ABI that took paths would have to invent an encoding rule for them.
///
/// # Safety
///
/// `name` must be a NUL-terminated UTF-8 string, and `source` must be readable
/// for `source_len` bytes or NULL with a zero length.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_config_machine(
    config: RsemuHandle,
    name: *const c_char,
    source: *const u8,
    source_len: usize,
) -> RsemuStatus {
    guard(|| {
        // SAFETY: the caller guarantees `name` is a NUL-terminated string that
        // stays valid for the call, which is `cstr`'s documented contract. Decoded
        // outside `with_config` so no lock is held across a borrow of caller
        // memory.
        let name = match unsafe { cstr(name) } {
            Ok(name) => String::from(name),
            Err(status) => return status,
        };
        // A NULL `source` is the catalog lookup, so it is the one place a null
        // pointer is an instruction rather than a mistake. A non-zero length with
        // it is a mistake, and saying so beats silently ignoring the length.
        let source = if source.is_null() {
            if source_len != 0 {
                return RsemuStatus::NullPointer;
            }
            None
        } else {
            // SAFETY: `source` is non-null here and the caller guarantees it is
            // readable for `source_len` bytes, which is `slice`'s contract.
            let Some(bytes) = (unsafe { slice(source, source_len) }) else {
                return RsemuStatus::NullPointer;
            };
            match core::str::from_utf8(bytes) {
                Ok(text) => Some(String::from(text)),
                Err(_) => return RsemuStatus::InvalidInput,
            }
        };
        with_config(config, move |cfg| {
            cfg.name = name;
            cfg.source = source;
            RsemuStatus::Ok
        })
    })
}

/// Binds `len` bytes to the media slot `slot`, as `--cart`, `--rom`, `--bios`
/// and `--disk` do.
///
/// The bytes are copied, so the caller may free them as soon as this returns.
/// Binding the same slot twice keeps the second image, the way a repeated
/// command-line option does.
///
/// # Safety
///
/// `slot` must be a NUL-terminated UTF-8 string, and `bytes` must be readable
/// for `len` bytes or NULL with a zero length.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_config_media(
    config: RsemuHandle,
    slot: *const c_char,
    bytes: *const u8,
    len: usize,
) -> RsemuStatus {
    guard(|| {
        // SAFETY: the caller guarantees a NUL-terminated `slot` and `len` readable
        // bytes at `bytes`; both helpers document that contract. Copied here, out
        // of the lock, so nothing borrows caller memory while a machine is locked.
        let decoded = unsafe { (cstr(slot), slice(bytes, len)) };
        let slot = match decoded.0 {
            Ok(slot) => String::from(slot),
            Err(status) => return status,
        };
        let Some(image) = decoded.1 else {
            return RsemuStatus::NullPointer;
        };
        let image = image.to_vec();
        with_config(config, move |cfg| {
            cfg.media.retain(|(name, _)| *name != slot);
            cfg.media.push((slot, image));
            RsemuStatus::Ok
        })
    })
}

/// Overrides a `param` in the description, as `rsemu run … -p ram=8M` does.
///
/// Both strings are copied. A parameter the description does not declare is
/// reported when the machine is built, not here.
///
/// # Safety
///
/// `key` and `value` must be NUL-terminated UTF-8 strings.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_config_param(
    config: RsemuHandle,
    key: *const c_char,
    value: *const c_char,
) -> RsemuStatus {
    guard(|| {
        // SAFETY: the caller guarantees both pointers are NUL-terminated strings
        // that stay valid for the call; `cstr` documents that contract.
        let decoded = unsafe { (cstr(key), cstr(value)) };
        let key = match decoded.0 {
            Ok(key) => String::from(key),
            Err(status) => return status,
        };
        let value = match decoded.1 {
            Ok(value) => String::from(value),
            Err(status) => return status,
        };
        with_config(config, move |cfg| {
            cfg.params.retain(|(name, _)| *name != key);
            cfg.params.push((key, value));
            RsemuStatus::Ok
        })
    })
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/// Builds the machine `config` describes and writes its handle to `out`.
///
/// The configuration is not consumed: it stays valid, can be adjusted and
/// built again, and must be freed with [`rsemu_free`] like any other handle.
/// On failure `*out` is set to `RSEMU_INVALID_HANDLE` and the reason is on the
/// **configuration's** handle, so `rsemu_last_error(config, …)` is where a
/// caller looks — there is no machine to carry it.
///
/// A media slot the caller did not bind falls back to whatever rsemu itself
/// ships for it, which is the courtesy `rsemu run` extends: an `apple1` with no
/// `rom` gets rsemu's own monitor, a `pc-at` with no `bios` gets the firmware
/// this repository assembles. The fallback applies only to catalog machines,
/// because it is keyed on the machine's name.
///
/// # Safety
///
/// `out` must point at a writable `rsemu_handle`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_new(
    config: RsemuHandle,
    out: *mut RsemuHandle,
) -> RsemuStatus {
    if out.is_null() {
        return RsemuStatus::NullPointer;
    }
    // SAFETY: the caller guarantees `out` points at a writable `rsemu_handle`;
    // the null case is handled above. Written before anything can fail so the
    // caller never reads an uninitialised handle out of a failed call.
    unsafe { *out = RSEMU_INVALID_HANDLE };

    guard(|| {
        // The configuration is copied out and its lock released before the
        // build runs: `build` constructs devices that take locks of their own,
        // and holding the outermost lock across that is exactly the pattern the
        // rank ladder exists to reject.
        let Some(slot) = lookup(config) else {
            return RsemuStatus::BadHandle;
        };
        let copy = {
            let slot = slot.lock();
            let Body::Config(cfg) = &slot.body else {
                return RsemuStatus::BadHandle;
            };
            Config {
                name: cfg.name.clone(),
                source: cfg.source.clone(),
                media: cfg.media.clone(),
                params: cfg.params.clone(),
            }
        };
        match build(&copy) {
            Ok(machine) => {
                let handle = insert(Body::Machine(alloc::boxed::Box::new(Running {
                    machine,
                    poisoned: false,
                })));
                // SAFETY: `out` was null-checked above and the caller
                // guarantees it stays writable for the call.
                unsafe { *out = handle };
                set_error(config, "");
                RsemuStatus::Ok
            }
            Err(e) => {
                let status = RsemuStatus::of(&e);
                set_error(config, e);
                status
            }
        }
    })
}

/// Frees the object `handle` names, whichever kind it is.
///
/// `RSEMU_BAD_HANDLE` if it names nothing — which is what a double free, a
/// free of a handle that was never created, and a free of an uninitialised
/// variable all look like. None of those is undefined behaviour here: the
/// handle is an id, so a wrong one is looked up and not found rather than
/// dereferenced.
///
/// Freeing a machine another thread is still running is safe. The handle stops
/// resolving immediately; the machine itself is dropped when that call
/// returns.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_free(handle: RsemuHandle) -> RsemuStatus {
    guard(|| {
        if remove(handle) {
            RsemuStatus::Ok
        } else {
            RsemuStatus::BadHandle
        }
    })
}

/// Writes the message explaining `handle`'s last failure.
///
/// The text `rsemu` would have printed: for a description error that is
/// `file:line:col`, the message and a caret under the offending token. UTF-8,
/// not NUL-terminated; `*out_len` carries the length. Empty when the last call
/// succeeded, and also for the argument errors a status code already explains
/// on its own — a NULL, a bad handle, an index out of range. There is nothing
/// to add to those that `rsemu_strerror` does not already say.
///
/// Works on a poisoned machine, and on the configuration whose
/// [`rsemu_machine_new`] failed.
///
/// # Safety
///
/// As `rsemu_build_info`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_last_error(
    handle: RsemuHandle,
    out: *mut c_char,
    out_len: *mut usize,
) -> RsemuStatus {
    guard(|| {
        let Some(slot) = lookup(handle) else {
            return RsemuStatus::BadHandle;
        };
        let message = slot.lock().error.clone();
        // SAFETY: forwarded verbatim to the shared helper, whose contract is
        // the one this function documents; the caller upholds it.
        unsafe { out_write(message.as_bytes(), out.cast::<u8>(), out_len) }
    })
}

// ---------------------------------------------------------------------------
// Running
// ---------------------------------------------------------------------------

/// Advances the machine by `nanos` nanoseconds of **virtual** time.
///
/// Virtual, not wall-clock: this is the unit the whole determinism story is
/// told in, and two runs of the same machine over the same span produce the
/// same [`rsemu_machine_state_hash`] on any host. How long the call takes is a
/// property of the host; how far the machine moves is not.
///
/// Additive, so driving a machine in slices and driving it in one call reach
/// the same place. That is what lets an embedder interleave its own work
/// without changing the result.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_run_ns(machine: RsemuHandle, nanos: u64) -> RsemuStatus {
    with_machine(machine, |m| {
        m.run_for(crate::core::clock::GlobalTime::from_nanos(nanos))
    })
}

/// Writes how far the machine has run, in nanoseconds of virtual time.
///
/// # Safety
///
/// `out` must point at a writable `uint64_t`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_now_ns(machine: RsemuHandle, out: *mut u64) -> RsemuStatus {
    if out.is_null() {
        return RsemuStatus::NullPointer;
    }
    with_machine(machine, |m| {
        let now = m.now().as_nanos();
        // SAFETY: `out` was null-checked above and the caller guarantees it
        // points at a writable `uint64_t` for the duration of the call.
        unsafe { *out = now };
        Ok(())
    })
}

/// Resets the machine. `kind` is one of the `RSEMU_RESET_*` constants.
#[unsafe(no_mangle)]
pub extern "C" fn rsemu_machine_reset(machine: RsemuHandle, kind: i32) -> RsemuStatus {
    let kind = match kind {
        RSEMU_RESET_COLD => ResetKind::Cold,
        RSEMU_RESET_WARM => ResetKind::Warm,
        RSEMU_RESET_BUS => ResetKind::Bus,
        _ => return RsemuStatus::InvalidInput,
    };
    with_machine(machine, |m| {
        m.reset(kind);
        Ok(())
    })
}

// ---------------------------------------------------------------------------
// Guest memory
// ---------------------------------------------------------------------------

/// Writes how many address spaces the machine has.
///
/// # Safety
///
/// `out` must point at a writable `uint32_t`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_space_count(
    machine: RsemuHandle,
    out: *mut u32,
) -> RsemuStatus {
    if out.is_null() {
        return RsemuStatus::NullPointer;
    }
    with_machine(machine, |m| {
        let count = m.spaces().len() as u32;
        // SAFETY: `out` was null-checked above and the caller guarantees it
        // points at a writable `uint32_t` for the duration of the call.
        unsafe { *out = count };
        Ok(())
    })
}

/// Writes the name of address space `index`, as the description spells it.
///
/// These are the names [`rsemu_machine_read`] and [`rsemu_machine_write`] take.
///
/// # Safety
///
/// As `rsemu_build_info`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_space_name(
    machine: RsemuHandle,
    index: u32,
    out: *mut c_char,
    out_len: *mut usize,
) -> RsemuStatus {
    let mut status = RsemuStatus::Ok;
    let outcome = with_machine(machine, |m| {
        let Some(entry) = m.spaces().get(index as usize) else {
            status = RsemuStatus::InvalidInput;
            return Ok(());
        };
        // SAFETY: forwarded verbatim to the shared helper, whose contract is
        // the one this function documents; the caller upholds it.
        status = unsafe { out_write(entry.name().as_bytes(), out.cast::<u8>(), out_len) };
        Ok(())
    });
    if outcome == RsemuStatus::Ok {
        status
    } else {
        outcome
    }
}

/// Copies `len` bytes of guest memory at `addr` into `out`.
///
/// `space` names the address space; NULL means the machine's first one, which
/// is the CPU bus on every machine rsemu ships.
///
/// The access is a **debug** access: it must not pop a FIFO, clear a status
/// bit or advance a pointer, exactly as a gdb read must not. Reading through
/// this ABI therefore cannot change what the guest goes on to do.
///
/// The bytes are copied. rsemu does not hand out pointers into guest memory —
/// guest RAM is addressed by byte offset so that it can live in a
/// `SharedArrayBuffer`, and a pointer into it would be invalidated by the next
/// remap or snapshot restore anyway.
///
/// # Safety
///
/// `space` must be NULL or a NUL-terminated UTF-8 string, and `out` must be
/// writable for `len` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_read(
    machine: RsemuHandle,
    space: *const c_char,
    addr: u64,
    out: *mut u8,
    len: usize,
) -> RsemuStatus {
    guard(|| {
        // SAFETY: the caller guarantees `space` is null or NUL-terminated;
        // `cstr_opt` documents that contract.
        let space = match unsafe { cstr_opt(space) } {
            Ok(space) => space,
            Err(status) => return status,
        };
        // SAFETY: the caller guarantees `out` is writable for `len` bytes and is
        // not aliased by guest memory, which rsemu owns.
        let Some(dst) = (unsafe { slice_mut(out, len) }) else {
            return RsemuStatus::NullPointer;
        };
        let mut status = RsemuStatus::Ok;
        let outcome = with_machine(machine, |m| {
            let Some(target) = space_of(m, space) else {
                status = RsemuStatus::InvalidInput;
                return Ok(());
            };
            if let Err(e) = target.read_bytes(addr, dst, MemAttrs::DEBUG) {
                return Err(crate::Error::Bus(e));
            }
            Ok(())
        });
        if outcome == RsemuStatus::Ok {
            status
        } else {
            outcome
        }
    })
}

/// Copies `len` bytes from `bytes` into guest memory at `addr`.
///
/// The mirror of [`rsemu_machine_read`], on the same terms: `space` may be
/// NULL for the first space, and the access is a debug access, so writing a
/// device register through this ABI does not run the side effects a guest
/// store would. Use it to plant a program, patch a value or seed RAM before a
/// run — not to impersonate the guest.
///
/// # Safety
///
/// `space` must be NULL or a NUL-terminated UTF-8 string, and `bytes` must be
/// readable for `len` bytes or NULL with a zero length.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_write(
    machine: RsemuHandle,
    space: *const c_char,
    addr: u64,
    bytes: *const u8,
    len: usize,
) -> RsemuStatus {
    guard(|| {
        // SAFETY: the caller guarantees `space` is null or NUL-terminated;
        // `cstr_opt` documents that contract.
        let space = match unsafe { cstr_opt(space) } {
            Ok(space) => space,
            Err(status) => return status,
        };
        // SAFETY: the caller guarantees `bytes` is readable for `len` bytes; a
        // null pointer with a zero length is the empty write.
        let Some(src) = (unsafe { slice(bytes, len) }) else {
            return RsemuStatus::NullPointer;
        };
        let mut status = RsemuStatus::Ok;
        let outcome = with_machine(machine, |m| {
            let Some(target) = space_of(m, space) else {
                status = RsemuStatus::InvalidInput;
                return Ok(());
            };
            if let Err(e) = target.write_bytes(addr, src, MemAttrs::DEBUG) {
                return Err(crate::Error::Bus(e));
            }
            Ok(())
        });
        if outcome == RsemuStatus::Ok {
            status
        } else {
            outcome
        }
    })
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

/// Writes a snapshot of the whole machine.
///
/// Follows the buffer convention: call it once with a zero capacity to learn
/// the size, then again with a buffer that big. The snapshot is
/// self-describing and versioned; [`rsemu_machine_load`] restores it into a
/// machine built from the same description.
///
/// # Safety
///
/// `out_len` must point at a writable `size_t` holding the capacity of `out`,
/// and `out` must be writable for that many bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_save(
    machine: RsemuHandle,
    out: *mut u8,
    out_len: *mut usize,
) -> RsemuStatus {
    let mut status = RsemuStatus::Ok;
    let outcome = with_machine(machine, |m| {
        let bytes = m.save()?;
        // SAFETY: forwarded verbatim to the shared helper, whose contract is
        // the one this function documents; the caller upholds it.
        status = unsafe { out_write(&bytes, out, out_len) };
        Ok(())
    });
    if outcome == RsemuStatus::Ok {
        status
    } else {
        outcome
    }
}

/// Restores a snapshot taken by [`rsemu_machine_save`].
///
/// The machine must have been built from the same description: a snapshot
/// records the shape it was taken from and a mismatch is reported rather than
/// half-applied.
///
/// # Safety
///
/// `bytes` must be readable for `len` bytes, or NULL with a zero length.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_load(
    machine: RsemuHandle,
    bytes: *const u8,
    len: usize,
) -> RsemuStatus {
    guard(|| {
        // SAFETY: the caller guarantees `len` readable bytes at `bytes`; `slice`
        // documents that contract and handles the null case.
        let Some(src) = (unsafe { slice(bytes, len) }) else {
            return RsemuStatus::NullPointer;
        };
        with_machine(machine, |m| m.load(src))
    })
}

/// Writes the machine's deterministic state hash.
///
/// Two runs that took the same path produce the same value, on any host and
/// under any thread count in deterministic mode. This is the regression gate
/// an embedded rsemu is worth having for: run, hash, compare.
///
/// # Safety
///
/// `out` must point at a writable `uint64_t`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rsemu_machine_state_hash(
    machine: RsemuHandle,
    out: *mut u64,
) -> RsemuStatus {
    if out.is_null() {
        return RsemuStatus::NullPointer;
    }
    with_machine(machine, |m| {
        let hash = m.state_hash()?;
        // SAFETY: `out` was null-checked above and the caller guarantees it
        // points at a writable `uint64_t` for the duration of the call.
        unsafe { *out = hash };
        Ok(())
    })
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/// One catalog entry by index.
fn catalog_entry(index: u32) -> Option<&'static crate::machine::catalog::CatalogEntry> {
    crate::machine::catalog::machines()
        .get(index as usize)
        .copied()
}

/// The address space `name` selects, or the first one when `name` is absent.
fn space_of<'a>(
    machine: &'a Machine,
    name: Option<&str>,
) -> Option<&'a alloc::sync::Arc<crate::core::space::AddressSpace>> {
    match name {
        Some(name) => machine.space(name),
        None => machine.spaces().first().map(|entry| entry.space()),
    }
}

/// Turns a configuration into a machine.
fn build(config: &Config) -> crate::Result<Machine> {
    use crate::machine::catalog;

    let entry = catalog::machine(&config.name);
    let (name, source): (&str, &str) = match &config.source {
        Some(text) => (
            if config.name.is_empty() {
                "machine"
            } else {
                config.name.as_str()
            },
            text.as_str(),
        ),
        None => match entry {
            Some(entry) => (entry.name, entry.source),
            None => return Err(unknown_machine(&config.name)),
        },
    };

    let mut options = catalog::build_options()?;
    for (slot, bytes) in &config.media {
        options
            .realize
            .media
            .insert(slot.as_str(), bytes.as_slice());
    }
    // The same courtesy `rsemu run` extends: a slot the caller left empty on a
    // machine rsemu ships an image for gets that image. Only for a catalog
    // machine, because there is nothing but the name to key it on and guessing
    // from a caller's own description would be worse than not trying.
    if config.source.is_none()
        && let Some(entry) = entry
    {
        for slot in entry.media {
            if config.media.iter().any(|(bound, _)| bound == slot) {
                continue;
            }
            if let Some(image) = builtin_media(entry.name, slot) {
                options.realize.media.insert(*slot, image.as_slice());
            }
        }
    }
    for (key, value) in &config.params {
        options = options.with_param(key.clone(), value.clone());
    }
    crate::machine::build(name, source, &catalog::registry()?, &options)
}

/// The image rsemu itself ships for `slot` on `machine`, if it ships one.
///
/// Mirrors the CLI's `builtin_bios`/`builtin_rom`, and for the same reason: a
/// machine that demonstrates itself with no file from the user is the
/// difference between an emulator someone can try and one they cannot.
fn builtin_media(machine: &str, slot: &str) -> Option<Vec<u8>> {
    match (machine, slot) {
        #[cfg(all(feature = "fw-pcbios", feature = "machine-pc-at"))]
        ("pc-at", "bios") => Some(crate::fw::pcbios::image()),
        #[cfg(feature = "dev-apple1")]
        ("apple1", "rom") => Some(crate::dev::apple1::RSMON.to_vec()),
        #[cfg(feature = "dev-wdc")]
        ("beneater-6502", "rom") => Some(crate::dev::wdc::RSMON_IMAGE.to_vec()),
        _ => {
            let _ = (machine, slot);
            None
        }
    }
}

/// The error for a machine this build does not ship, listing what it does.
fn unknown_machine(name: &str) -> crate::Error {
    use alloc::string::ToString;

    let mut message = String::from("no machine named `");
    message.push_str(name);
    message.push_str("` in this build, and no description was given; it has ");
    let names: Vec<&str> = crate::machine::catalog::machines()
        .into_iter()
        .map(|m| m.name)
        .collect();
    if names.is_empty() {
        message.push_str("none (enable a `machine-*` feature)");
    } else {
        message.push_str(&names.join(", "));
    }
    crate::Error::Config {
        at: "catalog".to_string(),
        message,
    }
}
