//! The machinery every entry point in [`abi`](super::abi) shares: the panic
//! guard, the caller-owned-buffer convention, the argument decoders, and the
//! table that turns an [`RsemuHandle`] into an object.
//!
//! Nothing here is exported to C. It is a separate file so that
//! [`header`](super::header) can generate the header from
//! [`abi`](super::abi) *alone* and a test can assert that no `extern "C"`
//! function lives anywhere else — the invariant that makes "the header is
//! generated from one file" true rather than aspirational.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::core::sync::{Global, LockRank};
use crate::ffi::abi::{RsemuHandle, RsemuStatus};
use crate::machine::Machine;

// ---------------------------------------------------------------------------
// The panic guard
// ---------------------------------------------------------------------------

/// Runs `f`, turning a panic into [`RsemuStatus::Panic`] rather than an unwind
/// out of an `extern "C"` frame.
///
/// `purecrypto`'s `guard`, under its own name. `AssertUnwindSafe` is honest
/// here rather than a shrug: nothing this closure touches is shared with a
/// later call except a machine, and a machine a panic passed through is marked
/// poisoned by [`with_machine`] before the status is returned.
pub(super) fn guard(f: impl FnOnce() -> RsemuStatus) -> RsemuStatus {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or(RsemuStatus::Panic)
}

/// The same, for a call whose failure value is a sentinel rather than a status.
pub(super) fn guard_with<T>(sentinel: T, f: impl FnOnce() -> T) -> T {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or(sentinel)
}

// ---------------------------------------------------------------------------
// Argument decoding
// ---------------------------------------------------------------------------

/// Borrows `len` bytes at `ptr`.
///
/// `(NULL, 0)` is the empty slice, which is a legitimate media image and a
/// legitimate machine description; `(NULL, n > 0)` is a caller bug and is
/// reported rather than dereferenced.
///
/// # Safety
///
/// The caller guarantees `ptr` is readable for `len` bytes and unaliased by
/// anything rsemu writes for the duration of the call.
pub(super) unsafe fn slice<'a>(ptr: *const u8, len: usize) -> Option<&'a [u8]> {
    if len == 0 {
        return Some(&[]);
    }
    if ptr.is_null() {
        return None;
    }
    // SAFETY: the caller guarantees `len` readable bytes at `ptr`, and the
    // null case is handled above. The lifetime is the caller's to keep valid
    // for the duration of the call, which is the whole of `'a`'s use here.
    Some(unsafe { core::slice::from_raw_parts(ptr, len) })
}

/// Borrows `len` writable bytes at `ptr`, on the same terms as [`slice`].
///
/// # Safety
///
/// The caller guarantees `ptr` is writable for `len` bytes and is not aliased
/// by any other pointer this call touches.
pub(super) unsafe fn slice_mut<'a>(ptr: *mut u8, len: usize) -> Option<&'a mut [u8]> {
    if len == 0 {
        // SAFETY: a zero-length slice built on a dangling but aligned pointer
        // is the canonical empty slice; nothing reads or writes through it.
        return Some(unsafe {
            core::slice::from_raw_parts_mut(core::ptr::NonNull::<u8>::dangling().as_ptr(), 0)
        });
    }
    if ptr.is_null() {
        return None;
    }
    // SAFETY: the caller guarantees `len` writable, unaliased bytes at `ptr`;
    // the null case is handled above.
    Some(unsafe { core::slice::from_raw_parts_mut(ptr, len) })
}

/// Reads a NUL-terminated UTF-8 identifier.
///
/// `Err(NullPointer)` for `NULL`, `Err(InvalidInput)` for bytes that are not
/// UTF-8. There is no lossy path: a slot name rsemu cannot spell back is a
/// mistake worth reporting, not one worth guessing at.
///
/// # Safety
///
/// The caller guarantees `ptr` is either null or points at a NUL-terminated
/// byte string that stays valid for the duration of the call.
pub(super) unsafe fn cstr<'a>(ptr: *const core::ffi::c_char) -> Result<&'a str, RsemuStatus> {
    if ptr.is_null() {
        return Err(RsemuStatus::NullPointer);
    }
    // SAFETY: the caller guarantees a NUL terminator within the allocation and
    // that the bytes are not mutated while borrowed; the null case is above.
    let bytes = unsafe { core::ffi::CStr::from_ptr(ptr) };
    bytes.to_str().map_err(|_| RsemuStatus::InvalidInput)
}

/// The same, but `NULL` is a legitimate "not given" rather than an error.
///
/// # Safety
///
/// As [`cstr`].
pub(super) unsafe fn cstr_opt<'a>(
    ptr: *const core::ffi::c_char,
) -> Result<Option<&'a str>, RsemuStatus> {
    if ptr.is_null() {
        return Ok(None);
    }
    // SAFETY: delegated verbatim; `ptr` is non-null here and the caller's
    // guarantee is the one `cstr` documents.
    unsafe { cstr(ptr) }.map(Some)
}

// ---------------------------------------------------------------------------
// The output-buffer convention
// ---------------------------------------------------------------------------

/// Copies `data` out under the capacity-in / length-out rule.
///
/// `*out_len` is **always** written, whether the copy happened or not, so a
/// caller that passes a zero capacity learns the size it needs and a caller
/// whose buffer was too small learns it too. `out` is only dereferenced when
/// there is something to write and the capacity allows it.
///
/// # Safety
///
/// The caller guarantees `out_len` is either null or points at a writable
/// `size_t`, and that `out` is writable for `*out_len` bytes.
pub(super) unsafe fn out_write(data: &[u8], out: *mut u8, out_len: *mut usize) -> RsemuStatus {
    if out_len.is_null() {
        return RsemuStatus::NullPointer;
    }
    // SAFETY: the caller guarantees `out_len` points at a writable `size_t`;
    // the null case is handled above.
    let cap = unsafe { *out_len };
    // SAFETY: same pointer, same guarantee. Written before any early return so
    // that a caller who gets `BufferTooSmall` always learns the length.
    unsafe { *out_len = data.len() };
    if data.len() > cap {
        return RsemuStatus::BufferTooSmall;
    }
    if data.is_empty() {
        return RsemuStatus::Ok;
    }
    if out.is_null() {
        return RsemuStatus::NullPointer;
    }
    // SAFETY: `data.len() <= cap` was checked, and the caller guarantees `out`
    // is writable for `cap` bytes. `data` is rsemu's own allocation, so the
    // two regions cannot overlap.
    unsafe { core::ptr::copy_nonoverlapping(data.as_ptr(), out, data.len()) };
    RsemuStatus::Ok
}

// ---------------------------------------------------------------------------
// The handle table
// ---------------------------------------------------------------------------

/// A machine description that has not been built yet.
///
/// A builder rather than a `#[repr(C)]` struct the caller fills in: a struct
/// crossing the ABI is padding, field order and a layout the header has to
/// keep in step by hand, and this API needs none of that. Every argument here
/// is a scalar or a pointer, which is also why the generated header has no
/// `struct` definitions in it at all.
#[derive(Debug, Default)]
pub(super) struct Config {
    /// A catalog name, or the name diagnostics should use for `source`.
    pub(super) name: String,
    /// The description text. `None` means "look `name` up in the catalog".
    pub(super) source: Option<String>,
    /// Media slots the caller bound, in the order it bound them.
    pub(super) media: Vec<(String, Vec<u8>)>,
    /// `param` overrides, as `-p name=value` would give them.
    pub(super) params: Vec<(String, String)>,
}

/// A built machine and whether a panic has been through it.
#[derive(Debug)]
pub(super) struct Running {
    pub(super) machine: Machine,
    /// Set when a panic was caught inside a call that held `machine` mutably.
    /// Nothing may be assumed about the machine afterwards, so every later
    /// call refuses rather than running a machine with broken invariants.
    pub(super) poisoned: bool,
}

/// What a handle names.
///
/// The machine is boxed because it dwarfs a configuration by an order of
/// magnitude, and the table holds one entry per live object of either kind.
#[derive(Debug)]
pub(super) enum Body {
    Config(Config),
    Machine(Box<Running>),
}

/// One live object, plus the message its last failing call left.
#[derive(Debug)]
pub(super) struct Slot {
    pub(super) body: Body,
    /// Why the last call on this handle failed. Kept per handle rather than in
    /// a thread-local so that two threads driving two machines cannot overwrite
    /// each other's diagnosis.
    pub(super) error: String,
}

/// The process-wide table of live handles.
///
/// A `Global` because it lives in a `static` and is therefore reachable from
/// every thread in the process — `core::sync`'s rule, and the reason this is
/// not a `Mutex`. Ranked *outside* [`LockRank::MACHINE`] so that taking a
/// slot's own lock underneath it is a strictly increasing acquisition; in
/// practice the table guard is dropped before the slot's is taken, so the two
/// never nest anyway.
static TABLE: Global<BTreeMap<u64, Arc<Global<Slot>>>> =
    Global::with_rank(TABLE_RANK, BTreeMap::new());

/// Outside `MACHINE` — a lower number is taken first — because the table is
/// the only thing in rsemu that outlives a machine.
const TABLE_RANK: LockRank = LockRank::new(0x0100);

/// The id generator. Monotonic and never reused, which is what makes a stale
/// handle a reportable error instead of an alias for somebody else's machine.
static NEXT_ID: AtomicU64 = AtomicU64::new(1);

/// Files `body` in the table and returns the handle that names it.
pub(super) fn insert(body: Body) -> RsemuHandle {
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    let slot = Arc::new(Global::with_rank(
        LockRank::MACHINE,
        Slot {
            body,
            error: String::new(),
        },
    ));
    TABLE.lock().insert(id, slot);
    id
}

/// The slot `handle` names, or `None` if it names nothing.
///
/// The table lock is taken for the lookup and released before the caller does
/// anything with the result, so a machine that runs for a second does not hold
/// the table against every other handle in the process.
pub(super) fn lookup(handle: RsemuHandle) -> Option<Arc<Global<Slot>>> {
    TABLE.lock().get(&handle).cloned()
}

/// Drops the object `handle` names.
///
/// `false` if there was nothing there, which is what a double free, a free of
/// a handle that was never created, and a free of an uninitialised variable all
/// look like from here.
pub(super) fn remove(handle: RsemuHandle) -> bool {
    // Taken out of the table under the lock and dropped after it is released:
    // dropping a machine runs every device's `Drop`, and holding the outermost
    // lock across that would rank-violate the moment one of them takes its own.
    let taken = TABLE.lock().remove(&handle);
    taken.is_some()
}

/// How many handles are live. Test scaffolding; never exported to C.
#[cfg(test)]
pub(super) fn live() -> usize {
    TABLE.lock().len()
}

// ---------------------------------------------------------------------------
// Running a call against a handle
// ---------------------------------------------------------------------------

/// Runs `f` against the machine `handle` names, inside the panic guard.
///
/// The whole lifecycle of an instance call is here so that no entry point can
/// forget a piece of it: look the handle up, refuse a config handle, refuse a
/// poisoned machine, run the body, record the error text on failure, and mark
/// the machine poisoned if the body panicked.
pub(super) fn with_machine(
    handle: RsemuHandle,
    f: impl FnOnce(&mut Machine) -> Result<(), crate::Error>,
) -> RsemuStatus {
    guard(|| {
        let Some(slot) = lookup(handle) else {
            return RsemuStatus::BadHandle;
        };
        let mut slot = slot.lock();
        let outcome = {
            let Body::Machine(running) = &mut slot.body else {
                return RsemuStatus::BadHandle;
            };
            if running.poisoned {
                return RsemuStatus::Panic;
            }
            // Poisoned *before* the call rather than after it: if `f` unwinds,
            // the clear below never runs and the flag is already right.
            running.poisoned = true;
            let outcome =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(&mut running.machine)));
            if outcome.is_ok() {
                running.poisoned = false;
            }
            outcome
        };
        match outcome {
            Ok(Ok(())) => {
                slot.error.clear();
                RsemuStatus::Ok
            }
            Ok(Err(e)) => {
                let status = RsemuStatus::of(&e);
                slot.error = e.to_string();
                status
            }
            Err(_) => {
                slot.error = String::from(
                    "a panic was caught at the C ABI boundary; this machine is poisoned and \
                     every further call on its handle will return RSEMU_PANIC",
                );
                RsemuStatus::Panic
            }
        }
    })
}

/// Runs `f` against the config `handle` names, inside the panic guard.
pub(super) fn with_config(
    handle: RsemuHandle,
    f: impl FnOnce(&mut Config) -> RsemuStatus,
) -> RsemuStatus {
    guard(|| {
        let Some(slot) = lookup(handle) else {
            return RsemuStatus::BadHandle;
        };
        let mut slot = slot.lock();
        let Body::Config(config) = &mut slot.body else {
            return RsemuStatus::BadHandle;
        };
        f(config)
    })
}

/// Records `message` as the reason `handle`'s last call failed.
pub(super) fn set_error(handle: RsemuHandle, message: impl ToString) {
    if let Some(slot) = lookup(handle) {
        slot.lock().error = message.to_string();
    }
}
