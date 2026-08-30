//! WebAssembly entry points.
//!
//! Plain C-ABI exports and embedder-supplied imports, following purecrypto's
//! browser convention — deliberately **no** `wasm-bindgen`: the dependency
//! policy forbids it (`ROADMAP.md` §0), and the boundary is small enough that
//! it does not need a binding generator.
//!
//! The host supplies the JS glue: it instantiates the module, reads exported
//! memory directly, and provides the imports rsemu needs (a clock, entropy,
//! and — once the JIT lands — module compilation). See `ROADMAP.md` §11.
//!
//! # Build
//!
//! ```sh
//! cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
//!     --no-default-features --features wasm --release
//! ```
//!
//! # Status
//!
//! Enough surface to prove the target builds and links in CI from the first
//! commit, which is the point: wasm rots faster than anything else, and a
//! target that is not built every commit is a target that does not work.
//!
//! # `unsafe` in this module
//!
//! This is the **C ABI boundary**, one of the six subsystems `ROADMAP.md` §0
//! sanctions to opt back in. Two things here need it: `#[unsafe(no_mangle)]`,
//! which edition 2024 classifies as an unsafe attribute because duplicate
//! exported symbols are the linker's problem rather than the compiler's; and
//! the private `leaked` helper, which rebuilds a `&'static str` from a
//! pointer/length pair. The
//! allow is module-scoped rather than crate-wide, and every genuine `unsafe`
//! block below carries its own `// SAFETY:` argument.
#![allow(unsafe_code)]

use alloc::string::String;

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
}
