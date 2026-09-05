//! The shared object half of the level-3 dynamic-linking guest.
//!
//! `scripts/fetch-testdata.sh usermode-guests` builds this into the
//! git-ignored corpus directory alongside [`main.rs`](main), stages it under
//! `/lib/libgreet.so`, and `src/usermode/proof.rs` runs the pair under a
//! **real** dynamic loader taken from the host.
//!
//! It is `#![no_std]` and links no libc, which is the point rather than
//! minimalism: the loader is the thing under test, and a libc between it and
//! the program only adds instructions. What it exports is one of each kind of
//! thing a relocation can be about — a *datum* (`R_AARCH64_GLOB_DAT`) and a
//! *function* (`R_AARCH64_JUMP_SLOT`) — so a run that prints the right string
//! has had both resolved by somebody, and that somebody is not rsemu.

#![no_std]

/// A guest with no libc has no unwinder and nothing to say.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// The datum the executable reads through the global offset table.
#[unsafe(no_mangle)]
pub static GREETING: [u8; 24] = *b"hello from a shared obj\n";

/// The function the executable calls through the procedure linkage table.
#[unsafe(no_mangle)]
pub extern "C" fn greeting_len() -> usize {
    GREETING.len()
}
