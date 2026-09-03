//! The level-3 milestone guest: a statically linked Linux program.
//!
//! `scripts/fetch-testdata.sh usermode-guests` builds this for
//! `riscv64gc-unknown-linux-musl` into the git-ignored corpus directory, and
//! `src/usermode/proof.rs` runs the result. Nothing is downloaded and nothing
//! is committed but this source file — the corpus rule (CLAUDE.md, Testing)
//! applies to a compiler's output as much as to a downloaded ROM.
//!
//! It is ordinary `std` Rust on purpose. The point of the exercise is that the
//! guest was written without any knowledge of the emulator: it starts through
//! `musl`'s `_start` and `__libc_start_main`, finds its own program headers
//! through `AT_PHDR`, sets up thread-local storage, installs a stack-overflow
//! handler, sizes a heap with `brk`, and only then reaches `main`. A guest
//! hand-written to fit the seam would prove none of that.
//!
//! What it prints is chosen to be evidence rather than decoration: `argv` and
//! the environment come off the initial stack the loader built, so printing
//! them back is how a malformed one is caught. A wrong auxiliary vector
//! usually shows up earlier — as a fault before `main` — but a stack whose
//! `argc` is off by one starts fine and lies quietly.

fn main() {
    println!("hello from level 3");
    let args: Vec<String> = std::env::args().collect();
    println!("argv = {args:?}");
    println!("RSEMU = {:?}", std::env::var("RSEMU").ok());
}
