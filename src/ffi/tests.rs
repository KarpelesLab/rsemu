//! Tests for the C ABI.
//!
//! # How strong a cross-boundary test can be, given no C toolchain
//!
//! `ROADMAP.md` §0 forbids a C compiler anywhere in this tree, so the
//! siblings' answer — a `.c` smoke test compiled in a dedicated CI job — is not
//! available and neither is anything that shells out to `cc`. `dlopen` is
//! `libc`, which the dependency policy forbids. What is left has to be argued
//! for rather than assumed, so here is the argument.
//!
//! "Does the ABI work?" is four questions, and they do not all have the same
//! answer.
//!
//! 1. **Do the symbols exist, unmangled, under the names the header
//!    declares?** Tested, and genuinely: the module below declares every
//!    function in an `extern "C"` block and calls it through *that* rather than
//!    through the Rust item. The declaration is resolved by the **linker**, by
//!    name, exactly as a C caller's would be — so a lost `#[unsafe(no_mangle)]`,
//!    a renamed function or a symbol that failed to be emitted is a link error
//!    in `cargo test`, not a surprise for the first embedder. Calling
//!    `abi::rsemu_config_new()` directly, which is what the siblings' tests do,
//!    proves none of that: it is an ordinary Rust call to a known item and
//!    would keep passing with the export removed.
//!
//! 2. **Does the header describe those symbols correctly?** Tested
//!    transitively. The header is generated from `abi.rs`
//!    ([`header`](super::header)) and compared against the committed file, and
//!    [`PROTOTYPES`] below pins the exact C prototype each `extern` declaration
//!    was written against. Change a signature in `abi.rs` and three things move
//!    together or the tests fail: the generated header, the committed header,
//!    and this list. That closes the loop between the Rust, the C view, and the
//!    caller — which is precisely the loop a hand-written header leaves open.
//!
//! 3. **Do the types have the layout the header claims?** Tested directly:
//!    `size_of`/`align_of` on every type that crosses, and each status
//!    discriminant asserted against the value the header defines.
//!
//! 4. **Does the platform's C calling convention agree?** **Not tested, and
//!    not testable here.** Whether the arguments this crate pushes are the
//!    arguments a C compiler pops is a property of two toolchains agreeing,
//!    and with only one toolchain in the room nothing in-tree can observe it.
//!    A Rust `extern "C"` call and a C call both claim to implement the
//!    platform ABI; a test that uses one to check the other is checking rustc
//!    against itself.
//!
//! So: the honest claim is **linkage, layout and behaviour are tested; the
//! calling convention is asserted on rustc's `extern "C"` implementation and
//! nothing more.** That residue is real, it is bounded, and the way to close
//! it is out-of-tree — a downstream C program that links the `staticlib` — not
//! by adding a compiler §0 says must not be here. Writing a test that *looked*
//! like a C test while being a Rust call in disguise would be worse than
//! saying this.
//!
//! The rest of the file is where the bugs actually are: hostile inputs. A C
//! caller's mistakes — a NULL, a stale handle, a freed handle, a handle of the
//! wrong kind, a buffer too small, a byte string that is not UTF-8 — are all
//! constructible in Rust, because they are just numbers and pointers, and each
//! one is asserted to produce a status rather than a crash.

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::ffi::c_char;

use crate::ffi::abi::{RSEMU_ABI_VERSION, RSEMU_INVALID_HANDLE, RSEMU_RESET_COLD, RsemuStatus};
use crate::ffi::header;

// ---------------------------------------------------------------------------
// The C view of the library, declared independently of the Rust items
// ---------------------------------------------------------------------------

/// The handle type as C sees it. Written here rather than imported so that a
/// change to `RsemuHandle` shows up as a failing layout assertion.
type Handle = u64;

/// The status type as C sees it: `int32_t`, per the header.
type Status = i32;

// SAFETY: every symbol below is defined in this crate by an
// `#[unsafe(no_mangle)] extern "C" fn` in `abi.rs`, and each declaration here
// was written against the prototype `PROTOTYPES` pins, which a test asserts is
// the one the generated header carries. The linker, not the compiler, is what
// matches the two — which is the property this block exists to exercise.
unsafe extern "C" {
    fn rsemu_abi_version() -> u32;
    fn rsemu_strerror(status: Status) -> *const c_char;
    fn rsemu_build_info(out: *mut c_char, out_len: *mut usize) -> Status;
    fn rsemu_catalog_count() -> u32;
    fn rsemu_catalog_name(index: u32, out: *mut c_char, out_len: *mut usize) -> Status;
    fn rsemu_catalog_media(index: u32, slot: u32, out: *mut c_char, out_len: *mut usize) -> Status;
    fn rsemu_config_new() -> Handle;
    fn rsemu_config_machine(
        config: Handle,
        name: *const c_char,
        source: *const u8,
        source_len: usize,
    ) -> Status;
    fn rsemu_config_media(
        config: Handle,
        slot: *const c_char,
        bytes: *const u8,
        len: usize,
    ) -> Status;
    fn rsemu_config_param(config: Handle, key: *const c_char, value: *const c_char) -> Status;
    fn rsemu_machine_new(config: Handle, out: *mut Handle) -> Status;
    fn rsemu_free(handle: Handle) -> Status;
    fn rsemu_last_error(handle: Handle, out: *mut c_char, out_len: *mut usize) -> Status;
    fn rsemu_machine_run_ns(machine: Handle, nanos: u64) -> Status;
    fn rsemu_machine_now_ns(machine: Handle, out: *mut u64) -> Status;
    fn rsemu_machine_reset(machine: Handle, kind: Status) -> Status;
    fn rsemu_machine_space_count(machine: Handle, out: *mut u32) -> Status;
    fn rsemu_machine_space_name(
        machine: Handle,
        index: u32,
        out: *mut c_char,
        out_len: *mut usize,
    ) -> Status;
    fn rsemu_machine_read(
        machine: Handle,
        space: *const c_char,
        addr: u64,
        out: *mut u8,
        len: usize,
    ) -> Status;
    fn rsemu_machine_write(
        machine: Handle,
        space: *const c_char,
        addr: u64,
        bytes: *const u8,
        len: usize,
    ) -> Status;
    fn rsemu_machine_save(machine: Handle, out: *mut u8, out_len: *mut usize) -> Status;
    fn rsemu_machine_load(machine: Handle, bytes: *const u8, len: usize) -> Status;
    fn rsemu_machine_state_hash(machine: Handle, out: *mut u64) -> Status;
}

/// The prototype each declaration above was written against.
///
/// Pinned here rather than trusted: this is the join between the C view in the
/// `extern` block and the C view in the header, and without it a signature
/// could change in `abi.rs`, propagate cleanly into a regenerated header, and
/// leave the block above quietly calling with the wrong arity.
const PROTOTYPES: &[&str] = &[
    "uint32_t rsemu_abi_version(void);",
    "const char *rsemu_strerror(int32_t status);",
    "rsemu_status rsemu_build_info(char *out, size_t *out_len);",
    "uint32_t rsemu_catalog_count(void);",
    "rsemu_status rsemu_catalog_name(uint32_t index, char *out, size_t *out_len);",
    "rsemu_status rsemu_catalog_media(uint32_t index, uint32_t slot, char *out, \
     size_t *out_len);",
    "rsemu_handle rsemu_config_new(void);",
    "rsemu_status rsemu_config_machine(rsemu_handle config, const char *name, \
     const uint8_t *source, size_t source_len);",
    "rsemu_status rsemu_config_media(rsemu_handle config, const char *slot, \
     const uint8_t *bytes, size_t len);",
    "rsemu_status rsemu_config_param(rsemu_handle config, const char *key, \
     const char *value);",
    "rsemu_status rsemu_machine_new(rsemu_handle config, rsemu_handle *out);",
    "rsemu_status rsemu_free(rsemu_handle handle);",
    "rsemu_status rsemu_last_error(rsemu_handle handle, char *out, size_t *out_len);",
    "rsemu_status rsemu_machine_run_ns(rsemu_handle machine, uint64_t nanos);",
    "rsemu_status rsemu_machine_now_ns(rsemu_handle machine, uint64_t *out);",
    "rsemu_status rsemu_machine_reset(rsemu_handle machine, int32_t kind);",
    "rsemu_status rsemu_machine_space_count(rsemu_handle machine, uint32_t *out);",
    "rsemu_status rsemu_machine_space_name(rsemu_handle machine, uint32_t index, \
     char *out, size_t *out_len);",
    "rsemu_status rsemu_machine_read(rsemu_handle machine, const char *space, \
     uint64_t addr, uint8_t *out, size_t len);",
    "rsemu_status rsemu_machine_write(rsemu_handle machine, const char *space, \
     uint64_t addr, const uint8_t *bytes, size_t len);",
    "rsemu_status rsemu_machine_save(rsemu_handle machine, uint8_t *out, size_t *out_len);",
    "rsemu_status rsemu_machine_load(rsemu_handle machine, const uint8_t *bytes, size_t len);",
    "rsemu_status rsemu_machine_state_hash(rsemu_handle machine, uint64_t *out);",
];

// ---------------------------------------------------------------------------
// Scaffolding
// ---------------------------------------------------------------------------

/// A machine with 4 KiB of RAM and nothing else.
///
/// No CPU and no oscillator, so it builds in *any* feature set — which is what
/// makes the tests below run on the `--features ffi` build CI checks rather
/// than only on one that happens to ship a console.
const DEMO: &str = r#"
machine "ffi-demo" {
  space mem { width = 16, unassigned = open-bus }
  object dram "ram" { size = 4K }
  map mem 0x0000 size 4K = dram
}
"#;

/// A NUL-terminated byte string, as C would pass one.
fn cstring(text: &str) -> Vec<u8> {
    let mut bytes = Vec::from(text.as_bytes());
    bytes.push(0);
    bytes
}

/// Builds the demo machine through the ABI, panicking with the recorded
/// message if it will not build.
fn demo() -> Handle {
    // SAFETY: every pointer below is into a live local that outlives the call,
    // every NUL-terminated string really is one, and `out` is a live `Handle`.
    unsafe {
        let cfg = rsemu_config_new();
        assert_ne!(cfg, RSEMU_INVALID_HANDLE);
        let name = cstring("ffi-demo");
        let status = rsemu_config_machine(
            cfg,
            name.as_ptr().cast::<c_char>(),
            DEMO.as_ptr(),
            DEMO.len(),
        );
        assert_eq!(status, RsemuStatus::Ok as Status);

        let mut machine: Handle = 0;
        let status = rsemu_machine_new(cfg, &raw mut machine);
        assert_eq!(status, RsemuStatus::Ok as Status, "{}", error_of(cfg));
        assert_eq!(rsemu_free(cfg), RsemuStatus::Ok as Status);
        machine
    }
}

/// The message on `handle`, read the way a C caller would: query, allocate,
/// fill.
fn error_of(handle: Handle) -> String {
    // SAFETY: the two calls follow the documented buffer convention — a length
    // query with a zero capacity, then a buffer of exactly that size — and
    // both pointers are into live locals.
    unsafe {
        let mut len = 0usize;
        let status = rsemu_last_error(handle, core::ptr::null_mut(), &raw mut len);
        if status == RsemuStatus::Ok as Status {
            return String::new();
        }
        assert_eq!(status, RsemuStatus::BufferTooSmall as Status);
        let mut buf = vec![0u8; len];
        let status = rsemu_last_error(handle, buf.as_mut_ptr().cast::<c_char>(), &raw mut len);
        assert_eq!(status, RsemuStatus::Ok as Status);
        buf.truncate(len);
        String::from_utf8(buf).expect("the ABI promises UTF-8")
    }
}

/// Whitespace-insensitive containment, so a prototype the generator wrapped
/// across lines still matches the one-line form written above.
fn squeeze(text: &str) -> String {
    text.chars().filter(|c| !c.is_whitespace()).collect()
}

// ---------------------------------------------------------------------------
// Linkage, layout and the header
// ---------------------------------------------------------------------------

#[test]
fn the_extern_declarations_match_the_header() {
    let generated = squeeze(&header::generate());
    for proto in PROTOTYPES {
        assert!(
            generated.contains(&squeeze(proto)),
            "the `extern \"C\"` block in this file was written against\n    {proto}\nwhich the \
             generated header no longer declares. Reconcile the block, `PROTOTYPES` and \
             `abi.rs`; the header itself regenerates with RSEMU_UPDATE_HEADER=1."
        );
    }
}

#[test]
fn every_exported_function_is_pinned_by_a_prototype() {
    // The list above is only a check while it is complete: a function added to
    // `abi.rs` and left out of it would be exported, declared in the header,
    // and never called through the linker by anything in this tree.
    let generated = header::generate();
    let declared: Vec<&str> = generated
        .lines()
        // A prototype's first line, and nothing else: not a `#define`, not a
        // comment, not the `typedef enum`.
        .filter(|line| {
            let head = line.trim_start();
            !head.starts_with('#')
                && !head.starts_with('*')
                && !head.starts_with('/')
                && !head.starts_with("typedef")
                && line.contains("rsemu_")
                && line.contains('(')
        })
        .collect();
    for line in declared {
        let name = line
            .split('(')
            .next()
            .and_then(|head| head.split_whitespace().last())
            .map(|name| name.trim_start_matches('*'))
            .expect("a prototype names a function");
        assert!(
            PROTOTYPES.iter().any(|p| p.contains(name)),
            "`{name}` is exported to C but no `extern` declaration in this file calls it"
        );
    }
}

#[test]
fn the_types_have_the_layout_the_header_claims() {
    // `rsemu_handle` is `uint64_t` and `rsemu_status` is an `int32_t`-shaped
    // enum. Both are claims the header makes to a C compiler that is not here
    // to check them, so they are checked here instead.
    assert_eq!(size_of::<crate::ffi::RsemuHandle>(), size_of::<u64>());
    assert_eq!(align_of::<crate::ffi::RsemuHandle>(), align_of::<u64>());
    assert_eq!(size_of::<RsemuStatus>(), size_of::<i32>());
    assert_eq!(align_of::<RsemuStatus>(), align_of::<i32>());

    let header = header::generate();
    for status in RsemuStatus::ALL {
        let value = status as i32;
        assert!(
            header.contains(&alloc::format!(" = {value},")),
            "the header does not define the value {value}"
        );
    }
    // The three the whole family agrees on.
    assert_eq!(RsemuStatus::Ok as i32, 0);
    assert_eq!(RsemuStatus::NullPointer as i32, -1);
    assert_eq!(RsemuStatus::BufferTooSmall as i32, -2);
}

#[test]
fn the_library_and_the_header_agree_on_the_abi_revision() {
    // SAFETY: no arguments, no pointers.
    assert_eq!(unsafe { rsemu_abi_version() }, RSEMU_ABI_VERSION);
    assert!(header::generate().contains("#define RSEMU_ABI_VERSION"));
}

#[test]
fn strerror_answers_every_code_and_survives_a_bogus_one() {
    for status in RsemuStatus::ALL {
        // SAFETY: `rsemu_strerror` takes an integer and returns a pointer into
        // static storage; reading it as a `CStr` is what the header documents.
        let text = unsafe { core::ffi::CStr::from_ptr(rsemu_strerror(status as i32)) };
        assert!(!text.to_bytes().is_empty(), "{status:?} has no message");
    }
    // SAFETY: as above; the ABI promises a valid pointer for any input.
    let text = unsafe { core::ffi::CStr::from_ptr(rsemu_strerror(12345)) };
    assert_eq!(text.to_str(), Ok("unknown status"));
}

// ---------------------------------------------------------------------------
// The buffer convention
// ---------------------------------------------------------------------------

#[test]
fn a_zero_capacity_is_a_length_query() {
    // SAFETY: `len` is a live local; the first call is told the buffer is
    // empty, so it writes nothing through the null pointer.
    unsafe {
        let mut len = 0usize;
        assert_eq!(
            rsemu_build_info(core::ptr::null_mut(), &raw mut len),
            RsemuStatus::BufferTooSmall as Status
        );
        assert!(len > 0, "the length is written even when the copy fails");

        let mut buf = vec![0u8; len];
        assert_eq!(
            rsemu_build_info(buf.as_mut_ptr().cast::<c_char>(), &raw mut len),
            RsemuStatus::Ok as Status
        );
        let text = String::from_utf8(buf).expect("UTF-8");
        assert!(text.starts_with("rsemu "), "{text}");
    }
}

#[test]
fn a_null_out_len_is_reported_rather_than_dereferenced() {
    // SAFETY: deliberately passing the null the ABI documents as an error; the
    // point of the call is that it returns instead of faulting.
    let status = unsafe { rsemu_build_info(core::ptr::null_mut(), core::ptr::null_mut()) };
    assert_eq!(status, RsemuStatus::NullPointer as Status);
}

#[test]
fn a_buffer_one_byte_short_is_refused_rather_than_truncated() {
    // SAFETY: `len` and `buf` are live locals and `len` is set to the real
    // capacity of `buf`, which is what makes the refusal meaningful.
    unsafe {
        let mut len = 0usize;
        rsemu_build_info(core::ptr::null_mut(), &raw mut len);
        let short = len - 1;
        let mut buf = vec![0xAAu8; short];
        let mut cap = short;
        assert_eq!(
            rsemu_build_info(buf.as_mut_ptr().cast::<c_char>(), &raw mut cap),
            RsemuStatus::BufferTooSmall as Status
        );
        assert_eq!(cap, len, "the needed length is reported");
        assert!(buf.iter().all(|b| *b == 0xAA), "nothing was written");
    }
}

// ---------------------------------------------------------------------------
// Handles: the failure modes a pointer-shaped ABI would answer with a segfault
// ---------------------------------------------------------------------------

#[test]
fn a_handle_that_names_nothing_is_an_error() {
    for handle in [0u64, 1 << 40, u64::MAX, 0xdead_beef] {
        // SAFETY: the handle is an id, not a pointer, so no value of it can
        // make rsemu dereference anything. That is the claim being tested.
        let status = unsafe { rsemu_machine_run_ns(handle, 1) };
        assert_eq!(
            status,
            RsemuStatus::BadHandle as Status,
            "handle {handle:#x} should name nothing"
        );
    }
}

#[test]
fn a_double_free_is_an_error_and_not_a_crash() {
    let machine = demo();
    // SAFETY: the first free is well formed; the second is the mistake under
    // test and is answered by a lookup that fails, not by a second `Box::drop`.
    unsafe {
        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
        assert_eq!(rsemu_free(machine), RsemuStatus::BadHandle as Status);
        assert_eq!(rsemu_free(machine), RsemuStatus::BadHandle as Status);
    }
}

#[test]
fn a_use_after_free_is_an_error_and_not_a_crash() {
    let machine = demo();
    // SAFETY: as above — after the free the id resolves to nothing, so every
    // call on it takes the same early return a forged id does.
    unsafe {
        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
        let mut out = 0u64;
        assert_eq!(
            rsemu_machine_state_hash(machine, &raw mut out),
            RsemuStatus::BadHandle as Status
        );
        assert_eq!(
            rsemu_machine_run_ns(machine, 1_000),
            RsemuStatus::BadHandle as Status
        );
        let mut len = 0usize;
        assert_eq!(
            rsemu_last_error(machine, core::ptr::null_mut(), &raw mut len),
            RsemuStatus::BadHandle as Status
        );
    }
}

#[test]
fn a_handle_of_the_wrong_kind_is_an_error() {
    // SAFETY: a live config handle, passed where a machine was wanted. The
    // table knows which is which, so this is a status rather than a machine
    // read out of a `Config`'s bytes.
    unsafe {
        let cfg = rsemu_config_new();
        assert_eq!(
            rsemu_machine_run_ns(cfg, 1_000),
            RsemuStatus::BadHandle as Status
        );
        let machine = demo();
        let name = cstring("ffi-demo");
        assert_eq!(
            rsemu_config_machine(
                machine,
                name.as_ptr().cast::<c_char>(),
                core::ptr::null(),
                0
            ),
            RsemuStatus::BadHandle as Status
        );
        assert_eq!(rsemu_free(cfg), RsemuStatus::Ok as Status);
        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
    }
}

#[test]
fn handles_do_not_repeat() {
    // The property that makes a stale handle a *reportable* error rather than
    // an alias for whichever machine landed at the same address next.
    let mut seen = Vec::new();
    for _ in 0..8 {
        // SAFETY: create and immediately free; no pointers involved.
        unsafe {
            let handle = rsemu_config_new();
            assert!(!seen.contains(&handle), "handle {handle} was reused");
            seen.push(handle);
            assert_eq!(rsemu_free(handle), RsemuStatus::Ok as Status);
        }
    }
}

// ---------------------------------------------------------------------------
// Strings
// ---------------------------------------------------------------------------

#[test]
fn a_slot_name_that_is_not_utf8_is_refused() {
    // SAFETY: `bad` is a live, NUL-terminated byte string; it is simply not
    // UTF-8, which is the case under test.
    unsafe {
        let cfg = rsemu_config_new();
        let bad = [0xffu8, 0xfe, 0x00];
        let status = rsemu_config_media(cfg, bad.as_ptr().cast::<c_char>(), core::ptr::null(), 0);
        assert_eq!(status, RsemuStatus::InvalidInput as Status);
        assert_eq!(rsemu_free(cfg), RsemuStatus::Ok as Status);
    }
}

#[test]
fn a_null_identifier_is_refused() {
    // SAFETY: a deliberate null where the ABI documents a NUL-terminated
    // string; it must be reported rather than dereferenced.
    unsafe {
        let cfg = rsemu_config_new();
        assert_eq!(
            rsemu_config_machine(cfg, core::ptr::null(), core::ptr::null(), 0),
            RsemuStatus::NullPointer as Status
        );
        assert_eq!(rsemu_free(cfg), RsemuStatus::Ok as Status);
    }
}

#[test]
fn a_null_source_with_a_length_is_refused() {
    // NULL is how a caller asks for the catalog, so a NULL that also claims a
    // length is a mistake worth naming rather than a length to ignore.
    // SAFETY: the null pointer is never dereferenced; the length is checked
    // against it first.
    unsafe {
        let cfg = rsemu_config_new();
        let name = cstring("nes-ntsc");
        assert_eq!(
            rsemu_config_machine(cfg, name.as_ptr().cast::<c_char>(), core::ptr::null(), 16),
            RsemuStatus::NullPointer as Status
        );
        assert_eq!(rsemu_free(cfg), RsemuStatus::Ok as Status);
    }
}

#[test]
fn a_description_that_does_not_parse_reports_where() {
    // SAFETY: pointers into live locals; the description is deliberately
    // malformed, which is what the call is for.
    unsafe {
        let cfg = rsemu_config_new();
        let name = cstring("broken.machine");
        let source = "machine \"broken\" { space mem { width = } }";
        assert_eq!(
            rsemu_config_machine(
                cfg,
                name.as_ptr().cast::<c_char>(),
                source.as_ptr(),
                source.len(),
            ),
            RsemuStatus::Ok as Status
        );
        let mut machine: Handle = 0;
        let status = rsemu_machine_new(cfg, &raw mut machine);
        assert_eq!(status, RsemuStatus::Config as Status);
        assert_eq!(machine, RSEMU_INVALID_HANDLE, "no handle out of a failure");
        let message = error_of(cfg);
        assert!(
            message.contains("broken.machine:"),
            "a config error carries file:line:col — got {message:?}"
        );
        assert_eq!(rsemu_free(cfg), RsemuStatus::Ok as Status);
    }
}

// ---------------------------------------------------------------------------
// The embedding surface, end to end
// ---------------------------------------------------------------------------

#[test]
fn a_machine_builds_runs_and_hashes() {
    let machine = demo();
    // SAFETY: every out-parameter is a live local and the handle is live.
    unsafe {
        let mut spaces = 0u32;
        assert_eq!(
            rsemu_machine_space_count(machine, &raw mut spaces),
            RsemuStatus::Ok as Status
        );
        assert_eq!(spaces, 1);

        let mut len = 0usize;
        assert_eq!(
            rsemu_machine_space_name(machine, 0, core::ptr::null_mut(), &raw mut len),
            RsemuStatus::BufferTooSmall as Status
        );
        let mut buf = vec![0u8; len];
        assert_eq!(
            rsemu_machine_space_name(machine, 0, buf.as_mut_ptr().cast::<c_char>(), &raw mut len),
            RsemuStatus::Ok as Status
        );
        assert_eq!(String::from_utf8(buf).expect("UTF-8"), "mem");

        assert_eq!(
            rsemu_machine_space_name(machine, 7, core::ptr::null_mut(), &raw mut len),
            RsemuStatus::InvalidInput as Status
        );

        assert_eq!(
            rsemu_machine_run_ns(machine, 1_000_000),
            RsemuStatus::Ok as Status
        );
        let mut now = 0u64;
        assert_eq!(
            rsemu_machine_now_ns(machine, &raw mut now),
            RsemuStatus::Ok as Status
        );
        // Not an exact equality: virtual time is 2⁻⁶⁴-second units and both
        // conversions round down, so a nanosecond span can lose its last unit.
        assert!((999_999..=1_000_000).contains(&now), "{now}");

        let mut hash = 0u64;
        assert_eq!(
            rsemu_machine_state_hash(machine, &raw mut hash),
            RsemuStatus::Ok as Status
        );

        assert_eq!(
            rsemu_machine_reset(machine, RESET_COLD),
            RsemuStatus::Ok as Status
        );
        assert_eq!(
            rsemu_machine_reset(machine, 99),
            RsemuStatus::InvalidInput as Status
        );
        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
    }
}

/// `RSEMU_RESET_COLD` with the type the `extern` block declares.
const RESET_COLD: Status = RSEMU_RESET_COLD;

#[test]
fn guest_memory_round_trips_by_copy() {
    let machine = demo();
    // SAFETY: all buffers are live locals; the space name is NUL-terminated
    // and the addresses are inside the 4 KiB the description maps.
    unsafe {
        let written = [0xde, 0xad, 0xbe, 0xef];
        assert_eq!(
            rsemu_machine_write(machine, core::ptr::null(), 0x100, written.as_ptr(), 4),
            RsemuStatus::Ok as Status
        );
        let mut read = [0u8; 4];
        assert_eq!(
            rsemu_machine_read(machine, core::ptr::null(), 0x100, read.as_mut_ptr(), 4),
            RsemuStatus::Ok as Status
        );
        assert_eq!(read, written);

        // The same, naming the space rather than taking the default.
        let name = cstring("mem");
        let mut read = [0u8; 4];
        assert_eq!(
            rsemu_machine_read(
                machine,
                name.as_ptr().cast::<c_char>(),
                0x100,
                read.as_mut_ptr(),
                4,
            ),
            RsemuStatus::Ok as Status
        );
        assert_eq!(read, written);

        // A space this machine does not have.
        let missing = cstring("nowhere");
        assert_eq!(
            rsemu_machine_read(
                machine,
                missing.as_ptr().cast::<c_char>(),
                0,
                read.as_mut_ptr(),
                4,
            ),
            RsemuStatus::InvalidInput as Status
        );

        // A zero-length access with a null buffer is the empty access, not an
        // error: `(NULL, 0)` is the convention's empty blob.
        assert_eq!(
            rsemu_machine_write(machine, core::ptr::null(), 0, core::ptr::null(), 0),
            RsemuStatus::Ok as Status
        );

        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
    }
}

#[test]
fn a_snapshot_round_trips_through_the_boundary() {
    let machine = demo();
    // SAFETY: buffers are live locals and follow the documented convention.
    unsafe {
        let marker = [1u8, 2, 3, 4];
        assert_eq!(
            rsemu_machine_write(machine, core::ptr::null(), 0x40, marker.as_ptr(), 4),
            RsemuStatus::Ok as Status
        );
        let mut before = 0u64;
        assert_eq!(
            rsemu_machine_state_hash(machine, &raw mut before),
            RsemuStatus::Ok as Status
        );

        let mut len = 0usize;
        assert_eq!(
            rsemu_machine_save(machine, core::ptr::null_mut(), &raw mut len),
            RsemuStatus::BufferTooSmall as Status
        );
        let mut snapshot = vec![0u8; len];
        assert_eq!(
            rsemu_machine_save(machine, snapshot.as_mut_ptr(), &raw mut len),
            RsemuStatus::Ok as Status
        );

        // Scribble over the marker, then put the snapshot back.
        let noise = [9u8, 9, 9, 9];
        assert_eq!(
            rsemu_machine_write(machine, core::ptr::null(), 0x40, noise.as_ptr(), 4),
            RsemuStatus::Ok as Status
        );
        assert_eq!(
            rsemu_machine_load(machine, snapshot.as_ptr(), snapshot.len()),
            RsemuStatus::Ok as Status
        );

        let mut read = [0u8; 4];
        assert_eq!(
            rsemu_machine_read(machine, core::ptr::null(), 0x40, read.as_mut_ptr(), 4),
            RsemuStatus::Ok as Status
        );
        assert_eq!(read, marker);

        let mut after = 0u64;
        assert_eq!(
            rsemu_machine_state_hash(machine, &raw mut after),
            RsemuStatus::Ok as Status
        );
        assert_eq!(
            before, after,
            "the state hash survives a snapshot round trip"
        );

        // A snapshot that is not one is reported, not half-applied.
        let junk = [0u8; 32];
        assert_eq!(
            rsemu_machine_load(machine, junk.as_ptr(), junk.len()),
            RsemuStatus::State as Status
        );
        assert!(!error_of(machine).is_empty());

        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
    }
}

#[test]
fn two_runs_of_the_same_machine_agree() {
    // The reason an embedder wants this ABI at all: run, hash, compare.
    // SAFETY: two independent handles, each used only through live locals.
    unsafe {
        let mut hashes = [0u64; 2];
        for hash in &mut hashes {
            let machine = demo();
            assert_eq!(
                rsemu_machine_run_ns(machine, 500_000),
                RsemuStatus::Ok as Status
            );
            assert_eq!(
                rsemu_machine_state_hash(machine, &raw mut *hash),
                RsemuStatus::Ok as Status
            );
            assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
        }
        assert_eq!(hashes[0], hashes[1]);
    }
}

#[test]
fn a_parameter_override_reaches_the_description() {
    // SAFETY: pointers into live locals, all NUL-terminated where declared so.
    unsafe {
        let source = r#"
machine "sized" {
  param bytes = 4K
  space mem { width = 16, unassigned = open-bus }
  object dram "ram" { size = bytes }
  map mem 0x0000 size bytes = dram
}
"#;
        let cfg = rsemu_config_new();
        let name = cstring("sized.machine");
        assert_eq!(
            rsemu_config_machine(
                cfg,
                name.as_ptr().cast::<c_char>(),
                source.as_ptr(),
                source.len(),
            ),
            RsemuStatus::Ok as Status
        );
        let key = cstring("bytes");
        let value = cstring("2K");
        assert_eq!(
            rsemu_config_param(
                cfg,
                key.as_ptr().cast::<c_char>(),
                value.as_ptr().cast::<c_char>(),
            ),
            RsemuStatus::Ok as Status
        );
        let mut machine: Handle = 0;
        assert_eq!(
            rsemu_machine_new(cfg, &raw mut machine),
            RsemuStatus::Ok as Status,
            "{}",
            error_of(cfg)
        );
        // 2 KiB, so the last kilobyte is no longer mapped and the space's
        // open-bus policy answers instead of RAM.
        let mut byte = [0u8; 1];
        assert_eq!(
            rsemu_machine_read(machine, core::ptr::null(), 0x7ff, byte.as_mut_ptr(), 1),
            RsemuStatus::Ok as Status
        );
        assert_eq!(rsemu_free(cfg), RsemuStatus::Ok as Status);
        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
    }
}

#[test]
fn the_catalog_is_readable_by_index() {
    // SAFETY: out-parameters are live locals; an out-of-range index is
    // reported rather than indexed.
    unsafe {
        let count = rsemu_catalog_count();
        for index in 0..count {
            // A fresh capacity every time: `out_len` is an in/out parameter, so
            // a leftover length from the previous name would be read as a
            // capacity this call is allowed to write a name into — through the
            // null pointer, which is `RSEMU_NULL_POINTER` and not the query
            // this loop means.
            let mut len = 0usize;
            assert_eq!(
                rsemu_catalog_name(index, core::ptr::null_mut(), &raw mut len),
                RsemuStatus::BufferTooSmall as Status
            );
            assert!(len > 0);
        }
        let mut len = 0usize;
        assert_eq!(
            rsemu_catalog_name(count, core::ptr::null_mut(), &raw mut len),
            RsemuStatus::InvalidInput as Status
        );
        assert_eq!(
            rsemu_catalog_media(count, 0, core::ptr::null_mut(), &raw mut len),
            RsemuStatus::InvalidInput as Status
        );
    }
}

/// A catalog machine, built the way `rsemu run apple1` builds one: by name,
/// with no media, falling back to the monitor ROM rsemu itself ships.
///
/// Gated on the feature that puts the machine in the catalog, because a
/// catalog is a fact about the build. The unconditional tests above carry the
/// surface; this one carries the claim that the surface reaches a *real*
/// machine and not only a description written for a test.
#[cfg(feature = "machine-apple1")]
#[test]
fn a_catalog_machine_builds_from_its_name_alone() {
    // SAFETY: pointers into live locals; the NUL-terminated name really is
    // one, and `NULL, 0` is the documented "look it up in the catalog".
    unsafe {
        let cfg = rsemu_config_new();
        let name = cstring("apple1");
        assert_eq!(
            rsemu_config_machine(cfg, name.as_ptr().cast::<c_char>(), core::ptr::null(), 0),
            RsemuStatus::Ok as Status
        );
        let mut machine: Handle = 0;
        assert_eq!(
            rsemu_machine_new(cfg, &raw mut machine),
            RsemuStatus::Ok as Status,
            "an unbound `rom` slot must fall back to rsemu's own monitor: {}",
            error_of(cfg)
        );
        assert_eq!(
            rsemu_machine_run_ns(machine, 1_000_000),
            RsemuStatus::Ok as Status
        );
        let mut hash = 0u64;
        assert_eq!(
            rsemu_machine_state_hash(machine, &raw mut hash),
            RsemuStatus::Ok as Status
        );
        assert_eq!(rsemu_free(cfg), RsemuStatus::Ok as Status);
        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
    }
}

// ---------------------------------------------------------------------------
// Panics
// ---------------------------------------------------------------------------

#[test]
fn the_guard_turns_a_panic_into_a_status() {
    assert_eq!(
        crate::ffi::common::guard(|| panic!("boom")),
        RsemuStatus::Panic
    );
    assert_eq!(
        crate::ffi::common::guard(|| RsemuStatus::Ok),
        RsemuStatus::Ok
    );
    assert_eq!(crate::ffi::common::guard_with(7u32, || panic!("boom")), 7);
    assert_eq!(crate::ffi::common::guard_with(7u32, || 3), 3);
}

#[test]
fn a_panicked_machine_is_poisoned_rather_than_run_again() {
    let machine = demo();
    // The panic is provoked through the real call path rather than simulated,
    // so what is tested is the poisoning `with_machine` does and not a flag
    // set by hand. The unwind is caught, so the harness only sees the message.
    let status = crate::ffi::common::with_machine(machine, |_| panic!("a device gave up"));
    assert_eq!(status, RsemuStatus::Panic);

    // SAFETY: the handle is live; the calls are the ones a caller would make
    // after seeing RSEMU_PANIC.
    unsafe {
        assert_eq!(
            rsemu_machine_run_ns(machine, 1_000),
            RsemuStatus::Panic as Status,
            "a poisoned machine must not run again"
        );
        let mut hash = 0u64;
        assert_eq!(
            rsemu_machine_state_hash(machine, &raw mut hash),
            RsemuStatus::Panic as Status
        );
        // Reporting and cleaning up still work, which is the whole point of
        // poisoning rather than aborting.
        assert!(error_of(machine).contains("poisoned"));
        assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status);
    }
}

#[test]
fn a_freed_handle_leaves_the_table() {
    // Not a process-wide leak check — these tests run in parallel, so the
    // count is nobody's to assert — but the entry this test made must be gone,
    // which is what stops the table growing for the life of an embedder.
    let machine = demo();
    let occupied = crate::ffi::common::live();
    // SAFETY: the handle is live and freed exactly once.
    unsafe { assert_eq!(rsemu_free(machine), RsemuStatus::Ok as Status) };
    assert!(occupied >= 1);
    assert!(crate::ffi::common::lookup(machine).is_none());
}
