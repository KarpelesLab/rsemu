//! `include/rsemu.h`, generated from [`abi`](super::abi).
//!
//! # Why generated, and why generated *this* way
//!
//! `purecrypto` and `kataan` both write their header by hand and check it with
//! a C smoke test in CI. rsemu cannot copy that half of the pattern:
//! `ROADMAP.md` §0 forbids a C toolchain anywhere near this tree, so a
//! hand-written header here would have **nothing** checking it — the one gap
//! the siblings' design leaves, and the one this module closes.
//!
//! `cbindgen` is the obvious answer and is not available: the dependency policy
//! lists seven permitted crates and it is not one of them. So the generator is
//! ours, it is forty lines of line-oriented parsing, and it reads the real
//! source rather than a table describing it. That last part is the whole point.
//! A generator driven by a hand-maintained list of signatures drifts exactly as
//! a hand-maintained header does, one edit later; a generator that parses
//! `abi.rs` cannot, because `abi.rs` is what the compiler compiled.
//!
//! Two properties fall out of parsing rather than tabulating:
//!
//! * **A type with no C spelling fails the build.** The type map has no default
//!   case. Add a `*mut Foo` to a signature and the header test panics naming
//!   it, which forces the question "what is this in C?" to be answered by
//!   whoever changed the signature.
//! * **The ABI has to stay in one file.** Everything C can see lives in
//!   `abi.rs` and a test asserts that no other file in this module exports a
//!   symbol, so "generated from the source" is not quietly true of only part
//!   of it.
//!
//! # Regenerating
//!
//! ```sh
//! RSEMU_UPDATE_HEADER=1 cargo test --features ffi header
//! ```
//!
//! The test writes `include/rsemu.h` and then fails, so a regeneration is
//! always a visible step and never something a passing test did behind
//! somebody's back.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

/// The source the header is generated from. One file, by construction.
const SOURCE: &str = include_str!("abi.rs");

/// What goes above everything the generator emits.
///
/// The conventions a C caller needs before the first prototype makes sense,
/// in the shape `purecrypto.h` and `kataan.h` use.
const PREAMBLE: &str = r#"/*
 * rsemu -- the C ABI.
 *
 * GENERATED FILE. Do not edit: it is produced from `src/ffi/abi.rs` by
 * `src/ffi/header.rs`, and a test fails when the two disagree. To change the
 * ABI, change `abi.rs`; to update this file afterwards, run
 *
 *     RSEMU_UPDATE_HEADER=1 cargo test --features ffi header
 *
 * Build the library it declares with:
 *
 *     cargo rustc --lib --release --features ffi --crate-type staticlib
 *     cargo rustc --lib --release --features ffi --crate-type cdylib
 *
 * Conventions
 * -----------
 *
 * Status codes. Every fallible call returns rsemu_status: 0 is success and
 * every failure is negative. The numeric values are part of the ABI.
 *
 * Output buffers belong to the caller. A call that produces bytes takes
 * (out, out_len). *out_len is the capacity on entry and the length on return,
 * and it is written whether the call succeeded or not -- so passing a zero
 * capacity is a length query, and RSEMU_BUFFER_TOO_SMALL tells you how much to
 * allocate. rsemu never hands back a pointer you have to free.
 *
 * Bytes are pointer + length; identifiers are NUL-terminated. Media images,
 * machine descriptions and snapshots are (ptr, len) and need no terminator;
 * slot names, parameter names and machine names are const char *. (NULL, 0) is
 * a valid empty blob, (NULL, n > 0) is RSEMU_NULL_POINTER.
 *
 * Text is UTF-8 and is NOT NUL-terminated on the way out; *out_len carries the
 * length. Text on the way in is validated, and invalid UTF-8 is
 * RSEMU_INVALID_INPUT rather than a silent replacement character.
 *
 * Handles are integers, not pointers, and 0 is never valid. Ids are never
 * reused and nothing a caller invents is ever dereferenced, so a double free,
 * a use-after-free and a forged handle are all RSEMU_BAD_HANDLE rather than
 * undefined behaviour. Calls on one handle serialise; different handles are
 * independent, so no external mutex is needed.
 *
 * Panics do not cross this boundary. A caught panic is RSEMU_PANIC and the
 * machine it happened in is poisoned: only rsemu_last_error and rsemu_free
 * keep working on it. This requires an unwinding build; under
 * -C panic=abort a panic aborts the process instead.
 */

#ifndef RSEMU_H
#define RSEMU_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
"#;

/// What goes below it.
const POSTAMBLE: &str = r#"
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RSEMU_H */
"#;

/// The C spelling of a Rust type that crosses the boundary.
///
/// `None` is not a fallback — every caller of this turns it into a panic that
/// names the type. Adding a type to the ABI means adding it here, deliberately.
fn c_type(rust: &str) -> Option<&'static str> {
    Some(match rust.trim() {
        "u8" => "uint8_t",
        "u16" => "uint16_t",
        "u32" => "uint32_t",
        "u64" => "uint64_t",
        "i32" => "int32_t",
        "usize" => "size_t",
        "RsemuHandle" => "rsemu_handle",
        "RsemuStatus" => "rsemu_status",
        "*const c_char" => "const char *",
        "*mut c_char" => "char *",
        "*const u8" => "const uint8_t *",
        "*mut u8" => "uint8_t *",
        "*mut u32" => "uint32_t *",
        "*mut u64" => "uint64_t *",
        "*mut usize" => "size_t *",
        "*mut RsemuHandle" => "rsemu_handle *",
        _ => return None,
    })
}

/// `BusBadAccess` to `RSEMU_BUS_BAD_ACCESS`.
fn screaming(name: &str) -> String {
    let mut out = String::from("RSEMU");
    for ch in name.chars() {
        if ch.is_ascii_uppercase() {
            out.push('_');
        }
        out.push(ch.to_ascii_uppercase());
    }
    out
}

/// Typography rsemu's prose uses, spelled the way a C compiler's source
/// character set is guaranteed to accept.
///
/// The header is the one file here a toolchain outside this repository has to
/// read, and MSVC in a non-UTF-8 codepage warns on bytes above 127 even inside
/// a comment. Unknown non-ASCII is a panic rather than a silent `?`, on the
/// same principle as [`c_type`]: the generator refuses instead of guessing.
fn ascii(text: &str) -> String {
    let mut out = String::new();
    for ch in text.chars() {
        match ch {
            _ if ch.is_ascii() => out.push(ch),
            '\u{2014}' | '\u{2013}' => out.push_str("--"),
            '\u{2026}' => out.push_str("..."),
            '\u{2018}' | '\u{2019}' => out.push('\''),
            '\u{201c}' | '\u{201d}' => out.push('"'),
            '\u{00d7}' => out.push('x'),
            _ => panic!(
                "`{ch}` has no ASCII spelling for the header; add one to `ascii` or reword the \
                 doc comment in abi.rs"
            ),
        }
    }
    out
}

/// A rustdoc line as C prose: intra-doc links flattened, headings unwound.
///
/// `[`rsemu_free`]` and ``[`Error`](crate::Error)`` both become `` `…` ``,
/// because a C reader has no rustdoc to follow the link into.
fn plain(text: &str) -> String {
    let text = match text.strip_prefix("# ") {
        Some(heading) => return format!("{heading}:"),
        None => text,
    };
    let text = ascii(text);
    let text = text.as_str();
    let mut out = String::new();
    let mut rest = text;
    while let Some(at) = rest.find("[`") {
        out.push_str(&rest[..at]);
        rest = &rest[at + 2..];
        let Some(end) = rest.find("`]") else {
            out.push_str("[`");
            break;
        };
        out.push('`');
        out.push_str(&rest[..end]);
        out.push('`');
        rest = &rest[end + 2..];
        if let Some(tail) = rest.strip_prefix('(')
            && let Some(close) = tail.find(')')
        {
            rest = &tail[close + 1..];
        }
    }
    out.push_str(rest);
    out
}

/// Renders accumulated doc lines as a C block comment.
fn comment(doc: &[String], indent: &str) -> String {
    if doc.is_empty() {
        return String::new();
    }
    let mut out = format!("{indent}/*\n");
    for line in doc {
        let line = plain(line);
        let line = line.trim_end();
        assert!(
            !line.contains("*/"),
            "a doc comment in abi.rs would close the C comment it is rendered into: {line}"
        );
        if line.is_empty() {
            out.push_str(&format!("{indent} *\n"));
        } else {
            out.push_str(&format!("{indent} * {line}\n"));
        }
    }
    out.push_str(&format!("{indent} */\n"));
    out
}

/// One `name: type` parameter, as C declares it.
fn parameter(text: &str) -> String {
    let (name, ty) = text
        .split_once(':')
        .unwrap_or_else(|| panic!("`{text}` is not a `name: type` parameter"));
    let c = c_type(ty).unwrap_or_else(|| {
        panic!(
            "the C ABI has no spelling for `{}`; add one to `c_type`",
            ty.trim()
        )
    });
    if c.ends_with('*') {
        format!("{c}{}", name.trim())
    } else {
        format!("{c} {}", name.trim())
    }
}

/// The whole header, as the file on disk should read.
///
/// # Panics
///
/// If `abi.rs` uses a type this generator has no C spelling for, or writes a
/// doc comment that would close the C comment it is rendered into. Both are
/// meant to be loud: they are the two ways the header could otherwise become
/// quietly wrong.
pub fn generate() -> String {
    let mut out = String::from(PREAMBLE);
    let mut doc: Vec<String> = Vec::new();
    let mut lines = SOURCE.lines().peekable();

    while let Some(line) = lines.next() {
        let trimmed = line.trim();

        if let Some(text) = trimmed.strip_prefix("///") {
            doc.push(text.strip_prefix(' ').unwrap_or(text).to_string());
            continue;
        }
        // Attributes sit between a doc comment and the item it documents, so
        // they must not clear what has been collected.
        if trimmed.starts_with("#[") {
            continue;
        }

        if let Some(rest) = trimmed.strip_prefix("pub type RsemuHandle = ") {
            let ty = rest.trim_end_matches(';');
            let c = c_type(ty).unwrap_or_else(|| panic!("no C spelling for `{ty}`"));
            out.push('\n');
            out.push_str(&comment(&doc, ""));
            out.push_str(&format!("typedef {c} rsemu_handle;\n"));
        } else if let Some(rest) = trimmed.strip_prefix("pub const RSEMU_") {
            let (name, rest) = rest
                .split_once(':')
                .unwrap_or_else(|| panic!("`{trimmed}` is not a constant"));
            let (ty, value) = rest
                .split_once('=')
                .unwrap_or_else(|| panic!("`{trimmed}` has no initialiser"));
            let c = c_type(ty).unwrap_or_else(|| panic!("no C spelling for `{}`", ty.trim()));
            let value = value.trim().trim_end_matches(';');
            out.push('\n');
            out.push_str(&comment(&doc, ""));
            out.push_str(&format!(
                "#define RSEMU_{} (({c}){value})\n",
                name.trim_end()
            ));
        } else if trimmed.starts_with("pub enum RsemuStatus") {
            out.push('\n');
            out.push_str(&comment(&doc, ""));
            out.push_str("typedef enum rsemu_status {\n");
            let mut variant_doc: Vec<String> = Vec::new();
            for line in lines.by_ref() {
                let trimmed = line.trim();
                if trimmed == "}" {
                    break;
                }
                if let Some(text) = trimmed.strip_prefix("///") {
                    variant_doc.push(text.strip_prefix(' ').unwrap_or(text).to_string());
                    continue;
                }
                let Some((name, value)) = trimmed.trim_end_matches(',').split_once('=') else {
                    variant_doc.clear();
                    continue;
                };
                out.push_str(&comment(&variant_doc, "    "));
                variant_doc.clear();
                out.push_str(&format!(
                    "    {} = {},\n",
                    screaming(name.trim()),
                    value.trim()
                ));
            }
            out.push_str("} rsemu_status;\n");
        } else if trimmed.starts_with("pub extern \"C\" fn")
            || trimmed.starts_with("pub unsafe extern \"C\" fn")
        {
            // A signature rustfmt wrapped is joined back into one line before
            // it is parsed, so the parser never has to know how wide the
            // formatter's column happens to be.
            let mut signature = String::from(trimmed);
            while !signature.trim_end().ends_with('{') {
                let next = lines.next().expect("an unterminated signature in abi.rs");
                signature.push(' ');
                signature.push_str(next.trim());
            }
            out.push('\n');
            out.push_str(&comment(&doc, ""));
            out.push_str(&function(&signature));
        } else if trimmed.is_empty() && doc.is_empty() {
            continue;
        } else {
            doc.clear();
            continue;
        }
        doc.clear();
    }

    out.push_str(POSTAMBLE);
    assert!(
        out.is_ascii(),
        "the header must be plain ASCII; `ascii` missed something"
    );
    out
}

/// One prototype, from the Rust signature with its body brace still on it.
fn function(signature: &str) -> String {
    let after_fn = signature
        .split_once("fn ")
        .expect("a signature names a function")
        .1;
    let (name, rest) = after_fn
        .split_once('(')
        .expect("a signature has a parameter list");
    let name = name.trim();
    let close = rest.rfind(')').expect("a signature closes its parameters");
    let params = &rest[..close];
    let ret = match rest[close + 1..].split_once("->") {
        Some((_, ty)) => {
            let ty = ty.trim().trim_end_matches('{').trim();
            c_type(ty).unwrap_or_else(|| panic!("no C spelling for return type `{ty}`"))
        }
        None => "void",
    };

    // `filter` before `map` because rustfmt leaves a trailing comma on a
    // signature it wrapped, and an empty tail is not a parameter.
    let params: Vec<String> = params
        .split(',')
        .filter(|p| !p.trim().is_empty())
        .map(parameter)
        .collect();
    let joined = if params.is_empty() {
        String::from("void")
    } else {
        params.join(", ")
    };
    let gap = if ret.ends_with('*') { "" } else { " " };
    let one_line = format!("{ret}{gap}{name}({joined});\n");
    if one_line.len() <= 80 {
        return one_line;
    }
    // Deterministic wrapping, so the generated file does not depend on how
    // long a parameter name happens to be this week.
    let mut out = format!("{ret}{gap}{name}(\n");
    for (at, param) in params.iter().enumerate() {
        let comma = if at + 1 == params.len() { "" } else { "," };
        out.push_str(&format!("    {param}{comma}\n"));
    }
    out.push_str(");\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The committed header, as it is right now.
    const COMMITTED: &str = include_str!("../../include/rsemu.h");

    /// The check the siblings do not have: the header and the Rust agree.
    ///
    /// A signature edited in `abi.rs` and nowhere else fails here rather than
    /// in somebody's build. On a mismatch, `RSEMU_UPDATE_HEADER=1` rewrites the
    /// file — and the test still fails, so the regeneration lands as a visible
    /// diff in the same commit as the change that needed it.
    #[test]
    fn the_header_matches_the_rust() {
        let generated = generate();
        if generated == COMMITTED {
            return;
        }
        if std::env::var_os("RSEMU_UPDATE_HEADER").is_some() {
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/include/rsemu.h");
            std::fs::write(path, &generated).expect("include/rsemu.h is writable");
            panic!("include/rsemu.h was regenerated; review the diff and commit it");
        }
        // The first differing line, because a 300-line diff in a test failure
        // is not a diagnosis.
        let at = generated
            .lines()
            .zip(COMMITTED.lines())
            .position(|(a, b)| a != b);
        let detail = match at {
            Some(at) => format!(
                "line {}:\n  generated: {:?}\n  committed: {:?}",
                at + 1,
                generated.lines().nth(at).unwrap_or(""),
                COMMITTED.lines().nth(at).unwrap_or("")
            ),
            None => format!(
                "one file is a prefix of the other: {} generated lines, {} committed",
                generated.lines().count(),
                COMMITTED.lines().count()
            ),
        };
        panic!(
            "include/rsemu.h no longer matches src/ffi/abi.rs at {detail}\n\
             regenerate with: RSEMU_UPDATE_HEADER=1 cargo test --features ffi header"
        );
    }

    /// "Generated from one file" is only a guarantee while it is the truth.
    ///
    /// A `#[unsafe(no_mangle)]` in `common.rs` would be exported to C and
    /// invisible to the generator, so the header would be silently incomplete
    /// — the exact failure mode generating it was meant to remove.
    #[test]
    fn nothing_outside_abi_rs_is_exported_to_c() {
        let dir = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/src/ffi"));
        let mut offenders: Vec<String> = Vec::new();
        for entry in std::fs::read_dir(dir).expect("src/ffi is readable") {
            let path = entry.expect("a readable directory entry").path();
            if path.file_name().and_then(|n| n.to_str()) == Some("abi.rs") {
                continue;
            }
            if path.extension().and_then(|e| e.to_str()) != Some("rs") {
                continue;
            }
            let text = std::fs::read_to_string(&path).expect("a readable source file");
            for (at, line) in text.lines().enumerate() {
                let trimmed = line.trim();
                if trimmed.starts_with("//") {
                    continue;
                }
                // The attribute and the definition, spelled exactly as an
                // export is — so this scan does not trip over its own source.
                let exported = trimmed.starts_with("#[unsafe(no_mangle)]")
                    || trimmed.starts_with("pub extern \"C\" fn")
                    || trimmed.starts_with("pub unsafe extern \"C\" fn");
                if exported {
                    offenders.push(format!("{}:{}", path.display(), at + 1));
                }
            }
        }
        assert!(
            offenders.is_empty(),
            "everything C can see must live in src/ffi/abi.rs, which is what the header is \
             generated from:\n  {}",
            offenders.join("\n  ")
        );
    }

    #[test]
    fn every_status_code_reaches_the_header() {
        let generated = generate();
        for status in crate::ffi::abi::RsemuStatus::ALL {
            let name = screaming(&format!("{status:?}"));
            assert!(
                generated.contains(&format!("{name} = {},", status as i32)),
                "`{name}` is missing from the generated header"
            );
        }
    }

    #[test]
    fn an_unmapped_type_has_no_c_spelling() {
        // The property the whole design rests on: the generator refuses rather
        // than inventing a plausible-looking C type.
        assert!(c_type("*mut Machine").is_none());
        assert_eq!(c_type("*const c_char"), Some("const char *"));
    }

    #[test]
    fn doc_links_flatten() {
        assert_eq!(plain("see [`rsemu_free`] first"), "see `rsemu_free` first");
        assert_eq!(
            plain("one [`Error`](crate::Error) enum"),
            "one `Error` enum"
        );
        assert_eq!(plain("# Safety"), "Safety:");
    }

    /// The nearest thing to a C compiler this repository is allowed to have.
    ///
    /// `ROADMAP.md` §0 forbids one, so nothing here can *parse* the header the
    /// way `cc` would. What can be checked without one is the class of mistake
    /// a text generator actually makes: an unterminated comment, an unbalanced
    /// brace or parenthesis, a declaration that lost its semicolon, a header
    /// guard that does not close. Each of those would break every C caller at
    /// once, and each is visible from here.
    #[test]
    fn the_generated_header_is_well_formed() {
        let text = generate();

        // Comments first: everything after this is code, and C comments do not
        // nest, so an inner `/*` is a bug in the generator's escaping.
        let mut code = String::new();
        let mut rest = text.as_str();
        while let Some(open) = rest.find("/*") {
            code.push_str(&rest[..open]);
            let body = &rest[open + 2..];
            let close = body.find("*/").expect("every comment is closed");
            assert!(
                !body[..close].contains("/*"),
                "a nested `/*` in a comment: C comments do not nest"
            );
            rest = &body[close + 2..];
        }
        code.push_str(rest);

        let braces = code.chars().filter(|c| *c == '{').count() as i64
            - code.chars().filter(|c| *c == '}').count() as i64;
        assert_eq!(braces, 0, "unbalanced braces");
        let parens = code.chars().filter(|c| *c == '(').count() as i64
            - code.chars().filter(|c| *c == ')').count() as i64;
        assert_eq!(parens, 0, "unbalanced parentheses");

        assert_eq!(code.matches("#ifndef RSEMU_H").count(), 1);
        assert_eq!(code.matches("#endif").count(), 3, "one per #ifndef/#ifdef");
        assert_eq!(code.matches("extern \"C\" {").count(), 1);

        // Every statement-shaped line ends the way C wants it to. A wrapped
        // parameter list is the exception — its last parameter carries no
        // comma — so those lines are skipped as a block rather than checked
        // one at a time.
        let mut in_params = false;
        for line in code.lines() {
            let line = line.trim();
            if in_params {
                in_params = !line.starts_with(')');
                continue;
            }
            if line.is_empty() || line.starts_with('#') || line == "}" || line == "{" {
                continue;
            }
            if line.ends_with('(') {
                in_params = true;
                continue;
            }
            assert!(
                line.ends_with(';') || line.ends_with(',') || line.ends_with('{'),
                "`{line}` is not a complete C declaration"
            );
        }
        assert!(!in_params, "a parameter list is never closed");
    }
}
