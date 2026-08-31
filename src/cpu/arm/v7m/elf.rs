//! A minimal ELF32 reader, enough to load what `clang -c` produces.
//!
//! # Why an object file and not an executable
//!
//! `riscv-tests` links its corpus, and `cpu::riscv::elf` therefore reads
//! program headers. That is not available here: `clang` targets
//! `thumbv7em-none-eabi` happily, but a *linker* for it is a separate
//! package, and on a machine with only host binutils there is none —
//! `ld` on an x86-64 box supports `elf_x86_64` and three friends and nothing
//! else.
//!
//! So the corpus is assembled rather than linked: each test is one `.S` file
//! with a single `.text` section, an explicit `.org`-driven layout, and no
//! symbolic references that would need relocating. That is not a compromise
//! in coverage — the instructions under test are identical — and it removes
//! a toolchain dependency the test would otherwise silently skip on.
//!
//! It does mean **the loader must check its assumptions**, because an
//! assembly file that grows a relocation would otherwise load as silently
//! wrong bytes. [`Object::parse`] therefore rejects a file with more than one
//! allocatable section, or with any relocation against one.
//!
//! # Sources
//!
//! The ELF32 header, section-header and symbol-table layouts from the *Tool
//! Interface Standard (TIS) Executable and Linking Format Specification*,
//! version 1.2, and the ARM-specific section types from *ELF for the Arm
//! Architecture* (ARM IHI 0044). Both are freely published.

use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

/// Why a file could not be loaded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum ElfError {
    /// Not an ELF file at all.
    NotElf,
    /// Not a 32-bit little-endian ELF for ARM.
    NotArm32,
    /// A header ran past the end of the file.
    Truncated(&'static str),
    /// The file has a shape this loader deliberately does not handle.
    Unsupported(String),
}

impl fmt::Display for ElfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ElfError::NotElf => f.write_str("not an ELF file"),
            ElfError::NotArm32 => f.write_str("not a 32-bit little-endian ARM ELF"),
            ElfError::Truncated(what) => write!(f, "truncated: {what}"),
            ElfError::Unsupported(why) => write!(f, "unsupported: {why}"),
        }
    }
}

/// One assembled object.
#[derive(Debug, Clone)]
pub(super) struct Object {
    /// The single allocatable section's bytes, to be loaded at address zero.
    pub(super) image: Vec<u8>,
    /// Symbol names and their values, which are addresses because the section
    /// starts at zero.
    ///
    /// Nothing in the corpus needs a symbol today — the layout is fixed by
    /// `.org` and the result word is at a known address — but a test that
    /// wanted to find a label rather than count bytes would look here, and
    /// dropping the table would be the wrong kind of tidy.
    #[allow(dead_code)]
    pub(super) symbols: Vec<(String, u32)>,
}

/// Read a little-endian integer of `len` bytes.
fn le(bytes: &[u8], at: usize, len: usize, what: &'static str) -> Result<u64, ElfError> {
    let end = at.checked_add(len).ok_or(ElfError::Truncated(what))?;
    let slice = bytes.get(at..end).ok_or(ElfError::Truncated(what))?;
    let mut value = 0u64;
    for (i, byte) in slice.iter().enumerate() {
        value |= u64::from(*byte) << (8 * i);
    }
    Ok(value)
}

/// Read a NUL-terminated string.
fn cstr(bytes: &[u8], at: usize) -> String {
    let rest = bytes.get(at..).unwrap_or(&[]);
    let end = rest.iter().position(|b| *b == 0).unwrap_or(rest.len());
    String::from_utf8_lossy(&rest[..end]).into_owned()
}

impl Object {
    /// Parse an ELF32 relocatable object.
    ///
    /// # Errors
    ///
    /// If the file is not a 32-bit little-endian ARM ELF, if it is
    /// truncated, if it has more than one allocatable section, or if
    /// anything relocates against that section — see the module docs for why
    /// the last two are errors rather than something to work around.
    pub(super) fn parse(bytes: &[u8]) -> Result<Object, ElfError> {
        if bytes.len() < 52 || bytes[..4] != [0x7f, b'E', b'L', b'F'] {
            return Err(ElfError::NotElf);
        }
        // EI_CLASS == ELFCLASS32, EI_DATA == ELFDATA2LSB, e_machine == EM_ARM.
        if bytes[4] != 1 || bytes[5] != 1 || le(bytes, 18, 2, "e_machine")? != 40 {
            return Err(ElfError::NotArm32);
        }
        let shoff = le(bytes, 32, 4, "e_shoff")? as usize;
        let shentsize = le(bytes, 46, 2, "e_shentsize")? as usize;
        let shnum = le(bytes, 48, 2, "e_shnum")? as usize;
        let shstrndx = le(bytes, 50, 2, "e_shstrndx")? as usize;

        let section = |i: usize| -> Result<Section, ElfError> {
            let at = shoff + i * shentsize;
            Ok(Section {
                name: le(bytes, at, 4, "sh_name")? as usize,
                kind: le(bytes, at + 4, 4, "sh_type")?,
                flags: le(bytes, at + 8, 4, "sh_flags")?,
                offset: le(bytes, at + 16, 4, "sh_offset")? as usize,
                size: le(bytes, at + 20, 4, "sh_size")? as usize,
                link: le(bytes, at + 24, 4, "sh_link")? as usize,
                info: le(bytes, at + 28, 4, "sh_info")? as usize,
                entsize: le(bytes, at + 36, 4, "sh_entsize")? as usize,
            })
        };

        let shstr = section(shstrndx)?.offset;
        let mut image: Option<Vec<u8>> = None;
        let mut alloc_index = None;
        let mut symbols = Vec::new();

        for i in 0..shnum {
            let sh = section(i)?;
            // SHF_ALLOC.
            if sh.flags & 0x2 == 0 {
                continue;
            }
            if alloc_index.is_some() {
                return Err(ElfError::Unsupported(alloc::format!(
                    "more than one allocatable section; `{}` is the second. \
                     Keep the whole test in `.text`.",
                    cstr(bytes, shstr + sh.name)
                )));
            }
            alloc_index = Some(i);
            // SHT_PROGBITS is content; SHT_NOBITS is `.bss`, which would need
            // an address this layout does not give it.
            if sh.kind != 1 {
                return Err(ElfError::Unsupported(
                    "the allocatable section has no contents".to_string(),
                ));
            }
            let end = sh.offset + sh.size;
            image = Some(
                bytes
                    .get(sh.offset..end)
                    .ok_or(ElfError::Truncated("section contents"))?
                    .to_vec(),
            );
        }
        let Some(image) = image else {
            return Err(ElfError::Unsupported("no allocatable section".to_string()));
        };
        let alloc_index = alloc_index.expect("set with the image");

        for i in 0..shnum {
            let sh = section(i)?;
            // SHT_REL and SHT_RELA against the section we loaded mean the
            // assembler could not resolve something, and the bytes are a lie.
            if (sh.kind == 9 || sh.kind == 4) && sh.info == alloc_index && sh.size != 0 {
                return Err(ElfError::Unsupported(alloc::format!(
                    "{} relocation(s) against the loaded section; the test must not \
                     reference a symbol whose address the assembler cannot compute",
                    sh.size / sh.entsize.max(1)
                )));
            }
            // SHT_SYMTAB.
            if sh.kind != 2 {
                continue;
            }
            let strtab = section(sh.link)?.offset;
            let entsize = sh.entsize.max(16);
            for k in 0..(sh.size / entsize) {
                let at = sh.offset + k * entsize;
                let name = le(bytes, at, 4, "st_name")? as usize;
                let value = le(bytes, at + 4, 4, "st_value")? as u32;
                let shndx = le(bytes, at + 14, 2, "st_shndx")? as usize;
                if shndx != alloc_index || name == 0 {
                    continue;
                }
                symbols.push((cstr(bytes, strtab + name), value));
            }
        }
        Ok(Object { image, symbols })
    }

    /// The address of a symbol, if the object defines one by that name.
    #[must_use]
    #[allow(dead_code)]
    pub(super) fn symbol(&self, name: &str) -> Option<u32> {
        self.symbols
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| *v)
    }
}

/// One section header, in the fields this loader reads.
struct Section {
    name: usize,
    kind: u64,
    flags: u64,
    offset: usize,
    size: usize,
    link: usize,
    info: usize,
    entsize: usize,
}
