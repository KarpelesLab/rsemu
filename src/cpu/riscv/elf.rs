//! A minimal ELF reader, for loading conformance binaries into guest memory.
//!
//! `riscv-tests` ships each test as a small statically linked ELF that signals
//! its result by storing to a symbol called `tohost`, so running the suite
//! needs exactly two things from an ELF file: the `PT_LOAD` segments and the
//! symbol table. That is what this reads, and nothing more — there is no
//! relocation processing, no dynamic linking and no section-to-segment
//! mapping, because a test binary needs none of it.
//!
//! It lives here rather than in a shared module because it is *this core's*
//! test scaffolding. If a second architecture needs an ELF loader, the two
//! should be merged deliberately rather than by one reaching into the other.
//!
//! `no_std + alloc`: the parser takes a byte slice and never touches the
//! filesystem, so the same code loads an image a browser handed us.
//!
//! # Sources
//!
//! The ELF specification (System V ABI, generic and RISC-V processor
//! supplements). Field offsets are transcribed from the `Elf64_Ehdr`,
//! `Elf64_Phdr`, `Elf64_Shdr` and `Elf64_Sym` structure definitions and their
//! 32-bit counterparts.

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;

/// Why an ELF file could not be read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ElfError {
    /// The file does not start with `\x7fELF`.
    NotElf,
    /// The file is 64-bit where 32 was expected, or is neither.
    BadClass(u8),
    /// The file is big-endian; RISC-V images are little-endian.
    BigEndian,
    /// The file is for another architecture.
    NotRiscv(u16),
    /// A header or a segment runs off the end of the file.
    Truncated(&'static str),
}

impl fmt::Display for ElfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ElfError::NotElf => f.write_str("not an ELF file"),
            ElfError::BadClass(c) => write!(f, "unknown ELF class {c}"),
            ElfError::BigEndian => f.write_str("big-endian ELF; RISC-V images are little-endian"),
            ElfError::NotRiscv(m) => write!(f, "ELF machine {m} is not RISC-V (243)"),
            ElfError::Truncated(what) => write!(f, "truncated ELF: {what} runs off the end"),
        }
    }
}

/// One loadable segment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Segment {
    /// The physical address to load it at.
    ///
    /// The *physical* address, not the virtual one: a bare-metal test image is
    /// loaded where it will actually run, and the two differ in exactly the
    /// linker scripts that matter.
    pub addr: u64,
    /// The bytes from the file.
    pub bytes: Vec<u8>,
    /// How many bytes the segment occupies in memory, which may exceed
    /// `bytes.len()` — the difference is `.bss` and must be zeroed.
    pub mem_len: u64,
}

/// A parsed ELF image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Elf {
    /// Whether the file is 64-bit.
    pub is_64: bool,
    /// The entry point.
    pub entry: u64,
    /// Every `PT_LOAD` segment, in file order.
    pub segments: Vec<Segment>,
    /// Global symbols, as `(name, value)` in symbol-table order.
    pub symbols: Vec<(String, u64)>,
}

/// Read a little-endian integer of `len` bytes at `at`.
fn le(bytes: &[u8], at: usize, len: usize, what: &'static str) -> Result<u64, ElfError> {
    let slice = bytes.get(at..at + len).ok_or(ElfError::Truncated(what))?;
    let mut v = 0u64;
    for (i, b) in slice.iter().enumerate() {
        v |= u64::from(*b) << (8 * i);
    }
    Ok(v)
}

/// A NUL-terminated string from a string table.
fn cstr(bytes: &[u8], at: usize) -> String {
    let rest = bytes.get(at..).unwrap_or(&[]);
    let end = rest.iter().position(|b| *b == 0).unwrap_or(rest.len());
    String::from_utf8_lossy(&rest[..end]).into_owned()
}

impl Elf {
    /// Parse an ELF image.
    ///
    /// # Errors
    ///
    /// If the magic, class, byte order or machine is wrong, or any header runs
    /// off the end of the file. A truncated file is an error rather than a
    /// panic: this parser is pointed at whatever a `--corpus` directory holds.
    pub fn parse(bytes: &[u8]) -> Result<Elf, ElfError> {
        if bytes.len() < 20 || bytes[..4] != [0x7f, b'E', b'L', b'F'] {
            return Err(ElfError::NotElf);
        }
        let is_64 = match bytes[4] {
            1 => false,
            2 => true,
            other => return Err(ElfError::BadClass(other)),
        };
        if bytes[5] != 1 {
            return Err(ElfError::BigEndian);
        }
        let machine = le(bytes, 18, 2, "machine")? as u16;
        // EM_RISCV. Refusing anything else is not pedantry: loading an x86
        // binary would produce a hart executing noise and a mystery.
        if machine != 243 {
            return Err(ElfError::NotRiscv(machine));
        }

        // Header field offsets differ between the two classes; everything else
        // below is shared.
        let (w, e_entry, e_phoff, e_shoff, e_phentsize, e_phnum, e_shentsize, e_shnum, e_shstrndx) =
            if is_64 {
                (
                    8usize, 24usize, 32usize, 40usize, 54usize, 56usize, 58usize, 60usize, 62usize,
                )
            } else {
                (
                    4usize, 24usize, 28usize, 32usize, 42usize, 44usize, 46usize, 48usize, 50usize,
                )
            };
        let _ = e_shstrndx;
        let entry = le(bytes, e_entry, w, "entry")?;
        let phoff = le(bytes, e_phoff, w, "program header offset")? as usize;
        let shoff = le(bytes, e_shoff, w, "section header offset")? as usize;
        let phentsize = le(bytes, e_phentsize, 2, "program header size")? as usize;
        let phnum = le(bytes, e_phnum, 2, "program header count")? as usize;
        let shentsize = le(bytes, e_shentsize, 2, "section header size")? as usize;
        let shnum = le(bytes, e_shnum, 2, "section header count")? as usize;

        let mut segments = Vec::new();
        for i in 0..phnum {
            let at = phoff + i * phentsize;
            let p_type = le(bytes, at, 4, "segment type")?;
            if p_type != 1 {
                continue; // Only PT_LOAD is loaded.
            }
            let (offset, paddr, filesz, memsz) = if is_64 {
                (
                    le(bytes, at + 8, 8, "segment offset")?,
                    le(bytes, at + 24, 8, "segment address")?,
                    le(bytes, at + 32, 8, "segment file size")?,
                    le(bytes, at + 40, 8, "segment memory size")?,
                )
            } else {
                (
                    le(bytes, at + 4, 4, "segment offset")?,
                    le(bytes, at + 12, 4, "segment address")?,
                    le(bytes, at + 16, 4, "segment file size")?,
                    le(bytes, at + 20, 4, "segment memory size")?,
                )
            };
            let start = offset as usize;
            let end = start + filesz as usize;
            let data = bytes
                .get(start..end)
                .ok_or(ElfError::Truncated("a segment"))?;
            segments.push(Segment {
                addr: paddr,
                bytes: data.to_vec(),
                mem_len: memsz,
            });
        }

        // The symbol table, for `tohost` and friends. A stripped binary simply
        // has none, which is not an error.
        let mut symbols = Vec::new();
        for i in 0..shnum {
            let at = shoff + i * shentsize;
            let Ok(sh_type) = le(bytes, at + 4, 4, "section type") else {
                break;
            };
            // SHT_SYMTAB.
            if sh_type != 2 {
                continue;
            }
            let (link, offset, size, entsize) = if is_64 {
                (
                    le(bytes, at + 40, 4, "symbol table link")?,
                    le(bytes, at + 24, 8, "symbol table offset")?,
                    le(bytes, at + 32, 8, "symbol table size")?,
                    le(bytes, at + 56, 8, "symbol size")?,
                )
            } else {
                (
                    le(bytes, at + 24, 4, "symbol table link")?,
                    le(bytes, at + 16, 4, "symbol table offset")?,
                    le(bytes, at + 20, 4, "symbol table size")?,
                    le(bytes, at + 36, 4, "symbol size")?,
                )
            };
            if entsize == 0 {
                continue;
            }
            // The linked section is the string table the names live in.
            let str_at = shoff + link as usize * shentsize;
            let str_off = if is_64 {
                le(bytes, str_at + 24, 8, "string table offset")?
            } else {
                le(bytes, str_at + 16, 4, "string table offset")?
            } as usize;

            let count = (size / entsize) as usize;
            for s in 0..count {
                let sym = offset as usize + s * entsize as usize;
                let name_off = le(bytes, sym, 4, "symbol name")? as usize;
                let value = if is_64 {
                    le(bytes, sym + 8, 8, "symbol value")?
                } else {
                    le(bytes, sym + 4, 4, "symbol value")?
                };
                if name_off == 0 {
                    continue;
                }
                symbols.push((cstr(bytes, str_off + name_off), value));
            }
        }

        Ok(Elf {
            is_64,
            entry,
            segments,
            symbols,
        })
    }

    /// The value of a symbol, if the image names one.
    #[must_use]
    pub fn symbol(&self, name: &str) -> Option<u64> {
        self.symbols
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| *v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// Build a minimal but genuine 64-bit RISC-V ELF with one loadable
    /// segment and one symbol, so the parser is tested against a real layout
    /// rather than against itself.
    fn tiny_elf() -> Vec<u8> {
        let phoff = 64usize;
        let phentsize = 56usize;
        let shoff = phoff + phentsize;
        let shentsize = 64usize;
        let shnum = 3usize;
        let payload_off = shoff + shnum * shentsize;
        let payload = [0x13u8, 0x00, 0x00, 0x00]; // one nop
        let symtab_off = payload_off + payload.len();
        let symentsize = 24usize;
        let strtab_off = symtab_off + 2 * symentsize;
        let names = b"\0tohost\0";

        let mut f = vec![0u8; strtab_off + names.len()];
        f[..4].copy_from_slice(&[0x7f, b'E', b'L', b'F']);
        f[4] = 2; // 64-bit
        f[5] = 1; // little-endian
        f[6] = 1; // version
        f[16..18].copy_from_slice(&2u16.to_le_bytes()); // ET_EXEC
        f[18..20].copy_from_slice(&243u16.to_le_bytes()); // EM_RISCV
        f[24..32].copy_from_slice(&0x8000_0000u64.to_le_bytes()); // entry
        f[32..40].copy_from_slice(&(phoff as u64).to_le_bytes());
        f[40..48].copy_from_slice(&(shoff as u64).to_le_bytes());
        f[54..56].copy_from_slice(&(phentsize as u16).to_le_bytes());
        f[56..58].copy_from_slice(&1u16.to_le_bytes()); // phnum
        f[58..60].copy_from_slice(&(shentsize as u16).to_le_bytes());
        f[60..62].copy_from_slice(&(shnum as u16).to_le_bytes());

        // One PT_LOAD segment.
        let p = phoff;
        f[p..p + 4].copy_from_slice(&1u32.to_le_bytes()); // PT_LOAD
        f[p + 8..p + 16].copy_from_slice(&(payload_off as u64).to_le_bytes());
        f[p + 16..p + 24].copy_from_slice(&0x8000_0000u64.to_le_bytes()); // vaddr
        f[p + 24..p + 32].copy_from_slice(&0x8000_0000u64.to_le_bytes()); // paddr
        f[p + 32..p + 40].copy_from_slice(&(payload.len() as u64).to_le_bytes());
        f[p + 40..p + 48].copy_from_slice(&8u64.to_le_bytes()); // memsz > filesz

        // Section 1 is the symbol table, section 2 its string table.
        let s = shoff + shentsize;
        f[s + 4..s + 8].copy_from_slice(&2u32.to_le_bytes()); // SHT_SYMTAB
        f[s + 24..s + 32].copy_from_slice(&(symtab_off as u64).to_le_bytes());
        f[s + 32..s + 40].copy_from_slice(&(2 * symentsize as u64).to_le_bytes());
        f[s + 40..s + 44].copy_from_slice(&2u32.to_le_bytes()); // link
        f[s + 56..s + 64].copy_from_slice(&(symentsize as u64).to_le_bytes());
        let s = shoff + 2 * shentsize;
        f[s + 4..s + 8].copy_from_slice(&3u32.to_le_bytes()); // SHT_STRTAB
        f[s + 24..s + 32].copy_from_slice(&(strtab_off as u64).to_le_bytes());
        f[s + 32..s + 40].copy_from_slice(&(names.len() as u64).to_le_bytes());

        f[payload_off..payload_off + payload.len()].copy_from_slice(&payload);
        // Symbol 0 is the reserved null entry; symbol 1 is `tohost`.
        let y = symtab_off + symentsize;
        f[y..y + 4].copy_from_slice(&1u32.to_le_bytes()); // name offset
        f[y + 8..y + 16].copy_from_slice(&0x8000_1000u64.to_le_bytes());
        f[strtab_off..strtab_off + names.len()].copy_from_slice(names);
        f
    }

    #[test]
    fn a_minimal_image_parses() {
        let elf = Elf::parse(&tiny_elf()).unwrap();
        assert!(elf.is_64);
        assert_eq!(elf.entry, 0x8000_0000);
        assert_eq!(elf.segments.len(), 1);
        assert_eq!(elf.segments[0].addr, 0x8000_0000);
        assert_eq!(elf.segments[0].bytes, [0x13, 0, 0, 0]);
        assert_eq!(elf.segments[0].mem_len, 8, "bss must be visible as a gap");
        assert_eq!(elf.symbol("tohost"), Some(0x8000_1000));
        assert_eq!(elf.symbol("fromhost"), None);
    }

    #[test]
    fn the_wrong_kind_of_file_is_rejected_rather_than_guessed_at() {
        assert_eq!(
            Elf::parse(b"not an elf at all").unwrap_err(),
            ElfError::NotElf
        );
        let mut f = tiny_elf();
        f[4] = 7;
        assert_eq!(Elf::parse(&f).unwrap_err(), ElfError::BadClass(7));
        let mut f = tiny_elf();
        f[5] = 2;
        assert_eq!(Elf::parse(&f).unwrap_err(), ElfError::BigEndian);
        let mut f = tiny_elf();
        f[18] = 62; // EM_X86_64
        assert!(matches!(Elf::parse(&f), Err(ElfError::NotRiscv(_))));
    }

    #[test]
    fn a_truncated_file_is_an_error_not_a_panic() {
        let full = tiny_elf();
        for cut in [20, 40, 64, 80, full.len() - 1] {
            let _ = Elf::parse(&full[..cut]);
        }
        assert!(Elf::parse(&full[..70]).is_err());
    }
}
