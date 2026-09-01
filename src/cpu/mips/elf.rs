//! A minimal 32-bit ELF reader, for loading programs into guest memory.
//!
//! Two jobs and nothing more: the `PT_LOAD` segments and the symbol table.
//! That is enough to load a bare-metal test image, and enough for a level-3
//! consumer to find an entry point. There is no relocation processing, no
//! dynamic linking and no section-to-segment mapping, because a statically
//! linked program needs none of it — and because a loader is properly an
//! operating system's job (`ROADMAP.md` §2.1), so anything more belongs
//! downstream rather than here.
//!
//! Unlike the RISC-V core's reader this one handles **both byte orders**, for
//! the same reason the core does: MIPS is bi-endian and a `mips-` toolchain
//! and a `mipsel-` toolchain produce files that differ in nothing else.
//!
//! `no_std + alloc`: the parser takes a byte slice and never touches the
//! filesystem, so the same code loads an image a browser handed us.
//!
//! # Sources
//!
//! The ELF specification (System V ABI, generic supplement, plus the MIPS
//! processor supplement for the machine number). Field offsets are transcribed
//! from the `Elf32_Ehdr`, `Elf32_Phdr`, `Elf32_Shdr` and `Elf32_Sym` structure
//! definitions.

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;

use super::isa::Endian;

/// The `e_machine` value that means MIPS.
pub const EM_MIPS: u16 = 8;

/// Why an ELF file could not be read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ElfError {
    /// The file does not start with `\x7fELF`.
    NotElf,
    /// The file is 64-bit; this core is 32-bit and so are its images.
    Not32Bit(u8),
    /// The `EI_DATA` byte is neither little- nor big-endian.
    BadEndian(u8),
    /// The file is for another architecture.
    NotMips(u16),
    /// A header or a segment runs off the end of the file.
    Truncated(&'static str),
}

impl fmt::Display for ElfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ElfError::NotElf => f.write_str("not an ELF file"),
            ElfError::Not32Bit(c) => write!(f, "ELF class {c} is not 32-bit"),
            ElfError::BadEndian(d) => write!(f, "unknown ELF data encoding {d}"),
            ElfError::NotMips(m) => write!(f, "ELF machine {m} is not MIPS ({EM_MIPS})"),
            ElfError::Truncated(what) => write!(f, "truncated ELF: {what} runs off the end"),
        }
    }
}

/// One loadable segment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Segment {
    /// The **physical** address to load it at.
    ///
    /// Not the virtual one: a bare-metal image is loaded where it will really
    /// run, and on MIPS the two differ in exactly the linker scripts that
    /// matter — a program linked to run at `0x8000_0000` in `kseg0` is loaded
    /// at physical zero.
    pub addr: u32,
    /// The virtual address the segment is linked for.
    pub vaddr: u32,
    /// The bytes from the file.
    pub bytes: Vec<u8>,
    /// How many bytes the segment occupies in memory, which may exceed
    /// `bytes.len()` — the difference is `.bss` and must be zeroed.
    pub mem_len: u32,
}

/// A parsed ELF image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Elf {
    /// Which byte order the file is in, which is also the byte order the
    /// processor pin must be strapped to.
    pub endian: Endian,
    /// The entry point, as a virtual address.
    pub entry: u32,
    /// Every `PT_LOAD` segment, in file order.
    pub segments: Vec<Segment>,
    /// Global symbols, as `(name, value)` in symbol-table order.
    pub symbols: Vec<(String, u32)>,
}

/// Read an integer of `len` bytes at `at`, in the file's byte order.
fn num(
    bytes: &[u8],
    at: usize,
    len: usize,
    endian: Endian,
    what: &'static str,
) -> Result<u32, ElfError> {
    let slice = bytes
        .get(at..at.checked_add(len).ok_or(ElfError::Truncated(what))?)
        .ok_or(ElfError::Truncated(what))?;
    let mut v = 0u32;
    if endian.is_big() {
        for b in slice {
            v = (v << 8) | u32::from(*b);
        }
    } else {
        for (i, b) in slice.iter().enumerate() {
            v |= u32::from(*b) << (8 * i);
        }
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
    /// panic: this parser is pointed at whatever a corpus directory holds.
    pub fn parse(bytes: &[u8]) -> Result<Elf, ElfError> {
        if bytes.len() < 52 || bytes[..4] != [0x7f, b'E', b'L', b'F'] {
            return Err(ElfError::NotElf);
        }
        if bytes[4] != 1 {
            return Err(ElfError::Not32Bit(bytes[4]));
        }
        let endian = match bytes[5] {
            1 => Endian::Little,
            2 => Endian::Big,
            other => return Err(ElfError::BadEndian(other)),
        };
        let machine = num(bytes, 18, 2, endian, "e_machine")? as u16;
        if machine != EM_MIPS {
            return Err(ElfError::NotMips(machine));
        }
        let entry = num(bytes, 24, 4, endian, "e_entry")?;
        let phoff = num(bytes, 28, 4, endian, "e_phoff")? as usize;
        let shoff = num(bytes, 32, 4, endian, "e_shoff")? as usize;
        let phentsize = num(bytes, 42, 2, endian, "e_phentsize")? as usize;
        let phnum = num(bytes, 44, 2, endian, "e_phnum")? as usize;
        let shentsize = num(bytes, 46, 2, endian, "e_shentsize")? as usize;
        let shnum = num(bytes, 48, 2, endian, "e_shnum")? as usize;

        let mut segments = Vec::new();
        for i in 0..phnum {
            let at = phoff + i * phentsize;
            // PT_LOAD is 1; every other kind is metadata a flat image ignores.
            if num(bytes, at, 4, endian, "p_type")? != 1 {
                continue;
            }
            let offset = num(bytes, at + 4, 4, endian, "p_offset")? as usize;
            let vaddr = num(bytes, at + 8, 4, endian, "p_vaddr")?;
            let paddr = num(bytes, at + 12, 4, endian, "p_paddr")?;
            let filesz = num(bytes, at + 16, 4, endian, "p_filesz")? as usize;
            let memsz = num(bytes, at + 20, 4, endian, "p_memsz")?;
            let data = bytes
                .get(
                    offset
                        ..offset
                            .checked_add(filesz)
                            .ok_or(ElfError::Truncated("a segment"))?,
                )
                .ok_or(ElfError::Truncated("a segment"))?;
            segments.push(Segment {
                addr: paddr,
                vaddr,
                bytes: data.to_vec(),
                mem_len: memsz,
            });
        }

        // The symbol table, if there is one. A stripped image simply has none,
        // which is not an error — it just cannot be used with a suite that
        // signals through a named symbol.
        let mut symbols = Vec::new();
        for i in 0..shnum {
            let at = shoff + i * shentsize;
            // SHT_SYMTAB is 2.
            if num(bytes, at, 4, endian, "sh_type").unwrap_or(0) != 2 {
                continue;
            }
            let offset = num(bytes, at + 16, 4, endian, "sh_offset")? as usize;
            let size = num(bytes, at + 20, 4, endian, "sh_size")? as usize;
            let link = num(bytes, at + 24, 4, endian, "sh_link")? as usize;
            let entsize = num(bytes, at + 36, 4, endian, "sh_entsize")? as usize;
            if entsize == 0 || link >= shnum {
                continue;
            }
            let strtab = shoff + link * shentsize;
            let stroff = num(bytes, strtab + 16, 4, endian, "sh_offset")? as usize;
            let strsize = num(bytes, strtab + 20, 4, endian, "sh_size")? as usize;
            let strings = bytes.get(stroff..stroff + strsize).unwrap_or(&[]);
            for k in 0..(size / entsize) {
                let sym = offset + k * entsize;
                let name = num(bytes, sym, 4, endian, "st_name")? as usize;
                let value = num(bytes, sym + 4, 4, endian, "st_value")?;
                if name == 0 {
                    continue;
                }
                symbols.push((cstr(strings, name), value));
            }
        }

        Ok(Elf {
            endian,
            entry,
            segments,
            symbols,
        })
    }

    /// Look a symbol up by name.
    #[must_use]
    pub fn symbol(&self, name: &str) -> Option<u32> {
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

    /// Build a minimal ELF32 with one `PT_LOAD` segment and no sections.
    fn image(endian: Endian, machine: u16, class: u8) -> Vec<u8> {
        let put = |out: &mut Vec<u8>, at: usize, len: usize, v: u32| {
            for i in 0..len {
                let byte = if endian.is_big() {
                    (v >> (8 * (len - 1 - i))) as u8
                } else {
                    (v >> (8 * i)) as u8
                };
                out[at + i] = byte;
            }
        };
        let mut out = vec![0u8; 52 + 32 + 8];
        out[..4].copy_from_slice(&[0x7f, b'E', b'L', b'F']);
        out[4] = class;
        out[5] = if endian.is_big() { 2 } else { 1 };
        out[6] = 1;
        put(&mut out, 16, 2, 2); // e_type = ET_EXEC
        put(&mut out, 18, 2, u32::from(machine));
        put(&mut out, 24, 4, 0x8000_0400); // e_entry
        put(&mut out, 28, 4, 52); // e_phoff
        put(&mut out, 42, 2, 32); // e_phentsize
        put(&mut out, 44, 2, 1); // e_phnum
        put(&mut out, 46, 2, 40); // e_shentsize
        // The one program header.
        put(&mut out, 52, 4, 1); // PT_LOAD
        put(&mut out, 56, 4, 84); // p_offset
        put(&mut out, 60, 4, 0x8000_0400); // p_vaddr
        put(&mut out, 64, 4, 0x0000_0400); // p_paddr
        put(&mut out, 68, 4, 8); // p_filesz
        put(&mut out, 72, 4, 16); // p_memsz — eight bytes of .bss
        out[84..92].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        out
    }

    #[test]
    fn a_little_endian_image_parses() {
        let elf = Elf::parse(&image(Endian::Little, EM_MIPS, 1)).expect("it parses");
        assert_eq!(elf.endian, Endian::Little);
        assert_eq!(elf.entry, 0x8000_0400);
        assert_eq!(elf.segments.len(), 1);
        let seg = &elf.segments[0];
        // The *physical* address is what a flat load uses.
        assert_eq!(seg.addr, 0x0000_0400);
        assert_eq!(seg.vaddr, 0x8000_0400);
        assert_eq!(seg.bytes, vec![1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(seg.mem_len, 16, "eight bytes of .bss beyond the file");
    }

    #[test]
    fn a_big_endian_image_parses_the_same_way() {
        // The whole reason this reader carries a byte order: `mips-` and
        // `mipsel-` toolchains produce files that differ in nothing else.
        let elf = Elf::parse(&image(Endian::Big, EM_MIPS, 1)).expect("it parses");
        assert_eq!(elf.endian, Endian::Big);
        assert_eq!(elf.entry, 0x8000_0400);
        assert_eq!(elf.segments[0].addr, 0x0000_0400);
    }

    #[test]
    fn the_wrong_architecture_is_refused_rather_than_loaded() {
        // 243 is RISC-V. Loading it would produce a processor executing
        // another architecture's bytes, which fails much later and much less
        // clearly.
        assert_eq!(
            Elf::parse(&image(Endian::Little, 243, 1)),
            Err(ElfError::NotMips(243))
        );
        assert_eq!(
            Elf::parse(&image(Endian::Little, EM_MIPS, 2)),
            Err(ElfError::Not32Bit(2))
        );
        assert_eq!(Elf::parse(b"not an elf at all!!!"), Err(ElfError::NotElf));
    }

    #[test]
    fn a_truncated_file_is_an_error_rather_than_a_panic() {
        let mut bytes = image(Endian::Little, EM_MIPS, 1);
        bytes.truncate(60);
        assert!(matches!(Elf::parse(&bytes), Err(ElfError::Truncated(_))));
    }
}
