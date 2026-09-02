//! A minimal ELF64 reader, for loading conformance binaries into guest memory.
//!
//! `rustc --target aarch64-unknown-none` emits a statically linked ELF and
//! there is no way to ask it for a flat image, so running a built corpus needs
//! exactly three things out of the file: the machine it is for, the entry
//! point, and the `PT_LOAD` segments. That is what this reads. No relocation
//! processing, no dynamic linking, no section table — a bare-metal test binary
//! has none of it, and a reader that handled them would be code no test
//! exercises.
//!
//! # Why there are two of these in the tree
//!
//! `cpu/riscv/elf.rs` is the other, and it says in its own header that if a
//! second architecture needs an ELF loader "the two should be merged
//! deliberately rather than by one reaching into the other". This is that
//! second architecture, and this file is deliberately *not* that merge:
//! `ROADMAP.md` §6.1.1's rule is to let duplication become real and visible and
//! then extract against two working consumers, rather than to guess at the
//! seam with one. The two now exist and disagree in exactly two places — the
//! machine check, and the symbol table, which that one needs and this one does
//! not. That is the evidence a merge would be built on.
//!
//! `no_std + alloc`: the parser takes a byte slice and never touches a
//! filesystem, so the same code loads an image a browser handed us.
//!
//! # Sources
//!
//! The ELF specification (System V ABI, generic ABI plus the AArch64
//! processor supplement). Field offsets are transcribed from `Elf64_Ehdr` and
//! `Elf64_Phdr`.

use alloc::vec::Vec;
use core::fmt;

/// The `e_machine` value the AArch64 supplement assigns.
pub(super) const EM_AARCH64: u16 = 183;

/// `PT_LOAD`.
const PT_LOAD: u32 = 1;

/// Why an ELF file could not be read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum ElfError {
    /// The file does not start with `\x7fELF`.
    NotElf,
    /// The file is not 64-bit. A `rustc` AArch64 image always is.
    NotElf64(u8),
    /// The file is big-endian.
    BigEndian,
    /// The file is for another architecture.
    NotAarch64(u16),
    /// A header or a segment runs off the end of the file.
    Truncated(&'static str),
}

impl fmt::Display for ElfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ElfError::NotElf => f.write_str("not an ELF file"),
            ElfError::NotElf64(c) => write!(f, "ELF class {c} is not 64-bit"),
            ElfError::BigEndian => f.write_str("big-endian ELF; A64 images are little-endian"),
            ElfError::NotAarch64(m) => write!(f, "ELF machine {m} is not AArch64 ({EM_AARCH64})"),
            ElfError::Truncated(what) => write!(f, "truncated ELF: {what} runs off the end"),
        }
    }
}

/// One loadable segment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Segment {
    /// Where to load it.
    ///
    /// `p_paddr`, not `p_vaddr`: a bare-metal image runs with the MMU off, and
    /// the physical address is the one the bus will see. For the images this
    /// loads the two are equal — but taking the virtual one would be a bug
    /// waiting for the first linker script that separates them.
    pub addr: u64,
    /// The bytes from the file.
    pub bytes: Vec<u8>,
    /// How much memory it occupies, which is at least `bytes.len()`: the
    /// excess is `.bss` and must be zeroed.
    pub mem_len: u64,
}

/// The parts of an ELF file a conformance run needs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Elf {
    /// The entry point.
    pub entry: u64,
    /// The loadable segments, in file order.
    pub segments: Vec<Segment>,
}

/// Read a little-endian value out of `bytes` at `at`.
fn u16_at(bytes: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes(bytes.get(at..at + 2)?.try_into().ok()?))
}

fn u32_at(bytes: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?))
}

fn u64_at(bytes: &[u8], at: usize) -> Option<u64> {
    Some(u64::from_le_bytes(bytes.get(at..at + 8)?.try_into().ok()?))
}

impl Elf {
    /// Parse an ELF64 AArch64 image.
    ///
    /// # Errors
    ///
    /// If it is not one, or if a header runs off the end of the file.
    pub(super) fn parse(bytes: &[u8]) -> Result<Elf, ElfError> {
        if bytes.len() < 64 || bytes[..4] != *b"\x7fELF" {
            return Err(ElfError::NotElf);
        }
        if bytes[4] != 2 {
            return Err(ElfError::NotElf64(bytes[4]));
        }
        if bytes[5] != 1 {
            return Err(ElfError::BigEndian);
        }
        let machine = u16_at(bytes, 18).ok_or(ElfError::Truncated("e_machine"))?;
        if machine != EM_AARCH64 {
            return Err(ElfError::NotAarch64(machine));
        }
        let entry = u64_at(bytes, 24).ok_or(ElfError::Truncated("e_entry"))?;
        let phoff = u64_at(bytes, 32).ok_or(ElfError::Truncated("e_phoff"))? as usize;
        let phentsize = u16_at(bytes, 54).ok_or(ElfError::Truncated("e_phentsize"))? as usize;
        let phnum = u16_at(bytes, 56).ok_or(ElfError::Truncated("e_phnum"))? as usize;

        let mut segments = Vec::new();
        for i in 0..phnum {
            let at = phoff + i * phentsize;
            let kind = u32_at(bytes, at).ok_or(ElfError::Truncated("p_type"))?;
            if kind != PT_LOAD {
                continue;
            }
            let offset = u64_at(bytes, at + 8).ok_or(ElfError::Truncated("p_offset"))? as usize;
            let paddr = u64_at(bytes, at + 24).ok_or(ElfError::Truncated("p_paddr"))?;
            let filesz = u64_at(bytes, at + 32).ok_or(ElfError::Truncated("p_filesz"))? as usize;
            let memsz = u64_at(bytes, at + 40).ok_or(ElfError::Truncated("p_memsz"))?;
            let data = bytes
                .get(offset..offset + filesz)
                .ok_or(ElfError::Truncated("a segment"))?;
            segments.push(Segment {
                addr: paddr,
                bytes: data.to_vec(),
                mem_len: memsz,
            });
        }
        Ok(Elf { entry, segments })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// Build a minimal ELF64 AArch64 file with one `PT_LOAD` segment, so the
    /// parser is tested against bytes rather than against a file that has to
    /// be built first.
    fn image(machine: u16, class: u8, endian: u8) -> Vec<u8> {
        let mut out = vec![0u8; 64 + 56 + 4];
        out[..4].copy_from_slice(b"\x7fELF");
        out[4] = class;
        out[5] = endian;
        out[16..18].copy_from_slice(&2u16.to_le_bytes()); // ET_EXEC
        out[18..20].copy_from_slice(&machine.to_le_bytes());
        out[24..32].copy_from_slice(&0x4000_0000u64.to_le_bytes()); // e_entry
        out[32..40].copy_from_slice(&64u64.to_le_bytes()); // e_phoff
        out[54..56].copy_from_slice(&56u16.to_le_bytes()); // e_phentsize
        out[56..58].copy_from_slice(&1u16.to_le_bytes()); // e_phnum
        let ph = 64;
        out[ph..ph + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
        out[ph + 8..ph + 16].copy_from_slice(&(64u64 + 56).to_le_bytes()); // p_offset
        out[ph + 16..ph + 24].copy_from_slice(&0x4000_0000u64.to_le_bytes()); // p_vaddr
        out[ph + 24..ph + 32].copy_from_slice(&0x4000_0000u64.to_le_bytes()); // p_paddr
        out[ph + 32..ph + 40].copy_from_slice(&4u64.to_le_bytes()); // p_filesz
        out[ph + 40..ph + 48].copy_from_slice(&16u64.to_le_bytes()); // p_memsz
        out[64 + 56..].copy_from_slice(&0xd503_201fu32.to_le_bytes()); // nop
        out
    }

    #[test]
    fn a_load_segment_comes_back_with_its_physical_address() {
        let elf = Elf::parse(&image(EM_AARCH64, 2, 1)).unwrap();
        assert_eq!(elf.entry, 0x4000_0000);
        assert_eq!(elf.segments.len(), 1);
        let seg = &elf.segments[0];
        assert_eq!(seg.addr, 0x4000_0000);
        assert_eq!(seg.bytes, 0xd503_201fu32.to_le_bytes());
        // `p_memsz` exceeds `p_filesz`: the rest is `.bss` and the caller
        // zeroes it, which is why both numbers survive parsing.
        assert_eq!(seg.mem_len, 16);
    }

    #[test]
    fn an_image_for_another_machine_is_refused() {
        // 243 is RISC-V, which is exactly the mistake worth catching: the two
        // corpora live under the same testdata root.
        assert_eq!(
            Elf::parse(&image(243, 2, 1)),
            Err(ElfError::NotAarch64(243))
        );
        assert_eq!(
            Elf::parse(&image(EM_AARCH64, 1, 1)),
            Err(ElfError::NotElf64(1))
        );
        assert_eq!(
            Elf::parse(&image(EM_AARCH64, 2, 2)),
            Err(ElfError::BigEndian)
        );
        assert_eq!(Elf::parse(b"not an elf"), Err(ElfError::NotElf));
    }

    #[test]
    fn a_truncated_segment_is_an_error_rather_than_a_panic() {
        let mut bytes = image(EM_AARCH64, 2, 1);
        bytes.truncate(64 + 56 + 2);
        assert_eq!(Elf::parse(&bytes), Err(ElfError::Truncated("a segment")));
    }
}
