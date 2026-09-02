//! Just enough AML to write a DSDT.
//!
//! # Why there is a byte encoder here at all
//!
//! Because a DSDT is not a table of fields — it is a **program**, in ACPI
//! Machine Language, and an operating system will not enumerate a q35 without
//! one. [`super::acpi`] generates every other table from the realized machine
//! as a struct of numbers; this is the one that needs a compiler, and this file
//! is the smallest compiler that emits what the board actually needs.
//!
//! It is *not* an ASL compiler and must not grow into one. What is here is the
//! grammar the DSDT in [`super::acpi::dsdt`] uses and nothing else:
//! `NameOp`, `ScopeOp`, `DeviceOp`, `PackageOp`, the integer prefixes, strings,
//! and `NameString`. Anything with control flow — `Method`, `If`, `Return` — is
//! absent, and a DSDT that needs one is the moment to ask whether the table
//! should be an input rather than an output.
//!
//! # The encoding
//!
//! *ACPI Specification* revision 6.5, §20.2 (`AML Grammar Definition`) and its
//! §20.2.4 (`Package Length Encoding`), §20.2.2 (`Name Objects Encoding`) and
//! §20.2.3 (`Data Objects Encoding`).
//!
//! **`PkgLength` is the awkward one** and is worth writing out, because
//! everything else is a byte:
//!
//! ```text
//!   bits 7:6   how many bytes follow this one, 0-3
//!   bit  5:4   must be zero when bits 7:6 are non-zero
//!   bits 3:0   the low four bits of the length, when bytes follow
//!   bits 5:0   the whole length, when none do (so up to 63)
//! ```
//!
//! and the length **counts the `PkgLength` field itself**, which is what makes
//! it self-referential: a package whose contents are 62 bytes encodes as one
//! byte holding 63, and one whose contents are 63 bytes needs two bytes and so
//! encodes 65. [`pkg_length`] does that fixed-point search rather than guessing.
//!
//! No emulator or firmware source was consulted (`CLAUDE.md`, provenance).

use alloc::vec::Vec;

/// `NameOp` (§20.2.5.1).
const NAME_OP: u8 = 0x08;
/// `ScopeOp` (§20.2.5.1).
const SCOPE_OP: u8 = 0x10;
/// `ExtOpPrefix`, the escape into the two-byte opcodes (§20.2.3).
const EXT_OP_PREFIX: u8 = 0x5b;
/// `DeviceOp`, after [`EXT_OP_PREFIX`] (§20.2.5.2).
const DEVICE_OP: u8 = 0x82;
/// `PackageOp` (§20.2.5.4).
const PACKAGE_OP: u8 = 0x12;
/// `BytePrefix` (§20.2.3).
const BYTE_PREFIX: u8 = 0x0a;
/// `WordPrefix` (§20.2.3).
const WORD_PREFIX: u8 = 0x0b;
/// `DWordPrefix` (§20.2.3).
const DWORD_PREFIX: u8 = 0x0c;
/// `StringPrefix` (§20.2.3).
const STRING_PREFIX: u8 = 0x0d;
/// `QWordPrefix` (§20.2.3).
const QWORD_PREFIX: u8 = 0x0e;
/// `BufferOp` (§20.2.5.4).
const BUFFER_OP: u8 = 0x11;
/// `ZeroOp` (§20.2.3).
const ZERO_OP: u8 = 0x00;
/// `OneOp` (§20.2.3).
const ONE_OP: u8 = 0x01;
/// `RootChar`, `\` (§20.2.2).
const ROOT_CHAR: u8 = 0x5c;
/// `DualNamePrefix` (§20.2.2).
const DUAL_NAME_PREFIX: u8 = 0x2e;
/// `MultiNamePrefix` (§20.2.2).
const MULTI_NAME_PREFIX: u8 = 0x2f;

/// Encode `len` bytes of contents as a `PkgLength`, which counts itself.
///
/// §20.2.4. The self-reference is why this is a search rather than a formula:
/// adding the field can push the total past a boundary that needs a wider
/// field. Three iterations is enough for every length AML can express, and the
/// loop is bounded so a mistake here cannot hang.
#[must_use]
pub fn pkg_length(len: usize) -> Vec<u8> {
    let mut width = 1usize;
    for _ in 0..4 {
        let total = len + width;
        let needed = if total <= 0x3f {
            1
        } else if total <= 0x0fff {
            2
        } else if total <= 0x0f_ffff {
            3
        } else {
            4
        };
        if needed == width {
            break;
        }
        width = needed;
    }
    let total = len + width;
    let mut out = Vec::with_capacity(width);
    if width == 1 {
        // Bits 7:6 zero means no following bytes and bits 5:0 are the length.
        out.push(total as u8);
        return out;
    }
    let follow = width - 1;
    out.push(((follow as u8) << 6) | (total & 0x0f) as u8);
    let mut rest = total >> 4;
    for _ in 0..follow {
        out.push(rest as u8);
        rest >>= 8;
    }
    out
}

/// Encode one four-character `NameSeg`, padded with `_` (§20.2.2).
///
/// A segment longer than four characters is truncated, and one shorter is
/// padded, because that is what an ASL compiler does with a name the
/// specification says is exactly four characters.
#[must_use]
pub fn name_seg(name: &str) -> [u8; 4] {
    let mut seg = [b'_'; 4];
    for (slot, byte) in seg.iter_mut().zip(name.bytes()) {
        *slot = byte;
    }
    seg
}

/// Encode a `NameString` — one or more segments, optionally rooted (§20.2.2).
///
/// `path` is written the way ASL writes it: `\_SB.PCI0`, or `_S5_` for a name
/// in the current scope.
#[must_use]
pub fn name_string(path: &str) -> Vec<u8> {
    let mut out = Vec::new();
    let rest = match path.strip_prefix('\\') {
        Some(rest) => {
            out.push(ROOT_CHAR);
            rest
        }
        None => path,
    };
    let segs: Vec<[u8; 4]> = rest.split('.').map(name_seg).collect();
    match segs.len() {
        // `NullName` — a rooted path with nothing after it is just `\`.
        0 => out.push(0x00),
        1 => {}
        2 => out.push(DUAL_NAME_PREFIX),
        n => {
            out.push(MULTI_NAME_PREFIX);
            out.push(n as u8);
        }
    }
    for seg in segs {
        out.extend_from_slice(&seg);
    }
    out
}

/// Encode an integer with the narrowest prefix that holds it (§20.2.3).
///
/// `Zero` and `One` are opcodes rather than constants, which is why 0 and 1 do
/// not get a `BytePrefix`.
#[must_use]
pub fn integer(value: u64) -> Vec<u8> {
    let mut out = Vec::new();
    match value {
        0 => out.push(ZERO_OP),
        1 => out.push(ONE_OP),
        v if v <= 0xff => {
            out.push(BYTE_PREFIX);
            out.push(v as u8);
        }
        v if v <= 0xffff => {
            out.push(WORD_PREFIX);
            out.extend_from_slice(&(v as u16).to_le_bytes());
        }
        v if v <= 0xffff_ffff => {
            out.push(DWORD_PREFIX);
            out.extend_from_slice(&(v as u32).to_le_bytes());
        }
        v => {
            out.push(QWORD_PREFIX);
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    out
}

/// Encode a null-terminated `String` (§20.2.3).
#[must_use]
pub fn string(text: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(text.len() + 2);
    out.push(STRING_PREFIX);
    out.extend_from_slice(text.as_bytes());
    out.push(0);
    out
}

/// `Name(path, value)` (§20.2.5.1).
#[must_use]
pub fn name(path: &str, value: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    out.push(NAME_OP);
    out.extend_from_slice(&name_string(path));
    out.extend_from_slice(value);
    out
}

/// `Package(n) { … }` (§20.2.5.4).
///
/// `elements` is already-encoded data objects, and `count` is how many of them
/// there are — the encoding carries both, and they have to agree.
#[must_use]
pub fn package(count: u8, elements: &[u8]) -> Vec<u8> {
    // Contents are the element count byte plus the elements.
    let mut body = Vec::with_capacity(elements.len() + 1);
    body.push(count);
    body.extend_from_slice(elements);
    let mut out = Vec::new();
    out.push(PACKAGE_OP);
    out.extend_from_slice(&pkg_length(body.len()));
    out.extend_from_slice(&body);
    out
}

/// `Device(path) { … }` (§20.2.5.2).
#[must_use]
pub fn device(path: &str, body: &[u8]) -> Vec<u8> {
    let mut inner = name_string(path);
    inner.extend_from_slice(body);
    let mut out = Vec::new();
    out.push(EXT_OP_PREFIX);
    out.push(DEVICE_OP);
    out.extend_from_slice(&pkg_length(inner.len()));
    out.extend_from_slice(&inner);
    out
}

/// `Scope(path) { … }` (§20.2.5.1).
#[must_use]
pub fn scope(path: &str, body: &[u8]) -> Vec<u8> {
    let mut inner = name_string(path);
    inner.extend_from_slice(body);
    let mut out = Vec::new();
    out.push(SCOPE_OP);
    out.extend_from_slice(&pkg_length(inner.len()));
    out.extend_from_slice(&inner);
    out
}

/// `Buffer(n) { … }` (§20.2.5.4).
///
/// The size is written as an ordinary integer term, which is what an ASL
/// compiler emits for a buffer whose length is known — and it is the form a
/// resource template takes.
#[must_use]
pub fn buffer(bytes: &[u8]) -> Vec<u8> {
    let mut body = integer(bytes.len() as u64);
    body.extend_from_slice(bytes);
    let mut out = Vec::new();
    out.push(BUFFER_OP);
    out.extend_from_slice(&pkg_length(body.len()));
    out.extend_from_slice(&body);
    out
}

// ---------------------------------------------------------------------------
// resource templates
// ---------------------------------------------------------------------------
//
// A `_CRS` is a `Buffer` holding a byte stream of *resource descriptors*
// terminated by an end tag — a different encoding from AML proper, defined in
// ACPI §6.4. The three descriptors below are the ones a host bridge's windows
// need, and they are all the same shape: §6.4.3.5's address space descriptors,
// in the 16-bit, 32-bit and 64-bit widths.

/// `Word Address Space Descriptor`, large resource type 8 (§6.4.3.5.3).
const WORD_ADDRESS_SPACE: u8 = 0x88;
/// `DWord Address Space Descriptor`, large resource type 7 (§6.4.3.5.2).
const DWORD_ADDRESS_SPACE: u8 = 0x87;
/// Resource type 0: a memory range (§6.4.3.5, Table 6.42).
const SPACE_MEMORY: u8 = 0x00;
/// Resource type 1: an I/O range.
const SPACE_IO: u8 = 0x01;
/// Resource type 2: a bus number range.
const SPACE_BUS: u8 = 0x02;
/// General flags for a window a bridge **produces** for the bus below it, with
/// both edges fixed: `_DEC` positive, `_MIF` and `_MAF` set (Table 6.43).
const FLAGS_PRODUCER_FIXED: u8 = 0x0c;
/// The same, for a range the device itself **consumes** — bit 0 set.
const FLAGS_CONSUMER_FIXED: u8 = 0x0d;
/// Memory type flags: read/write, non-cacheable, an ordinary memory range
/// (Table 6.44).
const MEMORY_RW_NONCACHEABLE: u8 = 0x01;
/// I/O type flags: `_RNG` = 3, the entire range (Table 6.45).
const IO_ENTIRE_RANGE: u8 = 0x03;
/// `EndTag`, small resource type 15 with one length byte (§6.4.2.9).
const END_TAG: u8 = 0x79;

/// One address space descriptor, at `width` bytes per address field.
///
/// §6.4.3.5's layout is identical in all three widths: a type byte, two flag
/// bytes, and then granularity, minimum, maximum, translation offset and
/// length, each `width` bytes wide and little-endian.
fn address_space(
    tag: u8,
    width: usize,
    kind: u8,
    general: u8,
    specific: u8,
    min: u64,
    max: u64,
) -> Vec<u8> {
    let len = 3 + 5 * width;
    let mut out = Vec::with_capacity(3 + len);
    out.push(tag);
    out.extend_from_slice(&(len as u16).to_le_bytes());
    out.push(kind);
    out.push(general);
    out.push(specific);
    // Granularity is zero for a fixed window: §6.4.3.5 defines it as the bits
    // that may vary, and neither edge of a fixed one does.
    for value in [0, min, max, 0, max.wrapping_sub(min).wrapping_add(1)] {
        out.extend_from_slice(&value.to_le_bytes()[..width]);
    }
    out
}

/// `WordBusNumber(…, min, max, 0, max - min + 1)`.
#[must_use]
pub fn bus_number_range(min: u8, max: u8) -> Vec<u8> {
    address_space(
        WORD_ADDRESS_SPACE,
        2,
        SPACE_BUS,
        FLAGS_PRODUCER_FIXED,
        0,
        u64::from(min),
        u64::from(max),
    )
}

/// `DWordIO(ResourceProducer, MinFixed, MaxFixed, PosDecode, EntireRange, …)`.
#[must_use]
pub fn dword_io(min: u32, max: u32) -> Vec<u8> {
    address_space(
        DWORD_ADDRESS_SPACE,
        4,
        SPACE_IO,
        FLAGS_PRODUCER_FIXED,
        IO_ENTIRE_RANGE,
        u64::from(min),
        u64::from(max),
    )
}

/// `DWordMemory(…, NonCacheable, ReadWrite, …)`.
///
/// `produced` says whether this is a window the bridge hands to the bus below
/// it or a range the device itself consumes — the difference is bit 0 of the
/// general flags, and it is the whole difference between "allocate BARs here"
/// and "this address is already spoken for".
#[must_use]
pub fn dword_memory(min: u32, max: u32, produced: bool) -> Vec<u8> {
    address_space(
        DWORD_ADDRESS_SPACE,
        4,
        SPACE_MEMORY,
        if produced {
            FLAGS_PRODUCER_FIXED
        } else {
            FLAGS_CONSUMER_FIXED
        },
        MEMORY_RW_NONCACHEABLE,
        u64::from(min),
        u64::from(max),
    )
}

/// Wrap already-encoded descriptors as a `ResourceTemplate` buffer.
///
/// The end tag's second byte is a checksum, and zero is the defined "no
/// checksum" value (§6.4.2.9) — which is what every ASL compiler emits, because
/// a template is not transmitted anywhere that could corrupt it.
#[must_use]
pub fn resource_template(descriptors: &[u8]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(descriptors.len() + 2);
    bytes.extend_from_slice(descriptors);
    bytes.push(END_TAG);
    bytes.push(0);
    buffer(&bytes)
}

/// `EisaId("PNP0A08")` as the DWord constant it compiles to (ACPI §19.6.30).
///
/// Three upper-case letters compressed to five bits each, then four hexadecimal
/// digits, packed big-endian into 32 bits — and then **byte-swapped**, because
/// AML stores the DWord little-endian and the identifier is compared as a
/// big-endian quantity. That is why `PNP0A03` is famously `0x030ad041` in a
/// hex dump.
///
/// Returns `None` for anything that is not three letters followed by four
/// hexadecimal digits.
#[must_use]
pub fn eisa_id(id: &str) -> Option<u32> {
    let bytes = id.as_bytes();
    if bytes.len() != 7 {
        return None;
    }
    let mut packed = 0u32;
    for &b in &bytes[..3] {
        if !b.is_ascii_uppercase() {
            return None;
        }
        packed = (packed << 5) | u32::from(b - b'@');
    }
    let mut hex = 0u32;
    for &b in &bytes[3..] {
        hex = (hex << 4) | (b as char).to_digit(16)?;
    }
    Some(((packed << 16) | hex).swap_bytes())
}

/// `Name(path, EisaId(id))`, or `None` if `id` is not one.
#[must_use]
pub fn name_eisa_id(path: &str, id: &str) -> Option<Vec<u8>> {
    let value = eisa_id(id)?;
    // Always a full `DWordPrefix`, never narrowed: an `_HID` that shrank to a
    // `WordPrefix` would still evaluate to the same integer, but every dump of
    // every real DSDT shows the wide form and matching it costs nothing.
    let mut encoded = Vec::with_capacity(5);
    encoded.push(DWORD_PREFIX);
    encoded.extend_from_slice(&value.to_le_bytes());
    Some(name(path, &encoded))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_short_package_length_is_one_byte_and_counts_itself() {
        // 10 bytes of contents plus the one-byte field is 11.
        assert_eq!(pkg_length(10), alloc::vec![11]);
        // 62 contents + 1 = 63, the largest a single byte holds.
        assert_eq!(pkg_length(62), alloc::vec![63]);
    }

    #[test]
    fn a_length_that_crosses_the_boundary_widens_and_re_counts() {
        // 63 contents would be 64 in one byte, which does not fit, so the
        // field widens to two and the total becomes 65.
        let encoded = pkg_length(63);
        assert_eq!(encoded.len(), 2);
        let total = usize::from(encoded[0] & 0x0f) | (usize::from(encoded[1]) << 4);
        assert_eq!(total, 65);
        assert_eq!(encoded[0] >> 6, 1, "one following byte");
    }

    #[test]
    fn a_package_length_decodes_back_to_itself_at_every_width() {
        // §20.2.4's field counts itself, so the only check that means anything
        // is a round trip: decode what was encoded and get the total back. The
        // sizes below straddle both boundaries, and the largest is the shape a
        // `_PRT` with one row per device number and pin actually reaches — over
        // a kilobyte, which is the two-byte field.
        for len in [0usize, 1, 61, 62, 63, 64, 4090, 4091, 4092, 5000, 70_000] {
            let encoded = pkg_length(len);
            let follow = usize::from(encoded[0] >> 6);
            assert_eq!(encoded.len(), follow + 1, "len {len}");
            let total = if follow == 0 {
                usize::from(encoded[0])
            } else {
                assert_eq!(encoded[0] & 0x30, 0, "bits 5:4 are zero when bytes follow");
                let mut total = usize::from(encoded[0] & 0x0f);
                for (i, byte) in encoded[1..].iter().enumerate() {
                    total |= usize::from(*byte) << (4 + 8 * i);
                }
                total
            };
            assert_eq!(total, len + encoded.len(), "len {len}");
        }
    }

    #[test]
    fn a_name_segment_is_four_characters_padded_with_underscores() {
        assert_eq!(&name_seg("_S5_"), b"_S5_");
        assert_eq!(&name_seg("PCI0"), b"PCI0");
        assert_eq!(&name_seg("SB"), b"SB__");
    }

    #[test]
    fn a_rooted_two_segment_path_takes_the_dual_prefix() {
        assert_eq!(name_string("\\_SB.PCI0"), b"\\\x2e_SB_PCI0".to_vec());
        assert_eq!(name_string("_S5_"), b"_S5_".to_vec());
    }

    #[test]
    fn zero_and_one_are_opcodes_and_everything_else_takes_a_prefix() {
        assert_eq!(integer(0), alloc::vec![0x00]);
        assert_eq!(integer(1), alloc::vec![0x01]);
        assert_eq!(integer(5), alloc::vec![0x0a, 0x05]);
        assert_eq!(integer(0x1234), alloc::vec![0x0b, 0x34, 0x12]);
        assert_eq!(
            integer(0x1234_5678),
            alloc::vec![0x0c, 0x78, 0x56, 0x34, 0x12]
        );
        assert_eq!(integer(1 << 40)[0], 0x0e);
    }

    #[test]
    fn the_eisa_ids_a_pci_host_bridge_uses_are_the_known_constants() {
        // The two every DSDT dump in the world shows.
        assert_eq!(eisa_id("PNP0A03"), Some(0x030a_d041));
        assert_eq!(eisa_id("PNP0A08"), Some(0x080a_d041));
        assert_eq!(eisa_id("PNP0C0F"), Some(0x0f0c_d041));
        assert_eq!(eisa_id("pnp0a03"), None, "the letters are upper case");
        assert_eq!(eisa_id("PNP0A0"), None, "seven characters, not six");
    }

    #[test]
    fn a_package_carries_both_its_count_and_its_length() {
        let mut elements = integer(5);
        elements.extend_from_slice(&integer(0));
        let encoded = package(2, &elements);
        assert_eq!(encoded[0], PACKAGE_OP);
        // One length byte, then the element count, then the two elements.
        assert_eq!(encoded[1], (encoded.len() - 1) as u8);
        assert_eq!(encoded[2], 2);
        assert_eq!(&encoded[3..], &[0x0a, 0x05, 0x00]);
    }

    #[test]
    fn a_device_wraps_its_body_in_a_two_byte_opcode() {
        let body = name("_ADR", &integer(0));
        let encoded = device("PCI0", &body);
        assert_eq!(&encoded[..2], &[EXT_OP_PREFIX, DEVICE_OP]);
        // Opcode, length, name, body — and the length covers everything after
        // the two opcode bytes.
        assert_eq!(usize::from(encoded[2]), encoded.len() - 2);
        assert_eq!(&encoded[3..7], b"PCI0");
    }

    #[test]
    fn a_string_is_null_terminated() {
        assert_eq!(string("hi"), alloc::vec![0x0d, b'h', b'i', 0]);
    }
}
