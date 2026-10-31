//! Reading a `LOADING.KWI`: the container a navi model's whole system ships
//! in on the map card.
//!
//! Only as much of the format as staging a boot needs: the header, the
//! partition manifest, and the boot descriptor (`pmb`) that tells the loader
//! where the kernel and the root go. The layout is
//! `docs/platforms/alphard-navi-kwi-format.md`'s, learnt by reading the
//! images themselves as data, and every claim this file relies on was checked
//! against all eleven models' images and the backup one.
//!
//! # The shape
//!
//! ```text
//!   0x00    header: version, a 12-byte signature, the model code at 0x18
//!   …       the manifest: a short header, then (addr, size, name) entries
//!   base    the logical image: the members back to back, at base + addr
//! ```
//!
//! Every container field is big-endian. The manifest's own length is in its
//! header, and the logical image starts where the manifest ends — the first
//! member, a BMP splash, carries its `BM` there. Members are found by name
//! and never by position.
//!
//! The four members a boot uses:
//!
//! | name | what it is |
//! | --- | --- |
//! | `pmb` | the boot descriptor, 512 bytes, little-endian (see [`BootDescriptor`]) |
//! | `rootfs` | a 256 KiB `PCRD` block followed by the ext2 root |
//! | `USRCONF` | the system manager's text configuration |
//! | `xipImage` | the raw execute-in-place kernel; its CRC-32 is in `pmb` |

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use crate::core::error::{Error, Result};

/// The 12 bytes at offset 4 that make a file a system KWI.
///
/// A map-data KWI (`HD0/ALLDATA.KWI`) carries different bytes, and is not
/// something a boot can use.
pub const SIGNATURE: [u8; 12] = [
    0x0f, 0x56, 0xa3, 0x00, 0x3c, 0x3c, 0x8a, 0x00, 0x07, 0x00, 0x01, 0x66,
];

/// The first 16 bytes of the manifest's header, which is how it is found:
/// its position is the same in every image seen, but nothing in the file
/// header points at it.
const MANIFEST_MAGIC: [u8; 16] = [
    0x00, 0x00, 0x00, 0x1c, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c,
];

/// How far into the file the manifest is looked for.
const MANIFEST_SEARCH: usize = 4 << 20;

/// The magic a boot descriptor opens with: a uImage's, which is what lets
/// U-Boot treat the staged kernel as one.
pub const PMB_MAGIC: u32 = 0x2705_1956;

/// How long the boot descriptor is.
pub const PMB_LEN: usize = 0x200;

/// Where in the descriptor USRCONF's address, length and sum are.
const PMB_USRCONF: usize = 0x1ec;

/// Where in the descriptor the version tag is: its last eight bytes.
const PMB_TAG: usize = 0x1f8;

/// One manifest entry, with its offset resolved to the file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Member {
    /// The member's name, as the manifest spells it.
    pub name: String,
    /// Where its first byte is in the file.
    pub offset: u64,
    /// How long it is.
    pub len: u64,
}

/// The boot descriptor: the `pmb` member, which the loader copies into memory
/// for U-Boot and which says where everything else goes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BootDescriptor {
    /// The start of the kernel's RAM: its data and BSS live from here.
    pub load_base: u32,
    /// Where the execute-in-place kernel is placed and entered.
    pub kernel_load: u32,
    /// How long the kernel is.
    pub kernel_size: u32,
    /// The CRC-32 of the kernel's bytes, which U-Boot checks before it jumps.
    pub kernel_crc: u32,
    /// Where the root filesystem's ext2 is placed.
    pub rootfs_load: u32,
    /// How long the `rootfs` member is.
    pub rootfs_size: u32,
    /// The kernel command line.
    pub cmdline: String,
    /// Where `USRCONF` is staged.
    pub usrconf_addr: u32,
    /// How long `USRCONF` is.
    pub usrconf_size: u32,
    /// The 32-bit additive sum of `USRCONF`'s bytes, taken as little-endian
    /// words.
    pub usrconf_sum: u32,
    /// The eight-character version tag the image carries (`VC5910KA`).
    pub tag: [u8; 8],
}

/// A parsed system KWI.
#[derive(Debug, Clone)]
pub struct Kwi {
    /// The four-character model code at offset 0x18: `HC59`, `HC60`, …
    pub model: String,
    /// Every manifest entry, in manifest order.
    pub members: Vec<Member>,
    /// The boot descriptor.
    pub pmb: BootDescriptor,
}

fn refuse(message: String) -> Error {
    Error::Config {
        at: String::from("navi.kwiboot"),
        message,
    }
}

fn be32(bytes: &[u8], at: usize) -> u32 {
    u32::from_be_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

fn le32(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

impl Kwi {
    /// Parse `image`, and check that what a boot needs is there and intact.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] naming what is wrong: not a system KWI, no manifest,
    /// a member missing or running past the end of the file, a boot
    /// descriptor without its magic, or a kernel whose CRC-32 is not the one
    /// the descriptor records — which is a damaged or modified image, and
    /// exactly what U-Boot would have refused too.
    pub fn parse(image: &[u8]) -> Result<Kwi> {
        if image.len() < 0x20 || image[4..16] != SIGNATURE {
            return Err(refuse(String::from(
                "this is not a system KWI: the 12 bytes at offset 4 are not the \
                 `0f 56 a3 00 3c 3c 8a 00 07 00 01 66` a `LOADING.KWI` carries",
            )));
        }
        let model: String = image[0x18..0x1c]
            .iter()
            .map(|&b| if b.is_ascii_graphic() { b as char } else { '?' })
            .collect();

        let window = &image[..image.len().min(MANIFEST_SEARCH)];
        let at = window
            .windows(MANIFEST_MAGIC.len())
            .position(|w| w == MANIFEST_MAGIC)
            .ok_or_else(|| {
                refuse(String::from(
                    "no partition manifest in the first 4 MiB of the KWI",
                ))
            })?;
        if at + 0x1e > image.len() {
            return Err(refuse(String::from("the KWI ends inside its manifest")));
        }
        let manifest_len = be32(image, at + 0x14) as usize;
        let table_len = usize::from(u16::from_be_bytes([image[at + 0x1c], image[at + 0x1d]]));
        let base = at + manifest_len;
        let table_end = at + 0x1e + table_len;
        if table_end > base || base > image.len() {
            return Err(refuse(format!(
                "the manifest at {at:#x} is inconsistent: its table ends at {table_end:#x} \
                 and the image it describes starts at {base:#x}"
            )));
        }

        let mut members = Vec::new();
        let mut p = at + 0x1e;
        while p + 10 <= table_end {
            let addr = u64::from(be32(image, p));
            let len = u64::from(be32(image, p + 4));
            let name_len = usize::from(u16::from_be_bytes([image[p + 8], image[p + 9]]));
            let name_end = p + 10 + name_len;
            if name_end > table_end {
                return Err(refuse(format!(
                    "manifest entry at {p:#x} runs past the end of its table"
                )));
            }
            let name = String::from_utf8_lossy(&image[p + 10..name_end]).into_owned();
            let offset = base as u64 + addr;
            if offset + len > image.len() as u64 {
                return Err(refuse(format!(
                    "member `{name}` ({len:#x} bytes at file offset {offset:#x}) runs past \
                     the end of the {:#x}-byte file: a truncated copy?",
                    image.len()
                )));
            }
            members.push(Member { name, offset, len });
            // Entries are 2-byte aligned.
            p = name_end + (name_len & 1);
        }

        let mut kwi = Kwi {
            model,
            members,
            pmb: BootDescriptor {
                load_base: 0,
                kernel_load: 0,
                kernel_size: 0,
                kernel_crc: 0,
                rootfs_load: 0,
                rootfs_size: 0,
                cmdline: String::new(),
                usrconf_addr: 0,
                usrconf_size: 0,
                usrconf_sum: 0,
                tag: [0; 8],
            },
        };
        let pmb = kwi.bytes(image, "pmb")?;
        if pmb.len() < PMB_LEN || le32(pmb, 0) != PMB_MAGIC {
            return Err(refuse(format!(
                "the `pmb` member is not a boot descriptor: it should be {PMB_LEN} bytes \
                 opening with {PMB_MAGIC:#010x}"
            )));
        }
        // The command line runs to the first NUL; the last 20 bytes of the
        // descriptor are USRCONF's placement and the version tag.
        let cmdline = &pmb[0x20..PMB_USRCONF];
        let cmdline = &cmdline[..cmdline
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(cmdline.len())];
        let mut tag = [0u8; 8];
        tag.copy_from_slice(&pmb[PMB_TAG..PMB_TAG + 8]);
        kwi.pmb = BootDescriptor {
            load_base: le32(pmb, 0x08),
            kernel_load: le32(pmb, 0x0c),
            kernel_size: le32(pmb, 0x10),
            kernel_crc: le32(pmb, 0x14),
            rootfs_load: le32(pmb, 0x18),
            rootfs_size: le32(pmb, 0x1c),
            cmdline: String::from_utf8_lossy(cmdline).into_owned(),
            usrconf_addr: le32(pmb, PMB_USRCONF),
            usrconf_size: le32(pmb, PMB_USRCONF + 4),
            usrconf_sum: le32(pmb, PMB_USRCONF + 8),
            tag,
        };

        let usrconf = kwi.bytes(image, "USRCONF")?;
        if usrconf.len() as u64 != u64::from(kwi.pmb.usrconf_size) {
            return Err(refuse(format!(
                "the boot descriptor says USRCONF is {:#x} bytes and the manifest's member \
                 is {:#x}",
                kwi.pmb.usrconf_size,
                usrconf.len()
            )));
        }
        let sum = word_sum(usrconf);
        if sum != kwi.pmb.usrconf_sum {
            return Err(refuse(format!(
                "USRCONF sums to {sum:#010x} and the boot descriptor records {:#010x}: the \
                 image is damaged or modified",
                kwi.pmb.usrconf_sum
            )));
        }

        let kernel = kwi.bytes(image, "xipImage")?;
        if kernel.len() as u64 != u64::from(kwi.pmb.kernel_size) {
            return Err(refuse(format!(
                "the boot descriptor says the kernel is {:#x} bytes and the manifest's \
                 `xipImage` is {:#x}",
                kwi.pmb.kernel_size,
                kernel.len()
            )));
        }
        let crc = crc32(kernel);
        if crc != kwi.pmb.kernel_crc {
            return Err(refuse(format!(
                "the kernel's CRC-32 is {crc:#010x} and the boot descriptor records {:#010x}: \
                 the image is damaged or modified, and U-Boot would refuse it too",
                kwi.pmb.kernel_crc
            )));
        }
        let rootfs = kwi.member("rootfs")?;
        if rootfs.len != u64::from(kwi.pmb.rootfs_size) {
            return Err(refuse(format!(
                "the boot descriptor says the root is {:#x} bytes and the manifest's \
                 `rootfs` is {:#x}",
                kwi.pmb.rootfs_size, rootfs.len
            )));
        }
        Ok(kwi)
    }

    /// The member called `name`.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the manifest has none.
    pub fn member(&self, name: &str) -> Result<&Member> {
        self.members
            .iter()
            .find(|m| m.name == name)
            .ok_or_else(|| refuse(format!("the KWI's manifest has no `{name}` member")))
    }

    /// The bytes of the member called `name`, out of `image`.
    ///
    /// # Errors
    ///
    /// As [`Kwi::member`].
    pub fn bytes<'a>(&self, image: &'a [u8], name: &str) -> Result<&'a [u8]> {
        let m = self.member(name)?;
        // `parse` checked every member against the file's length.
        Ok(&image[m.offset as usize..(m.offset + m.len) as usize])
    }
}

/// The 32-bit additive sum of `bytes` taken as little-endian words, a short
/// tail padded with zeros. The checksum the OSMEM records use.
#[must_use]
pub fn word_sum(bytes: &[u8]) -> u32 {
    bytes.chunks(4).fold(0u32, |sum, w| {
        let mut word = [0u8; 4];
        word[..w.len()].copy_from_slice(w);
        sum.wrapping_add(u32::from_le_bytes(word))
    })
}

/// CRC-32 as zlib and U-Boot compute it: reflected polynomial 0xedb88320,
/// initial value and final XOR all ones.
#[must_use]
pub fn crc32(bytes: &[u8]) -> u32 {
    const TABLE: [u32; 256] = {
        let mut table = [0u32; 256];
        let mut i = 0;
        while i < 256 {
            let mut c = i as u32;
            let mut k = 0;
            while k < 8 {
                c = if c & 1 != 0 {
                    0xedb8_8320 ^ (c >> 1)
                } else {
                    c >> 1
                };
                k += 1;
            }
            table[i] = c;
            i += 1;
        }
        table
    };
    let mut c = !0u32;
    for &b in bytes {
        c = TABLE[((c ^ u32::from(b)) & 0xff) as usize] ^ (c >> 8);
    }
    !c
}

impl core::fmt::Display for Member {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{} ({:#x} bytes at {:#x})",
            self.name, self.len, self.offset
        )
    }
}

/// A small, made-up system KWI with the real layout, for tests: no vendor
/// bytes, so it can live in the tree.
#[cfg(test)]
pub(crate) mod fake {
    use super::*;

    /// Where things go in the fake's boot descriptor.
    pub(crate) const LOAD_BASE: u32 = 0x6030_0000;
    /// Where the fake kernel goes.
    pub(crate) const KERNEL_LOAD: u32 = 0x6034_0000;
    /// Where the fake root goes.
    pub(crate) const ROOTFS_LOAD: u32 = 0x6036_0000;
    /// Where the fake USRCONF goes.
    pub(crate) const USRCONF_ADDR: u32 = 0x6010_c000;
    /// The fake's command line.
    pub(crate) const CMDLINE: &str = "root=/dev/pcrd quiet";

    /// The fake's members: kernel, root, USRCONF.
    pub(crate) fn members() -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let kernel: Vec<u8> = (0..0x400u32).map(|i| (i * 7) as u8).collect();
        let mut rootfs = b"PCRD".to_vec();
        rootfs.resize(0x300, 0x5a);
        let usrconf = b"SMNG_PROCESSNAME0\tNAVI\t1\n".to_vec();
        (kernel, rootfs, usrconf)
    }

    /// Build it. `model` is the four-character code.
    pub(crate) fn build(model: &[u8; 4]) -> Vec<u8> {
        let (kernel, rootfs, usrconf) = members();
        let mut pmb = Vec::new();
        for w in [
            PMB_MAGIC,
            0,
            LOAD_BASE,
            KERNEL_LOAD,
            kernel.len() as u32,
            crc32(&kernel),
            ROOTFS_LOAD,
            rootfs.len() as u32,
        ] {
            pmb.extend_from_slice(&w.to_le_bytes());
        }
        pmb.extend_from_slice(CMDLINE.as_bytes());
        pmb.resize(PMB_USRCONF, 0);
        for w in [USRCONF_ADDR, usrconf.len() as u32, word_sum(&usrconf)] {
            pmb.extend_from_slice(&w.to_le_bytes());
        }
        pmb.extend_from_slice(b"VC0000KA");
        assert_eq!(pmb.len(), PMB_LEN);

        // A splash first, as on the card, so nothing sits at logical 0 by luck.
        let members: [(&str, Vec<u8>); 5] = [
            ("loading.bmp", b"BM splash".to_vec()),
            ("pmb", pmb),
            ("rootfs", rootfs),
            ("USRCONF", usrconf),
            ("xipImage", kernel),
        ];
        let mut table = Vec::new();
        let mut addr = 0u32;
        for (name, bytes) in &members {
            table.extend_from_slice(&addr.to_be_bytes());
            table.extend_from_slice(&(bytes.len() as u32).to_be_bytes());
            table.extend_from_slice(&(name.len() as u16).to_be_bytes());
            table.extend_from_slice(name.as_bytes());
            if name.len() % 2 == 1 {
                table.push(0);
            }
            addr += bytes.len() as u32;
        }

        let mut out = Vec::new();
        out.extend_from_slice(&0x100u32.to_be_bytes());
        out.extend_from_slice(&SIGNATURE);
        out.extend_from_slice(&0x100u32.to_be_bytes());
        out.extend_from_slice(&1u32.to_be_bytes());
        out.extend_from_slice(model);
        out.resize(0x200, 0);
        let manifest = out.len();
        out.extend_from_slice(&MANIFEST_MAGIC);
        out.extend_from_slice(&0x7au32.to_be_bytes());
        out.extend_from_slice(&((0x1e + table.len()) as u32).to_be_bytes());
        out.extend_from_slice(&addr.to_be_bytes());
        out.extend_from_slice(&(table.len() as u16).to_be_bytes());
        out.extend_from_slice(&table);
        assert_eq!(out.len(), manifest + 0x1e + table.len());
        for (_, bytes) in &members {
            out.extend_from_slice(bytes);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_is_zlibs() {
        // The check value every CRC-32 catalogue gives for "123456789".
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn a_word_sum_pads_its_tail_with_zeros() {
        assert_eq!(word_sum(&[1, 0, 0, 0, 2, 0, 0, 0]), 3);
        assert_eq!(word_sum(&[1, 0, 0, 0, 0xff]), 0x100);
        assert_eq!(
            word_sum(&0xffff_ffffu32.to_le_bytes().repeat(2)),
            0xffff_fffe
        );
    }

    #[test]
    fn the_manifest_and_the_descriptor_parse() {
        let image = fake::build(b"HC60");
        let kwi = Kwi::parse(&image).expect("a well-formed KWI");
        assert_eq!(kwi.model, "HC60");
        let names: Vec<&str> = kwi.members.iter().map(|m| m.name.as_str()).collect();
        assert_eq!(
            names,
            ["loading.bmp", "pmb", "rootfs", "USRCONF", "xipImage"]
        );
        let (kernel, rootfs, usrconf) = fake::members();
        assert_eq!(kwi.bytes(&image, "xipImage").unwrap(), &kernel[..]);
        assert_eq!(kwi.bytes(&image, "rootfs").unwrap(), &rootfs[..]);
        assert_eq!(kwi.bytes(&image, "USRCONF").unwrap(), &usrconf[..]);
        assert_eq!(kwi.pmb.kernel_load, fake::KERNEL_LOAD);
        assert_eq!(kwi.pmb.rootfs_load, fake::ROOTFS_LOAD);
        assert_eq!(kwi.pmb.load_base, fake::LOAD_BASE);
        assert_eq!(kwi.pmb.usrconf_addr, fake::USRCONF_ADDR);
        assert_eq!(kwi.pmb.cmdline, fake::CMDLINE);
        assert_eq!(&kwi.pmb.tag, b"VC0000KA");
    }

    #[test]
    fn a_file_that_is_not_a_system_kwi_is_refused() {
        let e = Kwi::parse(&[0u8; 64])
            .expect_err("no signature")
            .to_string();
        assert!(e.contains("not a system KWI"), "{e}");
    }

    #[test]
    fn a_modified_kernel_is_refused_as_u_boot_would() {
        let mut image = fake::build(b"HC59");
        let last = image.len() - 1;
        image[last] ^= 1;
        let e = Kwi::parse(&image).expect_err("CRC mismatch").to_string();
        assert!(e.contains("CRC-32"), "{e}");
    }

    #[test]
    fn a_truncated_copy_says_so() {
        let image = fake::build(b"HC59");
        let e = Kwi::parse(&image[..image.len() - 16])
            .expect_err("short")
            .to_string();
        assert!(e.contains("truncated"), "{e}");
    }
}
