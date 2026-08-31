//! A writer for the flattened device tree (DTB) format.
//!
//! # Source
//!
//! *Devicetree Specification*, release v0.4, chapter 5 ("Flattened Devicetree
//! (DTB) Format") — <https://www.devicetree.org/specifications/>. Everything
//! here is that chapter and nothing else: the header layout of §5.2, the memory
//! reservation block of §5.3, the structure block tokens of §5.4 and the
//! strings block of §5.5. The document is published for exactly this purpose,
//! so no other source was consulted.
//!
//! # Why we generate rather than ship one
//!
//! `docs/platforms/riscv-virt.md` is explicit: the device tree is produced
//! *mechanically from the realized machine graph*, because a topology that
//! cannot describe itself is a topology that is only accidentally right. This
//! module is the encoder half of that; [`dt`](super::dt) is the half that walks
//! the machine.
//!
//! # Shape of the API
//!
//! An [`FdtWriter`] is a builder with a cursor: [`begin_node`] opens one,
//! [`end_node`] closes it, and property calls in between attach to whichever is
//! open. Nesting errors are caught at [`finish`] rather than by the type
//! system, because the natural way to build a tree from a machine graph is a
//! loop, not a nest of closures.
//!
//! [`begin_node`]: FdtWriter::begin_node
//! [`end_node`]: FdtWriter::end_node
//! [`finish`]: FdtWriter::finish

use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::error::{Error, Result};

/// `FDT_MAGIC` — the four bytes every DTB starts with (§5.2).
pub const FDT_MAGIC: u32 = 0xd00d_feed;

/// The format version this writer emits (§5.2). Version 17 is the current one
/// and the only one anything modern reads.
pub const FDT_VERSION: u32 = 17;

/// The oldest version whose readers can still parse what we emit (§5.2).
pub const FDT_LAST_COMP_VERSION: u32 = 16;

/// `FDT_BEGIN_NODE` (§5.4.1).
const TOK_BEGIN_NODE: u32 = 0x0000_0001;
/// `FDT_END_NODE` (§5.4.1).
const TOK_END_NODE: u32 = 0x0000_0002;
/// `FDT_PROP` (§5.4.1).
const TOK_PROP: u32 = 0x0000_0003;
/// `FDT_END` (§5.4.1).
const TOK_END: u32 = 0x0000_0009;

/// The header is ten big-endian 32-bit words (§5.2).
const HEADER_LEN: usize = 40;

/// Builds a flattened device tree.
///
/// Every integer in the output is big-endian, which is the format's own byte
/// order and has nothing to do with the guest's (§5.2).
#[derive(Debug, Default)]
pub struct FdtWriter {
    /// The structure block: tokens and inline property data (§5.4).
    structs: Vec<u8>,
    /// The strings block: property names, deduplicated (§5.5).
    strings: Vec<u8>,
    /// Name to offset in `strings`, so a name that appears in forty nodes is
    /// stored once. A `BTreeMap` rather than a hash map because nothing in
    /// rsemu may depend on hash order (`CLAUDE.md`, determinism) — and because
    /// this blob lands in guest memory, so a tree that differed run to run
    /// would make the machine's state hash differ too.
    interned: BTreeMap<String, u32>,
    /// Reserved memory ranges (§5.3).
    reservations: Vec<(u64, u64)>,
    /// How many nodes are open, so `finish` can refuse an unbalanced tree.
    depth: usize,
    /// The `boot_cpuid_phys` header field (§5.2).
    boot_cpu: u32,
}

impl FdtWriter {
    /// An empty tree.
    #[must_use]
    pub fn new() -> FdtWriter {
        FdtWriter::default()
    }

    /// Set the header's `boot_cpuid_phys` — the hart id the firmware entered on.
    pub fn set_boot_cpu(&mut self, hartid: u32) {
        self.boot_cpu = hartid;
    }

    /// Reserve a physical range so the client program leaves it alone (§5.3).
    pub fn reserve(&mut self, address: u64, size: u64) {
        self.reservations.push((address, size));
    }

    /// Open a node. `name` is the full node name including any `@unit-address`.
    ///
    /// The root node's name is the empty string, which is what the
    /// specification says and what every reader expects (§5.4.1).
    pub fn begin_node(&mut self, name: &str) {
        self.structs
            .extend_from_slice(&TOK_BEGIN_NODE.to_be_bytes());
        self.structs.extend_from_slice(name.as_bytes());
        self.structs.push(0);
        pad4(&mut self.structs);
        self.depth += 1;
    }

    /// Close the innermost open node.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if no node is open. An unbalanced tree is a bug in the
    /// caller, and encoding one produces a DTB that parses into the wrong
    /// shape rather than one that fails to parse — much worse.
    pub fn end_node(&mut self) -> Result<()> {
        if self.depth == 0 {
            return Err(malformed("`end_node` with no node open"));
        }
        self.depth -= 1;
        self.structs.extend_from_slice(&TOK_END_NODE.to_be_bytes());
        Ok(())
    }

    /// A property with an arbitrary byte value (§5.4.1's `FDT_PROP`).
    pub fn prop_bytes(&mut self, name: &str, value: &[u8]) {
        let name_off = self.intern(name);
        self.structs.extend_from_slice(&TOK_PROP.to_be_bytes());
        self.structs
            .extend_from_slice(&(value.len() as u32).to_be_bytes());
        self.structs.extend_from_slice(&name_off.to_be_bytes());
        self.structs.extend_from_slice(value);
        pad4(&mut self.structs);
    }

    /// A property with no value, as `ranges;` and `interrupt-controller;` are.
    pub fn prop_empty(&mut self, name: &str) {
        self.prop_bytes(name, &[]);
    }

    /// A property holding one null-terminated string.
    pub fn prop_str(&mut self, name: &str, value: &str) {
        let mut bytes = Vec::with_capacity(value.len() + 1);
        bytes.extend_from_slice(value.as_bytes());
        bytes.push(0);
        self.prop_bytes(name, &bytes);
    }

    /// A property holding a list of null-terminated strings, as `compatible` is.
    pub fn prop_str_list(&mut self, name: &str, values: &[&str]) {
        let mut bytes = Vec::new();
        for value in values {
            bytes.extend_from_slice(value.as_bytes());
            bytes.push(0);
        }
        self.prop_bytes(name, &bytes);
    }

    /// A property holding one 32-bit cell.
    pub fn prop_u32(&mut self, name: &str, value: u32) {
        self.prop_bytes(name, &value.to_be_bytes());
    }

    /// A property holding a 64-bit value as two cells, high cell first.
    ///
    /// The halves are not a 64-bit big-endian integer by coincidence: they are
    /// two independent cells that happen to concatenate the same way.
    pub fn prop_u64(&mut self, name: &str, value: u64) {
        self.prop_bytes(name, &value.to_be_bytes());
    }

    /// A property holding a list of 32-bit cells.
    pub fn prop_cells(&mut self, name: &str, cells: &[u32]) {
        let mut bytes = Vec::with_capacity(cells.len() * 4);
        for cell in cells {
            bytes.extend_from_slice(&cell.to_be_bytes());
        }
        self.prop_bytes(name, &bytes);
    }

    /// A `reg` property of `(address, size)` pairs in two-cell form.
    ///
    /// Two cells each is what a 64-bit board declares at the root, because its
    /// addresses do not fit in one.
    pub fn prop_reg64(&mut self, pairs: &[(u64, u64)]) {
        let mut cells = Vec::with_capacity(pairs.len() * 4);
        for (addr, size) in pairs {
            cells.push((addr >> 32) as u32);
            cells.push(*addr as u32);
            cells.push((size >> 32) as u32);
            cells.push(*size as u32);
        }
        self.prop_cells("reg", &cells);
    }

    /// Finish the tree and produce the DTB.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if a node is still open, or if the tree is so large
    /// that its offsets do not fit the header's 32-bit fields.
    pub fn finish(mut self) -> Result<Vec<u8>> {
        if self.depth != 0 {
            return Err(malformed(&format!(
                "{} node(s) left open at the end of the tree",
                self.depth
            )));
        }
        self.structs.extend_from_slice(&TOK_END.to_be_bytes());

        // §5.3: the reservation block is 8-byte aligned and terminated by an
        // all-zero entry. It is written even when empty — a reader looks for
        // the terminator, not for a count.
        let off_mem_rsvmap = HEADER_LEN;
        let mut rsv = Vec::with_capacity((self.reservations.len() + 1) * 16);
        for (address, size) in &self.reservations {
            rsv.extend_from_slice(&address.to_be_bytes());
            rsv.extend_from_slice(&size.to_be_bytes());
        }
        rsv.extend_from_slice(&0u64.to_be_bytes());
        rsv.extend_from_slice(&0u64.to_be_bytes());

        let off_dt_struct = off_mem_rsvmap + rsv.len();
        let off_dt_strings = off_dt_struct + self.structs.len();
        let total = off_dt_strings + self.strings.len();

        let fits = |v: usize| u32::try_from(v).map_err(|_| malformed("tree does not fit in 4 GiB"));

        let mut out = Vec::with_capacity(total);
        for word in [
            FDT_MAGIC,
            fits(total)?,
            fits(off_dt_struct)?,
            fits(off_dt_strings)?,
            fits(off_mem_rsvmap)?,
            FDT_VERSION,
            FDT_LAST_COMP_VERSION,
            self.boot_cpu,
            fits(self.strings.len())?,
            fits(self.structs.len())?,
        ] {
            out.extend_from_slice(&word.to_be_bytes());
        }
        out.extend_from_slice(&rsv);
        out.extend_from_slice(&self.structs);
        out.extend_from_slice(&self.strings);
        Ok(out)
    }

    /// The offset of `name` in the strings block, adding it if new.
    fn intern(&mut self, name: &str) -> u32 {
        if let Some(off) = self.interned.get(name) {
            return *off;
        }
        let off = self.strings.len() as u32;
        self.strings.extend_from_slice(name.as_bytes());
        self.strings.push(0);
        self.interned.insert(name.to_string(), off);
        off
    }
}

/// Pad `bytes` out to a 4-byte boundary with zeros.
///
/// Node names and property data in the structure block are 4-byte aligned
/// (§5.4.1); the tokens themselves then land aligned for free.
fn pad4(bytes: &mut Vec<u8>) {
    while !bytes.len().is_multiple_of(4) {
        bytes.push(0);
    }
}

/// A tree the writer refuses to encode.
fn malformed(message: &str) -> Error {
    Error::Config {
        at: "device tree".to_string(),
        message: message.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal reader, so the tests check the bytes against the format
    /// rather than against the writer's own idea of them.
    fn header(dtb: &[u8]) -> [u32; 10] {
        let mut out = [0u32; 10];
        for (i, slot) in out.iter_mut().enumerate() {
            let at = i * 4;
            *slot = u32::from_be_bytes([dtb[at], dtb[at + 1], dtb[at + 2], dtb[at + 3]]);
        }
        out
    }

    #[test]
    fn an_empty_root_is_a_valid_tree() {
        let mut w = FdtWriter::new();
        w.begin_node("");
        w.end_node().unwrap();
        let dtb = w.finish().unwrap();

        let h = header(&dtb);
        assert_eq!(h[0], FDT_MAGIC);
        assert_eq!(h[1] as usize, dtb.len(), "totalsize covers the whole blob");
        assert_eq!(h[5], FDT_VERSION);
        assert_eq!(h[6], FDT_LAST_COMP_VERSION);
        // The struct block is FDT_BEGIN_NODE, a padded empty name,
        // FDT_END_NODE, FDT_END: four words.
        assert_eq!(h[9], 16);
        // And an empty reservation block is still its terminator.
        assert_eq!(h[4] as usize, HEADER_LEN);
        assert_eq!(h[2] as usize, HEADER_LEN + 16);
    }

    #[test]
    fn a_property_name_appears_once_however_many_nodes_use_it() {
        // The strings block is the reason the format is compact at all, and a
        // writer that forgot to intern would still produce a readable tree —
        // so this has to be asserted rather than assumed.
        let mut w = FdtWriter::new();
        w.begin_node("");
        for name in ["a@0", "b@1", "c@2"] {
            w.begin_node(name);
            w.prop_str("compatible", "test");
            w.end_node().unwrap();
        }
        w.end_node().unwrap();
        let dtb = w.finish().unwrap();
        let h = header(&dtb);
        assert_eq!(h[8], "compatible".len() as u32 + 1);
    }

    #[test]
    fn an_unbalanced_tree_is_refused_rather_than_encoded() {
        let mut w = FdtWriter::new();
        w.begin_node("");
        let e = w.finish().unwrap_err().to_string();
        assert!(e.contains("open"), "{e}");

        let mut w = FdtWriter::new();
        assert!(w.end_node().is_err(), "nothing is open");
    }

    #[test]
    fn reservations_are_written_with_their_terminator() {
        let mut w = FdtWriter::new();
        w.reserve(0x8000_0000, 0x1000);
        w.begin_node("");
        w.end_node().unwrap();
        let dtb = w.finish().unwrap();
        let h = header(&dtb);
        let at = h[4] as usize;
        assert_eq!(&dtb[at..at + 8], &0x8000_0000u64.to_be_bytes());
        assert_eq!(&dtb[at + 8..at + 16], &0x1000u64.to_be_bytes());
        assert_eq!(&dtb[at + 16..at + 32], &[0u8; 16], "the terminator");
        assert_eq!(h[2] as usize, at + 32, "and the struct block follows it");
    }

    #[test]
    fn cells_and_reg_pairs_are_big_endian_whatever_the_guest_is() {
        let mut w = FdtWriter::new();
        w.begin_node("");
        w.prop_reg64(&[(0x1000_0000, 0x100)]);
        w.end_node().unwrap();
        let dtb = w.finish().unwrap();
        let needle = [
            0u8, 0, 0, 0, 0x10, 0, 0, 0, // address: high cell, then low
            0, 0, 0, 0, 0, 0, 1, 0, // size
        ];
        assert!(
            dtb.windows(needle.len()).any(|w| w == needle),
            "the reg cells are not where the format says"
        );
    }

    #[test]
    fn the_output_is_byte_identical_across_runs() {
        let build = || {
            let mut w = FdtWriter::new();
            w.begin_node("");
            w.prop_u32("#address-cells", 2);
            w.prop_u32("#size-cells", 2);
            w.begin_node("soc");
            w.prop_str_list("compatible", &["simple-bus"]);
            w.prop_empty("ranges");
            w.end_node().unwrap();
            w.end_node().unwrap();
            w.finish().unwrap()
        };
        assert_eq!(build(), build());
    }
}
