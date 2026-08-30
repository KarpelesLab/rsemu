//! By-name device construction — the machine file's entry point into Rust.
//!
//! A `.machine` file names a class (`object cpu "mos6502"`); this is what turns
//! that string into a device. It is also the introspection surface: `rsemu
//! devices` and `rsemu describe` read the same table the resolver validates
//! against, so the documentation cannot drift from the code.
//!
//! # Registration is explicit
//!
//! Classes are added by a `register` function per feature, not by link-time
//! magic. `compcol::factory` is the precedent for the *naming convention* only:
//! it is a compile-time `match` over feature-gated arms with no registration
//! API, whereas a machine assembled at runtime needs a mutable table.
//!
//! ```
//! # use rsemu::core::registry::Registry;
//! let reg = Registry::new();
//! assert!(reg.get("pci.nvme").is_none()); // no device features in this build
//! ```

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass};
use crate::core::error::{Error, Result};
use crate::core::props::Props;

/// The set of device classes this build can instantiate.
///
/// Ordered by class name. A `BTreeMap` rather than a hash map because iteration
/// order reaches the user — `rsemu devices` output, and the candidate list in an
/// unknown-class error — and CLAUDE.md forbids hash iteration order anywhere it
/// can be observed.
#[derive(Debug, Default)]
pub struct Registry {
    classes: BTreeMap<&'static str, &'static DeviceClass>,
}

impl Registry {
    /// An empty registry.
    pub fn new() -> Registry {
        Registry {
            classes: BTreeMap::new(),
        }
    }

    /// Add a class.
    ///
    /// Registering the same name twice is an error rather than a silent
    /// replacement: two classes claiming one name means two features collided,
    /// and the machine that results would depend on registration order.
    pub fn add(&mut self, class: &'static DeviceClass) -> Result<()> {
        if self.classes.contains_key(class.name) {
            return Err(Error::Config {
                at: class.name.to_string(),
                message: "device class registered twice".to_string(),
            });
        }
        self.classes.insert(class.name, class);
        Ok(())
    }

    /// Look up a class by name.
    pub fn get(&self, name: &str) -> Option<&'static DeviceClass> {
        self.classes.get(name).copied()
    }

    /// Every registered class, in name order.
    pub fn classes(&self) -> impl Iterator<Item = &'static DeviceClass> + '_ {
        self.classes.values().copied()
    }

    /// How many classes are registered.
    pub fn len(&self) -> usize {
        self.classes.len()
    }

    /// Whether nothing is registered.
    pub fn is_empty(&self) -> bool {
        self.classes.is_empty()
    }

    /// Construct a device by class name.
    ///
    /// The device is allocated and its properties validated; nothing observable
    /// happens until `realize` (`ROADMAP.md` §4.4).
    pub fn create(&self, name: &str, props: &Props) -> Result<Box<dyn Device>> {
        let class = self.get(name).ok_or_else(|| self.unknown(name))?;
        (class.construct)(props)
    }

    /// The error for a class this build does not have.
    ///
    /// A machine *is* a feature set, so the overwhelmingly likely cause is a
    /// missing Cargo feature rather than a typo — but a near-miss suggestion
    /// covers the typo without hiding the common case.
    fn unknown(&self, name: &str) -> Error {
        let mut message = String::from("unknown device class `");
        message.push_str(name);
        message.push_str("` (is its feature enabled?)");

        if let Some(near) = self.nearest(name) {
            message.push_str("; did you mean `");
            message.push_str(near);
            message.push_str("`?");
        }
        Error::Config {
            at: String::from("registry"),
            message,
        }
    }

    /// The registered name closest to `name`, if any is close enough.
    ///
    /// Prefix and substring matches first — `nvme` for `pci.nvme` is the shape
    /// people actually get wrong, and an edit distance over a dotted name scores
    /// that badly because the prefix dominates the length.
    fn nearest(&self, name: &str) -> Option<&'static str> {
        let mut best: Option<(usize, &'static str)> = None;
        for candidate in self.classes.keys().copied() {
            let score = if candidate == name {
                0
            } else if candidate.ends_with(name) || candidate.contains(name) {
                1
            } else {
                let tail = candidate.rsplit('.').next().unwrap_or(candidate);
                let want = name.rsplit('.').next().unwrap_or(name);
                let d = edit_distance(tail, want);
                // Scale the threshold with the name: one edit in three letters
                // is a different device, four in twenty is a slip.
                if d * 4 <= want.len().max(tail.len()) {
                    d + 1
                } else {
                    continue;
                }
            };
            if best.is_none_or(|(b, _)| score < b) {
                best = Some((score, candidate));
            }
        }
        best.map(|(_, name)| name)
    }
}

/// Levenshtein distance, iterative with a single row.
///
/// Only ever run on a failure path over a handful of short names, so the naive
/// form is the right one — no allocation beyond one row.
fn edit_distance(a: &str, b: &str) -> usize {
    let b_chars: Vec<char> = b.chars().collect();
    let mut row: Vec<usize> = (0..=b_chars.len()).collect();
    for (i, ca) in a.chars().enumerate() {
        let mut diagonal = row[0];
        row[0] = i + 1;
        for (j, cb) in b_chars.iter().enumerate() {
            let cost = usize::from(ca != *cb);
            let next = (row[j + 1] + 1).min(row[j] + 1).min(diagonal + cost);
            diagonal = row[j + 1];
            row[j + 1] = next;
        }
    }
    row[b_chars.len()]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
    use crate::core::props::{Props, Value};

    #[derive(Debug)]
    struct Dummy;

    impl Device for Dummy {
        fn class(&self) -> &'static DeviceClass {
            &DUMMY
        }
        fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
            Ok(())
        }
        fn reset(&self, _kind: ResetKind) {}
    }

    static DUMMY: DeviceClass = DeviceClass {
        name: "test.dummy",
        version: 1,
        summary: "a device that does nothing",
        properties: &[],
        construct: |_props| Ok(Box::new(Dummy)),
    };

    static OTHER: DeviceClass = DeviceClass {
        name: "pci.nvme",
        version: 1,
        summary: "another",
        properties: &[],
        construct: |_props| Ok(Box::new(Dummy)),
    };

    fn registry() -> Registry {
        let mut r = Registry::new();
        r.add(&DUMMY).unwrap();
        r.add(&OTHER).unwrap();
        r
    }

    #[test]
    fn a_registered_class_can_be_constructed() {
        let r = registry();
        let d = r.create("test.dummy", &Props::new()).unwrap();
        assert_eq!(d.class().name, "test.dummy");
    }

    #[test]
    fn registering_a_name_twice_is_refused() {
        let mut r = Registry::new();
        r.add(&DUMMY).unwrap();
        // Silent replacement would make the machine depend on feature ordering.
        let e = r.add(&DUMMY).unwrap_err().to_string();
        assert!(e.contains("twice"), "{e}");
    }

    #[test]
    fn an_unknown_class_blames_the_feature_first() {
        let r = registry();
        let e = r
            .create("dev.missing", &Props::new())
            .unwrap_err()
            .to_string();
        assert!(e.contains("dev.missing"), "{e}");
        assert!(e.contains("feature"), "{e}");
    }

    #[test]
    fn a_bare_name_suggests_its_qualified_class() {
        // The mistake people actually make: writing `nvme` for `pci.nvme`.
        let r = registry();
        let e = r.create("nvme", &Props::new()).unwrap_err().to_string();
        assert!(e.contains("did you mean `pci.nvme`?"), "{e}");
    }

    #[test]
    fn a_typo_in_the_tail_is_suggested() {
        let r = registry();
        let e = r.create("pci.nvne", &Props::new()).unwrap_err().to_string();
        assert!(e.contains("did you mean `pci.nvme`?"), "{e}");
    }

    #[test]
    fn an_unrelated_name_gets_no_suggestion() {
        // A wrong guess is worse than none: it sends the reader down a path
        // that was never going to work.
        let r = registry();
        let e = r
            .create("completely.different", &Props::new())
            .unwrap_err()
            .to_string();
        assert!(!e.contains("did you mean"), "{e}");
    }

    #[test]
    fn classes_iterate_in_name_order() {
        let r = registry();
        let names: Vec<&str> = r.classes().map(|c| c.name).collect();
        assert_eq!(names, ["pci.nvme", "test.dummy"]);
        assert_eq!(r.len(), 2);
        assert!(!r.is_empty());
    }

    #[test]
    fn properties_reach_the_constructor() {
        static TAKES: DeviceClass = DeviceClass {
            name: "test.sized",
            version: 1,
            summary: "wants a size",
            properties: &[],
            construct: |props| {
                let mut r = props.reader();
                let n: u64 = r.require("size")?;
                assert_eq!(n, 2048);
                Ok(Box::new(Dummy))
            },
        };
        let mut reg = Registry::new();
        reg.add(&TAKES).unwrap();
        let props = Props::new().with("size", Value::Size(2048));
        assert!(reg.create("test.sized", &props).is_ok());
    }
}
