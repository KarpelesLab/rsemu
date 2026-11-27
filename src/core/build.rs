//! Which rsemu this is.
//!
//! A machine is a feature set (`ROADMAP.md` §3), so "which rsemu is this?" is a
//! real question with a build-specific answer: a binary that cannot run a Game
//! Boy is not broken, it was built without `machine-gameboy`. `rsemu --version`,
//! the C ABI's `rsemu_build_info` and the browser module's version line all ask
//! here.
//!
//! # The list is the manifest's, and a test says so
//!
//! The answer used to be a hand-written run of `if cfg!(feature = "…")` that
//! named twenty-eight features of more than two hundred — `machine-nes` but not
//! `machine-gameboy` — because nothing made anybody add a line when they added
//! a feature. [`FEATURES`] is still written out, since `cfg!` needs a literal
//! and the crate has no build script, but it can no longer drift:
//!
//! * a name here that `Cargo.toml` does not declare is an `unexpected_cfgs`
//!   warning, which CI builds as an error;
//! * a feature `Cargo.toml` declares that is missing here fails
//!   `every_declared_feature_is_listed`, which reads the manifest.
//!
//! `default` is the one feature left out: it is a set of other features, not a
//! thing a build has.

use alloc::string::String;
use alloc::vec::Vec;

/// The crate version, as reported by `rsemu --version`.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Declare [`FEATURES`]: every name once, as the literal `cfg!` needs.
macro_rules! features {
    ($($name:literal,)*) => {
        /// Every feature the manifest declares, in the manifest's order, and
        /// whether this build has it.
        pub const FEATURES: &[(&str, bool)] = &[$(($name, cfg!(feature = $name)),)*];
    };
}

features! {
    "full",
    "std",
    "cli",
    "ffi",
    "gdb",
    "vnc",
    "window",
    "monitor",
    "trace",
    "display-png",
    "demo",
    "dev-nes-cart",
    "dev-nes-apu",
    "wasm",
    "wasm-threads",
    "dev-nes-io",
    "dev-nes-ppu",
    "machine-nes",
    "machine-snes",
    "dev-apple1",
    "machine-apple1",
    "float",
    "cpu-riscv",
    "cpu-riscv-lift",
    "cpu-x86-lift",
    "dev-mos8520",
    "dev-wdc",
    "machine-beneater",
    "bus-pci",
    "bus-spi",
    "bus-usb",
    "dev-usb-ehci",
    "dev-usb-chipidea",
    "dev-usb-dwc2",
    "dev-usb-hid",
    "dev-usb-hub",
    "dev-usb-msd",
    "dev-usb-xhci",
    "dev-usb-xhci-pci",
    "dev-st7272a",
    "dev-ssd1306",
    "dev-st77xx",
    "dev-lcdc",
    "machine-spi-panel",
    "machine-oled-spi",
    "machine-tft-spi",
    "ir",
    "jit",
    "jit-x86",
    "jit-arm64",
    "jit-wasm",
    "usermode",
    "accel-kvm",
    "cpu-mos6502",
    "cpu-w65c816",
    "cpu-arm-aprofile",
    "cpu-arm-aprofile-vfp",
    "cpu-arm-a64",
    "cpu-arm-a64-lift",
    "machine-a64-mini",
    "dev-arm",
    "dev-arm-mpcore",
    "dev-arm-l2c310",
    "machine-arm64-virt",
    "machine-arm926",
    "machine-alphard-navi",
    "dev-uart-ns16550",
    "dev-rcar",
    "dev-watchdog-pin",
    "dev-i2c-regfile",
    "dev-ak77xx",
    "dev-bu21023",
    "dev-rx8581",
    "dev-linux-printk",
    "dev-sgx",
    "dev-navi",
    "dev-riscv",
    "dev-virtio",
    "dev-fdt",
    "dev-flash-cfi",
    "dev-sd-card",
    "dev-keypad-matrix",
    "dev-stm32-sdmmc",
    "dev-stm32-sdio",
    "dev-flash-spinor",
    "dev-psram-qspi",
    "bus-i2c",
    "bus-swi",
    "bus-uart",
    "dev-at24c",
    "dev-st25dv",
    "dev-atecc",
    "dev-stm32-i2c",
    "dev-stm32-i2c-v2",
    "dev-stm32-spi",
    "dev-stm32-octospi",
    "dev-stm32-tim",
    "machine-spi-flash",
    "machine-riscv-virt",
    "cpu-arm-v7m",
    "cpu-arm-v7m-fp",
    "cpu-arm-v8m",
    "cpu-arm-v6m",
    "dev-stm32",
    "dev-stm32-dma",
    "dev-stm32-dmamux",
    "dev-stm32-exti",
    "dev-stm32-firewall",
    "dev-stm32-wdg",
    "dev-stm32-dbgmcu",
    "dev-stm32-crc",
    "dev-stm32-usbfs",
    "dev-stm32-hash",
    "dev-stm32-flash",
    "dev-stm32-rtc",
    "dev-stm32-rcc",
    "dev-stm32-pwr",
    "dev-stm32-icache",
    "dev-stm32-rng",
    "dev-stm32-ipcc",
    "dev-stm32-hsem",
    "dev-stm32-wb-cpu2",
    "machine-stm32f407",
    "machine-stm32u575",
    "machine-stm32u585",
    "machine-stm32wb55",
    "machine-stm32wb35",
    "cpu-z80",
    "machine-z80-mini",
    "machine-usb-mini",
    "machine-xhci-mini",
    "machine-xhci-pci-mini",
    "machine-hub-mini",
    "cpu-sm83",
    "cpu-spc700",
    "dev-gb",
    "dev-snes-cart",
    "dev-snes-dsp",
    "dev-snes-ppu",
    "dev-snes-apu",
    "dev-snes-scpu",
    "machine-gameboy",
    "dev-sms",
    "machine-sms",
    "cpu-x86",
    "dev-pc",
    "dev-pc-apic",
    "dev-pc-hpet",
    "dev-pc-debugcon",
    "dev-pc-video",
    "dev-pc-pci",
    "dev-pc-floppy",
    "dev-q35",
    "dev-ata-disk",
    "dev-ata-atapi",
    "dev-scsi",
    "dev-wd33c93",
    "dev-ncr53c710",
    "dev-ncr5380",
    "dev-linuxboot",
    "dev-dfuse",
    "dev-medium",
    "dev-disc",
    "dev-blk",
    "media-kickstart",
    "media-adf",
    "dev-pc-ide",
    "dev-pc-ide-pci",
    "dev-ahci",
    "machine-ahci-mini",
    "dev-nvme",
    "machine-nvme-mini",
    "machine-pc-at",
    "machine-pc-at-smp",
    "machine-pc-apic",
    "machine-q35",
    "machine-q35-linux",
    "machine-q35-linux-smp",
    "machine-q35-uefi",
    "machine-pc64",
    "fw-pcbios",
    "cpu-m68k",
    "cpu-m68k-lift",
    "cpu-mips",
    "machine-mips-mini",
    "cpu-sh4a",
    "machine-m68k-mini",
    "dev-mac",
    "dev-amiga",
    "dev-amiga-paula",
    "dev-amiga-floppy",
    "dev-amiga-denise",
    "dev-amiga-agnus",
    "dev-amiga-keyboard",
    "dev-amiga-mouse",
    "dev-amiga-akiko",
    "dev-amiga-cdrom",
    "dev-amiga-cd32pad",
    "dev-amiga-gayle",
    "dev-amiga-ide",
    "dev-amiga-ramsey",
    "dev-amiga-sdmac",
    "machine-amiga-a500",
    "machine-amiga-a600",
    "machine-amiga-a1200",
    "machine-amiga-a3000",
    "machine-amiga-a4000",
    "machine-amiga-a4000t",
    "machine-amiga-cd32",
    "dev-amiga-rtc",
    "machine-amiga-a500plus",
    "dev-net",
    "dev-ne2000",
    "net-pktkit",
    "machine-mac-plus",
    "machine-mac-classic",
    "machine-ne2k-mini",
}

/// The kinds a component feature comes in, by prefix, in the order they are
/// shown. Everything else — `std`, `cli`, `jit-x86`, `fw-pcbios` — describes
/// the build rather than a component and is shown first, under `build`.
const KINDS: [&str; 4] = ["cpu", "bus", "dev", "machine"];

/// The widest line [`build_info`] writes.
const WIDTH: usize = 78;

/// Which group a feature is listed under, and the name it has there.
fn classify(feature: &'static str) -> (&'static str, &'static str) {
    for kind in KINDS {
        if let Some(rest) = feature
            .strip_prefix(kind)
            .and_then(|rest| rest.strip_prefix('-'))
        {
            return (kind, rest);
        }
    }
    ("build", feature)
}

/// The features this build has, as `(group, name within the group)`.
fn enabled() -> impl Iterator<Item = (&'static str, &'static str)> {
    FEATURES
        .iter()
        .filter(|(_, on)| *on)
        .map(|(name, _)| classify(name))
}

/// How this build was configured, for a person to read.
///
/// The version on the first line, then one group per kind of feature, each
/// wrapped to a terminal's width — two hundred names on one line answered the
/// question in the sense that the answer was in there somewhere:
///
/// ```text
/// rsemu 0.0.12
///   build    std, cli, gdb, monitor, jit, jit-x86
///   cpu      mos6502, w65c816
///   dev      nes-cart, nes-ppu, nes-apu
///   machine  nes
/// ```
///
/// A group with nothing in it is left out, and the component groups drop the
/// prefix the heading already says. No trailing newline.
#[must_use]
pub fn build_info() -> String {
    let mut out = String::from("rsemu ");
    out.push_str(VERSION);
    for group in core::iter::once("build").chain(KINDS) {
        let names: Vec<&str> = enabled()
            .filter(|(kind, _)| *kind == group)
            .map(|(_, name)| name)
            .collect();
        if names.is_empty() {
            continue;
        }
        // Two spaces, the heading padded to the longest one, one more space.
        const INDENT: usize = 2 + "machine".len() + 2;
        out.push_str("\n  ");
        out.push_str(group);
        let mut column = 2 + group.len();
        for (i, name) in names.iter().enumerate() {
            let last = i + 1 == names.len();
            let width = name.len() + usize::from(!last);
            if i == 0 || column + 1 + width > WIDTH {
                if i != 0 {
                    out.push('\n');
                    column = 0;
                }
                while column < INDENT {
                    out.push(' ');
                    column += 1;
                }
            } else {
                out.push(' ');
                column += 1;
            }
            out.push_str(name);
            if !last {
                out.push(',');
            }
            column += width;
        }
    }
    out
}

/// The same answer on one line, for somewhere that has only one.
///
/// The build's own features by name and the components by count:
/// `rsemu 0.0.12 [std, wasm, demo; 3 cpu, 14 dev, 6 machine]`. A page footer
/// cannot hold two hundred names and does not need them — the page lists its
/// machines itself.
#[must_use]
pub fn build_line() -> String {
    let mut out = String::from("rsemu ");
    out.push_str(VERSION);
    out.push_str(" [");
    let build: Vec<&str> = enabled()
        .filter(|(kind, _)| *kind == "build")
        .map(|(_, name)| name)
        .collect();
    out.push_str(&build.join(", "));
    let mut first = true;
    for kind in KINDS {
        let count = enabled().filter(|(k, _)| *k == kind).count();
        if count == 0 {
            continue;
        }
        out.push_str(if first { "; " } else { ", " });
        first = false;
        out.push_str(&alloc::format!("{count} {kind}"));
    }
    out.push(']');
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::collections::BTreeSet;

    /// The feature names `Cargo.toml` declares, in order.
    ///
    /// A table header, then `name = [` at the start of a line until the next
    /// table. The manifest is this crate's own and is always in the package,
    /// so reading it at compile time costs nothing and needs no toolchain.
    fn declared() -> Vec<&'static str> {
        let manifest = include_str!("../../Cargo.toml");
        let mut names = Vec::new();
        let mut inside = false;
        for line in manifest.lines() {
            if line.starts_with('[') {
                inside = line.trim_end() == "[features]";
                continue;
            }
            if !inside || line.starts_with(|c: char| c.is_whitespace() || c == '#') {
                continue;
            }
            if let Some((name, _)) = line.split_once('=') {
                names.push(name.trim());
            }
        }
        names
    }

    /// The whole point of the module: nobody can add a feature and forget it.
    #[test]
    fn every_declared_feature_is_listed() {
        let declared = declared();
        assert!(
            declared.len() > 200 && declared.contains(&"std") && declared.contains(&"default"),
            "the manifest parser found {} features; it has stopped reading Cargo.toml",
            declared.len()
        );
        let listed: Vec<&str> = FEATURES.iter().map(|(name, _)| *name).collect();
        let want: Vec<&str> = declared.into_iter().filter(|n| *n != "default").collect();

        let have: BTreeSet<&str> = listed.iter().copied().collect();
        let missing: Vec<&&str> = want.iter().filter(|n| !have.contains(**n)).collect();
        assert!(
            missing.is_empty(),
            "Cargo.toml declares features `core::build::FEATURES` does not list, so \
             `rsemu --version` cannot report them — add them there: {missing:?}"
        );
        assert_eq!(have.len(), listed.len(), "a feature is listed twice");
        // The other direction is the compiler's (`unexpected_cfgs`), but the
        // order is ours: the manifest's, so the two can be read side by side.
        assert_eq!(listed, want, "FEATURES is not in Cargo.toml's order");
    }

    #[test]
    fn version_is_not_empty() {
        assert!(!VERSION.is_empty());
    }

    #[test]
    fn build_info_names_the_crate_and_groups_its_features() {
        let info = build_info();
        let mut lines = info.lines();
        assert_eq!(
            lines.next(),
            Some(alloc::format!("rsemu {VERSION}").as_str())
        );
        assert!(info.lines().all(|l| l.len() <= WIDTH), "{info}");
        assert!(!info.ends_with('\n'));
        // Everything this build has is in there, under the right heading, and
        // nothing it lacks is.
        for (name, on) in FEATURES {
            let (group, short) = classify(name);
            let listed = group_of(&info, group).split(", ").any(|n| n == short);
            assert_eq!(listed, *on, "`{name}` under `{group}` in:\n{info}");
        }
        #[cfg(feature = "std")]
        assert!(group_of(&info, "build").split(", ").any(|n| n == "std"));
    }

    /// One group's names, rejoined across the lines it was wrapped over.
    fn group_of(info: &str, group: &str) -> String {
        let mut names = String::new();
        let mut inside = false;
        for line in info.lines().skip(1) {
            let heading = line
                .get(2..)
                .and_then(|l| l.split(' ').next())
                .unwrap_or("");
            if !heading.is_empty() {
                inside = heading == group;
            }
            if inside {
                let body = line.get(11..).unwrap_or("");
                if !names.is_empty() {
                    names.push(' ');
                }
                names.push_str(body);
            }
        }
        names
    }

    #[test]
    fn the_one_line_form_is_one_line_and_counts_components() {
        let line = build_line();
        assert!(line.starts_with(&alloc::format!("rsemu {VERSION} [")));
        assert!(line.ends_with(']') && !line.contains('\n'), "{line}");
        for kind in KINDS {
            let count = FEATURES
                .iter()
                .filter(|(name, on)| *on && classify(name).0 == kind)
                .count();
            let said = line.contains(&alloc::format!("{count} {kind}"));
            assert_eq!(said, count != 0, "{kind} in `{line}`");
        }
    }

    #[test]
    fn a_feature_is_filed_under_its_kind() {
        assert_eq!(classify("machine-gameboy"), ("machine", "gameboy"));
        assert_eq!(classify("cpu-mos6502"), ("cpu", "mos6502"));
        assert_eq!(classify("dev-nes-ppu"), ("dev", "nes-ppu"));
        assert_eq!(classify("bus-pci"), ("bus", "pci"));
        // Not components: a JIT backend and a firmware are properties of the
        // build, and `device` is not `dev-`.
        assert_eq!(classify("jit-x86"), ("build", "jit-x86"));
        assert_eq!(classify("std"), ("build", "std"));
    }
}
