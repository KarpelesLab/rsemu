//! The register maps for the cores a debugger could not see at all.
//!
//! `MachineTarget` presents a device as a GDB thread only when this build has a
//! register map for its class, so a core with no entry in
//! `src/host/gdb/arch.rs` attaches with **zero threads**: the session connects,
//! `info threads` is empty, and there is nothing to read. That was the state of
//! `gameboy`, `stm32f407` and `m68k-mini`.
//!
//! A map is a table of byte offsets into a snapshot chunk, so the one thing
//! that can go wrong silently is an offset naming the wrong bytes — GDB would
//! show plausible numbers that are not the machine's. Every register below is
//! therefore compared against the core's *own* accessor rather than against a
//! constant, and every value is distinct so a table that is off by one entry
//! cannot pass.
//!
//! No `gdb` binary is involved and none is needed: a distribution's `gdb` is
//! built for its host, so on the overwhelmingly common x86-64 developer machine
//! none of these three would run anyway. `tests/gdb_real_client.rs` is where a
//! real client drives a real session.

#![cfg(all(
    feature = "gdb",
    any(
        feature = "cpu-sm83",
        feature = "cpu-arm-v7m",
        feature = "cpu-m68k",
        feature = "cpu-mips"
    )
))]

#[cfg(any(feature = "cpu-sm83", feature = "cpu-arm-v7m", feature = "cpu-m68k"))]
use std::sync::Arc;

#[cfg(any(feature = "cpu-sm83", feature = "cpu-arm-v7m", feature = "cpu-m68k"))]
use rsemu::host::gdb::{DebugTarget, MachineTarget};
#[cfg(any(feature = "cpu-sm83", feature = "cpu-arm-v7m", feature = "cpu-m68k"))]
use rsemu::machine::{Machine, catalog};

/// Build a one-core board, keeping the core so the map can be checked against
/// it.
#[cfg(any(feature = "cpu-sm83", feature = "cpu-arm-v7m", feature = "cpu-m68k"))]
fn board_with_core<T: rsemu::machine::Instance + Send + Sync + 'static>(
    name: &str,
    src: &str,
    class: &'static str,
    make: fn(&rsemu::core::props::Props) -> rsemu::core::Result<T>,
) -> (Machine, Arc<T>) {
    use rsemu::core::Captured;

    let cores: Arc<Captured<T>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut bindings = catalog::bindings().expect("this build's bindings");
    bindings.replace(class, move |props| {
        let cpu = Arc::new(make(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let options = rsemu::machine::BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(bindings);
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build(name, src, &registry, &options)
        .unwrap_or_else(|e| panic!("the {class} fixture does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the core");
    (machine, cpu)
}

/// Read register `index` as a little-endian integer.
#[cfg(any(feature = "cpu-sm83", feature = "cpu-arm-v7m", feature = "cpu-m68k"))]
fn reg(target: &MachineTarget<'_>, index: usize) -> u64 {
    let bytes = target
        .read_register(0, index)
        .unwrap_or_else(|e| panic!("register {index}: {e}"));
    let mut value = 0u64;
    for (i, byte) in bytes.iter().enumerate() {
        value |= u64::from(*byte) << (i * 8);
    }
    value
}

// ---------------------------------------------------------------------------
// Sharp SM83 — the Game Boy
// ---------------------------------------------------------------------------

#[cfg(feature = "cpu-sm83")]
const SM83_MINI: &str = r#"
machine "gdb-sm83" {
  osc xtal = 4194304 Hz
  space mem { width = 16 }
  object cpu "cpu.sm83" {
    clock  = xtal
    space  = mem
    engine = "interp"
  }
  object dram "ram" { size = 64K }
  map mem 0x0000 size 64K = dram
}
"#;

/// `gameboy`'s core, which used to attach with no threads at all.
#[cfg(feature = "cpu-sm83")]
#[test]
fn the_sm83_register_map_agrees_with_the_core() {
    use rsemu::cpu::sm83::Sm83;

    let (mut m, cpu) = board_with_core("gdb-sm83.machine", SM83_MINI, "cpu.sm83", Sm83::from_props);
    let mut target = MachineTarget::new(&mut m);
    assert_eq!(target.cpu_count(), 1, "the Game Boy's core is a thread now");
    let arch = target.arch(0).expect("a register map");
    assert_eq!(arch.class.name, "cpu.sm83");
    // Eight bytes and two halfwords: `a f b c d e h l sp pc`.
    assert_eq!(arch.packet_len(), 12);

    let mut regs = cpu.regs();
    regs.a = 0x11;
    regs.f = 0x20; // Only the high nibble is architectural; `F`'s low four
    regs.b = 0x33; // bits read back as zero on this core.
    regs.c = 0x44;
    regs.d = 0x55;
    regs.e = 0x66;
    regs.h = 0x77;
    regs.l = 0x88;
    regs.sp = 0xfffe;
    regs.pc = 0x0150;
    cpu.set_regs(regs);

    let regs = cpu.regs();
    for (index, (name, want)) in [
        ("a", u64::from(regs.a)),
        ("f", u64::from(regs.f)),
        ("b", u64::from(regs.b)),
        ("c", u64::from(regs.c)),
        ("d", u64::from(regs.d)),
        ("e", u64::from(regs.e)),
        ("h", u64::from(regs.h)),
        ("l", u64::from(regs.l)),
        ("sp", u64::from(regs.sp)),
        ("pc", u64::from(regs.pc)),
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(arch.regs[index].name, name);
        assert_eq!(reg(&target, index), want, "{name}");
    }
    assert_eq!(arch.regs[arch.pc].name, "pc");

    // A write, and the retirement counter that makes single-stepping mean one
    // instruction rather than one clock.
    target
        .write_register(0, 8, &0xc000u16.to_le_bytes())
        .expect("sp is writable");
    assert_eq!(cpu.regs().sp, 0xc000);
    // A wrong offset here would not fail loudly — the stepper would burn its
    // whole four-thousand-tick budget on every step and then report one anyway
    // — so the check is on how much a step cost, not merely that it cost
    // something.
    let before = cpu.cycles();
    target.step(0).expect("a step");
    let spent = cpu.cycles() - before;
    assert!(
        spent > 0 && spent < 64,
        "one step cost {spent} cycles, so the map's retirement counter is not \
         the core's"
    );
}

// ---------------------------------------------------------------------------
// ARMv7E-M — the STM32
// ---------------------------------------------------------------------------

#[cfg(feature = "cpu-arm-v7m")]
const V7M_MINI: &str = r#"
machine "gdb-v7m" {
  osc hclk = 168000000 Hz
  space mem { width = 32 }
  object cpu "cpu.arm.v7m" {
    clock  = hclk
    space  = mem
    part   = "cortex-m4"
  }
  object dram "ram" { size = 1M }
  map mem 0x08000000 size 1M = dram
}
"#;

/// `stm32f407`'s core.
///
/// The one map in the tree whose feature name buys something a user notices
/// immediately: `org.gnu.gdb.arm.m-profile` is how GDB learns the target is
/// M-profile, which is what makes an `EXC_RETURN` frame unwind instead of
/// ending at a magic address.
#[cfg(feature = "cpu-arm-v7m")]
#[test]
fn the_v7m_register_map_agrees_with_the_core() {
    use rsemu::cpu::arm::v7m::ArmV7m;

    let (mut m, cpu) = board_with_core(
        "gdb-v7m.machine",
        V7M_MINI,
        "cpu.arm.v7m",
        ArmV7m::from_props,
    );
    let mut target = MachineTarget::new(&mut m);
    assert_eq!(target.cpu_count(), 1, "the STM32's core is a thread now");
    let arch = target.arch(0).expect("a register map");
    assert_eq!(arch.class.name, "cpu.arm.v7m");
    assert_eq!(arch.feature, "org.gnu.gdb.arm.m-profile");
    assert_eq!(arch.architecture, Some("arm"));
    // `r0`-`r12`, `sp`, `lr`, `pc`, `xpsr`: seventeen words, which is exactly
    // what GDB's M-profile gdbarch asks that feature for.
    assert_eq!(arch.packet_len(), 68);

    let mut regs = cpu.regs();
    for (i, slot) in regs.r.iter_mut().enumerate() {
        *slot = 0xa5000000 | i as u32;
    }
    // A PC that is halfword-aligned and a Thumb `xPSR`, because those are the
    // only values this core will hold.
    regs.r[15] = 0x0800_0100;
    regs.xpsr = 0x0100_0000;
    cpu.set_regs(regs);

    let regs = cpu.regs();
    for i in 0..13 {
        assert_eq!(arch.regs[i].name, format!("r{i}"));
        assert_eq!(reg(&target, i), u64::from(regs.r[i]), "r{i}");
    }
    for (index, (name, want)) in [
        ("sp", regs.r[13]),
        ("lr", regs.r[14]),
        ("pc", regs.r[15]),
        ("xpsr", regs.xpsr),
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(arch.regs[13 + index].name, name);
        assert_eq!(reg(&target, 13 + index), u64::from(want), "{name}");
    }
    assert_eq!(arch.regs[arch.pc].name, "pc");

    // `sp` is `r[13]`, which is whichever of `MSP` and `PSP` is selected — the
    // chunk keeps the other one beside it, so no banking hook is needed and
    // this is the assertion that says so.
    assert_eq!(reg(&target, 13), u64::from(cpu.regs().r[13]));

    target
        .write_register(0, 16, &0x2100_0000u32.to_le_bytes())
        .expect("xpsr is writable");
    assert_eq!(cpu.regs().xpsr, 0x2100_0000);
    target
        .write_register(0, 13, &0x2001_0000u32.to_le_bytes())
        .expect("sp is writable");
    assert_eq!(cpu.regs().r[13], 0x2001_0000);

    // The retirement counter, which is what makes a single step one
    // *instruction* rather than one clock. An offset naming a field that never
    // changes would not fail loudly — the stepper would simply burn its whole
    // budget every time — so the check is that it does not.
    let before = cpu.cycles();
    target.step(0).expect("a step");
    let spent = cpu.cycles() - before;
    assert!(
        spent > 0 && spent < 64,
        "one step cost {spent} cycles, so the map's retirement counter is not \
         the core's"
    );
}

// ---------------------------------------------------------------------------
// MC68000
// ---------------------------------------------------------------------------

#[cfg(feature = "cpu-m68k")]
const M68K_MINI: &str = r#"
machine "gdb-m68k" {
  osc clk = 8000000 Hz
  space mem { width = 24, endian = "big" }
  object cpu "cpu.m68k" {
    clock  = clk
    space  = mem
    engine = "interp"
  }
  object dram "ram" { size = 1M }
  map mem 0x000000 size 1M = dram { endian = "big" }
}
"#;

/// `m68k-mini`'s core.
///
/// The board is big-endian and the map is a table of little-endian offsets,
/// which is not a contradiction: a snapshot chunk is flat little-endian
/// whatever the guest's byte order is (`core::state`). That this test passes on
/// a big-endian board is the check.
#[cfg(feature = "cpu-m68k")]
#[test]
fn the_m68k_register_map_agrees_with_the_core() {
    use rsemu::cpu::m68k::M68k;

    let (mut m, cpu) = board_with_core("gdb-m68k.machine", M68K_MINI, "cpu.m68k", M68k::from_props);
    let mut target = MachineTarget::new(&mut m);
    assert_eq!(target.cpu_count(), 1, "the 68000 is a thread now");
    let arch = target.arch(0).expect("a register map");
    assert_eq!(arch.class.name, "cpu.m68k");
    assert_eq!(arch.feature, "org.gnu.gdb.m68k.core");
    assert_eq!(arch.architecture, Some("m68k"));
    // `d0`-`d7`, `a0`-`a5`, `fp`, `sp`, `ps`, `pc`: eighteen words.
    assert_eq!(arch.packet_len(), 72);

    let mut regs = cpu.regs();
    for (i, slot) in regs.d.iter_mut().enumerate() {
        *slot = 0xd0000000 | i as u32;
    }
    for (i, slot) in regs.a.iter_mut().enumerate() {
        *slot = 0xa0000000 | i as u32;
    }
    regs.pc = 0x0000_4000;
    cpu.set_regs(regs);

    let regs = cpu.regs();
    for i in 0..8 {
        assert_eq!(arch.regs[i].name, format!("d{i}"));
        assert_eq!(reg(&target, i), u64::from(regs.d[i]), "d{i}");
    }
    for i in 0..6 {
        assert_eq!(arch.regs[8 + i].name, format!("a{i}"));
        assert_eq!(reg(&target, 8 + i), u64::from(regs.a[i]), "a{i}");
    }
    // GDB calls `a6` "fp" and `a7` "sp", and both are the address-register
    // array — `a[7]` is whichever stack pointer the supervisor bit selects,
    // with the other one beside it in the chunk.
    assert_eq!(arch.regs[14].name, "fp");
    assert_eq!(reg(&target, 14), u64::from(regs.a[6]));
    assert_eq!(arch.regs[15].name, "sp");
    assert_eq!(reg(&target, 15), u64::from(regs.a[7]));
    // `ps` is the one register no offset can name: GDB declares it thirty-two
    // bits wide and the core keeps sixteen, so it is zero-extended on the way
    // out and truncated on the way back.
    assert_eq!(arch.regs[16].name, "ps");
    assert_eq!(reg(&target, 16), u64::from(regs.sr));
    assert_eq!(arch.regs[17].name, "pc");
    assert_eq!(reg(&target, 17), u64::from(regs.pc));
    assert_eq!(arch.regs[arch.pc].name, "pc");

    target
        .write_register(0, 16, &0x0000_2700u32.to_le_bytes())
        .expect("ps is writable");
    assert_eq!(cpu.regs().sr, 0x2700, "supervisor, all interrupts masked");
    assert_eq!(
        reg(&target, 16),
        0x2700,
        "ps does not read back what was written to it"
    );
    target
        .write_register(0, 17, &0x0000_8000u32.to_le_bytes())
        .expect("pc is writable");
    assert_eq!(cpu.regs().pc, 0x8000);

    // As on the other two: a retirement counter at the wrong offset makes a
    // step burn its whole budget rather than fail.
    let before = cpu.cycles();
    target.step(0).expect("a step");
    let spent = cpu.cycles() - before;
    assert!(
        spent > 0 && spent < 64,
        "one step cost {spent} cycles, so the map's retirement counter is not \
         the core's"
    );
}

// ---------------------------------------------------------------------------
// MIPS
// ---------------------------------------------------------------------------

/// `cpu.mips` has no map, on purpose, and this says so out loud.
///
/// Two things are wrong with it at once and neither is fixable from
/// `src/host/gdb/arch.rs`. GDB's `mips_gdbarch_init` requires
/// `org.gnu.gdb.mips.cpu`, `org.gnu.gdb.mips.cp0` **and**
/// `org.gnu.gdb.mips.fpu` and yields no gdbarch when any is missing — the FPU
/// feature is not optional, and this core models no FPU — so no honest
/// description it could emit would be accepted. And its cycle counter comes
/// after two `write_bytes` cache blobs, which are length-prefixed, so it has no
/// fixed offset and `RetireCounter` has no hook for one; single-stepping would
/// fall back to comparing the program counter, which is wrong for a branch to
/// itself.
///
/// When either is fixed, this test fails and points at the paragraph to delete.
#[cfg(feature = "cpu-mips")]
#[test]
fn mips_is_deliberately_unmapped() {
    assert!(
        rsemu::host::gdb::arch::for_class("cpu.mips").is_none(),
        "`cpu.mips` has a register map now — good. Delete this test and the \
         paragraph above `SM83` in `src/host/gdb/arch.rs` that explains why it \
         did not."
    );
}
