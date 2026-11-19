//! The built-binary corpus: small ARMv7E-M programs that check themselves.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. The differential
//! against `cpu::arm::aprofile` measures the sixteen-bit half of that; this
//! measures the other half — the whole of T32, `IT` blocks, and the exception
//! model, none of which ARMv5TE has anything to say about.
//!
//! # Built, never vendored
//!
//! The sources are here, in this file, as ordinary Rust string constants;
//! nothing binary is committed (`ROADMAP.md` §1, §12). At test time each one
//! is concatenated with a shared prologue, handed to `clang --target=
//! thumbv7em-none-eabi`, and the resulting object is loaded by
//! [`super::elf`]. If there is no `clang` the test says so and passes, so
//! `cargo test` stays hermetic — but where there *is* one, which is every
//! developer machine and the CI image, it runs without asking.
//!
//! # How a test signals its result
//!
//! The same convention `riscv-tests` uses, because it is a good one: a word
//! at [`TOHOST`] stays zero while the test runs, becomes `1` on success, and
//! becomes `(n << 1) | 1` when check *n* fails. A failure therefore names the
//! `CHECK` that failed rather than reporting a mood, and the line of assembly
//! is a `grep` away.
//!
//! # The shared prologue
//!
//! [`PROLOGUE`] gives every test a vector table, a trampoline that dispatches
//! through a RAM word so a test can install its own handler at run time, a
//! default handler that logs the exception number, and three macros:
//!
//! | Macro | Does |
//! | --- | --- |
//! | `LOADC reg, val` | a 32-bit constant via `MOVW`/`MOVT`, so no literal pool has to be in range |
//! | `CHECK n, reg, val` | fail check *n* unless `reg == val` |
//! | `CHECKF n, val` | fail check *n* unless `APSR[31:27]` is exactly `val` |
//!
//! `R11` and `R12` are the macros' scratch registers and do not survive a
//! `CHECK`.
//!
//! An ARMv6-M test (`.on_m0()`, `.on_m0plus()`) is assembled with
//! `--target=thumbv6m-none-eabi` against [`V6M_PROLOGUE`] instead, which
//! keeps the same result convention in the v6-M instruction set; so the
//! assembler also refuses any v7-M encoding written into a v6-M body by
//! mistake. [`v6m_encoding_set_matches_llvm`] cross-checks the v6-M decode
//! gate against LLVM's disassembler the same way, as a black box.
//!
//! # Why the corpus is assembled and not linked
//!
//! There is no ARM-capable linker on a machine with only host binutils, and
//! requiring one would mean the test skipped itself on most machines.
//! [`super::elf`] explains the consequences and the checks that keep them
//! honest.
//!
//! # Sources
//!
//! Every expected value in these tests comes from DDI 0403's A7.7 operation
//! pseudocode for the instruction in question, or from B1.5 for the exception
//! sequences. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::core::device::{Device, ResetKind};
use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};

use super::elf::Object;
use super::sys::Exception;
use super::{ArmV7m, Config};

/// Where the code image is loaded, and how much room it gets.
const FLASH_BASE: u64 = 0x0000_0000;
/// How much "flash" the corpus gets. Writable, because it is a `RamStore`;
/// nothing in the corpus writes to it, and a test that did would be finding a
/// real bug in itself.
const FLASH_SIZE: u64 = 0x0001_0000;
/// Where SRAM starts.
const RAM_BASE: u64 = 0x2000_0000;
/// How much SRAM the corpus gets.
const RAM_SIZE: u64 = 0x0001_0000;
/// The result word. At the *bottom* of SRAM: the stack grows down from the
/// top, and an exception frame landing on the result word is a debugging
/// afternoon nobody needs twice.
const TOHOST: u64 = RAM_BASE;
/// Where the default handler records what went wrong: the exception number
/// and the address that faulted.
const LOG: u64 = RAM_BASE + 8;
/// How many instructions a test may run before it is called a hang.
const STEP_LIMIT: u64 = 500_000;

/// The vector table, trampoline, default handler and macros every test gets.
pub(super) const PROLOGUE: &str = r#"
    .syntax unified
    .thumb
    .cpu cortex-m4
    .text

    @ The three control words live at the *bottom* of SRAM, not the top:
    @ the stack grows down from the top and an exception frame would
    @ otherwise land on them. Tests use 0x20000100 upward for scratch.
    .equ TOHOST,      0x20000000
    .equ HANDLER_PTR, 0x20000004
    .equ LOG,         0x20000008
    .equ STACK_TOP,   0x20010000

    @ The vector table. Every entry is a literal address, not a symbol: the
    @ corpus is assembled and never linked, so a symbolic reference here
    @ would need a relocation the loader deliberately refuses.
    .word STACK_TOP
    .word 0x201                  @ Reset -> _start
    .rept 30
    .word 0x81                   @ everything else -> vector_entry
    .endr

    @ Load a 32-bit constant without a literal pool, so nothing has to stay
    @ in range as a test grows.
    .macro LOADC reg, val
    movw \reg, #((\val) & 0xffff)
    movt \reg, #(((\val) >> 16) & 0xffff)
    .endm

    @ Fail check \n unless \reg holds \val. Clobbers r11 and r12.
    .macro CHECK n, reg, val
    LOADC r12, \val
    cmp \reg, r12
    beq 9000f
    movs r0, #\n
    b fail
9000:
    .endm

    @ Fail check \n unless APSR[31:27] is exactly \val. Clobbers r11 and r12.
    .macro CHECKF n, val
    mrs r12, apsr
    LOADC r11, 0xf8000000
    and r12, r12, r11
    LOADC r11, \val
    cmp r12, r11
    beq 9001f
    movs r0, #\n
    b fail
9001:
    .endm

    @ Every exception lands here. If the test has installed a handler in
    @ HANDLER_PTR, go there with LR still holding EXC_RETURN.
    .org 0x80
    .thumb_func
vector_entry:
    LOADC r12, HANDLER_PTR
    ldr r12, [r12]
    cmp r12, #0
    beq default_handler
    bx r12

    @ An exception the test did not ask for stops it dead, with the
    @ exception number and the faulting address where the runner can see
    @ them. Returning instead would re-run the faulting instruction for as
    @ long as the step limit allowed, which reports a timeout rather than the
    @ one fact worth knowing.
    .thumb_func
default_handler:
    mrs r0, ipsr
    LOADC r1, LOG
    str r0, [r1]
    ldr r0, [sp, #24]
    str r0, [r1, #4]
    LOADC r0, 0xffffffff
    LOADC r1, TOHOST
    str r0, [r1]
default_hang:
    b default_hang

    .org 0x200
    .thumb_func
_start:
"#;

/// The pass/fail tail every test gets.
const EPILOGUE: &str = r#"
pass:
    movs r0, #1
    b done
    .thumb_func
fail:
    lsls r0, r0, #1
    adds r0, r0, #1
done:
    LOADC r1, TOHOST
    str r0, [r1]
hang:
    b hang
"#;

/// The ARMv6-M prologue: the same contract as [`PROLOGUE`] — `TOHOST`,
/// `HANDLER_PTR` and `LOG` where they always are, a trampoline through
/// `HANDLER_PTR`, a default handler that logs and stops — written in the
/// v6-M instruction set, because `MOVW`/`MOVT`, the wide `B`, `AND` with a
/// high register and the backward `ADR` the v7-M prologue leans on are all
/// UNDEFINED on a Cortex-M0 (DDI 0419E A5.1.1).
///
/// The differences a test body has to know about:
///
/// | Macro | Does |
/// | --- | --- |
/// | `LOADC reg, val` | a 32-bit constant from an inline literal the macro branches over — **low registers only** (the v6-M `LDR` literal form takes `R0`–`R7`), flags preserved |
/// | `CHECK n, reg, val` | fail check *n* unless `reg == val`; any register but `R7` |
/// | `CHECKR n, a, b` | fail check *n* unless `a == b` |
/// | `CHECKF n, val` | fail check *n* unless `APSR[31:27]` is exactly `val` (so the absent `Q` must read zero) |
/// | `SETF val` | write `val` to `APSR` through `MSR` |
/// | `ADDR reg, label` | `reg = label | 1`, for a label *later* in the file (v6-M's `ADR` reaches forwards only); clobbers `R7` and the flags |
/// | `INSTALL label` | point `HANDLER_PTR` at `label`; clobbers `R6`, `R7` and the flags |
///
/// `R6` and `R7` are the macros' scratch registers and do not survive a
/// `CHECK`. The trampoline clobbers `R0`, which the exception frame restores
/// on return. A failing check reaches `fail` with `BL`, whose range covers
/// any test; a conditional `B` would not. The table has 48 entries — 16
/// system vectors and the 32 interrupts an ARMv6-M NVIC can have (B3.4) —
/// so the trampoline sits at `0xC0`.
#[cfg(feature = "cpu-arm-v6m")]
pub(super) const V6M_PROLOGUE: &str = r#"
    .syntax unified
    .thumb
    .text

    .equ TOHOST,      0x20000000
    .equ HANDLER_PTR, 0x20000004
    .equ LOG,         0x20000008
    .equ STACK_TOP,   0x20010000

    .word STACK_TOP
    .word 0x201                  @ Reset -> _start
    .rept 46
    .word 0xc1                   @ everything else -> vector_entry
    .endr

    .macro LOADC reg, val
    ldr \reg, 9002f
    b 9003f
    .align 2
9002:
    .word \val
9003:
    .endm

    .macro CHECK n, reg, val
    LOADC r7, \val
    cmp \reg, r7
    beq 9000f
    movs r0, #\n
    bl fail
9000:
    .endm

    .macro CHECKR n, a, b
    cmp \a, \b
    beq 9000f
    movs r0, #\n
    bl fail
9000:
    .endm

    .macro CHECKF n, val
    mrs r7, apsr
    lsrs r7, r7, #27
    lsls r7, r7, #27
    LOADC r6, \val
    cmp r7, r6
    beq 9001f
    movs r0, #\n
    bl fail
9001:
    .endm

    .macro SETF val
    LOADC r7, \val
    msr apsr, r7
    .endm

    @ The literal is the label's distance from itself, so the assembler
    @ resolves it and the unlinked object carries no relocation; `| 1`
    @ whether or not the label is a `.thumb_func` (which already is odd).
    .macro ADDR reg, label
    adr r7, 9004f
    ldr \reg, [r7]
    adds \reg, \reg, r7
    b 9005f
    .align 2
9004:
    .word ((\label) - 9004b) | 1
9005:
    .endm

    .macro INSTALL label
    ADDR r6, \label
    LOADC r7, HANDLER_PTR
    str r6, [r7]
    .endm

    .org 0xc0
    .thumb_func
vector_entry:
    LOADC r0, HANDLER_PTR
    ldr r0, [r0]
    cmp r0, #0
    beq default_handler
    bx r0

    .thumb_func
default_handler:
    mrs r0, ipsr
    LOADC r1, LOG
    str r0, [r1]
    ldr r0, [sp, #24]
    str r0, [r1, #4]
    movs r0, #0
    mvns r0, r0
    LOADC r1, TOHOST
    str r0, [r1]
default_hang:
    b default_hang

    .org 0x200
    .thumb_func
_start:
"#;

/// The ARMv6-M pass/fail tail. A test body that runs off its end passes.
#[cfg(feature = "cpu-arm-v6m")]
const V6M_EPILOGUE: &str = r#"
pass:
    movs r0, #1
    b done
    .thumb_func
fail:
    lsls r0, r0, #1
    adds r0, r0, #1
done:
    LOADC r1, TOHOST
    str r0, [r1]
hang:
    b hang
"#;

/// One test's name and body, and which part it runs on.
struct Test {
    name: &'static str,
    body: &'static str,
    /// The part to build it for. A floating-point test needs a Cortex-M4F,
    /// and it needs `clang` to accept the VFP mnemonics, which is why the
    /// part travels with the test rather than being a property of the runner.
    part: Config,
}

impl Test {
    const fn new(name: &'static str, body: &'static str) -> Test {
        Test {
            name,
            body,
            part: Config::CORTEX_M4,
        }
    }

    /// The same test on a Cortex-M4F.
    #[cfg(feature = "cpu-arm-v7m-fp")]
    const fn on_m4f(self) -> Test {
        Test {
            part: Config::CORTEX_M4F,
            ..self
        }
    }

    /// The same test on a Cortex-M33, assembled for Armv8-M Mainline.
    #[cfg(feature = "cpu-arm-v8m")]
    const fn on_m33(self) -> Test {
        Test {
            part: Config::CORTEX_M33,
            ..self
        }
    }

    /// The same test on a Cortex-M0, assembled for ARMv6-M with the v6-M
    /// prologue ([`V6M_PROLOGUE`]).
    #[cfg(feature = "cpu-arm-v6m")]
    const fn on_m0(self) -> Test {
        Test {
            part: Config::CORTEX_M0,
            ..self
        }
    }

    /// The same test on a Cortex-M0+ with its options — `VTOR`, the PMSAv6
    /// MPU, unprivileged Thread mode.
    #[cfg(feature = "cpu-arm-v6m")]
    const fn on_m0plus(self) -> Test {
        Test {
            part: Config::CORTEX_M0PLUS,
            ..self
        }
    }

    /// Whether this test's part has a floating-point unit, which decides the
    /// `-mfpu` `clang` needs.
    const fn wants_fpu(&self) -> bool {
        self.part.ext.fp.present()
    }
}

/// The corpus.
static CORPUS: &[Test] = &[
    Test::new("t32-dataproc", super::corpus::DATAPROC),
    Test::new("t32-shift", super::corpus::SHIFT),
    Test::new("t32-memory", super::corpus::MEMORY),
    Test::new("t32-multiply", super::corpus::MULTIPLY),
    Test::new("t32-bitfield", super::corpus::BITFIELD),
    Test::new("t32-branch", super::corpus::BRANCH),
    Test::new("t32-it", super::corpus::IT),
    Test::new("dsp-simd", super::corpus::DSP_SIMD),
    Test::new("dsp-multiply", super::corpus::DSP_MULTIPLY),
    Test::new("exceptions", super::corpus::EXCEPTIONS),
    Test::new("faults", super::corpus::FAULTS),
    Test::new("nvic-systick", super::corpus::NVIC_SYSTICK),
    Test::new("mpu", super::corpus::MPU),
    #[cfg(feature = "cpu-arm-v7m-fp")]
    Test::new("fp-arith", super::corpus::FP_ARITH).on_m4f(),
    #[cfg(feature = "cpu-arm-v7m-fp")]
    Test::new("fp-convert", super::corpus::FP_CONVERT).on_m4f(),
    #[cfg(feature = "cpu-arm-v7m-fp")]
    Test::new("fp-memory", super::corpus::FP_MEMORY).on_m4f(),
    #[cfg(feature = "cpu-arm-v7m-fp")]
    Test::new("fp-exceptions", super::corpus::FP_EXCEPTIONS).on_m4f(),
    #[cfg(feature = "cpu-arm-v8m")]
    Test::new("v8m-mainline", super::corpus::V8M_MAINLINE).on_m33(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-alu", super::corpus::V6M_ALU).on_m0(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-memory", super::corpus::V6M_MEMORY).on_m0(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-branch", super::corpus::V6M_BRANCH).on_m0(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-system", super::corpus::V6M_SYSTEM).on_m0(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-exceptions", super::corpus::V6M_EXCEPTIONS).on_m0(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-faults", super::corpus::V6M_FAULTS).on_m0(),
    // The instruction set and the exception model are the architecture's,
    // not the part's, so the same bodies run unchanged on a Cortex-M0+.
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-alu-m0plus", super::corpus::V6M_ALU).on_m0plus(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-memory-m0plus", super::corpus::V6M_MEMORY).on_m0plus(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-branch-m0plus", super::corpus::V6M_BRANCH).on_m0plus(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-exceptions-m0plus", super::corpus::V6M_EXCEPTIONS).on_m0plus(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-faults-m0plus", super::corpus::V6M_FAULTS).on_m0plus(),
    #[cfg(feature = "cpu-arm-v6m")]
    Test::new("v6m-m0plus", super::corpus::V6M_M0PLUS).on_m0plus(),
];

/// What running one test produced.
#[derive(Debug, PartialEq, Eq)]
enum Outcome {
    Pass,
    Failed {
        check: u64,
        pc: u32,
        exception: Exception,
        cfsr: u32,
        hfsr: u32,
    },
    /// The test hit an exception it did not install a handler for.
    Unhandled {
        exception: Exception,
        at: u32,
        cfsr: u32,
        hfsr: u32,
    },
    Timeout {
        pc: u32,
        exception: Exception,
        cfsr: u32,
        hfsr: u32,
    },
}

/// Assemble one test, returning the loadable object.
fn assemble(clang: &str, dir: &Path, test: &Test) -> Result<Object, String> {
    // A floating-point test gets the `CPACR` enable in front of it, because
    // every one of them would otherwise open with the same eight lines and
    // the first `NOCP` would be reported as a mysterious HardFault.
    #[cfg(feature = "cpu-arm-v7m-fp")]
    let enable = if test.wants_fpu() {
        super::corpus::ENABLE_FP
    } else {
        ""
    };
    #[cfg(not(feature = "cpu-arm-v7m-fp"))]
    let enable = "";
    // An ARMv6-M test gets the prologue written in its own instruction set.
    #[cfg(feature = "cpu-arm-v6m")]
    let (prologue, epilogue) = if test.part.is_v6m() {
        (V6M_PROLOGUE, V6M_EPILOGUE)
    } else {
        (PROLOGUE, EPILOGUE)
    };
    #[cfg(not(feature = "cpu-arm-v6m"))]
    let (prologue, epilogue) = (PROLOGUE, EPILOGUE);
    let source = format!("{prologue}{enable}{}{epilogue}", test.body);
    let src = dir.join(format!("{}.S", test.name));
    let obj = dir.join(format!("{}.o", test.name));
    std::fs::write(&src, source).map_err(|e| format!("writing {}: {e}", src.display()))?;
    // An Armv8-M test is assembled for the architecture it runs on, so an
    // encoding ARMv7-M does not have is accepted rather than refused.
    // An ARMv6-M test likewise, which also makes the assembler the first
    // check that the body stays inside the v6-M instruction set: a v7-M
    // encoding written by mistake is refused here rather than silently
    // HardFaulting at run time.
    let (target, cpu) = if test.part.is_v8m() {
        ("--target=thumbv8m.main-none-eabi", "-mcpu=cortex-m33")
    } else if test.part.is_v6m() && test.part.ext.mpu {
        ("--target=thumbv6m-none-eabi", "-mcpu=cortex-m0plus")
    } else if test.part.is_v6m() {
        ("--target=thumbv6m-none-eabi", "-mcpu=cortex-m0")
    } else {
        ("--target=thumbv7em-none-eabi", "-mcpu=cortex-m4")
    };
    let output = Command::new(clang)
        .args([
            target,
            cpu,
            "-mthumb",
            // A test that names `S0` needs the assembler told the unit is
            // there; one that does not is unaffected by the flag.
            if test.wants_fpu() {
                "-mfpu=fpv4-sp-d16"
            } else {
                "-mfpu=none"
            },
            "-c",
            "-o",
        ])
        .arg(&obj)
        .arg(&src)
        .output()
        .map_err(|e| format!("running {clang}: {e}"))?;
    if !output.status.success() {
        return Err(String::from_utf8_lossy(&output.stderr).trim().to_string());
    }
    let bytes = std::fs::read(&obj).map_err(|e| format!("reading {}: {e}", obj.display()))?;
    Object::parse(&bytes).map_err(|e| e.to_string())
}

/// Run one loaded object to completion.
fn run(object: &Object, part: Config) -> Outcome {
    let flash = Arc::new(RamStore::new(FLASH_SIZE));
    flash.write_at(0, &object.image).expect("image fits");
    let ram = Arc::new(RamStore::new(RAM_SIZE));

    // Nothing but flash and RAM is mapped, and an access outside them faults
    // rather than reading zero: a test that runs off the end should fail
    // loudly. The private peripheral bus is not mapped here at all — the core
    // owns it (DDI 0403 B3.1).
    let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
    {
        let mut topo = space.topology();
        topo.map(Region::ram("flash", Arc::clone(&flash)), FLASH_BASE)
            .expect("flash fits");
        topo.map(Region::ram("sram", Arc::clone(&ram)), RAM_BASE)
            .expect("RAM fits");
    }

    let cpu = ArmV7m::new(part);
    cpu.attach_space(Arc::new(space));
    Device::reset(&cpu, ResetKind::Cold);

    let tohost = TOHOST - RAM_BASE;
    let read_tohost = || {
        let mut v = 0u32;
        for k in 0..4 {
            v |= u32::from(ram.read_u8(tohost + k).unwrap_or(0)) << (8 * k);
        }
        v
    };

    // `RSEMU_V7M_TRACE=n` prints the last `n` instructions before the test
    // stopped. A corpus test that locks up reports a PC of `0xFFFFFFFE` and
    // nothing else, which is exactly when the trail matters.
    let trace: usize = std::env::var("RSEMU_V7M_TRACE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let mut recent: Vec<String> = Vec::new();
    let word_at = |offset: u64| {
        let mut v = 0u32;
        for k in 0..4 {
            v |= u32::from(ram.read_u8(offset + k).unwrap_or(0)) << (8 * k);
        }
        v
    };
    for _ in 0..STEP_LIMIT {
        if trace != 0 {
            let listed = cpu.disassemble(cpu.pc(), 1);
            let line = listed
                .first()
                .map_or_else(|| format!("{:08x}: ??", cpu.pc()), |l| format!("{l}"));
            if recent.len() == trace {
                recent.remove(0);
            }
            recent.push(line);
        }
        cpu.step();
        if cpu.is_locked_up() {
            for line in &recent {
                std::println!("  {line}");
            }
            let (cfsr, hfsr) = cpu.with_sys(|s| (s.cfsr, s.hfsr));
            return Outcome::Unhandled {
                exception: cpu.current_exception(),
                at: 0xffff_fffe,
                cfsr,
                hfsr,
            };
        }
        let status = read_tohost();
        if status != 0 {
            let (cfsr, hfsr) = cpu.with_sys(|s| (s.cfsr, s.hfsr));
            if status == 1 {
                return Outcome::Pass;
            }
            if trace != 0 && status != 1 {
                for line in &recent {
                    std::println!("  {line}");
                }
            }
            if status == u32::MAX {
                let log = LOG - RAM_BASE;
                return Outcome::Unhandled {
                    exception: Exception(word_at(log) as u16),
                    at: word_at(log + 4),
                    cfsr,
                    hfsr,
                };
            }
            return Outcome::Failed {
                check: u64::from(status >> 1),
                pc: cpu.pc(),
                exception: cpu.current_exception(),
                cfsr,
                hfsr,
            };
        }
    }
    let (cfsr, hfsr) = cpu.with_sys(|s| (s.cfsr, s.hfsr));
    Outcome::Timeout {
        pc: cpu.pc(),
        exception: cpu.current_exception(),
        cfsr,
        hfsr,
    }
}

/// Where to put the assembled corpus.
fn workdir() -> PathBuf {
    let dir = std::env::temp_dir().join("rsemu-v7m-corpus");
    let _ = std::fs::create_dir_all(&dir);
    dir
}

/// Which `clang` to use, if there is one.
fn find_clang() -> Option<String> {
    let candidate = std::env::var("RSEMU_V7M_CLANG").unwrap_or_else(|_| "clang".to_string());
    let ok = Command::new(&candidate)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false);
    ok.then_some(candidate)
}

/// Build and run the whole corpus.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. Where there is no `clang` this prints why and passes.
#[test]
fn built_corpus() {
    let Some(clang) = find_clang() else {
        std::println!(
            "conformance: no `clang` on PATH, so the built ARMv7E-M corpus did not run. \
             Install clang (any version that knows --target=thumbv7em-none-eabi) or set \
             RSEMU_V7M_CLANG."
        );
        return;
    };
    let dir = workdir();
    let only = std::env::var("RSEMU_V7M_ONLY").ok();

    let mut passed = 0usize;
    let mut failures: Vec<String> = Vec::new();
    for test in CORPUS {
        if let Some(only) = &only
            && !only.split(',').any(|f| test.name.contains(f.trim()))
        {
            continue;
        }
        let object = match assemble(&clang, &dir, test) {
            Ok(o) => o,
            Err(why) => {
                failures.push(format!("{}: did not assemble: {why}", test.name));
                continue;
            }
        };
        match run(&object, test.part) {
            Outcome::Pass => passed += 1,
            Outcome::Failed {
                check,
                pc,
                exception,
                cfsr,
                hfsr,
            } => failures.push(format!(
                "{}: check {check} failed (pc {pc:#010x}, in {exception}, \
                 cfsr {cfsr:#010x}, hfsr {hfsr:#010x})",
                test.name
            )),
            Outcome::Timeout {
                pc,
                exception,
                cfsr,
                hfsr,
            } => failures.push(format!(
                "{}: no result after {STEP_LIMIT} instructions (pc {pc:#010x}, \
                 in {exception}, cfsr {cfsr:#010x}, hfsr {hfsr:#010x})",
                test.name
            )),
            Outcome::Unhandled {
                exception,
                at,
                cfsr,
                hfsr,
            } => failures.push(format!(
                "{}: unhandled {exception} at {at:#010x} \
                 (cfsr {cfsr:#010x}, hfsr {hfsr:#010x})",
                test.name
            )),
        }
    }

    for failure in &failures {
        std::println!("FAIL {failure}");
    }
    std::println!(
        "built corpus: {passed} passed, {} failed, out of {} tests",
        failures.len(),
        CORPUS.len()
    );
    assert!(failures.is_empty(), "{} failing tests", failures.len());
}

/// The loader's guard rails are load-bearing, so check they fire.
#[test]
fn the_loader_rejects_a_relocation() {
    let Some(clang) = find_clang() else {
        return;
    };
    let dir = workdir();
    // `.word _start` cannot be resolved by the assembler, so it becomes a
    // relocation — exactly the case the loader must refuse rather than load
    // as zeroes.
    let test = Test::new("loader-guard", "    .word _start\n");
    match assemble(&clang, &dir, &test) {
        Err(why) => assert!(
            why.contains("relocation"),
            "expected a relocation complaint, got: {why}"
        ),
        Ok(_) => panic!("the loader accepted an object with a relocation"),
    }
}

/// Which `llvm-objdump` to use, if there is one.
#[cfg(feature = "cpu-arm-v6m")]
fn find_objdump() -> Option<String> {
    let candidate =
        std::env::var("RSEMU_V7M_OBJDUMP").unwrap_or_else(|_| "llvm-objdump".to_string());
    let ok = Command::new(&candidate)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false);
    ok.then_some(candidate)
}

/// The encodings the ARMv6-M encoding-set cross-check feeds LLVM: every
/// sixteen-bit halfword, and a structured sample of the thirty-two-bit
/// space — every first halfword against a fixed set of second halfwords
/// that walks each `op2` and the barrier and special-register fields, a few
/// pseudo-random ones, and the *whole* second-halfword space behind the
/// first halfwords where the permitted set lives (`MSR`, `MRS`, the
/// barriers, `UDF.W`).
#[cfg(feature = "cpu-arm-v6m")]
fn v6m_sample() -> Vec<(u16, u16)> {
    const FIXED: &[u16] = &[
        0x0000, 0x8000, 0x9000, 0xa000, 0xb000, 0xc000, 0xd000, 0xe000, 0xf000, 0xf800, 0x7fff,
        0xffff, 0x8f4f, 0x8f5f, 0x8f6f, 0x8f2f, 0x8f1f, 0x8f3f, 0x8f7f, 0x8800, 0x8008, 0x8010,
        0x8014, 0x8003, 0x8f00, 0xaf00, 0x8888,
    ];
    let mut out: Vec<(u16, u16)> = (0..=0xffffu16)
        .filter(|&h| !super::isa::is_32bit(h))
        .map(|h| (h, 0))
        .collect();
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    for first in 0xe800..=0xffffu16 {
        for &second in FIXED {
            out.push((first, second));
        }
        for _ in 0..8 {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            out.push((first, seed as u16));
        }
    }
    for first in [0xf3bfu16, 0xf3ef, 0xf380, 0xf7f0] {
        for second in 0..=0xffffu16 {
            out.push((first, second));
        }
    }
    out
}

/// Why the core's ARMv6-M decoder and LLVM's may legitimately disagree
/// about whether an encoding exists. Each is UNPREDICTABLE or behaves
/// identically either way; none is a place where a defined instruction runs
/// differently.
#[cfg(feature = "cpu-arm-v6m")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum V6mDisagreement {
    /// `UDF.W`: DDI 0419E Table A5-10 allocates it as permanently
    /// UNDEFINED; LLVM's ARMv6-M disassembler does not list it. The core
    /// decodes it to `Udf`, which traps as UNDEFINED — the same HardFault an
    /// unknown encoding takes.
    UdfWide,
    /// `MRS`/`MSR` naming a special register ARMv6-M does not have
    /// (`BASEPRI`, `FAULTMASK`, …): UNPREDICTABLE (B4.2.2, B4.2.3). LLVM
    /// refuses the encoding; the core reads zero and ignores the write.
    UnlistedSysm,
    /// An encoding with a should-be-one/should-be-zero bit flipped —
    /// `(1)`/`(0)` in the encoding diagrams — or naming `SP`/`PC` where the
    /// instruction forbids it: UNPREDICTABLE. LLVM refuses it; the core
    /// executes the instruction the remaining bits name.
    ShouldBeBits,
    /// `PUSH`, `POP`, `LDM` or `STM` with an empty register list:
    /// UNPREDICTABLE (`BitCount(registers) < 1`). LLVM refuses it; the core
    /// decodes it and treats it as UNDEFINED at execution, as on ARMv7-M.
    EmptyList,
}

/// Classify one disagreement, or `None` if it is unexplained.
#[cfg(feature = "cpu-arm-v6m")]
fn classify_v6m(first: u16, second: u16) -> Option<V6mDisagreement> {
    if !super::isa::is_32bit(first) {
        // BX/BLX: `0100 0111 L Rm (0)(0)(0)` (A6.7.14, A6.7.15).
        if first & 0xff00 == 0x4700 && first & 0b111 != 0 {
            return Some(V6mDisagreement::ShouldBeBits);
        }
        // CPS: `1011 0110 011 im (0)(0)(1)(0)` (B4.2.1).
        if first & 0xffe0 == 0xb660 && first & 0xf != 0b0010 {
            return Some(V6mDisagreement::ShouldBeBits);
        }
        // PUSH, POP, STM, LDM naming no register at all (A6.7.25, A6.7.49,
        // A6.7.50, A6.7.58).
        if matches!(first, 0xb400 | 0xbc00) || (first & 0xf000 == 0xc000 && first & 0xff == 0) {
            return Some(V6mDisagreement::EmptyList);
        }
        return None;
    }
    let op1 = (first >> 4) & 0x7f;
    let op2 = (second >> 12) & 0b111;
    if op1 == 0b111_1111 && op2 == 0b010 {
        return Some(V6mDisagreement::UdfWide);
    }
    if op2 & 0b101 != 0 {
        return None;
    }
    // The `(0)`/`(1)` bits and UNPREDICTABLE registers of each permitted
    // system encoding (B4.2.2 MRS, B4.2.3 MSR, A6.7.21/A6.7.20/A6.7.34 the
    // barriers): hw1 bit 4 and hw2 bit 13 are `(0)` throughout.
    let (canonical, sysm) = match op1 {
        // MSR: `11110 0 1110 0 (0) Rn | 10 (0) 0 (1)(0)(0)(0) SYSm`.
        0b011_1000 | 0b011_1001 => {
            let rn = first & 0xf;
            (
                first & 0x10 == 0 && second & 0x2f00 == 0x0800 && rn != 13 && rn != 15,
                Some(second & 0xff),
            )
        }
        // MRS: `11110 0 1111 1 (0)(1)(1)(1)(1) | 10 (0) 0 Rd SYSm`.
        0b011_1110 | 0b011_1111 => {
            let rd = (second >> 8) & 0xf;
            (
                first & 0x1f == 0x0f && second & 0x2000 == 0 && rd != 13 && rd != 15,
                Some(second & 0xff),
            )
        }
        // DSB/DMB/ISB: `11110 0 111 01 1 (1)(1)(1)(1) | 10 (0) 0 (1)(1)(1)(1) op option`.
        0b011_1011 => (first & 0xf == 0xf && second & 0x2f00 == 0x0f00, None),
        _ => return None,
    };
    if !canonical {
        return Some(V6mDisagreement::ShouldBeBits);
    }
    // The special registers ARMv6-M lists: APSR, IAPSR, EAPSR, xPSR, IPSR,
    // EPSR, IEPSR, MSP, PSP, PRIMASK, CONTROL.
    match sysm {
        Some(sysm) if !matches!(sysm, 0..=3 | 5..=9 | 16 | 20) => {
            Some(V6mDisagreement::UnlistedSysm)
        }
        _ => None,
    }
}

/// Cross-check the set of encodings ARMv6-M has against LLVM's ARMv6-M
/// disassembler, used as a black box (`ROADMAP.md` §1 permits running any
/// tool): an encoding the core decodes to [`Insn::Undefined`] must be one
/// `llvm-objdump --triple=thumbv6m-none-eabi --mcpu=cortex-m0` calls
/// `<unknown>`, and the other way round, except for the classified,
/// UNPREDICTABLE-or-equivalent cases in [`V6mDisagreement`].
///
/// LLVM is an independent reading of the same tables (DDI 0419E A5.2,
/// A5.3), not an oracle; agreement on several hundred thousand encodings is
/// what makes a transcription error in `super::v6m::decode` unlikely, and a
/// mismatch names the encoding to go and look up.
///
/// Skips with a message where there is no `clang` or `llvm-objdump`.
#[cfg(feature = "cpu-arm-v6m")]
#[test]
fn v6m_encoding_set_matches_llvm() {
    use alloc::collections::BTreeMap;
    use core::fmt::Write as _;

    use super::isa::{Insn, decode_for};

    let (Some(clang), Some(objdump)) = (find_clang(), find_objdump()) else {
        std::println!(
            "conformance: no `clang` or `llvm-objdump`, so the ARMv6-M encoding set was not \
             cross-checked against LLVM."
        );
        return;
    };
    let sample = v6m_sample();
    let mut src = String::from("    .syntax unified\n    .thumb\n    .text\n");
    for &(first, second) in &sample {
        if super::isa::is_32bit(first) {
            let _ = writeln!(src, "    .inst.w 0x{first:04x}{second:04x}");
        } else {
            let _ = writeln!(src, "    .inst.n 0x{first:04x}");
        }
    }
    let dir = workdir();
    let asm = dir.join("v6m-encodings.S");
    let obj = dir.join("v6m-encodings.o");
    std::fs::write(&asm, src).expect("writing the encoding list");
    let built = Command::new(&clang)
        .args(["--target=thumbv6m-none-eabi", "-mcpu=cortex-m0", "-c", "-o"])
        .arg(&obj)
        .arg(&asm)
        .output()
        .expect("running clang");
    assert!(
        built.status.success(),
        "clang refused the encoding list: {}",
        String::from_utf8_lossy(&built.stderr)
    );
    let listed = Command::new(&objdump)
        .args(["-d", "--triple=thumbv6m-none-eabi", "--mcpu=cortex-m0"])
        .arg(&obj)
        .output()
        .expect("running llvm-objdump");
    assert!(listed.status.success(), "llvm-objdump failed");
    let text = String::from_utf8_lossy(&listed.stdout);

    // Each line is `addr: hex [hex]  mnemonic`; key it by address.
    let mut llvm: BTreeMap<u64, bool> = BTreeMap::new();
    for line in text.lines() {
        let Some((addr, rest)) = line.trim_start().split_once(':') else {
            continue;
        };
        let Ok(addr) = u64::from_str_radix(addr, 16) else {
            continue;
        };
        llvm.insert(addr, !rest.contains("<unknown>"));
    }

    let mut addr = 0u64;
    let mut agreed = 0usize;
    let mut classified: BTreeMap<V6mDisagreement, usize> = BTreeMap::new();
    let mut unexplained = 0usize;
    for &(first, second) in &sample {
        let wide = super::isa::is_32bit(first);
        let llvm_defined = *llvm
            .get(&addr)
            .unwrap_or_else(|| panic!("llvm-objdump lost sync at {addr:#x}"));
        let core_defined = decode_for(super::Arch::V6m, first, second) != Insn::Undefined;
        addr += if wide { 4 } else { 2 };
        if llvm_defined == core_defined {
            agreed += 1;
            continue;
        }
        if let Some(why) = classify_v6m(first, second) {
            *classified.entry(why).or_default() += 1;
            continue;
        }
        unexplained += 1;
        if unexplained <= 40 {
            let enc = if wide {
                format!("{first:04x} {second:04x}")
            } else {
                format!("{first:04x}")
            };
            std::println!(
                "  {enc}: core says {}, LLVM says {}",
                if core_defined { "defined" } else { "UNDEFINED" },
                if llvm_defined { "defined" } else { "unknown" }
            );
        }
    }
    std::println!(
        "v6-M encoding set: {agreed} of {} agree with LLVM; classified: {classified:?}",
        sample.len()
    );
    assert_eq!(
        unexplained, 0,
        "unexplained disagreements with LLVM's ARMv6-M decoder"
    );
}
