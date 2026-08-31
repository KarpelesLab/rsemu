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

/// One test's name and body.
struct Test {
    name: &'static str,
    body: &'static str,
}

/// The corpus.
static CORPUS: &[Test] = &[
    Test {
        name: "t32-dataproc",
        body: super::corpus::DATAPROC,
    },
    Test {
        name: "t32-shift",
        body: super::corpus::SHIFT,
    },
    Test {
        name: "t32-memory",
        body: super::corpus::MEMORY,
    },
    Test {
        name: "t32-multiply",
        body: super::corpus::MULTIPLY,
    },
    Test {
        name: "t32-bitfield",
        body: super::corpus::BITFIELD,
    },
    Test {
        name: "t32-branch",
        body: super::corpus::BRANCH,
    },
    Test {
        name: "t32-it",
        body: super::corpus::IT,
    },
    Test {
        name: "dsp-simd",
        body: super::corpus::DSP_SIMD,
    },
    Test {
        name: "dsp-multiply",
        body: super::corpus::DSP_MULTIPLY,
    },
    Test {
        name: "exceptions",
        body: super::corpus::EXCEPTIONS,
    },
    Test {
        name: "faults",
        body: super::corpus::FAULTS,
    },
    Test {
        name: "nvic-systick",
        body: super::corpus::NVIC_SYSTICK,
    },
    Test {
        name: "mpu",
        body: super::corpus::MPU,
    },
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
    let source = format!("{PROLOGUE}{}{EPILOGUE}", test.body);
    let src = dir.join(format!("{}.S", test.name));
    let obj = dir.join(format!("{}.o", test.name));
    std::fs::write(&src, source).map_err(|e| format!("writing {}: {e}", src.display()))?;
    let output = Command::new(clang)
        .args([
            "--target=thumbv7em-none-eabi",
            "-mcpu=cortex-m4",
            "-mthumb",
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
fn run(object: &Object) -> Outcome {
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

    let cpu = ArmV7m::new(Config::CORTEX_M4);
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
        match run(&object) {
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
    let test = Test {
        name: "loader-guard",
        body: "    .word _start\n",
    };
    match assemble(&clang, &dir, &test) {
        Err(why) => assert!(
            why.contains("relocation"),
            "expected a relocation complaint, got: {why}"
        ),
        Ok(_) => panic!("the loader accepted an object with a relocation"),
    }
}
