//! Two x86 cores, one address space, one contended word — the defect the bus
//! lock closes, at guest level.
//!
//! `LOCK XADD` is a read-modify-write the architecture requires to be
//! indivisible to every other observer. The interpreter performs it as an
//! `Exec::read_mem` and then an `Exec::write_mem`, with a window between them,
//! and until [`AddressSpace::bus_lock`] existed nothing closed that window: a
//! sibling core's locked increment landed inside it and was overwritten, so two
//! cores adding one twenty thousand times each produced fewer than forty
//! thousand. `pc-at-smp`, `q35-linux-smp` and `pc-apic` all declare two
//! processors and `ThreadingMode::Parallel` is implemented, so the window is
//! reachable on a shipping board.
//!
//! This is the instrument, not a unit test of the lock: it runs two real
//! interpreters on two host threads over one `RamStore`, exactly the shape of
//! the usermode threaded guest that found the AArch64 and RISC-V half of the
//! same defect. Injecting the defect back in — making `Exec::locks_the_bus`
//! answer `false` — turns the forty thousand into about thirty-five: 34 271,
//! 36 567, 36 408 and 35 884 over four debug runs, and 33 952 in release.
//!
//! # `std::thread` here rather than `core::sync::Pool`
//!
//! `CLAUDE.md`'s prohibition covers `core/`, `cpu/`, `dev/`, `machine/` and
//! `ir/` — the code that has to build for wasm and for bare metal. A test that
//! exists to make two cores collide has to be able to say what a thread is,
//! and `tests/kvm_freedos.rs` makes the same call for the same reason.
//!
//! [`AddressSpace::bus_lock`]: rsemu::core::space::AddressSpace::bus_lock

#![cfg(all(feature = "cpu-x86", feature = "std"))]

use std::sync::Arc;

use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Config, Variant, X86};

/// Where the contended counter lives.
const COUNTER: u64 = 0x2000;

/// How many times each core increments it. Twenty thousand a side, so the
/// answer is forty thousand and fits the sixteen bits `CX` and the counter
/// both have.
const ITERATIONS: u16 = 20_000;

/// The reset vector's far jump: `jmp 0000:0000`, which is how a real ROM
/// leaves the top of the address space. Both cores execute it and both land on
/// the same program, which is what makes them contend.
const RESET_JUMP: &[u8] = &[0xea, 0x00, 0x00, 0x00, 0x00];

/// The guest program, hand-assembled from *Intel SDM* volume 2.
///
/// ```text
///   0000  b9 20 4e        mov cx, 20000
///   0003  b8 01 00        mov ax, 1          ; top
///   0006  f0 0f c1 06     lock xadd [0x2000], ax
///         00 20
///   000c  e2 f5           loop top
///   000e  f4              hlt
/// ```
///
/// `0F C1 /r` is `XADD Ev, Gv`, a 486 addition; the ModRM byte `06` is the
/// 16-bit direct-address form, so the destination is `ds:0x2000` and `DS` is
/// zero out of reset. `F0` is the `LOCK` prefix — the only difference between
/// the two programs this file runs.
fn program(locked: bool) -> Vec<u8> {
    let mut code = vec![
        0xb9,
        (ITERATIONS & 0xff) as u8,
        (ITERATIONS >> 8) as u8, // mov cx, ITERATIONS
        0xb8,
        0x01,
        0x00, // top: mov ax, 1
    ];
    if locked {
        code.push(0xf0);
    }
    code.extend_from_slice(&[0x0f, 0xc1, 0x06, 0x00, 0x20]); // xadd [0x2000], ax
    // The backward displacement counts from the byte after the `loop`, so it
    // moves with the prefix.
    let back = if locked { 0xf5 } else { 0xf6 };
    code.extend_from_slice(&[0xe2, back, 0xf4]); // loop top ; hlt
    code
}

/// One 32-bit space with four megabytes of RAM at zero and a ROM window at the
/// top, holding `code` at physical zero.
fn space(code: &[u8]) -> Arc<AddressSpace> {
    let space = Arc::new(AddressSpace::new("mem", 32));
    space
        .topology()
        .map(Region::ram("ram", Arc::new(RamStore::new(0x40_0000))), 0)
        .expect("4 MiB at zero");
    space
        .topology()
        .map(
            Region::ram("rom", Arc::new(RamStore::new(0x1_0000))),
            0xffff_0000,
        )
        .expect("64 KiB at the top");
    space
        .write_bytes(0, code, MemAttrs::DEFAULT)
        .expect("the program lands");
    space
        .write_bytes(0xffff_fff0, RESET_JUMP, MemAttrs::DEFAULT)
        .expect("the reset vector lands");
    space
}

/// Run two 486 cores over `space` on two host threads until both halt, and
/// return the counter they were both incrementing.
fn contend(space: &Arc<AddressSpace>) -> u64 {
    let cores: Vec<Arc<X86>> = (0..2)
        .map(|_| {
            let cpu = Arc::new(X86::new(Config::default().with_variant(Variant::I80486)));
            cpu.attach_space(Arc::clone(space));
            cpu
        })
        .collect();

    std::thread::scope(|s| {
        for cpu in &cores {
            let cpu = Arc::clone(cpu);
            s.spawn(move || {
                // A generous ceiling: the program is four instructions an
                // iteration plus a reset sequence and a far jump. Stopping on
                // the count rather than only on `step() == 0` keeps a
                // regression from hanging the suite.
                for _ in 0..(u64::from(ITERATIONS) * 8 + 64) {
                    if cpu.step() == 0 {
                        return;
                    }
                }
                panic!("a core never reached its `hlt`");
            });
        }
    });

    space
        .read(COUNTER, Width::U16, MemAttrs::DEFAULT)
        .expect("the counter reads back")
}

/// The gate: forty thousand locked increments produce forty thousand.
#[test]
fn two_cores_do_not_lose_a_locked_increment() {
    let space = space(&program(true));
    let total = contend(&space);
    assert_eq!(
        total,
        u64::from(ITERATIONS) * 2,
        "a locked read-modify-write was interleaved with another core's"
    );
    assert!(
        space.bus_lock().taken() >= u64::from(ITERATIONS) * 2,
        "every one of those increments took the bus"
    );
    assert!(!space.bus_lock().held(), "and gave it back");
}

/// The control, and the reason it asserts an inequality rather than a loss.
///
/// Without the `F0` prefix nothing serialises the read against the write, and
/// this is the arrangement that used to lose updates. How *many* it loses is a
/// property of the host's scheduler, and on a machine that happens to run the
/// two threads one after the other it loses none — so the assertion is the
/// only one that is true on every host. What it does pin is the other
/// direction: an unlocked instruction must not take the bus, or the "costs
/// nothing unless you asked for it" claim is false.
#[test]
fn an_unlocked_read_modify_write_does_not_take_the_bus() {
    let space = space(&program(false));
    let total = contend(&space);
    assert!(
        total <= u64::from(ITERATIONS) * 2,
        "an increment cannot happen more often than it was executed"
    );
    assert_eq!(
        space.bus_lock().taken(),
        0,
        "no `LOCK` prefix, no bus lock, no cost"
    );
}
