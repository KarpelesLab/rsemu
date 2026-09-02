// A64 conformance: loads, stores, the exclusive monitor, and unaligned access.
//
// Copyright (c) Karpeles Lab Inc. MIT. Written from DDI 0487; no emulator
// source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// What is being asserted, and by whom
// ---------------------------------------------------------------------------
//
// Most of this is a property rather than a value: a byte written and read back
// is the byte, a signed halfword load sign-extends, a `compare_exchange` that
// should fail does. Those need no external oracle, because the property is the
// specification. What the file buys is the *route* to them — `core`'s atomics
// lower to an `LDAXR`/`STLXR` retry loop that no unit test in this crate
// writes by hand, and the addressing modes come out of LLVM's selector rather
// than out of a table somebody typed.
//
// The reservation-granule cases at the end are the exception: those are read
// off DDI 0487 B2.9 and are ours, and they are the ones that would catch an
// exclusive monitor implemented as a plain flag.

#![no_std]
#![no_main]

use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

include!("rt.rs");

/// A scratch area. `#[repr(C, align(16))]` so the guest can reason about the
/// alignment of every offset into it, which the unaligned cases depend on.
#[repr(C, align(16))]
struct Scratch([u8; 256]);

static mut SCRATCH: Scratch = Scratch([0; 256]);

static COUNTER32: AtomicU32 = AtomicU32::new(0);
static COUNTER64: AtomicU64 = AtomicU64::new(0);

/// Read the scratch area as a byte.
fn peek(at: usize) -> u8 {
    unsafe { read_volatile(addr_of!((*addr_of!(SCRATCH)).0[at])) }
}

/// Write one byte of the scratch area.
fn poke(at: usize, value: u8) {
    unsafe { write_volatile(addr_of_mut!((*addr_of_mut!(SCRATCH)).0[at]), value) }
}

/// The base of the scratch area as a mutable pointer.
fn base() -> *mut u8 {
    unsafe { addr_of_mut!((*addr_of_mut!(SCRATCH)).0) }.cast::<u8>()
}

fn run() -> Report {
    // ------------------------------------------------------------------
    // Widths and sign extension
    // ------------------------------------------------------------------
    for i in 0..64usize {
        poke(i, (i as u8).wrapping_mul(37).wrapping_add(11));
    }
    for i in 0..64usize {
        let want = (i as u8).wrapping_mul(37).wrapping_add(11);
        if peek(i) != want {
            return (1, u64::from(peek(i)), u64::from(want), i as u64);
        }
    }

    // Every width, aligned, written and read back through a volatile pointer
    // so the compiler cannot fold the pair away.
    let p = base();
    unsafe {
        write_volatile(p.cast::<u64>(), 0x0123_4567_89ab_cdef);
        let whole = read_volatile(p.cast::<u64>());
        if whole != 0x0123_4567_89ab_cdef {
            return (2, whole, 0x0123_4567_89ab_cdef, 0);
        }
        // Little-endian: the low byte is at the low address, which is the one
        // rule a byte-swapped implementation gets wrong everywhere at once.
        if read_volatile(p) != 0xef {
            return (2, u64::from(read_volatile(p)), 0xef, 1);
        }
        if read_volatile(p.cast::<u16>()) != 0xcdef {
            return (2, u64::from(read_volatile(p.cast::<u16>())), 0xcdef, 2);
        }
        if read_volatile(p.cast::<u32>()) != 0x89ab_cdef {
            return (2, u64::from(read_volatile(p.cast::<u32>())), 0x89ab_cdef, 3);
        }
        // The signed loads: `LDRSB`, `LDRSH`, `LDRSW`, each to both 32-bit and
        // 64-bit destinations.
        let b = read_volatile(p.cast::<i8>()) as i64 as u64;
        if b != 0xffff_ffff_ffff_ffef {
            return (2, b, 0xffff_ffff_ffff_ffef, 4);
        }
        let h = read_volatile(p.cast::<i16>()) as i64 as u64;
        if h != 0xffff_ffff_ffff_cdef {
            return (2, h, 0xffff_ffff_ffff_cdef, 5);
        }
        let w = read_volatile(p.cast::<i32>()) as i64 as u64;
        if w != 0xffff_ffff_89ab_cdef {
            return (2, w, 0xffff_ffff_89ab_cdef, 6);
        }
        let b32 = read_volatile(p.cast::<i8>()) as i32 as u32 as u64;
        if b32 != 0xffff_ffef {
            return (2, b32, 0xffff_ffef, 7);
        }
    }

    // ------------------------------------------------------------------
    // Unaligned access
    // ------------------------------------------------------------------
    //
    // `SCTLR_EL1.A` resets to zero, so an unaligned normal load or store is
    // permitted and must give the same answer as the aligned one at the same
    // address. Every offset from 1 to 7 is tried because an implementation
    // that splits the access into bytes and reassembles them wrongly is only
    // wrong at some of them.
    for off in 1..8usize {
        let value = 0xfedc_ba98_7654_3210u64 ^ (off as u64 * 0x0101_0101_0101_0101);
        unsafe {
            let q = base().add(off);
            q.cast::<u64>().write_unaligned(value);
            let back = q.cast::<u64>().read_unaligned();
            if back != value {
                return (3, back, value, off as u64);
            }
            // ... and the bytes really are where an unaligned store puts them.
            for k in 0..8usize {
                let want = (value >> (8 * k)) as u8;
                if peek(off + k) != want {
                    return (3, u64::from(peek(off + k)), u64::from(want), 100 + k as u64);
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // Atomics: the `LDAXR`/`STLXR` retry loop LLVM emits
    // ------------------------------------------------------------------
    COUNTER32.store(0, Ordering::SeqCst);
    COUNTER64.store(0, Ordering::SeqCst);
    for i in 0..64u64 {
        COUNTER32.fetch_add(i as u32, Ordering::SeqCst);
        COUNTER64.fetch_add(i.wrapping_mul(0x1_0000_0001), Ordering::SeqCst);
    }
    let want32 = (0..64u32).sum::<u32>();
    if COUNTER32.load(Ordering::SeqCst) != want32 {
        return (4, u64::from(COUNTER32.load(Ordering::SeqCst)), u64::from(want32), 0);
    }
    let want64 = (0..64u64).map(|i| i.wrapping_mul(0x1_0000_0001)).sum::<u64>();
    if COUNTER64.load(Ordering::SeqCst) != want64 {
        return (4, COUNTER64.load(Ordering::SeqCst), want64, 1);
    }

    // A compare-and-swap that must succeed, then one that must fail and
    // report the value that was actually there.
    COUNTER64.store(0xdead_beef, Ordering::SeqCst);
    if COUNTER64
        .compare_exchange(0xdead_beef, 0x1234, Ordering::SeqCst, Ordering::SeqCst)
        .is_err()
    {
        return (5, COUNTER64.load(Ordering::SeqCst), 0xdead_beef, 0);
    }
    match COUNTER64.compare_exchange(0xdead_beef, 9, Ordering::SeqCst, Ordering::SeqCst) {
        Ok(_) => return (5, 1, 0, 1),
        Err(actual) => {
            if actual != 0x1234 {
                return (5, actual, 0x1234, 2);
            }
        }
    }
    if COUNTER64.swap(7, Ordering::SeqCst) != 0x1234 {
        return (5, COUNTER64.load(Ordering::SeqCst), 0x1234, 3);
    }
    if COUNTER64.fetch_or(0x30, Ordering::SeqCst) != 7 {
        return (5, COUNTER64.load(Ordering::SeqCst), 7, 4);
    }
    if COUNTER64.load(Ordering::SeqCst) != 0x37 {
        return (5, COUNTER64.load(Ordering::SeqCst), 0x37, 5);
    }

    // ------------------------------------------------------------------
    // The exclusive monitor's own rules (DDI 0487 B2.9)
    // ------------------------------------------------------------------
    //
    // Written in assembly because no Rust construct can express them: a
    // `STXR` after a plain store to the same location must *fail*, which is
    // exactly the case an implementation that treats `LDXR` as "remember an
    // address, always succeed" gets wrong.
    let slot = base().cast::<u64>();
    unsafe {
        write_volatile(slot, 100);
        let status: u64;
        let value: u64;
        core::arch::asm!(
            "ldxr {value}, [{p}]",
            "str {other}, [{p}]",       // an ordinary store clears the reservation
            "stxr {status:w}, {new}, [{p}]",
            p = in(reg) slot,
            value = out(reg) value,
            other = in(reg) 200u64,
            new = in(reg) 300u64,
            status = out(reg) status,
            options(nostack),
        );
        if value != 100 {
            return (6, value, 100, 0);
        }
        // `STXR` reports 1 on failure, and the memory must still hold what the
        // intervening store put there.
        if status != 1 {
            return (6, status, 1, 1);
        }
        if read_volatile(slot) != 200 {
            return (6, read_volatile(slot), 200, 2);
        }

        // The same sequence with nothing in between must succeed.
        let status2: u64;
        core::arch::asm!(
            "ldxr {value}, [{p}]",
            "stxr {status:w}, {new}, [{p}]",
            p = in(reg) slot,
            value = out(reg) _,
            new = in(reg) 400u64,
            status = out(reg) status2,
            options(nostack),
        );
        if status2 != 0 {
            return (6, status2, 0, 3);
        }
        if read_volatile(slot) != 400 {
            return (6, read_volatile(slot), 400, 4);
        }

        // `CLREX` clears the reservation, so the store-exclusive after it
        // fails even though nothing wrote the location.
        let status3: u64;
        core::arch::asm!(
            "ldxr {value}, [{p}]",
            "clrex",
            "stxr {status:w}, {new}, [{p}]",
            p = in(reg) slot,
            value = out(reg) _,
            new = in(reg) 500u64,
            status = out(reg) status3,
            options(nostack),
        );
        if status3 != 1 {
            return (6, status3, 1, 5);
        }
        if read_volatile(slot) != 400 {
            return (6, read_volatile(slot), 400, 6);
        }
    }

    // ------------------------------------------------------------------
    // A byte-by-byte copy, which is where `memcpy` and the addressing modes
    // meet.
    // ------------------------------------------------------------------
    for i in 0..64usize {
        poke(i, (i as u8) ^ 0x5a);
    }
    let mut sum = 0u64;
    for i in 0..64usize {
        poke(128 + i, peek(i));
        sum = sum.wrapping_add(u64::from(peek(128 + i)));
    }
    let want: u64 = (0..64u64).map(|i| u64::from((i as u8) ^ 0x5a)).sum();
    if sum != want {
        return (7, sum, want, 0);
    }
    PASS
}
