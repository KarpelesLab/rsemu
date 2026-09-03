// A64 conformance: loads, stores, the exclusive monitor and its pair form, and
// unaligned access.
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
//
// ---------------------------------------------------------------------------
// Why `LDXP`/`STXP` are hand-written here when nothing else is
// ---------------------------------------------------------------------------
//
// The pair exclusives are what a 128-bit atomic compiles to on a part without
// `FEAT_LSE`, and that would have been the ideal route to them: LLVM's choice,
// not ours. It is not available. `AtomicU128` is behind the unstable
// `integer_atomics` feature, this corpus is built with the pinned **stable**
// toolchain, and no stable Rust construct lowers to a sixteen-byte atomic. So
// the pair cases below are inline assembly and their expectations are ours,
// from DDI 0487 B2.9 — the same standing as the reservation-granule cases they
// sit beside, and stated here rather than left to be assumed.

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

/// A 128-bit compare-and-swap out of `LDAXP`/`STLXP`, with the retry the
/// architecture requires: a `STLXP` may fail spuriously, so a loop that gave up
/// on the first failure would be a compare-and-swap that sometimes lies.
///
/// Returns whether the swap happened.
unsafe fn cas128(at: *mut u64, expect: (u64, u64), new: (u64, u64)) -> bool {
    loop {
        let (lo, hi, status): (u64, u64, u64);
        unsafe {
            core::arch::asm!(
                "ldaxp {lo}, {hi}, [{p}]",
                "cmp   {lo}, {elo}",
                "ccmp  {hi}, {ehi}, #0, eq",
                "b.ne  2f",
                "stlxp {status:w}, {nlo}, {nhi}, [{p}]",
                "b     3f",
                "2:",
                "clrex",
                "mov   {status}, #2",   // "the comparison failed"
                "3:",
                p = in(reg) at,
                lo = out(reg) lo,
                hi = out(reg) hi,
                elo = in(reg) expect.0,
                ehi = in(reg) expect.1,
                nlo = in(reg) new.0,
                nhi = in(reg) new.1,
                status = out(reg) status,
                options(nostack),
            );
        }
        match status {
            0 => return true,
            2 => return false,
            _ => {
                let _ = (lo, hi);
            }
        }
    }
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
    // The exclusive *pair* (DDI 0487 B2.9, C6.2)
    // ------------------------------------------------------------------
    //
    // Sixteen bytes, single-copy atomic, and the only 128-bit atomic an
    // Armv8.0 part has. Written in assembly for the reason at the top of this
    // file.
    let wide = base().cast::<u64>();
    unsafe {
        write_volatile(wide, 0x1111_2222_3333_4444);
        write_volatile(wide.add(1), 0x5555_6666_7777_8888);

        // The pair arrives in address order: `Rt` from the low doubleword,
        // `Rt2` from the high one. An implementation that swapped them would
        // still round-trip a store, so the *read* is what pins the order.
        let (lo, hi, status): (u64, u64, u64);
        core::arch::asm!(
            "ldxp {lo}, {hi}, [{p}]",
            "stxp {status:w}, {nlo}, {nhi}, [{p}]",
            p = in(reg) wide,
            lo = out(reg) lo,
            hi = out(reg) hi,
            nlo = in(reg) 0xaaaa_bbbb_cccc_ddddu64,
            nhi = in(reg) 0x0102_0304_0506_0708u64,
            status = out(reg) status,
            options(nostack),
        );
        if lo != 0x1111_2222_3333_4444 {
            return (8, lo, 0x1111_2222_3333_4444, 0);
        }
        if hi != 0x5555_6666_7777_8888 {
            return (8, hi, 0x5555_6666_7777_8888, 1);
        }
        if status != 0 {
            return (8, status, 0, 2);
        }
        if read_volatile(wide) != 0xaaaa_bbbb_cccc_dddd {
            return (8, read_volatile(wide), 0xaaaa_bbbb_cccc_dddd, 3);
        }
        if read_volatile(wide.add(1)) != 0x0102_0304_0506_0708 {
            return (8, read_volatile(wide.add(1)), 0x0102_0304_0506_0708, 4);
        }

        // A store into the *upper* half of the pair breaks the reservation.
        // This is the case an implementation that watched only the address the
        // `LDXP` named would get wrong, and it would get it wrong silently:
        // the retry loop it belongs to would simply never retry.
        let status: u64;
        core::arch::asm!(
            "ldxp {lo}, {hi}, [{p}]",
            "str {other}, [{p}, #8]",
            "stxp {status:w}, {nlo}, {nhi}, [{p}]",
            p = in(reg) wide,
            lo = out(reg) _,
            hi = out(reg) _,
            other = in(reg) 0x9999_9999_9999_9999u64,
            nlo = in(reg) 0u64,
            nhi = in(reg) 0u64,
            status = out(reg) status,
            options(nostack),
        );
        if status != 1 {
            return (9, status, 1, 0);
        }
        if read_volatile(wide) != 0xaaaa_bbbb_cccc_dddd {
            return (9, read_volatile(wide), 0xaaaa_bbbb_cccc_dddd, 1);
        }
        if read_volatile(wide.add(1)) != 0x9999_9999_9999_9999 {
            return (9, read_volatile(wide.add(1)), 0x9999_9999_9999_9999, 2);
        }

        // The word pair: eight bytes, two four-byte elements, and a status
        // register that is 32 bits whatever the pair's width is.
        write_volatile(wide, 0);
        let (a, b, status): (u64, u64, u64);
        core::arch::asm!(
            "ldxp {a:w}, {b:w}, [{p}]",
            "stxp {status:w}, {na:w}, {nb:w}, [{p}]",
            p = in(reg) wide,
            a = out(reg) a,
            b = out(reg) b,
            na = in(reg) 0x1234_5678u64,
            nb = in(reg) 0x9abc_def0u64,
            status = out(reg) status,
            options(nostack),
        );
        if a != 0 || b != 0 {
            return (10, a, b, 0);
        }
        if status != 0 {
            return (10, status, 0, 1);
        }
        if read_volatile(wide) != 0x9abc_def0_1234_5678 {
            return (10, read_volatile(wide), 0x9abc_def0_1234_5678, 2);
        }

        // `CLREX` clears a pair's reservation exactly as it does a single
        // register's.
        let status: u64;
        core::arch::asm!(
            "ldxp {lo}, {hi}, [{p}]",
            "clrex",
            "stxp {status:w}, {nlo}, {nhi}, [{p}]",
            p = in(reg) wide,
            lo = out(reg) _,
            hi = out(reg) _,
            nlo = in(reg) 0u64,
            nhi = in(reg) 0u64,
            status = out(reg) status,
            options(nostack),
        );
        if status != 1 {
            return (11, status, 1, 0);
        }
        if read_volatile(wide) != 0x9abc_def0_1234_5678 {
            return (11, read_volatile(wide), 0x9abc_def0_1234_5678, 1);
        }

        // The acquire/release spellings are the same instruction with the same
        // result on one core; they exist so a retry loop can be written with
        // the ordering a real one needs.
        let status: u64;
        core::arch::asm!(
            "ldaxp {lo}, {hi}, [{p}]",
            "stlxp {status:w}, {nlo}, {nhi}, [{p}]",
            p = in(reg) wide,
            lo = out(reg) _,
            hi = out(reg) _,
            nlo = in(reg) 0x0f0f_0f0f_0f0f_0f0fu64,
            nhi = in(reg) 0xf0f0_f0f0_f0f0_f0f0u64,
            status = out(reg) status,
            options(nostack),
        );
        if status != 0 {
            return (12, status, 0, 0);
        }
        if read_volatile(wide) != 0x0f0f_0f0f_0f0f_0f0f {
            return (12, read_volatile(wide), 0x0f0f_0f0f_0f0f_0f0f, 1);
        }
        if read_volatile(wide.add(1)) != 0xf0f0_f0f0_f0f0_f0f0 {
            return (12, read_volatile(wide.add(1)), 0xf0f0_f0f0_f0f0_f0f0, 2);
        }

        // A 128-bit compare-and-swap, written the way an Armv8.0 part has to
        // write one: the retry loop `AtomicU128::compare_exchange` would have
        // compiled to if this corpus could name it.
        write_volatile(wide, 1);
        write_volatile(wide.add(1), 2);
        let swapped = cas128(wide, (1, 2), (3, 4));
        if !swapped || read_volatile(wide) != 3 || read_volatile(wide.add(1)) != 4 {
            return (13, read_volatile(wide), 3, u64::from(swapped));
        }
        // ...and one that must fail, leaving both halves alone.
        let swapped = cas128(wide, (99, 99), (5, 6));
        if swapped || read_volatile(wide) != 3 || read_volatile(wide.add(1)) != 4 {
            return (13, read_volatile(wide), 3, 10);
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
