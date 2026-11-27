#![no_main]
//! Every MMIO window of a whole shipped board, poked through the address space
//! the board's own processor would use.
//!
//! `CLAUDE.md` asks for a fuzz target on "every MMIO surface", and the targets
//! beside this one cover about fifteen devices one at a time, each in a rig
//! built for it. The tree has well over a hundred more — thirty STM32
//! peripherals, the R-Car set, the PC's 8259/8254/8042/RTC/FDC/IDE/VGA, the
//! Macintosh VIA/SCC/IWM, the Amiga's custom chips and CIAs — and none of them
//! had ever seen an access a guest's driver would not make. Writing a rig per
//! device does not scale to that, and a rig is also the wrong shape for most
//! of them: what makes a peripheral fall over is rarely its register decode
//! alone, it is a register write that reaches *out* — a DMA request, an
//! interrupt line, a remap, a timer armed on the scheduler — into neighbours a
//! rig does not have.
//!
//! So this target realizes a board from `machine::catalog` exactly as
//! `rsemu run` does, finds every range of every address space that is not
//! plain RAM, and aims the fuzzer at those. "Every address space" matters on
//! the PC, whose port I/O is one: the 8259s, the 8254, the 8042, the RTC, the
//! floppy and IDE controllers and the VGA registers are all reached this way.
//! The properties, in increasing strength:
//!
//! 1. **No access panics.** Any offset, any width, any value, at any window:
//!    a bus fault is an answer, a panic is not. In a debug build of the library
//!    that includes every `+`, `-`, `<<` and index the access reaches.
//! 2. **Time passing does not panic either.** Between accesses the machine is
//!    run for a short span, so a timer the last write armed with a period of
//!    zero, or an event scheduled at `u64::MAX`, actually fires.
//! 3. **A debug read has no side effects** (`CLAUDE.md`, Devices: "a debugger
//!    read must not pop a FIFO, clear a status bit, or advance a pointer").
//!    The whole machine is saved, a window is read with `MemAttrs::DEBUG`, and
//!    the machine is saved again: the two must be byte-identical. This is the
//!    one that needs no knowledge of the device at all, and it is checked for
//!    every device on the board at once.
//! 4. **Save and load are inverses** from whatever state the accesses left:
//!    load what was just saved, save again, compare. A field `save` writes
//!    and `load` drops, or one `load` reconstructs differently, shows up as
//!    the chunk that differs.
//! 5. **A dropped machine is freed.** Nothing asserts it; LeakSanitizer does,
//!    because a board is built and dropped on every input. The first run
//!    found that none of them was — `tests/machine_teardown.rs` has the
//!    account — and every board here is clean now, so a leak this target
//!    reports is a cycle somebody has just introduced.
//!
//! What it does *not* do is replace the per-device targets. Those drive
//! structures the guest builds in RAM behind a register write (a queue, a
//! transfer descriptor, a PRP list); this one reaches RAM only by accident.
//!
//! # Input encoding
//!
//! ```text
//!   byte 0                board, modulo the number of `BOARDS`
//!   then a stream of ops:
//!   0x00 ww ww oo oo oo vv*  write: window, offset, then 1/2/4/8 value bytes
//!   0x01 ww ww oo oo oo      read
//!   0x02 nn                  run the machine for (nn + 1) * 500 ns
//!   0x03 ww ww oo oo oo      debug-read 16 bytes there; state must not move
//!   0x04                     save, load, save: a round trip
//! ```
//!
//! The low two bits of the opcode's *high nibble* choose the width, so one
//! byte carries both and a mutated corpus stays productive. The window index
//! is taken modulo the number of windows and the offset modulo the window's
//! length, so no input is rejected for aiming at nothing.

use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::state::StateReader;
use rsemu::core::value::Width;
use rsemu::machine::Machine;
use rsemu::machine::catalog;

/// A Cortex-M vector table and `b .`: stack pointer, reset vector, one
/// instruction. Enough for the board to realize and for its core to sit still
/// while the peripherals are poked.
const fn cortex_m(sp: u32, pc: u32) -> [u8; 10] {
    let (s, p) = (sp.to_le_bytes(), pc.to_le_bytes());
    [s[0], s[1], s[2], s[3], p[0], p[1], p[2], p[3], 0xfe, 0xe7]
}

static STM32F407: [u8; 10] = cortex_m(0x2002_0000, 0x0000_0009);
static STM32U5: [u8; 10] = cortex_m(0x2003_0000, 0x0800_0009);
static STM32WB: [u8; 10] = cortex_m(0x2000_8000, 0x0800_0009);
/// RISC-V `wfi`, then a branch back to it.
static RISCV_SPIN: [u8; 8] = [0x73, 0x00, 0x50, 0x10, 0x6f, 0xf0, 0xdf, 0xff];
/// A 68000 reset vector pair and `BRA .`.
static M68K_MAC: [u8; 10] = [0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x60, 0xfe];
static M68K_AMIGA: [u8; 10] = [0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x60, 0xfe];

/// One board: its catalog name, what goes in the media slots that need
/// something, and the parameters to override.
struct Board {
    name: &'static str,
    media: &'static [(&'static str, &'static [u8])],
    /// `--param` overrides. Every one of them makes a memory *smaller* and
    /// none touches a peripheral: a board is realized from its `.machine`
    /// text on every input, so that the same bytes always meet the same
    /// machine, and both that and each whole-machine snapshot are linear in
    /// the RAM.
    params: &'static [(&'static str, &'static str)],
}

/// The boards.
///
/// Chosen for the devices they carry that no other target reaches. A media
/// slot not named here gets an empty image, which every one of these boards
/// reads as "nothing inserted".
///
/// `alphard-navi` is not here at all, and that is a cost: its DRAM is a fixed
/// gigabyte, which under AddressSanitizer is ten seconds to allocate and free,
/// so one input is a timeout. The R-Car peripherals it carries are therefore
/// still unfuzzed.
static BOARDS: &[Board] = &[
    Board {
        name: "stm32u585",
        media: &[("firmware", &STM32U5)],
        params: &[],
    },
    Board {
        name: "pc-at",
        media: &[
            ("bios", &[0; 128 * 1024]),
            ("vgabios", &[0; 32 * 1024]),
            ("floppy", &[0; 1_474_560]),
        ],
        params: &[("extmem", "1M")],
    },
    Board {
        name: "riscv-virt",
        media: &[("firmware", &RISCV_SPIN)],
        params: &[("ram", "2M"), ("storage", "64K"), ("flash", "256K")],
    },
    Board {
        name: "mac-plus",
        media: &[("macrom", &M68K_MAC)],
        params: &[],
    },
    Board {
        name: "stm32f407",
        media: &[("firmware", &STM32F407)],
        params: &[],
    },
    Board {
        name: "stm32wb55",
        media: &[("firmware", &STM32WB)],
        params: &[],
    },
    Board {
        name: "amiga-a500",
        media: &[("kickstart", &M68K_AMIGA)],
        params: &[],
    },
];

/// Realize `board` the way `rsemu run` does: the catalog's own text, registry
/// and build options, with the media and parameters above.
fn realize(board: &Board) -> rsemu::core::Result<Machine> {
    let name = board.name;
    let entry = catalog::machine(name).ok_or_else(|| rsemu::core::Error::Config {
        at: String::from("fuzz"),
        message: format!("`{name}` is not in this build's catalog"),
    })?;
    let mut options = catalog::build_options()?;
    for slot in entry.media {
        let bytes = board
            .media
            .iter()
            .find(|(s, _)| s == slot)
            .map_or(&[][..], |(_, b)| *b);
        options.realize.media.insert(*slot, bytes);
    }
    for (param, value) in board.params {
        options = options.with_param(*param, *value);
    }
    rsemu::machine::build(entry.name, entry.source, registry(), &options)
}

/// The class registry, built once.
///
/// It is the same for every board and nothing writes to it. The build options
/// are *not* shared this way, and must not be: they carry the table of host
/// objects a build's devices rendezvous through, so a second machine built
/// from the same options would meet the first one's buses.
fn registry() -> &'static rsemu::core::Registry {
    static REGISTRY: OnceLock<rsemu::core::Registry> = OnceLock::new();
    REGISTRY.get_or_init(|| catalog::registry().expect("no two classes share a name"))
}

/// Cap on operations per input. Each can run the machine or save it, so this
/// bounds an iteration's wall time; the phase gate counts a timeout as a
/// failure and the harness must not manufacture one.
const MAX_OPS: usize = 96;
/// Cap on the two operations that save the whole machine.
const MAX_SAVES: usize = 6;

/// One non-RAM range of one address space.
struct Window {
    space: usize,
    start: u64,
    len: u64,
}

/// Every range that is not plain RAM, across every space the board has.
///
/// Collected into a list and the view dropped before anything is accessed: a
/// write may remap a region, and that takes the topology lock the view holds.
/// A list that has gone stale by then is harmless — an access to what is now
/// unmapped is a bus fault, which is an answer.
fn windows(machine: &Machine) -> Vec<Window> {
    let mut out = Vec::new();
    for (space, entry) in machine.spaces().iter().enumerate() {
        let view = entry.space().view();
        for e in view.flat_view().entries() {
            if !e.is_direct_ram() && !e.is_empty() {
                out.push(Window {
                    space,
                    start: e.start(),
                    len: e.len(),
                });
            }
        }
    }
    out
}

/// A cursor over the input that never fails: past the end it reads zeroes.
struct Input<'a> {
    data: &'a [u8],
    at: usize,
}

impl Input<'_> {
    fn done(&self) -> bool {
        self.at >= self.data.len()
    }

    fn u8(&mut self) -> u8 {
        let b = self.data.get(self.at).copied().unwrap_or(0);
        self.at += 1;
        b
    }

    fn uint(&mut self, bytes: usize) -> u64 {
        let mut v = 0u64;
        for i in 0..bytes {
            v |= u64::from(self.u8()) << (8 * i);
        }
        v
    }

    /// A window and an address inside it.
    fn target(&mut self, windows: &[Window]) -> (usize, u64) {
        let w = &windows[self.uint(2) as usize % windows.len()];
        let offset = self.uint(3) % w.len;
        // Wrapping: a window may end at the top of a 64-bit space.
        (w.space, w.start.wrapping_add(offset))
    }
}

/// Which chunks two snapshots of one machine disagree about, for the message.
fn differing(a: &[u8], b: &[u8]) -> String {
    let (Ok(ra), Ok(rb)) = (StateReader::new(a), StateReader::new(b)) else {
        return String::from("(a snapshot the machine wrote does not parse)");
    };
    let mut out = String::new();
    for info in ra.chunks() {
        let Ok((_, _, da)) = ra.load_raw(info.path) else {
            continue;
        };
        match rb.load_raw(info.path) {
            Ok((_, _, db)) if da == db => {}
            Ok((_, _, db)) => {
                let at = da
                    .iter()
                    .zip(db)
                    .position(|(x, y)| x != y)
                    .unwrap_or_else(|| da.len().min(db.len()));
                out.push_str(&format!(
                    "\n  {} [{}]: {} vs {} bytes, first difference at offset {at}",
                    info.path,
                    info.class,
                    da.len(),
                    db.len()
                ));
            }
            Err(_) => out.push_str(&format!("\n  {} is missing from the second", info.path)),
        }
    }
    out
}

fuzz_target!(|data: &[u8]| {
    let mut input = Input { data, at: 0 };
    let board = &BOARDS[usize::from(input.u8()) % BOARDS.len()];
    let name = board.name;

    // A board that will not realize is a harness bug, never an input's fault:
    // nothing the fuzzer chose has been looked at yet beyond the board index.
    let mut machine = realize(board).unwrap_or_else(|e| panic!("`{name}` does not realize: {e}"));

    let windows = windows(&machine);
    assert!(
        !windows.is_empty(),
        "`{name}` has no MMIO window at all, so this target would poke nothing"
    );

    let mut saves = 0usize;
    for _ in 0..MAX_OPS {
        if input.done() {
            break;
        }
        let op = input.u8();
        let width = match (op >> 4) & 3 {
            0 => Width::U8,
            1 => Width::U16,
            2 => Width::U32,
            _ => Width::U64,
        };
        match op & 0x0f {
            0 => {
                let (space, addr) = input.target(&windows);
                let value = input.uint(1 << ((op >> 4) & 3));
                let _ =
                    machine.spaces()[space]
                        .space()
                        .write(addr, width, value, MemAttrs::DEFAULT);
            }
            1 => {
                let (space, addr) = input.target(&windows);
                let _ = machine.spaces()[space]
                    .space()
                    .read(addr, width, MemAttrs::DEFAULT);
            }
            2 => {
                let nanos = (u64::from(input.u8()) + 1) * 500;
                // An error here is the guest's processor faulting or the
                // machine refusing to go on, both of which are answers.
                let _ = machine.run_for(GlobalTime::from_nanos(nanos));
            }
            3 if saves < MAX_SAVES => {
                saves += 1;
                let (space, addr) = input.target(&windows);
                let Ok(before) = machine.save() else { continue };
                for i in 0..16u64 {
                    let _ = machine.spaces()[space].space().read(
                        addr.wrapping_add(i),
                        Width::U8,
                        MemAttrs::DEBUG,
                    );
                }
                let _ = machine.spaces()[space]
                    .space()
                    .read(addr, width, MemAttrs::DEBUG);
                let after = machine.save().expect("the machine saved a moment ago");
                assert!(
                    before == after,
                    "`{name}`: a debug read at {addr:#x} in space {space} changed \
                     the machine's state:{}",
                    differing(&before, &after)
                );
            }
            4 if saves < MAX_SAVES => {
                saves += 1;
                let Ok(first) = machine.save() else { continue };
                machine
                    .load(&first)
                    .unwrap_or_else(|e| panic!("`{name}` refused its own snapshot: {e}"));
                let second = machine.save().expect("the machine saved a moment ago");
                assert!(
                    first == second,
                    "`{name}`: save, load, save is not a round trip:{}",
                    differing(&first, &second)
                );
            }
            _ => {}
        }
    }
});
