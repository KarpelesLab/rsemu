#![no_main]
//! A snapshot that says something false, loaded into a real machine and run.
//!
//! `state_decoder` proves the container reader survives garbage, and
//! `state_roundtrip` that the primitives read back what they wrote. Neither
//! reaches a device's `load`, and that is where a save state somebody was
//! *handed* does its damage: the container is well formed, every chunk is the
//! right class at the right version, and one field inside one chunk is a value
//! the device could never have reached — a prescaler past its division, a
//! cursor past its buffer, a bit counter at zero. A `load` that believes it
//! leaves the device one step from an arithmetic overflow or a slice out of
//! range, and the panic arrives later, in the middle of a run, far from the
//! file that caused it.
//!
//! `CLAUDE.md` puts the bar where a user needs it: a snapshot is a file, and
//! `load` returns [`Error::State`](rsemu::core::Error::State) for one it cannot
//! honour. It never panics, and neither does the machine it accepted.
//!
//! # What one input does
//!
//! 1. Picks a board. Each is realized from the catalog once, as `rsemu run`
//!    does, and kept.
//! 2. Puts the board back to its **baseline** — its own snapshot after a
//!    short run, so that counters are mid-count and FIFOs are not all empty —
//!    by loading it. That load has to succeed whatever the last input did to
//!    the machine, a refused load included; it is asserted, because "a
//!    snapshot defines the whole machine" is the promise every restore makes.
//! 3. Rebuilds the baseline with one device chunk's payload doctored — a
//!    handful of bytes overwritten, and optionally the payload cut short —
//!    and loads that. Everything else in the container is byte for byte what
//!    the machine wrote. An `Err` is the right answer to most inputs and ends
//!    the iteration.
//! 4. If it loaded, the machine is **used**: it runs, every register window is
//!    read the way a debugger reads it, it runs again, and it is saved. A
//!    state `load` accepted has to be one the device can live in.
//!
//! The three machine-level chunks (`/clock`, `/sched`, `/wires`) are left
//! alone: they belong to `core`, and this target is about devices.
//!
//! # Why the machine is kept
//!
//! `machine_mmio` realizes its board afresh on every input, so that the same
//! bytes always meet the same machine. That costs most of an iteration, and
//! here it buys nothing a load does not already promise: step 2 is a restore,
//! and a restore that left something of the previous input behind is exactly
//! the kind of defect this target exists to find. A crash that does not
//! reproduce from its own input alone says so, and the input before it is the
//! other half.
//!
//! # The boards
//!
//! A board is here once it survives a campaign, and the list only grows. The
//! ones not yet on it are not known to be clean — they are not yet looked at.
//! `snes-ntsc` is known *not* to be: a snapshot that claims the S-CPU owes
//! 2^38 master clocks is believed, and the machine spends them.
//!
//! # Run it with `-a`
//!
//! ```sh
//! cargo +nightly fuzz run -a machine_snapshot -- -timeout=20
//! ```
//!
//! `cargo fuzz` builds in release, where an arithmetic overflow wraps instead
//! of panicking — and most of what a false snapshot does to a device is an
//! overflow. `-a` turns the checks back on. Without it this target still
//! catches a slice out of range or a refused baseline, and misses the rest.

use std::cell::RefCell;
use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::state::{StateReader, StateWriter};
use rsemu::core::value::Width;
use rsemu::machine::Machine;
use rsemu::machine::catalog;
use rsemu::machine::machine::{CLOCK_PATH, SCHED_PATH, WIRE_PATH};

/// An iNES image the NES can read its cartridge's wiring out of: the sixteen
/// byte header (`NES<EOF>`, one 16 KiB PRG unit, one 8 KiB CHR unit) and
/// zeroes — which is `BRK` for ever, and that is a program.
static NES_CART: [u8; 16 + 16 * 1024 + 8 * 1024] = {
    let mut image = [0u8; 16 + 16 * 1024 + 8 * 1024];
    image[0] = b'N';
    image[1] = b'E';
    image[2] = b'S';
    image[3] = 0x1a;
    image[4] = 1;
    image[5] = 1;
    image
};

/// A Cortex-M vector table and `b .`.
const fn cortex_m(sp: u32, pc: u32) -> [u8; 10] {
    let (s, p) = (sp.to_le_bytes(), pc.to_le_bytes());
    [s[0], s[1], s[2], s[3], p[0], p[1], p[2], p[3], 0xfe, 0xe7]
}

static STM32U5: [u8; 10] = cortex_m(0x2003_0000, 0x0800_0009);

/// One board: its catalog name, its media and the parameters that shrink it.
struct Board {
    name: &'static str,
    media: &'static [(&'static str, &'static [u8])],
    params: &'static [(&'static str, &'static str)],
}

static BOARDS: &[Board] = &[
    Board {
        name: "nes-ntsc",
        media: &[("cart", &NES_CART)],
        params: &[],
    },
    Board {
        name: "gameboy",
        media: &[("cart", &[0; 32 * 1024])],
        params: &[],
    },
    Board {
        name: "stm32u585",
        media: &[("firmware", &STM32U5)],
        params: &[],
    },
];

/// The class registry, built once: the same for every board, and read only.
fn registry() -> &'static rsemu::core::Registry {
    static REGISTRY: OnceLock<rsemu::core::Registry> = OnceLock::new();
    REGISTRY.get_or_init(|| catalog::registry().expect("no two classes share a name"))
}

/// Realize `board` the way `rsemu run` does, and reset it.
fn realize(board: &Board) -> Machine {
    let name = board.name;
    let entry =
        catalog::machine(name).unwrap_or_else(|| panic!("`{name}` is not in this build's catalog"));
    let mut options = catalog::build_options().expect("this build's options");
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
    let mut machine = rsemu::machine::build(entry.name, entry.source, registry(), &options)
        .unwrap_or_else(|e| panic!("`{name}` does not realize: {e}"));
    machine.reset(ResetKind::Cold);
    machine.sweep();
    machine
}

/// How long the baseline runs before it is saved, and how long a machine that
/// accepted a doctored snapshot is run each time. Short: an iteration's cost
/// is two of these plus a realize.
const SLICE: GlobalTime = GlobalTime::from_nanos(300_000);

/// A board, realized, and the snapshot every input starts it from.
struct Rig {
    machine: Machine,
    baseline: Vec<u8>,
}

thread_local! {
    /// One rig a board, built the first time an input picks it. libFuzzer
    /// runs a target on one thread, so this is one set for the process.
    static RIGS: RefCell<Vec<Option<Rig>>> = RefCell::new(BOARDS.iter().map(|_| None).collect());
}

fn rig(board: &Board) -> Rig {
    let mut machine = realize(board);
    // A faulting processor is an answer; the peripherals have still moved,
    // which is all this is for.
    let _ = machine.run_for(SLICE);
    let baseline = machine
        .save()
        .unwrap_or_else(|e| panic!("`{}` does not save: {e}", board.name));
    Rig { machine, baseline }
}

/// A cursor over the input that never fails: past the end it reads zeroes.
struct Input<'a> {
    data: &'a [u8],
    at: usize,
}

impl Input<'_> {
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

    fn done(&self) -> bool {
        self.at >= self.data.len()
    }
}

/// The most bytes one input overwrites.
const MAX_PATCHES: usize = 12;

/// `baseline` with one device chunk's payload doctored as `input` says.
///
/// Returns the snapshot and the path of the chunk that was changed, for the
/// message when something downstream breaks.
fn doctor(baseline: &[u8], input: &mut Input<'_>) -> (Vec<u8>, String) {
    let reader = StateReader::new(baseline).expect("a snapshot the machine wrote");
    let devices: Vec<_> = reader
        .chunks()
        .filter(|c| ![CLOCK_PATH, SCHED_PATH, WIRE_PATH].contains(&c.path) && c.len > 0)
        .collect();
    assert!(!devices.is_empty(), "a machine with no stateful device");
    let victim = devices[input.uint(2) as usize % devices.len()].path;
    let cut = input.u8();

    let mut writer = StateWriter::new(reader.shape().clone());
    for info in reader.chunks() {
        let (class, version, data) = reader.load_raw(info.path).expect("a chunk it listed");
        if info.path != victim {
            writer
                .raw_chunk(info.path, class, version, data)
                .expect("a chunk the machine wrote");
            continue;
        }
        let mut data = data.to_vec();
        for _ in 0..MAX_PATCHES {
            if input.done() {
                break;
            }
            let at = input.uint(3) as usize % data.len();
            let how = input.u8();
            let value = input.u8();
            data[at] = match how & 3 {
                0 => value,
                1 => data[at] ^ value,
                2 => 0xff,
                _ => 0,
            };
        }
        // One input in sixteen also loses its tail, which is what a download
        // that stopped early looks like from inside a chunk.
        if cut & 0x0f == 0x0f {
            let keep = usize::from(cut >> 4) * data.len() / 16;
            data.truncate(keep);
        }
        writer
            .raw_chunk(info.path, class, version, &data)
            .expect("a chunk of the class the machine wrote");
    }
    (
        writer.to_vec().expect("a container with the same chunks"),
        victim.to_string(),
    )
}

/// Read every register window the way a debugger does, a byte at a stride.
fn look(machine: &Machine) {
    for entry in machine.spaces() {
        let space = entry.space();
        let windows: Vec<(u64, u64)> = {
            let view = space.view();
            view.flat_view()
                .entries()
                .iter()
                .filter(|e| !e.is_direct_ram() && !e.is_empty())
                .map(|e| (e.start(), e.len()))
                .collect()
        };
        for (start, len) in windows {
            let step = (len / 64).max(1);
            let mut offset = 0;
            while offset < len {
                let _ = space.read(start.wrapping_add(offset), Width::U8, MemAttrs::DEBUG);
                offset += step;
            }
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let mut input = Input { data, at: 0 };
    let index = usize::from(input.u8()) % BOARDS.len();
    let board = &BOARDS[index];
    RIGS.with_borrow_mut(|rigs| {
        let Rig { machine, baseline } = rigs[index].get_or_insert_with(|| rig(board));
        let name = board.name;
        // Whatever the last input left behind, the board's own snapshot puts
        // it back. If it does not, a restore is not a restore.
        machine
            .load(baseline)
            .unwrap_or_else(|e| panic!("`{name}` refused its own baseline snapshot: {e}"));

        let (snapshot, _victim) = doctor(baseline, &mut input);
        // Most doctored snapshots are refused, and a refusal is the whole of
        // the right answer: nothing is asked of a machine that said no.
        if machine.load(&snapshot).is_err() {
            return;
        }
        // It said yes, so it has to be a machine. An error from running is
        // the guest's processor faulting, which is an answer; a panic is not.
        let _ = machine.run_for(SLICE);
        look(machine);
        let _ = machine.run_for(SLICE);
        let _ = machine.save();
    });
});
