//! The CMOS core against a real ROM: Ben Eater's Wozmon on a 65C51 ACIA.
//!
//! Unit tests prove that each instruction does what a datasheet says. This one
//! proves something a unit test cannot: that a **third-party ROM that never
//! knew rsemu existed** boots on this core, talks to a UART, parses what a
//! person types and answers correctly. It is the acceptance gate for the
//! W65C02S variant.
//!
//! # Why this ROM in particular
//!
//! Because it does not run on an NMOS part, and for a reason that is one byte
//! long. Its `ECHO` routine at `$ffef` is
//!
//! ```text
//! ffef: 48        PHA
//! fff0: 8d 00 50  STA $5000     ; the ACIA data register
//! fff3: a9 ff     LDA #$ff
//! fff5: 3a        DEC A         ; 65C02 only
//! fff6: d0 fd     BNE $fff5
//! fff8: 68        PLA
//! fff9: 60        RTS
//! ```
//!
//! `$3a` is `DEC A` on a W65C02S and an undocumented one-byte `NOP` on an NMOS
//! 6502. Decode it as the NOP and `A` never changes, **Z** never sets, and the
//! delay loop spins for ever — after the *first* character has already been
//! written to the ACIA, which is the detail that makes the failure so
//! confusing on real hardware and so easy to assert on here. Both variants are
//! run below and the difference is the test.
//!
//! # The board
//!
//! Ben Eater's breadboard computer, as much of it as a CPU test needs: 16 KiB
//! of RAM at `$0000`, a 65C51 ACIA at `$5000`, and 32 KiB of ROM at `$8000`
//! carrying the reset vector at `$fffc`. The 65C22 VIA at `$6000` is not
//! modelled because this ROM never touches it. The real board's devices belong
//! in `src/dev/`; this is a bare [`AddressSpace`] and one [`MemOps`], so the
//! test says something about the CPU rather than about a device model.
//!
//! # Provenance
//!
//! `wozmon.bin` is **Ben Eater's** port of the 1976 Woz Monitor to a 65C51
//! ACIA, published at <https://eater.net/6502> and released, with all the code
//! in his videos, under a **Creative Commons Attribution (CC-BY)** licence. The
//! monitor it derives from was published in the Apple-1 Operation Manual
//! without a copyright notice and is in the public domain; `docs/platforms/apple1.md`
//! records both determinations and their evidence.
//!
//! The ROM is **not committed** — `testdata/` is downloaded, never vendored
//! (`ROADMAP.md` §1, §12) — so this test is skipped, loudly, when it is absent.

use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use std::path::PathBuf;

use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
};
use crate::core::sync::{self, LockRank};

use super::{Config, Mos6502, Variant};

/// Where the ROM lives, and how it is addressed.
const ROM_BASE: u16 = 0x8000;
/// 16 KiB of RAM at the bottom of the map, as the board wires it.
const RAM_TOP: u16 = 0x3fff;
/// The 65C51's four registers.
const ACIA_BASE: u16 = 0x5000;
const ACIA_TOP: u16 = 0x5003;

/// The bit the ACIA's status register sets when a byte has arrived.
const RX_FULL: u8 = 0x08;
/// The bit it sets when the transmitter will accept another byte. Always, here:
/// nothing models the baud rate, and Wozmon's `ECHO` does not look anyway.
const TX_EMPTY: u8 = 0x10;

/// The board's memory map and its one device, behind the bus lock.
#[derive(Debug)]
struct Board(sync::Mutex<Inner>);

#[derive(Debug)]
struct Inner {
    ram: Vec<u8>,
    rom: Vec<u8>,
    /// Bytes waiting to be read out of the ACIA — what a person has typed.
    rx: VecDeque<u8>,
    /// Everything the ROM has written to the ACIA, in order.
    tx: Vec<u8>,
}

impl Board {
    fn new(rom: Vec<u8>) -> Board {
        assert_eq!(rom.len(), 0x8000, "the image is a 32 KiB ROM");
        Board(sync::Mutex::with_rank(
            // Below the CPU's own BUS-ranked lock, which is held across the
            // access — the nesting the ladder is drawn for.
            LockRank::DEVICE,
            Inner {
                ram: alloc::vec![0; usize::from(RAM_TOP) + 1],
                rom,
                rx: VecDeque::new(),
                tx: Vec::new(),
            },
        ))
    }

    /// Type at it.
    fn send(&self, text: &str) {
        let mut inner = self.0.lock();
        inner.rx.extend(text.as_bytes());
    }

    /// Everything the ROM has printed so far.
    fn output(&self) -> String {
        String::from_utf8_lossy(&self.0.lock().tx).into_owned()
    }

    fn printed(&self) -> usize {
        self.0.lock().tx.len()
    }
}

impl MemOps for Board {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let mut inner = self.0.lock();
        for (i, slot) in dst.iter_mut().enumerate() {
            let addr = (offset as u16).wrapping_add(i as u16);
            *slot = match addr {
                0..=RAM_TOP => inner.ram[usize::from(addr)],
                ACIA_BASE => {
                    // The data register pops the receive queue — but only for a
                    // real access. A debugger reading the port must not eat the
                    // guest's keystroke (CLAUDE.md, Devices).
                    if attrs.debug {
                        inner.rx.front().copied().unwrap_or(0)
                    } else {
                        inner.rx.pop_front().unwrap_or(0)
                    }
                }
                0x5001 => {
                    let ready = if inner.rx.is_empty() { 0 } else { RX_FULL };
                    TX_EMPTY | ready
                }
                0x5002 | 0x5003 => 0,
                ROM_BASE.. => inner.rom[usize::from(addr - ROM_BASE)],
                // The holes the board leaves — the VIA at $6000 among them.
                // Nothing drives the bus, so the CPU sees what it last put
                // there; the core models that as open bus and counts it.
                _ => return Err(crate::core::error::BusError::Unassigned),
            };
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let mut inner = self.0.lock();
        for (i, byte) in src.iter().enumerate() {
            let addr = (offset as u16).wrapping_add(i as u16);
            match addr {
                0..=RAM_TOP => inner.ram[usize::from(addr)] = *byte,
                ACIA_BASE => {
                    if !attrs.debug {
                        inner.tx.push(*byte);
                    }
                }
                // Status write is a reset, command and control are baud rate
                // and framing. Nothing here cares, and the ROM writes all three
                // on its way up.
                0x5001..=ACIA_TOP => {}
                // ROM. A write lands on nothing, which is what a ROM does.
                ROM_BASE.. => {}
                _ => return Err(crate::core::error::BusError::Unassigned),
            }
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// A CPU of the given variant, wired to a board with the ROM in it.
fn board(variant: Variant, rom: Vec<u8>) -> (Arc<Mos6502>, Arc<Board>) {
    let board = Arc::new(Board::new(rom));
    let space = AddressSpace::new("cpu", 16).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::io("board", 0x1_0000, board.clone()), 0)
        .expect("64 KiB fits in a 16-bit space");
    let cpu = Arc::new(Mos6502::new(Config::NMOS_6502.with_variant(variant)));
    cpu.attach_space(Arc::new(space));
    (cpu, board)
}

/// Step until `done`, or until the cycle budget runs out.
///
/// Returns whether it finished. The budget is what turns "Wozmon hangs" from a
/// test that never returns into a test that fails.
fn run_until(cpu: &Mos6502, budget: u64, mut done: impl FnMut() -> bool) -> bool {
    let mut spent = 0;
    while spent < budget {
        if done() {
            return true;
        }
        let n = cpu.step();
        if n == 0 {
            break;
        }
        spent += n;
    }
    done()
}

/// The ROM, if someone has fetched it.
fn rom() -> Option<Vec<u8>> {
    let root = match std::env::var("RSEMU_TESTDATA") {
        Ok(dir) => PathBuf::from(dir),
        Err(_) => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("testdata"),
    };
    let path = root.join("wozmon/wozmon.bin");
    match std::fs::read(&path) {
        Ok(bytes) if bytes.len() == 0x8000 => Some(bytes),
        Ok(bytes) => panic!(
            "{}: expected a 32 KiB image, got {} bytes",
            path.display(),
            bytes.len()
        ),
        Err(_) => {
            println!(
                "wozmon: {} is absent; fetch it with \
                 `scripts/fetch-testdata.sh wozmon --wozmon-url ...` \
                 (Ben Eater, CC-BY, https://eater.net/6502) to run this test",
                path.display()
            );
            None
        }
    }
}

/// The whole point: a 65C02 ROM reaches its prompt and answers a memory dump.
///
/// Not `#[ignore]`d, and not silent either — without the ROM it says which file
/// it wanted and passes, so `cargo test` stays hermetic (`ROADMAP.md` §12).
#[test]
fn wozmon_reaches_its_prompt_and_dumps_memory_on_a_65c02() {
    let Some(image) = rom() else { return };
    let (cpu, board) = board(Variant::Wdc65C02, image);

    // The reset vector at $fffc points into the ROM, which is the first thing
    // the core reads. Nothing else has to be arranged.
    assert!(
        run_until(&cpu, 5_000_000, || board.printed() >= 2),
        "the monitor never printed its banner; output so far {:?}",
        board.output()
    );
    assert_eq!(
        cpu.reg(super::Reg::Pc) & 0xff00,
        0xff00,
        "running in the ROM"
    );

    // Wozmon's banner is a backslash and a carriage return, which is how it
    // says "ready" — there is no `>` prompt in the 1976 design.
    assert_eq!(board.output(), "\\\r", "the Woz Monitor banner");

    // Now type at it. `FF00` then Enter asks for one byte, and the answer is
    // the first byte of the monitor itself: `$ff00` holds `$a9`, the `LDA #`
    // that starts its own initialisation.
    board.send("FF00\r");
    assert!(
        run_until(&cpu, 5_000_000, || board.output().contains("FF00: A9")),
        "no dump came back; output was {:?}",
        board.output()
    );

    // The whole exchange: banner, the echo of what was typed, then the answer.
    // Every character of it came out of `ECHO`, so every one of them went
    // through the `DEC A` at $fff5.
    let out = board.output();
    assert!(
        out.starts_with("\\\rFF00\r"),
        "the input was echoed: {out:?}"
    );
    assert!(out.ends_with("FF00: A9"), "and answered: {out:?}");
    assert_eq!(
        cpu.bus_faults().0,
        0,
        "the ROM stayed inside the memory map"
    );

    // Ask for a range as well, so the dump loop and not just the single-byte
    // path is exercised. `$fffa.$ffff` is the vector table, and the six bytes
    // that come back are the NMI, RESET and IRQ vectors of the ROM under test:
    // `$0f00`, `$ff00` and `$0000`, low byte first.
    board.send("FFFA.FFFF\r");
    let want = "FFFA: 00 0F 00 FF 00 00";
    assert!(
        run_until(&cpu, 5_000_000, || board.output().contains(want)),
        "the vector table came back wrong; output was {:?}",
        board.output()
    );
    // The whole session, for the record:
    //   "\\\rFF00\r\rFF00: A9\rFFFA.FFFF\r\rFFFA: 00 0F 00 FF 00 00"
    // banner, echo, answer, echo, answer.
}

/// The same ROM on an NMOS part: it stops after one character, for ever.
///
/// This is the bug the variant exists to fix, asserted rather than described.
/// `$3a` decodes as a one-byte `NOP`, so `LDA #$ff / NOP / BNE -3` never sets
/// **Z** and never leaves the loop — but `ECHO` writes to the ACIA *before* the
/// delay, so exactly one character escapes first.
#[test]
fn the_same_rom_hangs_on_an_nmos_part_after_one_character() {
    let Some(image) = rom() else { return };
    let (cpu, board) = board(Variant::Nmos6502, image);

    let reached = run_until(&cpu, 5_000_000, || board.printed() >= 2);
    assert!(!reached, "an NMOS 6502 cannot get past the delay loop");
    assert_eq!(
        board.output(),
        "\\",
        "the character written before the loop"
    );
    // Still fetching, not halted: it is spinning, which is the whole failure
    // mode. A person watching the terminal sees one byte and nothing more.
    assert!(!cpu.is_halted());
    assert_eq!(cpu.reg(super::Reg::Pc) & 0xfff0, 0xfff0, "stuck in ECHO");
}
