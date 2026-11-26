//! The STM32 debug support block, `DBGMCU`.
//!
//! One class, `st.dbgmcu`. It is the smallest peripheral on the die and the
//! only one whose customer is not the firmware: three writable words and one
//! read-only identifier, sitting at `0xE004_2000` on the **external** private
//! peripheral bus, put there so that a debugger — not the program — can say
//! what the chip should keep doing while the core is stopped at a breakpoint.
//!
//! Without it, a board that models a watchdog is a board you cannot debug. Set
//! a breakpoint in a main loop that kicks the `IWDG`, wait at the prompt for
//! half a second, and the part resets under you; the bug you were chasing is
//! gone and a reset is in its place. `DBG_IWDG_STOP` is the bit that stops
//! that, and every real STM32 debug script sets it before it sets a
//! breakpoint.
//!
//! # The registers
//!
//! Four of them, at `0xE004_2000` (RM0090 §38.16):
//!
//! | Offset | Register | What it does |
//! | --- | --- | --- |
//! | `0x00` | `IDCODE` | read-only: `DEV_ID[11:0]` and `REV_ID[31:16]` |
//! | `0x04` | `CR` | `DBG_SLEEP`, `DBG_STOP`, `DBG_STANDBY`, `TRACE_IOEN`, `TRACE_MODE` |
//! | `0x08` | `APB1_FZ` | one freeze bit per APB1 peripheral, `DBG_IWDG_STOP` among them |
//! | `0x0c` | `APB2_FZ` | the same for APB2: the four advanced and general-purpose timers on it |
//!
//! `DEV_ID` is `0x413` on an STM32F405/407/415/417 (RM0090 §38.6.1) and
//! `REV_ID` is the silicon revision, `0x1000` for revision A. Both are
//! construction properties rather than constants, because they are the one
//! thing a bootloader uses this block for: `DEV_ID` is how ST's own DFU
//! loader decides which flash geometry it is talking to.
//!
//! # Which part
//!
//! This is **RM0090's** `DBGMCU`, which is the F2/F4/F7 one. An L4's debug
//! block (RM0351 §44.9) is a different register map — `APB1FZR1` at `0x08`,
//! a second `APB1FZR2` at `0x0c` and `APB2FZR` at `0x10` — and it wants its
//! own bit table, not a fudge of this one. It is not written here because no
//! board in this tree is an L4 and a bit table nobody can check against a
//! board is worse than no bit table: the freeze bits below are each one line
//! of RM0090 §38.16.3 and §38.16.4.
//!
//! # The `u5` variant
//!
//! The STM32U5's block (`variant = "u5"`) lives at `0xE004_4000` and is the
//! wider L4-style map, per `DBGMCU_TypeDef` and the `DBGMCU_*_Pos`
//! definitions in ST CMSIS `stm32u585xx.h` (Apache-2.0) and RM0456's debug
//! chapter:
//!
//! | Offset | Register | Freeze pins |
//! | --- | --- | --- |
//! | `0x00` | `IDCODE` | — (`DEV_ID` `0x482` on a U575/U585, the `u5` default) |
//! | `0x04` | `CR` | — (`DBG_STOP` 1, `DBG_STANDBY` 2, `TRACE_IOEN` 4, `TRACE_CLKEN` 5, `TRACE_MODE` 6–7) |
//! | `0x08` | `APB1FZR1` | `tim2`…`tim7` 0–5, `wwdg` 11, `iwdg` 12, `i2c1` 21, `i2c2` 22 |
//! | `0x0C` | `APB1FZR2` | `i2c4` 1, `lptim2` 5 |
//! | `0x10` | `APB2FZR` | `tim1` 11, `tim8` 13, `tim15` 16, `tim16` 17, `tim17` 18 |
//! | `0x14` | `APB3FZR` | `i2c3` 10, `lptim1` 17, `lptim3` 18, `lptim4` 19, `rtc` 30 |
//! | `0x20` | `AHB1FZR` | `gpdma0`…`gpdma15` 0–15 |
//! | `0x28` | `AHB3FZR` | `lpdma0`…`lpdma3` 0–3 |
//!
//! The watchdog bits keep their F4 positions, in the register that took the
//! F4's `APB1_FZ` offset, so `wire dbgmcu.iwdg -> iwdg.freeze` is the same
//! line on both boards. There is no `DBG_SLEEP` on a U5. `DBGMCU_SR`, the
//! two `DBG_AUTH` mailboxes at `0xFC`–`0x108` and the CoreSight ID block at
//! `0xFD0` are not modelled: they serve a debugger's authentication protocol
//! rather than firmware, and none of them is in the CMSIS struct.
//!
//! # The `wb` variant
//!
//! The STM32WB55/WB35's block (`variant = "wb"`) is at the F4's
//! `0xE004_2000` with a freeze register per CPU per bus, per `DBGMCU_TypeDef`
//! and the `DBGMCU_*_Pos` definitions in ST CMSIS `stm32wb55xx.h` v1.12.4
//! (Apache-2.0) and RM0434 §41.8:
//!
//! | Offset | Register | Freeze pins |
//! | --- | --- | --- |
//! | `0x00` | `IDCODE` | — (`DEV_ID` `0x495` on a WB55 *and* a WB35, the `wb` default; `REV_ID` `0x2001` for WB55 rev Y / WB35 rev A, `0x2003` for rev X) |
//! | `0x04` | `CR` | — (`DBG_SLEEP` 0, `DBG_STOP` 1, `DBG_STANDBY` 2, `TRACE_IOEN` 5, `TRGOEN` 28) |
//! | `0x3C` | `APB1FZR1` | `tim2` 0, `rtc` 10, `wwdg` 11, `iwdg` 12, `i2c1` 21, `i2c3` 23, `lptim1` 31 |
//! | `0x40` | `C2APB1FZR1` | CPU2's: the same bits but `wwdg` — stored only |
//! | `0x44` | `APB1FZR2` | `lptim2` 5 |
//! | `0x48` | `C2APB1FZR2` | CPU2's `lptim2` — stored only |
//! | `0x4C` | `APB2FZR` | `tim1` 11, `tim16` 17, `tim17` 18 |
//! | `0x50` | `C2APB2FZR` | CPU2's — stored only |
//!
//! Only CPU1's bits drive pins: the level is "the bit and *the core* is
//! halted", and the one halt this machine reports ([`Device::debug_halt`]) is
//! the debugger's on CPU1. CPU2's registers read back what was written and
//! freeze nothing until there is a CPU2 halt to AND them with. (ST's SVD
//! misplaces `C2APB2FZR` at `0x48`; the CMSIS header and RM0434 Table 279 put
//! it at `0x50`.) As on the F4, "the DBGMCU registers are not reset by a
//! system reset, only by a power on reset" (RM0434 §41.8).
//!
//! # The freeze pins
//!
//! A stored bit that nothing consults is the defect this block exists to fix,
//! so every freeze bit this model knows is an **output pin**, and a board
//! wires it to the peripheral it freezes:
//!
//! ```text
//! wire dbgmcu.iwdg -> iwdg.freeze
//! wire dbgmcu.wwdg -> wwdg.freeze
//! ```
//!
//! The level on that pin is *the bit AND the core is halted*, which is the
//! hardware's own definition: `DBG_IWDG_STOP` reads "debug independent
//! watchdog stopped when core is halted", and a chip whose core is running
//! freezes nothing no matter what the debugger wrote.
//!
//! # How "the core is halted" gets here
//!
//! It is not a wire and it is not a register, because nothing *inside* the
//! machine knows: a gdb stub halts a core by declining to call the run loop,
//! and until this block existed there was no flag anywhere below `host/` that
//! said so. So it arrives on [`Device::debug_halt`], which
//! [`Machine::set_debug_halted`](crate::machine::Machine::set_debug_halted)
//! broadcasts and which the gdb session drives from the one place it already
//! computes running-versus-halted, once a turn, before it lets the machine
//! move. That is the whole seam; this is the only device in the tree that
//! overrides it, which is the point of a debug unit.
//!
//! # What a system reset does to it: nothing
//!
//! "This register is asynchronously reset by the POR … and not by the system
//! reset" (RM0090 §38.16.2). That is not a detail. The whole value of
//! `DBG_IWDG_STOP` is that it survives the resets the debugger is stepping
//! through, so [`Device::reset`] clears the registers on
//! [`ResetKind::Cold`] and leaves them alone otherwise.
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §38 "Debug support (DBG)": §38.6.1 for `IDCODE`,
//! §38.16.2 for `CR`, §38.16.3 for `APB1_FZ` and §38.16.4 for `APB2_FZ`. The
//! address is §38.16.1. ARM **DDI 0403** B3.1 for why `0xE004_2000` is a
//! vendor address at all: `0xE0040000`–`0xE00FFFFF` is the *external* private
//! peripheral bus, which the architecture leaves implementation-defined. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.dbgmcu";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes the registers occupy: `IDCODE` through `APB2_FZ`.
pub const REGISTER_BYTES: u64 = 0x10;

/// How many bytes the `u5` registers occupy: `IDCODE` through `AHB3FZR`.
pub const REGISTER_BYTES_U5: u64 = 0x2c;

/// How many bytes the `wb` registers occupy: `IDCODE` through `C2APB2FZR`.
pub const REGISTER_BYTES_WB: u64 = 0x54;

/// Where each of the WB's writable words is: `CR`, `APB1FZR1`, `C2APB1FZR1`,
/// `APB1FZR2`, `C2APB1FZR2`, `APB2FZR`, `C2APB2FZR`.
const OFFSETS_WB: [u64; MAX_WORDS] = [0x04, 0x3c, 0x40, 0x44, 0x48, 0x4c, 0x50];

/// The WB's writable bits, word by word — the `_Msk` lines of ST CMSIS
/// `stm32wb55xx.h` for each register ORed together (identical in
/// `stm32wb35xx.h`).
const WRITE_MASK_WB: [u32; MAX_WORDS] = [
    0x1000_0027,
    0x80a0_1c01,
    0x80a0_1401,
    0x0000_0020,
    0x0000_0020,
    0x0006_0800,
    0x0006_0800,
];

/// `DEV_ID` of a WB55 or WB35 (RM0434 §41.8.1).
const WB_DEV_ID: u64 = 0x495;

/// `REV_ID` of a WB55 rev Y or WB35 rev A (RM0434 §41.8.1).
const WB_REV_ID: u64 = 0x2001;

/// Where an STM32U5 puts the block (ST CMSIS `stm32u585xx.h`, `DBGMCU_BASE`).
pub const BASE_U5: u64 = 0xe004_4000;

/// The architectural base of the block on an STM32 (RM0090 §38.16.1).
///
/// Inside the *external* private peripheral bus, `0xE0040000`–`0xE00FFFFF`,
/// which ARM DDI 0403 B3.1 leaves to the implementation — which is why a chip
/// vendor may put a register file there at all.
pub const BASE: u64 = 0xe004_2000;

/// `CR`'s index in [`State::words`]; the register is at offset `0x04`.
const CR: usize = 0;
/// `APB1_FZ`'s index; offset `0x08`.
const APB1_FZ: usize = 1;
/// `APB2_FZ`'s index; offset `0x0c`.
const APB2_FZ: usize = 2;

/// How many writable words the F4 block has. `IDCODE` is not one of them.
const WORDS: usize = 3;

/// How many writable words the widest layout has: the U5's seven.
const MAX_WORDS: usize = 7;

/// Which bits of each word a guest write can set.
///
/// `CR`: `DBG_SLEEP` 0, `DBG_STOP` 1, `DBG_STANDBY` 2, `TRACE_IOEN` 5 and
/// `TRACE_MODE[7:6]` (RM0090 §38.16.2). `APB1_FZ` and `APB2_FZ` are the two
/// tables below, and everything outside them is reserved and reads as zero,
/// which is how firmware that probes for a peripheral this part does not have
/// finds out.
const WRITE_MASK: [u32; WORDS] = [0x0000_00e7, 0x06e0_1dff, 0x0007_0003];

/// Where each of the F4's writable words is.
const OFFSETS: [u64; WORDS] = [0x04, 0x08, 0x0c];

/// Where each of the U5's writable words is: `CR`, `APB1FZR1`, `APB1FZR2`,
/// `APB2FZR`, `APB3FZR`, `AHB1FZR`, `AHB3FZR`.
const OFFSETS_U5: [u64; MAX_WORDS] = [0x04, 0x08, 0x0c, 0x10, 0x14, 0x20, 0x28];

/// The U5's writable bits, word by word — each the `_Msk` lines of ST CMSIS
/// `stm32u585xx.h` for that register ORed together.
const WRITE_MASK_U5: [u32; MAX_WORDS] = [
    0x0000_00f6,
    0x0060_183f,
    0x0000_0022,
    0x0007_2800,
    0x400e_0400,
    0x0000_ffff,
    0x0000_000f,
];

/// Which register file the block has.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// RM0090's F2/F4/F7 block at `0xE004_2000`.
    #[default]
    F4,
    /// RM0456's STM32U5 block at `0xE004_4000`.
    U5,
    /// RM0434's STM32WB block at `0xE004_2000`, with CPU2's freeze registers.
    Wb,
}

/// One variant's register file, as data.
#[derive(Debug)]
struct Layout {
    /// The offset of each writable word.
    offsets: &'static [u64],
    /// The writable bits of each.
    masks: &'static [u32],
    /// The freeze pins.
    freeze: &'static [Freeze],
    /// How many bytes decode.
    bytes: u64,
}

/// The F4's register file.
static LAYOUT_F4: Layout = Layout {
    offsets: &OFFSETS,
    masks: &WRITE_MASK,
    freeze: FREEZE,
    bytes: REGISTER_BYTES,
};

/// The U5's register file.
static LAYOUT_U5: Layout = Layout {
    offsets: &OFFSETS_U5,
    masks: &WRITE_MASK_U5,
    freeze: FREEZE_U5,
    bytes: REGISTER_BYTES_U5,
};

/// The WB's register file.
static LAYOUT_WB: Layout = Layout {
    offsets: &OFFSETS_WB,
    masks: &WRITE_MASK_WB,
    freeze: FREEZE_WB,
    bytes: REGISTER_BYTES_WB,
};

impl Variant {
    fn layout(self) -> &'static Layout {
        match self {
            Variant::F4 => &LAYOUT_F4,
            Variant::U5 => &LAYOUT_U5,
            Variant::Wb => &LAYOUT_WB,
        }
    }

    /// `DEV_ID` out of the box: `0x413` for an F407, `0x482` for a U575/U585,
    /// `0x495` for a WB55/WB35.
    fn default_dev_id(self) -> u64 {
        match self {
            Variant::F4 => DEFAULT_DEV_ID,
            Variant::U5 => 0x482,
            Variant::Wb => WB_DEV_ID,
        }
    }

    /// `REV_ID` out of the box: revision A (`0x1000`), or on a WB the
    /// WB55 rev Y / WB35 rev A code, `0x2001`.
    fn default_rev_id(self) -> u64 {
        match self {
            Variant::F4 | Variant::U5 => DEFAULT_REV_ID,
            Variant::Wb => WB_REV_ID,
        }
    }
}

/// One freeze bit, as a pin a board can wire.
#[derive(Debug, Clone, Copy)]
struct Freeze {
    /// What the pin is called: the `DBG_xxx_STOP` name with the decoration
    /// removed, because `wire dbgmcu.iwdg -> iwdg.freeze` is what a machine
    /// file wants to read.
    pin: &'static str,
    /// Which of [`State::words`] the bit lives in.
    reg: usize,
    /// Which bit of it.
    bit: u32,
}

/// Every freeze bit RM0090 §38.16.3 and §38.16.4 give the F4.
///
/// The two the issue that asked for this block cares about are `wwdg` and
/// `iwdg`; the rest are here because leaving them out would make the register
/// half a model, and because a board that stops its timers at a breakpoint is
/// the other thing every debug script does.
static FREEZE: &[Freeze] = &[
    // DBGMCU_APB1_FZ (RM0090 §38.16.3).
    Freeze {
        pin: "tim2",
        reg: APB1_FZ,
        bit: 0,
    },
    Freeze {
        pin: "tim3",
        reg: APB1_FZ,
        bit: 1,
    },
    Freeze {
        pin: "tim4",
        reg: APB1_FZ,
        bit: 2,
    },
    Freeze {
        pin: "tim5",
        reg: APB1_FZ,
        bit: 3,
    },
    Freeze {
        pin: "tim6",
        reg: APB1_FZ,
        bit: 4,
    },
    Freeze {
        pin: "tim7",
        reg: APB1_FZ,
        bit: 5,
    },
    Freeze {
        pin: "tim12",
        reg: APB1_FZ,
        bit: 6,
    },
    Freeze {
        pin: "tim13",
        reg: APB1_FZ,
        bit: 7,
    },
    Freeze {
        pin: "tim14",
        reg: APB1_FZ,
        bit: 8,
    },
    Freeze {
        pin: "rtc",
        reg: APB1_FZ,
        bit: 10,
    },
    Freeze {
        pin: "wwdg",
        reg: APB1_FZ,
        bit: 11,
    },
    Freeze {
        pin: "iwdg",
        reg: APB1_FZ,
        bit: 12,
    },
    // The three I2C bits freeze the SMBUS timeout counter rather than the
    // peripheral, which is why they are named for the timeout in the manual.
    Freeze {
        pin: "i2c1",
        reg: APB1_FZ,
        bit: 21,
    },
    Freeze {
        pin: "i2c2",
        reg: APB1_FZ,
        bit: 22,
    },
    Freeze {
        pin: "i2c3",
        reg: APB1_FZ,
        bit: 23,
    },
    Freeze {
        pin: "can1",
        reg: APB1_FZ,
        bit: 25,
    },
    Freeze {
        pin: "can2",
        reg: APB1_FZ,
        bit: 26,
    },
    // DBGMCU_APB2_FZ (RM0090 §38.16.4).
    Freeze {
        pin: "tim1",
        reg: APB2_FZ,
        bit: 0,
    },
    Freeze {
        pin: "tim8",
        reg: APB2_FZ,
        bit: 1,
    },
    Freeze {
        pin: "tim9",
        reg: APB2_FZ,
        bit: 16,
    },
    Freeze {
        pin: "tim10",
        reg: APB2_FZ,
        bit: 17,
    },
    Freeze {
        pin: "tim11",
        reg: APB2_FZ,
        bit: 18,
    },
];

/// Every CPU1 freeze bit ST CMSIS `stm32wb55xx.h` gives the WB. `reg`
/// indexes [`OFFSETS_WB`]; CPU2's registers (indices 2, 4 and 6)
/// have no pins — see the module header.
static FREEZE_WB: &[Freeze] = &[
    // APB1FZR1.
    fz("tim2", 1, 0),
    fz("rtc", 1, 10),
    fz("wwdg", 1, 11),
    fz("iwdg", 1, 12),
    fz("i2c1", 1, 21),
    fz("i2c3", 1, 23),
    fz("lptim1", 1, 31),
    // APB1FZR2.
    fz("lptim2", 3, 5),
    // APB2FZR.
    fz("tim1", 5, 11),
    fz("tim16", 5, 17),
    fz("tim17", 5, 18),
];

/// A U5 freeze bit, in one line.
const fn fz(pin: &'static str, reg: usize, bit: u32) -> Freeze {
    Freeze { pin, reg, bit }
}

/// Every freeze bit ST CMSIS `stm32u585xx.h` gives the U5, by `DBGMCU_*_Pos`.
/// `reg` indexes [`OFFSETS_U5`].
static FREEZE_U5: &[Freeze] = &[
    // APB1FZR1.
    fz("tim2", 1, 0),
    fz("tim3", 1, 1),
    fz("tim4", 1, 2),
    fz("tim5", 1, 3),
    fz("tim6", 1, 4),
    fz("tim7", 1, 5),
    fz("wwdg", 1, 11),
    fz("iwdg", 1, 12),
    fz("i2c1", 1, 21),
    fz("i2c2", 1, 22),
    // APB1FZR2.
    fz("i2c4", 2, 1),
    fz("lptim2", 2, 5),
    // APB2FZR.
    fz("tim1", 3, 11),
    fz("tim8", 3, 13),
    fz("tim15", 3, 16),
    fz("tim16", 3, 17),
    fz("tim17", 3, 18),
    // APB3FZR.
    fz("i2c3", 4, 10),
    fz("lptim1", 4, 17),
    fz("lptim3", 4, 18),
    fz("lptim4", 4, 19),
    fz("rtc", 4, 30),
    // AHB1FZR.
    fz("gpdma0", 5, 0),
    fz("gpdma1", 5, 1),
    fz("gpdma2", 5, 2),
    fz("gpdma3", 5, 3),
    fz("gpdma4", 5, 4),
    fz("gpdma5", 5, 5),
    fz("gpdma6", 5, 6),
    fz("gpdma7", 5, 7),
    fz("gpdma8", 5, 8),
    fz("gpdma9", 5, 9),
    fz("gpdma10", 5, 10),
    fz("gpdma11", 5, 11),
    fz("gpdma12", 5, 12),
    fz("gpdma13", 5, 13),
    fz("gpdma14", 5, 14),
    fz("gpdma15", 5, 15),
    // AHB3FZR.
    fz("lpdma0", 6, 0),
    fz("lpdma1", 6, 1),
    fz("lpdma2", 6, 2),
    fz("lpdma3", 6, 3),
];

/// The reset value of `DEV_ID` — an STM32F405/407/415/417 (RM0090 §38.6.1).
const DEFAULT_DEV_ID: u64 = 0x413;

/// The reset value of `REV_ID`: revision A.
const DEFAULT_REV_ID: u64 = 0x1000;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything a guest write can change.
///
/// `IDCODE` is not here: it is a construction property, so it is not state a
/// snapshot has to carry. Neither is the `halted` level — that belongs to
/// whatever is debugging, not to the machine, exactly as `st.iwdg` keeps its
/// own freeze input out of its chunk.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct State {
    /// `CR`, `APB1_FZ`, `APB2_FZ`, already masked — or, on a `u5`, the seven
    /// words of [`OFFSETS_U5`]. An F4 leaves the last four at zero.
    words: [u32; MAX_WORDS],
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// Which register file.
    variant: Variant,
    /// `IDCODE`, assembled once from the two properties.
    idcode: u32,
    /// Whether whatever is debugging has the core halted.
    ///
    /// An atomic rather than a field of [`State`] for the reason `st.iwdg`
    /// gives for its own freeze flag: it is read on every refresh and it is
    /// **not** the machine's state.
    halted: AtomicBool,
    /// The connected freeze outputs, as indices into [`FREEZE`].
    outputs: Mutex<Vec<(usize, WireSource)>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("idcode", &self.idcode)
            .field("halted", &self.halted.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Drive every connected freeze pin to what the registers now say.
    ///
    /// Called with **no lock held**: a sink is a peripheral that may call
    /// straight back into this device, so the state is copied out first, the
    /// wire list second, and the outward `set` calls last (`CLAUDE.md`,
    /// "Concurrency"). `st.rcc` refreshes its gate pins the same way and for
    /// the same reason.
    fn refresh_outputs(&self) {
        let words = self.state.lock().words;
        let halted = self.halted.load(Ordering::Acquire);
        let pending: Vec<(WireSource, Level)> = {
            let outputs = self.outputs.lock();
            outputs
                .iter()
                .map(|(index, source)| {
                    let bit = self.variant.layout().freeze[*index];
                    // "…stopped when core is halted": the bit alone freezes
                    // nothing on a running chip.
                    let high = halted && words[bit.reg] & (1 << bit.bit) != 0;
                    (source.clone(), Level::from_bool(high))
                })
                .collect()
        };
        for (source, level) in pending {
            source.set(level);
        }
    }

    /// Whatever is debugging halted or resumed the core.
    fn set_halted(&self, halted: bool) {
        self.halted.store(halted, Ordering::Release);
        self.refresh_outputs();
    }

    /// Read one register.
    fn read_register(&self, offset: u64) -> u32 {
        if offset == 0x00 {
            return self.idcode;
        }
        match self.word_at(offset) {
            Some(index) => self.state.lock().words[index],
            None => 0,
        }
    }

    /// Which writable word is at `offset`, if any.
    fn word_at(&self, offset: u64) -> Option<usize> {
        self.variant
            .layout()
            .offsets
            .iter()
            .position(|&at| at == offset)
    }

    /// Write one register. Returns whether a freeze pin may have moved.
    fn write_register(&self, offset: u64, value: u32) -> bool {
        // "IDCODE … read-only", and so is every hole between the words.
        let Some(index) = self.word_at(offset) else {
            return false;
        };
        let mut state = self.state.lock();
        state.words[index] = value & self.variant.layout().masks[index];
        index != CR
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        // Nothing here clears or advances on a read, and the block is not on
        // a clock domain, so a debug read is the same read (`ROADMAP.md` §15,
        // invariant 5) — which is just as well, because reading `IDCODE` out
        // of a stopped machine is exactly what a debugger does first.
        let _ = attrs;
        let bytes = self.read_register(offset & !3).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `APB1_FZ` would stop a watchdog the guest is
            // relying on, or start one it had frozen — the pins move either
            // way. There is no harmless version, and the debugger that wants
            // this bit set has the `halted` pin instead.
            return Err(BusError::BadAccess);
        }
        let value = u32::from_le_bytes([*a, *b, *c, *d]);
        if self.write_register(offset & !3, value) {
            // Outside the critical section, as `refresh_outputs` requires.
            self.refresh_outputs();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 debug support block.
#[derive(Debug)]
pub struct Dbgmcu {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Dbgmcu {
    /// Validate `props` and build the block.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Dbgmcu> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "f4", &["f4", "u5", "wb"])? {
            "u5" => Variant::U5,
            "wb" => Variant::Wb,
            _ => Variant::F4,
        };
        // Twelve bits and sixteen: a value that does not fit is a machine file
        // that has confused `DEV_ID` with the part number.
        let dev_id = r.or_range("dev-id", variant.default_dev_id(), 0..=0xfff)? as u32;
        let rev_id = r.or_range("rev-id", variant.default_rev_id(), 0..=0xffff)? as u32;
        r.finish()?;
        Ok(Dbgmcu::with_variant(dev_id, rev_id, variant))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_id(dev_id: u32, rev_id: u32) -> Dbgmcu {
        Dbgmcu::with_variant(dev_id, rev_id, Variant::F4)
    }

    /// Build one of either register file.
    #[must_use]
    pub fn with_variant(dev_id: u32, rev_id: u32, variant: Variant) -> Dbgmcu {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            variant,
            idcode: (rev_id << 16) | (dev_id & 0xfff),
            halted: AtomicBool::new(false),
            outputs: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        });
        let region = Arc::new(Region::io(
            "dbgmcu",
            variant.layout().bytes,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Dbgmcu { regs, region }
    }

    /// Which register file this block has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// What `IDCODE` reads as.
    #[must_use]
    pub fn idcode(&self) -> u32 {
        self.regs.idcode
    }

    /// Whether the block believes the core is halted.
    #[must_use]
    pub fn halted(&self) -> bool {
        self.regs.halted.load(Ordering::Acquire)
    }

    /// Tell it the core halted or resumed, as the `halted` pin does.
    ///
    /// The route a host that is not driving a wire takes — and the route a
    /// test takes.
    pub fn set_halted(&self, halted: bool) {
        self.regs.set_halted(halted);
    }

    /// Whether the freeze pin called `pin` would be asserted right now.
    ///
    /// The bit *and* the halt, which is what the pin carries.
    #[must_use]
    pub fn freezing(&self, pin: &str) -> bool {
        let freeze = self.regs.variant.layout().freeze;
        let Some(bit) = freeze.iter().find(|f| f.pin == pin) else {
            return false;
        };
        self.halted() && self.regs.state.lock().words[bit.reg] & (1 << bit.bit) != 0
    }
}

impl Device for Dbgmcu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the freeze lines.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // "This register is asynchronously reset by the POR … and not by the
        // system reset" (RM0090 §38.16.2), which is the point of the block: a
        // debugger sets `DBG_IWDG_STOP` once and it survives every reset it
        // then steps the firmware through — including the watchdog reset the
        // bit exists to prevent.
        if kind != ResetKind::Cold {
            return;
        }
        *self.regs.state.lock() = State::default();
        self.regs.refresh_outputs();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        // As many words as the variant has, so an F4's chunk is the three it
        // always was.
        let n = self.regs.variant.layout().masks.len();
        for word in &state.words[..n] {
            w.write_u32(*word)?;
        }
        Ok(())
        // `idcode` is a construction property and `halted` belongs to whatever
        // is debugging; neither is the machine's state.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::default();
        let masks = self.regs.variant.layout().masks;
        for (word, &mask) in state.words.iter_mut().zip(masks) {
            let value = r.read_u32()?;
            if value & !mask != 0 {
                return Err(Error::State(format!(
                    "snapshot has a DBGMCU register with reserved bits set: {value:#010x}"
                )));
            }
            *word = value;
        }
        *self.regs.state.lock() = state;
        self.regs.refresh_outputs();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let index = self
            .regs
            .variant
            .layout()
            .freeze
            .iter()
            .position(|f| f.pin == port)
            .ok_or_else(|| Error::Config {
                at: String::from(port),
                message: String::from(
                    "a DBGMCU drives one freeze pin per DBG_xxx_STOP bit; see `rsemu describe \
                     st.dbgmcu`",
                ),
            })?;
        self.regs.outputs.lock().push((index, source));
        self.regs.refresh_outputs();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        // Every freeze pin idles low out of reset, which a fresh net already
        // is — but a machine wired after a snapshot load has bits that
        // survived it, and a debugger that was already halted when the
        // snapshot was taken is halted again the moment it reattaches.
        self.regs.refresh_outputs();
    }

    fn debug_halt(&self, halted: bool) {
        self.regs.set_halted(halted);
    }
}

impl Instance for Dbgmcu {}

/// The `st.dbgmcu` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 debug support: IDCODE, the low-power debug bits and the peripheral freeze bits",
    properties: &[
        PropertySpec {
            name: "dev-id",
            kind: ValueKind::Uint,
            required: false,
            summary: "IDCODE.DEV_ID, twelve bits: 0x413 on an F405/407/415/417 (the f4 default), \
                      0x482 on a U575/U585 (the u5 default), 0x495 on a WB55/WB35 (the wb \
                      default)",
        },
        PropertySpec {
            name: "rev-id",
            kind: ValueKind::Uint,
            required: false,
            summary: "IDCODE.REV_ID, sixteen bits: the silicon revision, 0x1000 for rev A (0x2001 \
                      on a wb: WB55 rev Y, WB35 rev A)",
        },
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "register file: f4 (default; RM0090, at 0xE0042000), u5 (RM0456, at \
                      0xE0044000) or wb (RM0434, at 0xE0042000)",
        },
    ],
    construct: |props| Ok(Box::new(Dbgmcu::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Dbgmcu::new(props)?)))
}

/// What the validator should know about `st.dbgmcu`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("dev-id", ValueKind::Uint).range(0, 0xfff))
        .prop(PropSchema::new("rev-id", ValueKind::Uint).range(0, 0xffff))
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "u5", "wb"]))
        .region("")
        .region("regs");
    // Both variants' pins: the schema does not know which one a board names,
    // and `connect` refuses a pin the chosen variant does not have.
    let mut seen: Vec<&str> = Vec::new();
    for bit in FREEZE.iter().chain(FREEZE_U5).chain(FREEZE_WB) {
        if !seen.contains(&bit.pin) {
            seen.push(bit.pin);
            schema = schema.port(bit.pin, PortDir::Out);
        }
    }
    schema
}

#[cfg(test)]
mod tests;
