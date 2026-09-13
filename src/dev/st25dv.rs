//! The STMicroelectronics ST25DV04K / ST25DV16K / ST25DV64K dynamic NFC tag.
//!
//! A dual-interface EEPROM: the same array is reachable from an I²C host on two
//! device addresses *and* from an ISO/IEC 15693 reader over a 13.56 MHz field.
//! Between the two sits a 256-byte **mailbox** — the "dynamic" in dynamic tag —
//! plus a `GPO` interrupt pin, an energy-harvesting element, and four
//! password-protected memory areas.
//!
//! # Source
//!
//! ST **DS10925 rev 9** (*ST25DV04K ST25DV16K ST25DV64K — dynamic NFC/RFID tag
//! IC with 4-Kbit, 16-Kbit or 64-Kbit EEPROM, and fast transfer mode
//! capability*), cited by table and section number throughout, together with
//! ISO/IEC 15693-3 for the command and response framing. No emulator source of
//! any licence was consulted (`ROADMAP.md` §1).
//!
//! # The two halves
//!
//! ```text
//!        I2C host                     ST25DV                       RF reader
//!   ┌──────────────┐         ┌────────────────────────┐       ┌───────────────┐
//!   │ st.i2c   ────┼── 0x53 ─┤ user EEPROM            ├── 23h ┤ rf::Reader    │
//!   │              │   0x57  │ system configuration   │   A0h │  (a host door)│
//!   │  gpio.exti ◄─┼── GPO ──┤ dynamic registers      │   ADh │               │
//!   └──────────────┘         │ mailbox (256 B)        ├── ACh └───────────────┘
//!                            └────────────────────────┘
//! ```
//!
//! The left-hand side is ordinary device work: an [`I2cSlave`] on the
//! [`atmel::at24c`](crate::dev::atmel::at24c) pattern, with two device
//! addresses instead of one and a much larger register file.
//!
//! The right-hand side is **not** inside the machine. A reader tapping the tag
//! is host input crossing into a deterministic simulation, so it goes through
//! the record/replay seam like a keystroke: [`rf`] declares a
//! [`HostKind::door`](crate::core::hosts::HostKind::door) and a recorded
//! session replays to the same state hash. See that module for the whole
//! argument.
//!
//! # What is modelled
//!
//! * **Both I²C device addresses** (§6.3, Table 88): `1010 E2 1 1`, so `0x53`
//!   for user memory, dynamic registers and the mailbox and `0x57` for the
//!   system configuration area. A 16-bit big-endian byte address follows.
//! * **Byte and sequential write** (§6.4), up to 256 bytes in one command, with
//!   every inhibition rule of §6.4.1 and §6.4.2: area protection through
//!   `I2CSS`, the ban on crossing an area border inside one sequence, the ban on
//!   writing EEPROM at all while fast transfer mode is enabled (the write path
//!   *is* the mailbox buffer), read-only registers, and the 256-byte cap.
//! * **The internally self-timed write cycle** (§6.4.2, §6.4.3): tW per
//!   four-byte EEPROM page touched, during which the part NACKs its own
//!   address, so acknowledge polling works exactly as the flow chart says.
//! * **Sequential read** (§6.5.3) with no roll-over anywhere: past an area
//!   border, past the end of memory, past the end of the mailbox message, the
//!   part returns `0xFF` forever.
//! * **The I²C security session** (§6.6.1): the present-password command —
//!   address `0900h`, eight bytes, validation code `09h`, the same eight bytes
//!   again — and `I2C_SSO_Dyn`. Validation code `07h` is the write-password
//!   command (§6.6.2).
//! * **The mailbox** (§5.1): `MB_MODE`, `MB_EN`, `HOST_PUT_MSG`/`RF_PUT_MSG`,
//!   `HOST_MISS_MSG`/`RF_MISS_MSG`, `HOST_CURRENT_MSG`/`RF_CURRENT_MSG`,
//!   `MB_LEN_Dyn` as length−1, the rule that an I²C write must start at `2008h`
//!   and an RF write at offset 0, that a read never clears the *writer's* flag,
//!   and the `MB_WDG` watchdog that frees a message nobody read.
//! * **`GPO`** (§5.2) with the `IT_TIME` pulse of Eq. (1), in both output
//!   styles: open drain (`-IE`) idles high-Z and pulls to ground, CMOS (`-JF`)
//!   idles low and drives a positive pulse.
//! * **`IT_STS_Dyn`** (Table 32), which is *not* `GPO`'s bit layout — the one
//!   `FIELD_CHANGE_EN` enable bit becomes two status bits, `FIELD_FALLING` and
//!   `FIELD_RISING`, and everything above it shifts by one. It clears on read
//!   and a debug read does not clear it.
//! * **Field detect and energy harvesting flags** (§5.3): `EH_MODE`, and
//!   `EH_CTRL_Dyn`'s `EH_EN`, `EH_ON`, `FIELD_ON` and `VCC_ON`.
//! * **RF management** (§5.4): `RF_MNGT`/`RF_MNGT_Dyn`, so an I²C host can make
//!   the tag answer error `0Fh` (`RF_DISABLE`) or go silent (`RF_SLEEP`).
//! * **The RF command set** the mailbox needs: Inventory, Read/Write Multiple
//!   Blocks, Read/Write Configuration, Read/Write Dynamic Configuration,
//!   Read/Write Message, Read Message Length, Present/Write Password and Manage
//!   GPO, each with the asymmetries Table 11 and Table 12 record — `I2CSS` and
//!   `LOCK_CCFILE` have no RF address at all, and `I2C_SSO_Dyn`, `IT_STS_Dyn`
//!   and `MB_LEN_Dyn` are I²C-only dynamic registers.
//!
//! # What is not
//!
//! * **The analogue half of energy harvesting.** `EH_MODE`, `EH_EN`, `EH_ON`,
//!   `FIELD_ON` and `VCC_ON` are digital state a driver reads and they are all
//!   here; `V_EH` is a current source whose output is a *voltage*, and
//!   [`core::wire`](crate::core::wire) carries logic levels. There is nothing
//!   to drive and inventing a logic pin for it would be a lie about the part.
//!   A board that wants harvested power modelled needs an analogue seam this
//!   tree does not have; until then `EH_ON` is the whole observable effect, and
//!   it is exactly what firmware branches on.
//! * **RF field *strength*.** The reader door says the field is present or
//!   absent. Anything finer — the modulation index, the distance at which
//!   harvesting starts working — is analogue for the same reason.
//! * **The RF physical layer**: SOF/EOF, the two data rates, Manchester
//!   subcarriers, anticollision slots, CRC16. A door delivers a *command*, not
//!   a waveform, so `RF_ACTIVITY` (a level that lasts from request EOF to
//!   response EOF) has nowhere to live — the status bit is set, the pin is not
//!   driven from it, and `Shared::gpo_level` says so in the source.
//! * `Lock Block`, `Write AFI`/`DSFID` and the extended (32-bit address)
//!   command set. The tag answers `02h`, "command not recognized", which is
//!   what a real part does for a command it does not implement.
//!
//! # `no_std`
//!
//! All of it, `alloc` included. The reader door is `no_std` too: what needs an
//! operating system is the thing *holding* a reader, not the tag.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::i2c::wires::{SlaveWires, SlaveWiresState, pin as line};
use crate::bus::i2c::{Ack, Address, Direction, I2cBus, I2cSlave, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Drive, Level, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

pub mod rf;

#[cfg(test)]
mod tests;

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.st25dv";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The seven-bit address of the user half: `1010 0 1 1` (§6.3, Table 88, with
/// `E2 = 0`). A machine file sees it as `0xA6`/`0xA7` on the wire.
pub const USER_ADDRESS: u8 = 0b101_0011;

/// The seven-bit address of the system configuration half, `E2 = 1`.
pub const SYSTEM_ADDRESS: u8 = 0b101_0111;

/// Where the dynamic registers start in the `E2 = 0` address space (Table 12).
pub const DYN_BASE: u16 = 0x2000;

/// Where the fast transfer mode mailbox starts (Table 13).
pub const MAILBOX_BASE: u16 = 0x2008;

/// How many bytes the mailbox holds (§4.5).
pub const MAILBOX_SIZE: usize = 256;

/// The last mailbox byte's I²C address.
const MAILBOX_END: u16 = MAILBOX_BASE + MAILBOX_SIZE as u16 - 1;

/// Where the I²C password lives in the `E2 = 1` space (Table 11).
pub const I2C_PWD_BASE: u16 = 0x0900;

/// The most bytes one I²C write command may carry (§6.4.2, Table 89).
pub const MAX_SEQUENTIAL_WRITE: u32 = 256;

/// One internal EEPROM page, in bytes (§6.4.2: "internally organized in pages
/// of 4 bytes long"). A sequential write costs tW per page it touches.
pub const EEPROM_PAGE: u64 = 4;

/// One RF block, in bytes — what `BLK_SIZE` reports as `03h`, which is the
/// size minus one (Table 81).
pub const BLOCK_SIZE: u64 = 4;

/// tW, the internal write cycle, in ticks of this device's clock domain.
///
/// §6.4.3 and the AC characteristics give 5 ms. In ticks rather than seconds
/// because the time path has no floats (`CLAUDE.md`) and a device does not own
/// a frequency: a board clocking this part from a 1 MHz domain gets 5 ms from
/// the default.
pub const DEFAULT_WRITE_TICKS: u64 = 5_000;

/// The `IT_TIME = 0` GPO pulse, in ticks: 301 µs at a 1 MHz domain (Eq. (1)).
pub const DEFAULT_IT_TICKS: u64 = 301;

/// The mailbox watchdog's unit, in ticks: 30 ms at a 1 MHz domain (Table 17,
/// "Watch dog duration = 2^(MB_WDG-1) x 30ms").
pub const DEFAULT_WDG_TICKS: u64 = 30_000;

/// The numerator Eq. (1) is scaled by, so the pulse divides exactly.
///
/// `301 µs − IT_TIME × 37.65 µs` in hundredths of a microsecond is
/// `30100 − IT_TIME × 3765`, which is integers all the way down — and
/// `CLAUDE.md` has no floats in the time path.
const IT_SCALE: u64 = 30_100;

/// The per-step subtrahend of Eq. (1), on [`IT_SCALE`]'s scale.
const IT_STEP: u64 = 3_765;

// ---------------------------------------------------------------------------
// Register addresses and bits
// ---------------------------------------------------------------------------

/// System configuration register addresses, `E2 = 1` (Table 11).
pub mod sys {
    /// Enable/disable interrupts on GPO.
    pub const GPO: u16 = 0x0000;
    /// Interrupt pulse duration.
    pub const IT_TIME: u16 = 0x0001;
    /// Energy harvesting strategy after power-on.
    pub const EH_MODE: u16 = 0x0002;
    /// RF interface state after power-on.
    pub const RF_MNGT: u16 = 0x0003;
    /// Area 1 RF access protection.
    pub const RFA1SS: u16 = 0x0004;
    /// Area 1 ending point.
    pub const ENDA1: u16 = 0x0005;
    /// Area 2 RF access protection.
    pub const RFA2SS: u16 = 0x0006;
    /// Area 2 ending point.
    pub const ENDA2: u16 = 0x0007;
    /// Area 3 RF access protection.
    pub const RFA3SS: u16 = 0x0008;
    /// Area 3 ending point.
    pub const ENDA3: u16 = 0x0009;
    /// Area 4 RF access protection.
    pub const RFA4SS: u16 = 0x000a;
    /// Areas 1 to 4 I²C access protection. **No RF address** (Table 11).
    pub const I2CSS: u16 = 0x000b;
    /// Blocks 0 and 1 RF write protection. **No RF address**.
    pub const LOCK_CCFILE: u16 = 0x000c;
    /// Fast transfer mode state after power-on.
    pub const MB_MODE: u16 = 0x000d;
    /// Mailbox watchdog.
    pub const MB_WDG: u16 = 0x000e;
    /// Protect RF write access to the configuration registers.
    pub const LOCK_CFG: u16 = 0x000f;
    /// The last writable system register.
    pub const LAST_WRITABLE: u16 = LOCK_CFG;
    /// DSFID lock status, read-only over I²C.
    pub const LOCK_DSFID: u16 = 0x0010;
    /// AFI lock status, read-only over I²C.
    pub const LOCK_AFI: u16 = 0x0011;
    /// DSFID, read-only over I²C.
    pub const DSFID: u16 = 0x0012;
    /// AFI, read-only over I²C.
    pub const AFI: u16 = 0x0013;
    /// Memory size in blocks, minus one, little-endian over two bytes.
    pub const MEM_SIZE: u16 = 0x0014;
    /// Block size in bytes, minus one.
    pub const BLK_SIZE: u16 = 0x0016;
    /// The ISO/IEC 15693 IC reference.
    pub const IC_REF: u16 = 0x0017;
    /// The eight-byte unique identifier, byte 0 (LSB) first.
    pub const UID: u16 = 0x0018;
    /// The IC revision.
    pub const IC_REV: u16 = 0x0020;
}

/// Dynamic register addresses, `E2 = 0` (Table 12).
pub mod dyn_reg {
    use super::DYN_BASE;

    /// GPO control: a copy of `GPO` whose bit 7 the host may toggle freely.
    pub const GPO_CTRL: u16 = DYN_BASE;
    /// ST reserved.
    pub const RESERVED: u16 = DYN_BASE + 1;
    /// Energy harvesting management and power-source status.
    pub const EH_CTRL: u16 = DYN_BASE + 2;
    /// RF interface usage management.
    pub const RF_MNGT: u16 = DYN_BASE + 3;
    /// I²C security session status. **No RF address**.
    pub const I2C_SSO: u16 = DYN_BASE + 4;
    /// Interrupt status, cleared by reading it. **No RF address**.
    pub const IT_STS: u16 = DYN_BASE + 5;
    /// Fast transfer mode control and status.
    pub const MB_CTRL: u16 = DYN_BASE + 6;
    /// Length of the message in the mailbox, minus one. **No RF address**.
    pub const MB_LEN: u16 = DYN_BASE + 7;
}

/// `GPO` and `GPO_CTRL_Dyn` bits (Table 26, Table 30 — the same layout).
pub mod gpo_bit {
    /// GPO level is driven by the Manage GPO command.
    pub const RF_USER_EN: u8 = 1 << 0;
    /// GPO level follows an RF command in progress.
    pub const RF_ACTIVITY_EN: u8 = 1 << 1;
    /// Manage GPO may request a pulse.
    pub const RF_INTERRUPT_EN: u8 = 1 << 2;
    /// A pulse when the RF field appears or disappears.
    pub const FIELD_CHANGE_EN: u8 = 1 << 3;
    /// A pulse when RF completes a Write Message.
    pub const RF_PUT_MSG_EN: u8 = 1 << 4;
    /// A pulse when RF reads the last byte of a message.
    pub const RF_GET_MSG_EN: u8 = 1 << 5;
    /// A pulse when RF completes a write into EEPROM.
    pub const RF_WRITE_EN: u8 = 1 << 6;
    /// The output stage itself.
    pub const GPO_EN: u8 = 1 << 7;
}

/// `IT_STS_Dyn` bits (Table 32).
///
/// **Not [`gpo_bit`]'s layout.** `FIELD_CHANGE_EN` is one enable bit and
/// becomes two status bits, so everything from `RF_PUT_MSG` up sits one
/// position higher than its enable. Getting this wrong is the classic ST25DV
/// driver bug.
pub mod it_bit {
    /// Manage GPO set the pin.
    pub const RF_USER: u8 = 1 << 0;
    /// An RF access happened.
    pub const RF_ACTIVITY: u8 = 1 << 1;
    /// Manage GPO requested an interrupt.
    pub const RF_INTERRUPT: u8 = 1 << 2;
    /// The RF field went away.
    pub const FIELD_FALLING: u8 = 1 << 3;
    /// The RF field appeared.
    pub const FIELD_RISING: u8 = 1 << 4;
    /// RF put a message in the mailbox.
    pub const RF_PUT_MSG: u8 = 1 << 5;
    /// RF read a message out of the mailbox, reaching its end.
    pub const RF_GET_MSG: u8 = 1 << 6;
    /// RF wrote EEPROM.
    pub const RF_WRITE: u8 = 1 << 7;
}

/// `MB_CTRL_Dyn` bits (Table 19).
pub mod mb_bit {
    /// Fast transfer mode is enabled. The only writable bit.
    pub const MB_EN: u8 = 1 << 0;
    /// The I²C host put a message in the mailbox.
    pub const HOST_PUT_MSG: u8 = 1 << 1;
    /// The RF reader put a message in the mailbox.
    pub const RF_PUT_MSG: u8 = 1 << 2;
    /// The I²C host did not read an RF message before the watchdog fired.
    pub const HOST_MISS_MSG: u8 = 1 << 4;
    /// The RF reader did not read an I²C message before the watchdog fired.
    pub const RF_MISS_MSG: u8 = 1 << 5;
    /// The message in the mailbox came from I²C.
    pub const HOST_CURRENT_MSG: u8 = 1 << 6;
    /// The message in the mailbox came from RF.
    pub const RF_CURRENT_MSG: u8 = 1 << 7;
}

/// `EH_CTRL_Dyn` bits (Table 37).
pub mod eh_bit {
    /// Energy harvesting is requested.
    pub const EH_EN: u8 = 1 << 0;
    /// Energy harvesting is running.
    pub const EH_ON: u8 = 1 << 1;
    /// An RF field is present.
    pub const FIELD_ON: u8 = 1 << 2;
    /// `VCC` is present and low-power-down is not forced.
    pub const VCC_ON: u8 = 1 << 3;
}

/// `RF_MNGT` and `RF_MNGT_Dyn` bits (Table 40, Table 42).
pub mod rf_bit {
    /// RF commands are interpreted but answered with error `0Fh`.
    pub const RF_DISABLE: u8 = 1 << 0;
    /// The RF interface is silent.
    pub const RF_SLEEP: u8 = 1 << 1;
}

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt output (§5.2).
    pub const GPO: &str = "gpo";
    /// The low-power-down input, on the 10-ball and 12-pin packages.
    ///
    /// High forces low-power-down, which clears `VCC_ON` — and with it the
    /// mailbox, which §5.1.2 says needs `VCC`.
    pub const LPD: &str = "lpd";
    /// The wire line number the `LPD` sink answers on, past
    /// [`crate::bus::i2c::wires::pin::SDA`] so one device can host the two bus
    /// lines and this one without their numbers colliding.
    pub const LPD_LINE: u32 = 2;
}

// ---------------------------------------------------------------------------
// Density
// ---------------------------------------------------------------------------

/// Which member of the family this is.
///
/// The three parts differ in user memory size, in `MEM_SIZE`, in `IC_REF` and
/// in the factory `ENDAx` values, and in nothing else (Table 6, Table 79,
/// Table 83).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Density {
    /// ST25DV04K: 4 Kbit, 512 bytes, 128 blocks.
    K4,
    /// ST25DV16K: 16 Kbit, 2048 bytes, 512 blocks.
    K16,
    /// ST25DV64K: 64 Kbit, 8192 bytes, 2048 blocks.
    K64,
}

impl Density {
    /// Every spelling a machine description may use, for a validator.
    pub const NAMES: &'static [&'static str] = &["4K", "16K", "64K"];

    /// Parse the spelling a machine description uses.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Density> {
        match name {
            "4K" | "4k" => Some(Density::K4),
            "16K" | "16k" => Some(Density::K16),
            "64K" | "64k" => Some(Density::K64),
            _ => None,
        }
    }

    /// The spelling a machine description uses.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Density::K4 => "4K",
            Density::K16 => "16K",
            Density::K64 => "64K",
        }
    }

    /// User memory, in bytes.
    #[must_use]
    pub const fn bytes(self) -> u64 {
        match self {
            Density::K4 => 512,
            Density::K16 => 2048,
            Density::K64 => 8192,
        }
    }

    /// User memory, in four-byte RF blocks.
    #[must_use]
    pub const fn blocks(self) -> u64 {
        self.bytes() / BLOCK_SIZE
    }

    /// `IC_REF` (Table 83). The 16K and the 64K share `26h`.
    #[must_use]
    pub const fn ic_ref(self) -> u8 {
        match self {
            Density::K4 => 0x24,
            Density::K16 | Density::K64 => 0x26,
        }
    }

    /// The factory `ENDA1`/`ENDA2`/`ENDA3`, which is the end of memory and so
    /// means "one area" (Table 6).
    #[must_use]
    pub const fn enda_max(self) -> u8 {
        match self {
            Density::K4 => 0x0f,
            Density::K16 => 0x3f,
            Density::K64 => 0xff,
        }
    }
}

impl fmt::Display for Density {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// How the `GPO` output stage is built.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GpoStyle {
    /// The `-IE` parts: open drain. Idle is high-Z, asserted is pulled to
    /// ground, and the board supplies the pull-up.
    #[default]
    OpenDrain,
    /// The `-JF` parts: CMOS. §5.2.1 defines them by inverting the open-drain
    /// curve, so idle is driven low and asserted is driven high.
    Cmos,
}

impl GpoStyle {
    /// Every spelling, for a validator.
    pub const NAMES: &'static [&'static str] = &["open-drain", "cmos"];

    /// Parse the spelling a machine description uses.
    #[must_use]
    pub fn from_name(name: &str) -> Option<GpoStyle> {
        match name {
            "open-drain" => Some(GpoStyle::OpenDrain),
            "cmos" => Some(GpoStyle::Cmos),
            _ => None,
        }
    }

    /// The spelling a machine description uses.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            GpoStyle::OpenDrain => "open-drain",
            GpoStyle::Cmos => "cmos",
        }
    }

    /// What the pin does while no interrupt is asserted.
    #[must_use]
    const fn idle(self) -> Drive {
        match self {
            GpoStyle::OpenDrain => Drive::HiZ,
            GpoStyle::Cmos => Drive::Low,
        }
    }

    /// What the pin does while one is.
    #[must_use]
    const fn asserted(self) -> Drive {
        match self {
            GpoStyle::OpenDrain => Drive::Low,
            GpoStyle::Cmos => Drive::High,
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// "Nothing scheduled".
const NO_EVENT: u64 = u64::MAX;

/// Where the I²C transaction is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    /// Not addressed.
    #[default]
    Idle,
    /// Addressed for a write; the next byte is the address MSB (Table 90).
    AddrHi,
    /// The address LSB.
    AddrLo,
    /// Data bytes.
    Writing,
    /// The 17 bytes of a present- or write-password command (§6.6).
    Password,
    /// Addressed for a read.
    Reading,
    /// A byte was refused, so the part waits for a whole new instruction
    /// (§6.4: "ST25DVxxx enters in I2C dead state").
    Dead,
}

/// One staged EEPROM byte, waiting for the STOP that commits it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Staged {
    /// Where, in the flat space the commit walks: user bytes first, then the
    /// system area at [`SYS_STAGE`].
    at: u32,
    /// What.
    byte: u8,
}

/// Where a staged system-area byte's address starts, above any user array.
const SYS_STAGE: u32 = 0x10_0000;

/// Everything a snapshot has to carry, plus the parts that are deliberately
/// derived and are documented as such at their declaration.
#[derive(Debug, Clone)]
struct State {
    /// Domain ticks simulated.
    ticks: u64,

    // -- EEPROM ------------------------------------------------------------
    /// User memory.
    user: Vec<u8>,
    /// The writable system configuration registers, `0000h`..=`000Fh`.
    sys: [u8; 16],
    /// `LOCK_DSFID`, `LOCK_AFI`, `DSFID`, `AFI` — read-only over I²C.
    ident: [u8; 4],
    /// The I²C password (Table 11, `0900h`..`0907h`), MSB first.
    pwd: [u8; 8],
    /// The four RF passwords, index 0 being the configuration password.
    rf_pwd: [[u8; 8]; 4],

    // -- dynamic registers -------------------------------------------------
    /// `GPO_CTRL_Dyn`.
    gpo_dyn: u8,
    /// `EH_CTRL_Dyn`'s `EH_EN`. The other three bits are derived.
    eh_en: bool,
    /// `RF_MNGT_Dyn`.
    rf_mngt_dyn: u8,
    /// `I2C_SSO_Dyn`.
    sso: bool,
    /// `IT_STS_Dyn`.
    it_sts: u8,
    /// `MB_CTRL_Dyn`.
    mb_ctrl: u8,
    /// `MB_LEN_Dyn`: the message length minus one.
    mb_len: u8,
    /// The mailbox buffer.
    mailbox: Vec<u8>,

    // -- RF sessions -------------------------------------------------------
    /// Whether the RF configuration security session is open.
    rf_cfg_session: bool,
    /// Which RF user password opened a session, 1 to 3, or 0 for none.
    rf_user_session: u8,

    // -- timing ------------------------------------------------------------
    /// The tick the internal write cycle ends on.
    busy_until: u64,
    /// Whether one is running.
    busy: bool,
    /// The tick the mailbox watchdog fires on.
    wdg_until: u64,
    /// Whether it is armed.
    wdg_armed: bool,
    /// The tick the GPO pulse ends on.
    pulse_until: u64,
    /// Whether one is running.
    pulsing: bool,
    /// The level Manage GPO last forced, for `RF_USER`.
    rf_user: bool,

    // -- the I²C transaction ----------------------------------------------
    /// Where in a transaction the part is.
    phase: Phase,
    /// Which half was addressed: `true` for `E2 = 1`, the system area.
    e2: bool,
    /// The internal byte address counter (§6.4).
    addr: u16,
    /// How many data bytes this write command has carried (§6.4.2's cap).
    written: u32,
    /// Whether any byte of it was refused, which cancels the whole programming.
    refused: bool,
    /// Bytes staged for the EEPROM, committed at the STOP.
    staged: Vec<Staged>,
    /// Which user area the first staged byte belonged to, so a border crossing
    /// can be refused (§6.4.2).
    area: Option<u8>,
    /// Whether this write touched the mailbox, and how many bytes it put there.
    mb_written: u32,
    /// Whether the read counter reached the end of the mailbox message, which
    /// frees the mailbox at the following STOP (§5.1.2).
    mb_read_end: bool,
    /// The 17 bytes of a password command.
    pwd_buf: Vec<u8>,

    // -- derived, and never serialized -------------------------------------
    /// Whether an RF field is present.
    ///
    /// **Not saved.** It is the reader's state, not the tag's — the same
    /// argument `atmel.at24c` makes about its `WP` pin and `keypad.matrix`
    /// makes about the levels it senses. A snapshot loaded somewhere else has
    /// no reader over it, so the field comes back absent and the host door
    /// re-announces one if there is one.
    field: bool,
    /// Whether `LPD` is held high, which is `VCC_ON` inverted.
    ///
    /// Not saved either, and for the same reason: it is another device's pin.
    lpd: bool,
}

/// Everything both halves of the part reach.
struct Shared {
    state: Mutex<State>,
    /// Which member of the family.
    density: Density,
    /// How the GPO output stage is built.
    gpo_style: GpoStyle,
    /// tW, in ticks, per four-byte EEPROM page.
    write_ticks: u64,
    /// The `IT_TIME = 0` pulse, in ticks.
    it_ticks: u64,
    /// The mailbox watchdog's unit, in ticks.
    wdg_ticks: u64,
    /// The unique identifier, byte 0 (LSB) first, as the I²C map presents it
    /// (Table 85).
    uid: [u8; 8],
    /// `IC_REV`.
    ic_rev: u8,
    /// Domain ticks simulated, for the scheduler's lock-free question.
    ticks: AtomicU64,
    /// The next tick something is due on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The catch-up handle, once the machine has given us one.
    lazy: Mutex<Option<LazyHandle>>,
    /// The `GPO` output, once a machine has wired it.
    gpo: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("St25dvShared");
        s.field("density", &self.density);
        s.field("gpo", &self.gpo_style);
        match self.state.try_lock() {
            Some(state) => s
                .field("phase", &state.phase)
                .field("addr", &alloc::format!("{:#06x}", state.addr))
                .field("field", &state.field)
                .field("mb_ctrl", &alloc::format!("{:#04x}", state.mb_ctrl))
                .field("busy", &state.busy),
            None => s.field("state", &"<in use>"),
        };
        s.finish()
    }
}

// ---------------------------------------------------------------------------
// Shared: derived reads
// ---------------------------------------------------------------------------

impl Shared {
    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        let mut next = NO_EVENT;
        let mut soonest = |at: u64| {
            let at = at.max(state.ticks.saturating_add(1));
            if at < next {
                next = at;
            }
        };
        if state.busy {
            soonest(state.busy_until);
        }
        if state.wdg_armed {
            soonest(state.wdg_until);
        }
        if state.pulsing {
            soonest(state.pulse_until);
        }
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Where this device's clock domain has got to.
    ///
    /// **Not [`LazyHandle::sync`]**, for the reason `atmel.at24c` records: the
    /// bus reaches this device from inside the bit engine's lock and `sync`
    /// would re-enter it. Asking where the domain *is* answers every question
    /// this part has and touches nothing.
    fn now(&self, state: &State) -> u64 {
        let handle = self.lazy.lock().clone();
        match handle {
            Some(handle) => handle.present_tick().max(state.ticks),
            None => state.ticks,
        }
    }

    /// Whether the internal write cycle is still running (§6.4.3).
    fn is_busy(&self, state: &State) -> bool {
        state.busy && self.now(state) < state.busy_until
    }

    /// `EH_CTRL_Dyn`, assembled (Table 37).
    ///
    /// Only `EH_EN` is stored; `EH_ON`, `FIELD_ON` and `VCC_ON` are facts about
    /// the present moment, so they are computed rather than kept — the
    /// "derived state is never serialized" rule in one register.
    fn eh_ctrl(&self, state: &State) -> u8 {
        let mut v = 0;
        if state.eh_en {
            v |= eh_bit::EH_EN;
        }
        // "EH_ON set reflects the EH_EN bit value" (§5.3.2), and §5.3.2's state
        // diagram only delivers with a field: no field, nothing to harvest.
        if state.eh_en && state.field {
            v |= eh_bit::EH_ON;
        }
        if state.field {
            v |= eh_bit::FIELD_ON;
        }
        if state.vcc_on() {
            v |= eh_bit::VCC_ON;
        }
        v
    }

    /// The GPO pulse this part emits, in ticks (Eq. (1)).
    fn pulse_ticks(&self, state: &State) -> u64 {
        let it = u64::from(state.sys[sys::IT_TIME as usize] & 0b111);
        // 301 µs − IT_TIME × 37.65 µs, on a scale of a hundredth of a
        // microsecond so the arithmetic is exact. IT_TIME is three bits, so
        // the subtraction cannot go below 30100 − 7 × 3765 = 3745.
        let scaled = IT_SCALE - it * IT_STEP;
        self.it_ticks.saturating_mul(scaled) / IT_SCALE
    }

    /// The mailbox watchdog, in ticks (Table 17), or `None` for infinite.
    fn wdg_duration(&self, state: &State) -> Option<u64> {
        let wdg = u32::from(state.sys[sys::MB_WDG as usize] & 0b111);
        if wdg == 0 {
            // "If MD_WDG = 0, then watchdog duration is infinite".
            return None;
        }
        Some(self.wdg_ticks.saturating_mul(1u64 << (wdg - 1)))
    }

    /// What the `GPO` pin is doing.
    ///
    /// `RF_ACTIVITY` is absent on purpose: Table 26 makes it a *level* that
    /// lasts from a request's EOF to its response's EOF, and a door delivers a
    /// command at one virtual instant, so the level would have zero width. The
    /// status bit is still set — firmware polling `IT_STS_Dyn` sees the access.
    fn gpo_level(&self, state: &State) -> Drive {
        if state.gpo_dyn & gpo_bit::GPO_EN == 0 {
            // Table 33: either enable at zero leaves the pin idle. Bit 7 of
            // GPO_CTRL_Dyn "is prevalent over" the one in GPO, and GPO's copy
            // reaches GPO_CTRL_Dyn whenever GPO is written, so this one bit is
            // the whole table.
            return self.gpo_style.idle();
        }
        if state.gpo_dyn & gpo_bit::RF_USER_EN != 0 && state.rf_user {
            // §5.2.1: "RF_USER is prevalent over all other GPO events".
            return self.gpo_style.asserted();
        }
        if state.pulsing {
            return self.gpo_style.asserted();
        }
        self.gpo_style.idle()
    }

    /// Drive `GPO` to match `state`. **Call with the state lock released.**
    fn refresh_gpo(&self, drive: Drive) {
        let source = self.gpo.lock().clone();
        if let Some(source) = source {
            source.drive(drive);
        }
    }
}

impl State {
    /// Whether `VCC` is present and low-power-down is not forced (Table 37).
    ///
    /// A tag on an I²C bus is powered by the board that hosts the bus, so the
    /// only thing that can take `VCC_ON` away is the `LPD` pin.
    const fn vcc_on(&self) -> bool {
        !self.lpd
    }

    /// Whether fast transfer mode is usable: authorised, enabled, and powered
    /// (§5.1.2, "VCC supply source is mandatory to activate this feature").
    const fn ftm(&self) -> bool {
        self.sys[sys::MB_MODE as usize] & 1 != 0
            && self.mb_ctrl & mb_bit::MB_EN != 0
            && self.vcc_on()
    }

    /// Whether the mailbox holds a message nobody has read.
    const fn mb_busy(&self) -> bool {
        self.mb_ctrl & (mb_bit::HOST_PUT_MSG | mb_bit::RF_PUT_MSG) != 0
    }

    /// How many bytes the message in the mailbox has.
    const fn mb_message_len(&self) -> usize {
        self.mb_len as usize + 1
    }

    /// Which user area byte `at` belongs to, 0 to 3 (§4.2).
    ///
    /// `ENDAi` counts 32-byte steps over I²C: "End Area i = 32 × ENDAi + 31"
    /// (Table 6). The last area always ends at the last user byte, so an
    /// `ENDAi` at the end of memory means the area after it does not exist.
    fn area_of(&self, at: u64) -> u8 {
        for (index, reg) in [sys::ENDA1, sys::ENDA2, sys::ENDA3].into_iter().enumerate() {
            let end = 32 * u64::from(self.sys[reg as usize]) + 31;
            if at <= end {
                #[allow(clippy::cast_possible_truncation)]
                return index as u8;
            }
        }
        3
    }

    /// The two `I2CSS` bits for an area (Table 52).
    fn i2css(&self, area: u8) -> u8 {
        (self.sys[sys::I2CSS as usize] >> (2 * area)) & 0b11
    }

    /// Whether an I²C read of `area` is allowed (Table 52).
    ///
    /// Area 1 is "Read always allowed" for all four codes; the others read-lock
    /// on `10` and `11`.
    fn i2c_can_read(&self, area: u8) -> bool {
        if area == 0 {
            return true;
        }
        self.i2css(area) & 0b10 == 0 || self.sso
    }

    /// Whether an I²C write of `area` is allowed (Table 52).
    ///
    /// The low bit of the code is the write rule for every area — `00` and `10`
    /// are "Write always allowed", `01` and `11` want the session — which is
    /// why the areas differ on reading and agree on writing.
    fn i2c_can_write(&self, area: u8) -> bool {
        self.i2css(area) & 0b01 == 0 || self.sso
    }

    /// The two `RFAxSS` read/write protection bits (Table 44).
    fn rfass(&self, area: u8) -> u8 {
        let reg = match area {
            0 => sys::RFA1SS,
            1 => sys::RFA2SS,
            2 => sys::RFA3SS,
            _ => sys::RFA4SS,
        };
        (self.sys[reg as usize] >> 2) & 0b11
    }

    /// Which RF password opens `area`'s user session (Table 44, `PWD_CTRL_Ax`).
    fn rf_pwd_of(&self, area: u8) -> u8 {
        let reg = match area {
            0 => sys::RFA1SS,
            1 => sys::RFA2SS,
            2 => sys::RFA3SS,
            _ => sys::RFA4SS,
        };
        self.sys[reg as usize] & 0b11
    }

    /// Whether the RF user session that governs `area` is open.
    fn rf_area_open(&self, area: u8) -> bool {
        let want = self.rf_pwd_of(area);
        // `00` means no password can open this area's session at all.
        want != 0 && self.rf_user_session == want
    }

    /// Whether an RF read of `area` is allowed.
    ///
    /// **Area 1 is not like the others.** Table 44 makes every `RFA1SS` code
    /// "Read always allowed", which §4.2 states outright — "Area1 is always
    /// readable" — while Tables 46, 48 and 50 read-lock areas 2 to 4 on `10`
    /// and `11`. That is the asymmetry, and it is on the *read* side; the write
    /// rule below is the same for all four.
    fn rf_can_read(&self, area: u8) -> bool {
        if area == 0 {
            return true;
        }
        self.rfass(area) & 0b10 == 0 || self.rf_area_open(area)
    }

    /// Whether an RF write of `area` is allowed (Table 44).
    fn rf_can_write(&self, area: u8) -> bool {
        match self.rfass(area) {
            0b00 => true,
            0b11 => false,
            _ => self.rf_area_open(area),
        }
    }
}

// ---------------------------------------------------------------------------
// Timed events
// ---------------------------------------------------------------------------

impl Shared {
    /// Simulate forward to `target` domain ticks.
    fn advance_to(&self, target: u64) {
        let drive = {
            let mut state = self.state.lock();
            if target <= state.ticks {
                return;
            }
            state.ticks = target;
            if state.busy && target >= state.busy_until {
                state.busy = false;
            }
            if state.pulsing && target >= state.pulse_until {
                state.pulsing = false;
            }
            if state.wdg_armed && target >= state.wdg_until {
                self.watchdog_fired(&mut state);
            }
            self.publish(&state);
            self.gpo_level(&state)
        };
        // Outward, with the lock released: the re-entrancy contract in
        // `core::device`.
        self.refresh_gpo(drive);
    }

    /// The mailbox watchdog timed out (§5.1.2).
    ///
    /// "When a time-out occurs, the mailbox is considered free, and the
    /// `HOST_MISS_MSG` or `RF_MISS_MSG` bits is set" — whichever side failed to
    /// *read* it. The data is not cleared.
    fn watchdog_fired(&self, state: &mut State) {
        state.wdg_armed = false;
        if state.mb_ctrl & mb_bit::HOST_PUT_MSG != 0 {
            // The host put it and RF never collected it.
            state.mb_ctrl &= !mb_bit::HOST_PUT_MSG;
            state.mb_ctrl |= mb_bit::RF_MISS_MSG;
        }
        if state.mb_ctrl & mb_bit::RF_PUT_MSG != 0 {
            state.mb_ctrl &= !mb_bit::RF_PUT_MSG;
            state.mb_ctrl |= mb_bit::HOST_MISS_MSG;
        }
    }

    /// Arm the watchdog on a message that has just been put in the mailbox.
    fn arm_watchdog(&self, state: &mut State) {
        match self.wdg_duration(state) {
            Some(ticks) => {
                let now = self.now(state);
                state.wdg_until = now.saturating_add(ticks);
                state.wdg_armed = true;
            }
            None => state.wdg_armed = false,
        }
    }

    /// Raise the interrupt status bits in `status`, pulsing `GPO` if the
    /// matching enable in `GPO_CTRL_Dyn` is set.
    ///
    /// `enables` is the enable mask, which is *not* `status`: Table 32 splits
    /// `FIELD_CHANGE_EN` into two status bits, so the caller passes both.
    fn raise(&self, state: &mut State, status: u8, enables: u8) {
        // "When enabled, RF events are reported in IT_STS_Dyn register even if
        // GPO output is disabled" — so the status bits land unconditionally.
        state.it_sts |= status;
        if enables == 0 || state.gpo_dyn & enables == 0 {
            return;
        }
        let ticks = self.pulse_ticks(state);
        let now = self.now(state);
        state.pulse_until = now.saturating_add(ticks);
        state.pulsing = true;
    }
}

// ---------------------------------------------------------------------------
// The I2C face
// ---------------------------------------------------------------------------

/// What one I²C byte address refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Target {
    /// User memory, at this byte offset.
    User(u64),
    /// A system configuration register.
    System(u16),
    /// The I²C password, at this byte offset.
    Password(usize),
    /// A dynamic register.
    Dynamic(u16),
    /// The mailbox, at this byte offset.
    Mailbox(usize),
    /// Nothing. Reads answer `0xFF`, writes are refused.
    Nowhere,
}

impl Shared {
    /// Decode an I²C address against the half that was addressed.
    fn target(&self, state: &State, addr: u16) -> Target {
        let at = u64::from(addr);
        if state.e2 {
            return match addr {
                0x0000..=0x0023 => Target::System(addr),
                I2C_PWD_BASE..=0x0907 => Target::Password((addr - I2C_PWD_BASE) as usize),
                _ => Target::Nowhere,
            };
        }
        if at < self.density.bytes() {
            return Target::User(at);
        }
        match addr {
            DYN_BASE..=dyn_reg::MB_LEN => Target::Dynamic(addr),
            MAILBOX_BASE..=MAILBOX_END => Target::Mailbox((addr - MAILBOX_BASE) as usize),
            _ => Target::Nowhere,
        }
    }

    /// What a read of `addr` hands over, with no side effect of any kind.
    ///
    /// Every side effect lives in [`Shared::read_taken`], which is what makes
    /// [`I2cSlave::peek`] — the bus-level spelling of
    /// [`MemAttrs::debug`](crate::core::space::MemAttrs::debug) — free.
    fn read_byte(&self, state: &State, addr: u16) -> u8 {
        match self.target(state, addr) {
            Target::User(at) => {
                // §6.5: a read of a protected area releases the bus, and the
                // host reads FFh.
                #[allow(clippy::cast_possible_truncation)]
                if state.i2c_can_read(state.area_of(at)) {
                    state.user.get(at as usize).copied().unwrap_or(0xff)
                } else {
                    0xff
                }
            }
            Target::System(reg) => match reg {
                0x0000..=sys::LOCK_CFG => state.sys[reg as usize],
                sys::LOCK_DSFID..=sys::AFI => state.ident[(reg - sys::LOCK_DSFID) as usize],
                sys::MEM_SIZE => {
                    // MEM_SIZE is the block count minus one (Table 79).
                    #[allow(clippy::cast_possible_truncation)]
                    {
                        (self.density.blocks() - 1) as u8
                    }
                }
                0x0015 => {
                    #[allow(clippy::cast_possible_truncation)]
                    {
                        ((self.density.blocks() - 1) >> 8) as u8
                    }
                }
                #[allow(clippy::cast_possible_truncation)]
                sys::BLK_SIZE => (BLOCK_SIZE - 1) as u8,
                sys::IC_REF => self.density.ic_ref(),
                sys::UID..=0x001f => self.uid[(reg - sys::UID) as usize],
                sys::IC_REV => self.ic_rev,
                _ => 0xff,
            },
            // Table 11, footnote 8: "Read access is granted if I2C security
            // session is open."
            Target::Password(i) if state.sso => state.pwd[i],
            Target::Password(_) => 0xff,
            Target::Dynamic(reg) => match reg {
                dyn_reg::GPO_CTRL => state.gpo_dyn,
                dyn_reg::EH_CTRL => self.eh_ctrl(state),
                dyn_reg::RF_MNGT => state.rf_mngt_dyn,
                dyn_reg::I2C_SSO => u8::from(state.sso),
                dyn_reg::IT_STS => state.it_sts,
                dyn_reg::MB_CTRL => state.mb_ctrl,
                dyn_reg::MB_LEN => state.mb_len,
                // `2001h` is ST reserved and read-only (Table 12).
                _ => 0x00,
            },
            Target::Mailbox(off) => {
                // §5.1.2: "data out is set to FFh when the counter reaches the
                // message end", and the mailbox is unreadable with fast
                // transfer mode off.
                if state.ftm() && off < state.mb_message_len() {
                    state.mailbox[off]
                } else {
                    0xff
                }
            }
            Target::Nowhere => 0xff,
        }
    }

    /// Apply the side effects of the byte at `addr` having gone out.
    fn read_taken(&self, state: &mut State, addr: u16) {
        match self.target(state, addr) {
            Target::Dynamic(dyn_reg::IT_STS) => {
                // "Once read the ITSTS_Dyn register is cleared (set to 00h)."
                state.it_sts = 0;
            }
            // The last byte of the message went out. §5.1.2 frees the mailbox
            // at the *following STOP*, not here.
            Target::Mailbox(off) if state.ftm() && off + 1 == state.mb_message_len() => {
                state.mb_read_end = true;
            }
            _ => {}
        }
        // §6.5: "the device's internal address counter is incremented by one".
        // No roll-over anywhere, and a read past the end simply keeps
        // answering FFh, so a saturating step is the whole rule.
        state.addr = state.addr.saturating_add(1);
    }

    /// Take one data byte of a write command, answering §6.4.1's inhibitions.
    fn write_byte(&self, state: &mut State, byte: u8) -> Ack {
        if state.written >= MAX_SEQUENTIAL_WRITE {
            // "256 write occurrence have already been reached in the same
            // sequential write" (§6.4.2).
            return Ack::Nack;
        }
        let addr = state.addr;
        let ack = match self.target(state, addr) {
            Target::User(at) => self.write_user(state, at, byte),
            Target::System(reg) => self.write_system(state, reg, byte),
            // The password area is only reachable through the present- and
            // write-password commands (§6.6), which `write` routes to
            // `Phase::Password` before any data byte gets here.
            Target::Password(_) | Target::Nowhere => Ack::Nack,
            Target::Dynamic(reg) => self.write_dynamic(state, reg, byte),
            Target::Mailbox(off) => self.write_mailbox(state, off, byte),
        };
        if ack.is_ack() {
            state.addr = state.addr.saturating_add(1);
            state.written += 1;
        }
        ack
    }

    /// A user-memory byte.
    fn write_user(&self, state: &mut State, at: u64, byte: u8) -> Ack {
        // §6.4: "fast transfer mode must be deactivated before starting any
        // write operation in user or system memory" — the write path *is* the
        // mailbox buffer.
        if state.ftm() {
            return Ack::Nack;
        }
        #[allow(clippy::cast_possible_truncation)]
        let area = state.area_of(at);
        if !state.i2c_can_write(area) {
            return Ack::Nack;
        }
        match state.area {
            // §6.4.2: "area border crossing is forbidden".
            Some(first) if first != area => return Ack::Nack,
            Some(_) => {}
            None => state.area = Some(area),
        }
        #[allow(clippy::cast_possible_truncation)]
        state.staged.push(Staged {
            at: at as u32,
            byte,
        });
        Ack::Ack
    }

    /// A system configuration byte.
    fn write_system(&self, state: &mut State, reg: u16, byte: u8) -> Ack {
        if state.ftm() {
            return Ack::Nack;
        }
        // "Byte is in system memory and I2C security session is closed", and
        // everything from LOCK_DSFID up is read-only over I²C (Table 11).
        if !state.sso || reg > sys::LAST_WRITABLE {
            return Ack::Nack;
        }
        // §4.2: "If this rule is not respected … NoAck is returned in I2C, and
        // programming is not done."
        if (reg == sys::ENDA1 || reg == sys::ENDA2 || reg == sys::ENDA3)
            && !self.enda_ok(state, reg, byte)
        {
            return Ack::Nack;
        }
        state.staged.push(Staged {
            at: SYS_STAGE + u32::from(reg),
            byte,
        });
        Ack::Ack
    }

    /// §4.2's `ENDAi-1 < ENDAi ≤ ENDAi+1 = end of memory` rule.
    fn enda_ok(&self, state: &State, reg: u16, byte: u8) -> bool {
        let top = self.density.enda_max();
        let (e1, e2, e3) = (
            state.sys[sys::ENDA1 as usize],
            state.sys[sys::ENDA2 as usize],
            state.sys[sys::ENDA3 as usize],
        );
        match reg {
            sys::ENDA1 => byte <= e2 && e2 == top && e3 == top,
            sys::ENDA2 => byte > e1 && byte <= e3 && e3 == top,
            _ => byte > e2 && byte <= top,
        }
    }

    /// A dynamic register. Programming is immediate: there is no EEPROM here.
    fn write_dynamic(&self, state: &mut State, reg: u16, byte: u8) -> Ack {
        match reg {
            dyn_reg::GPO_CTRL => {
                // Table 29: bits 0-6 are read-only over I²C, bit 7 is not, and
                // no password is needed for it.
                state.gpo_dyn = (state.gpo_dyn & !gpo_bit::GPO_EN) | (byte & gpo_bit::GPO_EN);
                Ack::Ack
            }
            dyn_reg::EH_CTRL => {
                // Table 36: bit 0 read/write, bits 1-7 read-only.
                state.eh_en = byte & eh_bit::EH_EN != 0;
                Ack::Ack
            }
            dyn_reg::RF_MNGT => {
                state.rf_mngt_dyn = byte & (rf_bit::RF_DISABLE | rf_bit::RF_SLEEP);
                Ack::Ack
            }
            dyn_reg::MB_CTRL => {
                // Table 18: bit 0 read/write, bits 1-7 read-only. Table 15
                // gates it: "Enabling fast transfer mode is forbidden" unless
                // MB_MODE says otherwise.
                let want = byte & mb_bit::MB_EN != 0;
                if want && state.sys[sys::MB_MODE as usize] & 1 == 0 {
                    return Ack::Nack;
                }
                if want {
                    state.mb_ctrl |= mb_bit::MB_EN;
                } else {
                    // §5.1.2's state diagram: MB_EN = 0 empties the mailbox.
                    state.mb_ctrl = 0;
                    state.mb_len = 0;
                    state.wdg_armed = false;
                }
                Ack::Ack
            }
            // `2001h`, `I2C_SSO_Dyn`, `IT_STS_Dyn` and `MB_LEN_Dyn` are
            // read-only: "Byte is in dynamic registers area and is a Read Only
            // register" (§6.4.1).
            _ => Ack::Nack,
        }
    }

    /// A mailbox byte (§5.1.2, "I2C access to mailbox").
    fn write_mailbox(&self, state: &mut State, off: usize, byte: u8) -> Ack {
        if !state.ftm() {
            return Ack::Nack;
        }
        // "A I2C write operation must start from the first mailbox location",
        // and a message may not be overwritten while one is pending.
        if state.mb_written == 0 && off != 0 {
            return Ack::Nack;
        }
        if state.mb_busy() {
            return Ack::Nack;
        }
        state.mailbox[off] = byte;
        state.mb_written += 1;
        Ack::Ack
    }

    /// Commit a finished write command at the STOP (§6.4).
    fn commit(&self, state: &mut State) {
        if state.refused {
            // "If some bytes have been NotAck'ed, no internal programming is
            // done (0 byte written)."
            return;
        }
        if state.mb_written > 0 {
            // §5.1.2: the length lands in MB_LEN_Dyn, HOST_PUT_MSG goes up,
            // and the mailbox is shut to writers until somebody reads it.
            #[allow(clippy::cast_possible_truncation)]
            {
                state.mb_len = (state.mb_written - 1) as u8;
            }
            state.mb_ctrl |= mb_bit::HOST_PUT_MSG | mb_bit::HOST_CURRENT_MSG;
            state.mb_ctrl &= !mb_bit::RF_CURRENT_MSG;
            self.arm_watchdog(state);
        }
        if state.staged.is_empty() {
            return;
        }
        let mut pages: u64 = 0;
        let mut last_page: Option<u64> = None;
        let staged = core::mem::take(&mut state.staged);
        for Staged { at, byte } in staged {
            let page = u64::from(at) / EEPROM_PAGE;
            if last_page != Some(page) {
                pages += 1;
                last_page = Some(page);
            }
            if at >= SYS_STAGE {
                let reg = (at - SYS_STAGE) as usize;
                state.sys[reg] = byte;
                self.sys_written(state, reg as u16);
            } else if let Some(slot) = state.user.get_mut(at as usize) {
                *slot = byte;
            }
        }
        // §6.4.2: "total programming time is tW multiplied by the number of
        // internal EEPROM pages where the data must be programmed".
        state.busy_until = state
            .ticks
            .saturating_add(self.write_ticks.saturating_mul(pages.max(1)));
        state.busy = true;
    }

    /// A static register was written, so its dynamic image follows (§4.4).
    fn sys_written(&self, state: &mut State, reg: u16) {
        match reg {
            // "At power up, and each time GPO register is updated,
            // GPO_CTRL_Dyn content is copied from GPO register."
            sys::GPO => state.gpo_dyn = state.sys[sys::GPO as usize],
            // "each time RF_MNGT register it is updated, content of
            // RF_MNGT_Dyn register is copied from RF_MNGT register."
            sys::RF_MNGT => {
                state.rf_mngt_dyn =
                    state.sys[sys::RF_MNGT as usize] & (rf_bit::RF_DISABLE | rf_bit::RF_SLEEP);
            }
            // Table 38: "Writing 0 in EH_MODE at any time after boot will
            // automatically set EH_EN bit to 1"; writing 1 changes nothing.
            sys::EH_MODE => {
                if state.sys[sys::EH_MODE as usize] & 1 == 0 {
                    state.eh_en = true;
                }
            }
            // Table 19, footnote 1: "MB_EN bit is automatically reset to 0 if
            // MB_MODE register is reset to 0."
            sys::MB_MODE if state.sys[sys::MB_MODE as usize] & 1 == 0 => {
                state.mb_ctrl = 0;
                state.mb_len = 0;
                state.wdg_armed = false;
            }
            _ => {}
        }
    }

    /// Evaluate a finished password command (§6.6).
    ///
    /// Seventeen bytes: eight of password, a validation code, and the same
    /// eight again. `09h` presents, `07h` writes.
    fn password_command(&self, state: &mut State) {
        if state.pwd_buf.len() != 17 {
            return;
        }
        let first = &state.pwd_buf[0..8];
        let code = state.pwd_buf[8];
        let second = &state.pwd_buf[9..17];
        if first != second {
            // "If the two 64-bit passwords sent are not exactly the same, the
            // ST25DVxxx does not start the internal comparison."
            return;
        }
        let mut given = [0u8; 8];
        given.copy_from_slice(first);
        match code {
            0x09 => {
                // "If the values match, the I2C security session is open …
                // If the values do not match, the I2C security session is
                // closed."
                state.sso = given == state.pwd;
            }
            0x07 if state.sso => {
                state.pwd = given;
                // A write cycle, over the two password pages.
                state.busy_until = state
                    .ticks
                    .saturating_add(self.write_ticks.saturating_mul(2));
                state.busy = true;
            }
            _ => {}
        }
    }
}

impl I2cSlave for Shared {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        let mut state = self.state.lock();
        let Address::Seven(a) = address else {
            // §6.3 knows only the eight-bit device select byte.
            return Ack::Nack;
        };
        let e2 = match a {
            USER_ADDRESS => false,
            SYSTEM_ADDRESS => true,
            _ => {
                state.phase = Phase::Idle;
                return Ack::Nack;
            }
        };
        if self.is_busy(&state) {
            // §6.4.3: during the internal write cycle "the device disconnects
            // itself from the bus". This one line is acknowledge polling.
            state.phase = Phase::Idle;
            return Ack::Nack;
        }
        // A repeated START keeps the address counter, which is what makes a
        // random read work (§6.5.1); only the half may change.
        state.e2 = e2;
        state.phase = match dir {
            Direction::Write => Phase::AddrHi,
            Direction::Read => Phase::Reading,
        };
        Ack::Ack
    }

    fn write(&self, byte: u8) -> Ack {
        let mut state = self.state.lock();
        match state.phase {
            Phase::AddrHi => {
                state.addr = u16::from(byte) << 8;
                state.phase = Phase::AddrLo;
                Ack::Ack
            }
            Phase::AddrLo => {
                state.addr |= u16::from(byte);
                state.written = 0;
                state.refused = false;
                state.staged.clear();
                state.area = None;
                state.mb_written = 0;
                state.pwd_buf.clear();
                state.phase = if state.e2 && state.addr == I2C_PWD_BASE {
                    Phase::Password
                } else {
                    Phase::Writing
                };
                Ack::Ack
            }
            Phase::Writing => {
                let ack = self.write_byte(&mut state, byte);
                if !ack.is_ack() {
                    state.refused = true;
                    state.phase = Phase::Dead;
                }
                ack
            }
            Phase::Password => {
                if state.pwd_buf.len() >= 17 {
                    state.phase = Phase::Dead;
                    return Ack::Nack;
                }
                state.pwd_buf.push(byte);
                Ack::Ack
            }
            Phase::Idle | Phase::Reading | Phase::Dead => Ack::Nack,
        }
    }

    fn read(&self) -> u8 {
        let state = self.state.lock();
        if state.phase != Phase::Reading {
            return 0xff;
        }
        self.read_byte(&state, state.addr)
    }

    fn read_ack(&self, ack: Ack) {
        let mut state = self.state.lock();
        if state.phase != Phase::Reading {
            return;
        }
        let addr = state.addr;
        self.read_taken(&mut state, addr);
        if !ack.is_ack() {
            state.phase = Phase::Idle;
        }
    }

    fn stop(&self) {
        let drive = {
            let mut state = self.state.lock();
            match state.phase {
                Phase::Writing | Phase::Dead => self.commit(&mut state),
                Phase::Password => self.password_command(&mut state),
                _ => {}
            }
            if state.mb_read_end {
                // §5.1.2: "RF_PUT_MSG is cleared after reaching the STOP
                // consecutive to reading the last message byte". The data is
                // not cleared. Checked outside the match on purpose: a master
                // that NACKs the last byte — which §6.5.4 says it must — has
                // already sent this device back to `Idle` by the time the STOP
                // arrives, so keying this on `Phase::Reading` would fire only
                // for the one master that acknowledges its own last byte.
                state.mb_ctrl &= !mb_bit::RF_PUT_MSG;
                state.wdg_armed = false;
                state.mb_read_end = false;
            }
            state.phase = Phase::Idle;
            self.publish(&state);
            self.gpo_level(&state)
        };
        self.refresh_gpo(drive);
    }

    fn peek(&self) -> u8 {
        let state = self.state.lock();
        if state.phase != Phase::Reading {
            return 0xff;
        }
        // The whole reason `read_byte` has no side effects: a debug read must
        // not clear `IT_STS_Dyn` or free the mailbox (`CLAUDE.md`, devices).
        self.read_byte(&state, state.addr)
    }
}

// ---------------------------------------------------------------------------
// The low-power-down pin
// ---------------------------------------------------------------------------

/// The `LPD` input.
struct LpdSink {
    shared: Arc<Shared>,
}

impl fmt::Debug for LpdSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LpdSink").finish_non_exhaustive()
    }
}

impl WireSink for LpdSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        let drive = {
            let mut state = self.shared.state.lock();
            state.lpd = level.is_high();
            self.shared.gpo_level(&state)
        };
        self.shared.refresh_gpo(drive);
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An ST25DV04K/16K/64K dynamic NFC tag.
#[derive(Debug)]
pub struct St25dv {
    shared: Arc<Shared>,
    wires: Arc<SlaveWires>,
    /// The bus to hook onto at realize time, if the machine named one.
    bus: Option<Arc<I2cBus>>,
    /// The reader door this tag answers, opened in `new` and bound in
    /// `realize` — opening a host object is allocation, binding is outward.
    reader: Arc<rf::Reader>,
    /// The `LPD` pin, kept alive here because a net refers to its sinks weakly.
    lpd_pin: Mutex<Option<Arc<LpdSink>>>,
}

impl St25dv {
    /// Validate `props` and build the part.
    ///
    /// Properties:
    ///
    /// * `density` — `"4K"`, `"16K"` or `"64K"`. Defaults to `"64K"`.
    /// * `gpo` — `"open-drain"` for an `-IE` part or `"cmos"` for a `-JF` one
    ///   (§5.2.1). Defaults to open drain.
    /// * `uid` — the four low serial bytes of the unique identifier, as a
    ///   number. The top four bytes are fixed by ST (Table 85): `E0h`, `02h`,
    ///   the product code, and the serial's high byte.
    /// * `write-ticks` — tW per four-byte EEPROM page, in ticks of this
    ///   device's clock domain (§6.4.2). Defaults to
    ///   [`DEFAULT_WRITE_TICKS`].
    /// * `it-ticks` — the `IT_TIME = 0` GPO pulse, in ticks (Eq. (1)).
    ///   Defaults to [`DEFAULT_IT_TICKS`].
    /// * `wdg-ticks` — the mailbox watchdog's 30 ms unit, in ticks
    ///   (Table 17). Defaults to [`DEFAULT_WDG_TICKS`].
    /// * `password` — the factory I²C password, as a 64-bit number. Defaults
    ///   to zero, which is how ST delivers the part.
    /// * `image` — a media slot holding the initial user memory.
    /// * `bus` — the named [`I2cBus`] to hang off.
    /// * `reader` — the name of the RF reader door in [`rf`]. Defaults to
    ///   [`rf::DEFAULT_READER`].
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property; [`Error::Config`] for an
    /// unknown `density` or `gpo` spelling, an `image` longer than the array,
    /// or a host object of another kind already holding the reader's name.
    pub fn new(props: &Props) -> Result<St25dv> {
        let mut r = props.reader();
        let density = r.or_str("density", "64K")?.to_string();
        let gpo = r.or_str("gpo", GpoStyle::OpenDrain.name())?.to_string();
        let serial: u64 = r.or("uid", 0)?;
        let write_ticks: u64 = r.or("write-ticks", DEFAULT_WRITE_TICKS)?;
        let it_ticks: u64 = r.or("it-ticks", DEFAULT_IT_TICKS)?;
        let wdg_ticks: u64 = r.or("wdg-ticks", DEFAULT_WDG_TICKS)?;
        let password: u64 = r.or("password", 0)?;
        let image = r
            .optional_media("image")?
            .map(crate::core::props::Media::to_bytes);
        let bus_name = r.optional_str("bus")?.map(String::from);
        let reader_name = r.or_str("reader", rf::DEFAULT_READER)?.to_string();
        r.finish()?;

        let bad = |message: String| Error::Config {
            at: String::from(CLASS_NAME),
            message,
        };
        let density = Density::from_name(&density).ok_or_else(|| {
            bad(alloc::format!(
                "`density` is `{density}`; this family has {:?}",
                Density::NAMES
            ))
        })?;
        let gpo_style = GpoStyle::from_name(&gpo).ok_or_else(|| {
            bad(alloc::format!(
                "`gpo` is `{gpo}`; the -IE parts are `open-drain` and the -JF parts are `cmos` \
                 (datasheet §5.2.1)"
            ))
        })?;

        let size = density.bytes();
        let mut user = alloc::vec![0x00_u8; size as usize];
        if let Some(image) = image {
            if image.len() as u64 > size {
                return Err(bad(alloc::format!(
                    "`image` is {} bytes and an ST25DV{density} holds {size}",
                    image.len()
                )));
            }
            user[..image.len()].copy_from_slice(&image);
        }

        // Table 85: byte 7 is E0h, byte 6 the IC manufacturer code 02h, byte 5
        // the ST product code — which is IC_REF — and bytes 4..0 the serial.
        let mut uid = [0u8; 8];
        uid[7] = 0xe0;
        uid[6] = 0x02;
        uid[5] = density.ic_ref();
        for (i, slot) in uid[..5].iter_mut().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            {
                *slot = (serial >> (8 * i)) as u8;
            }
        }

        let mut pwd = [0u8; 8];
        for (i, slot) in pwd.iter_mut().enumerate() {
            // MSB first, as §6.6.1 sends it.
            #[allow(clippy::cast_possible_truncation)]
            {
                *slot = (password >> (8 * (7 - i))) as u8;
            }
        }

        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::fresh(density, user, pwd)),
            density,
            gpo_style,
            write_ticks,
            it_ticks,
            wdg_ticks,
            uid,
            ic_rev: 0x01,
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
            gpo: Mutex::with_rank(LockRank::WIRE, None),
        });
        // Both of these are get-or-create in the build's own host-object table
        // and nothing outside this machine can see either, which is why
        // `core::hosts` puts them in `new`. The *outward* halves — joining the
        // bus, letting the door reach this tag — are in `realize`.
        let bus = bus_name
            .as_deref()
            .map(|name| buses::attach(props, name))
            .transpose()?;
        let reader = rf::attach(props, &reader_name)?;
        let wires = Arc::new(SlaveWires::new(Arc::clone(&shared) as Arc<dyn I2cSlave>));
        Ok(St25dv {
            shared,
            wires,
            bus,
            reader,
            lpd_pin: Mutex::with_rank(LockRank::WIRE, None),
        })
    }

    /// Which member of the family this is.
    #[must_use]
    pub fn density(&self) -> Density {
        self.shared.density
    }

    /// This part as a bus device, for a controller that hands it whole bytes.
    #[must_use]
    pub fn slave(&self) -> Arc<dyn I2cSlave> {
        Arc::clone(&self.shared) as Arc<dyn I2cSlave>
    }

    /// The part's wire pins, for a controller that drives them directly.
    #[must_use]
    pub fn wires(&self) -> &Arc<SlaveWires> {
        &self.wires
    }

    /// The RF reader door this tag answers.
    #[must_use]
    pub fn reader(&self) -> &Arc<rf::Reader> {
        &self.reader
    }

    /// One byte of user memory, without touching the protocol state.
    #[must_use]
    pub fn byte(&self, at: u64) -> Option<u8> {
        let state = self.shared.state.lock();
        state.user.get(usize::try_from(at).ok()?).copied()
    }

    /// The whole user array, copied.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        self.shared.state.lock().user.clone()
    }

    /// One system configuration or dynamic register, as I²C would read it.
    ///
    /// The debug view: this moves no counter and clears no status.
    #[must_use]
    pub fn register(&self, system: bool, addr: u16) -> u8 {
        let mut state = self.shared.state.lock();
        let was = state.e2;
        state.e2 = system;
        let byte = self.shared.read_byte(&state, addr);
        state.e2 = was;
        byte
    }

    /// Whether the internal write cycle is running (§6.4.3).
    #[must_use]
    pub fn busy(&self) -> bool {
        let state = self.shared.state.lock();
        self.shared.is_busy(&state)
    }

    /// Whether the I²C security session is open (§6.6.1).
    #[must_use]
    pub fn session_open(&self) -> bool {
        self.shared.state.lock().sso
    }

    /// Whether an RF field is present, as the reader door last said.
    #[must_use]
    pub fn field(&self) -> bool {
        self.shared.state.lock().field
    }

    /// What the `GPO` pin is doing.
    #[must_use]
    pub fn gpo(&self) -> Drive {
        let state = self.shared.state.lock();
        self.shared.gpo_level(&state)
    }

    /// Domain ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Run the part until `target` domain ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

impl State {
    /// The factory state (§4.3's "Factory Value" columns).
    fn fresh(density: Density, user: Vec<u8>, pwd: [u8; 8]) -> State {
        let mut sys = [0u8; 16];
        // Table 26: FIELD_CHANGE_EN and GPO_EN are the two bits ST ships set.
        sys[sys::GPO as usize] = gpo_bit::FIELD_CHANGE_EN | gpo_bit::GPO_EN;
        // Table 28: IT_TIME = 011b, which Eq. (1) makes 188 µs.
        sys[sys::IT_TIME as usize] = 0b011;
        // Table 35: EH_MODE = 1, "EH on demand only".
        sys[sys::EH_MODE as usize] = 1;
        sys[sys::ENDA1 as usize] = density.enda_max();
        sys[sys::ENDA2 as usize] = density.enda_max();
        sys[sys::ENDA3 as usize] = density.enda_max();
        // Table 17: MB_WDG = 111b, 2^6 × 30 ms.
        sys[sys::MB_WDG as usize] = 0b111;
        State {
            ticks: 0,
            user,
            sys,
            ident: [0; 4],
            pwd,
            rf_pwd: [[0; 8]; 4],
            // §4.4: "At power up … GPO_CTRL_Dyn content is copied from GPO".
            gpo_dyn: sys[sys::GPO as usize],
            // Table 38: EH_MODE = 1 boots with EH_EN clear.
            eh_en: false,
            rf_mngt_dyn: 0,
            sso: false,
            it_sts: 0,
            mb_ctrl: 0,
            mb_len: 0,
            mailbox: alloc::vec![0; MAILBOX_SIZE],
            rf_cfg_session: false,
            rf_user_session: 0,
            busy_until: 0,
            busy: false,
            wdg_until: 0,
            wdg_armed: false,
            pulse_until: 0,
            pulsing: false,
            rf_user: false,
            phase: Phase::Idle,
            e2: false,
            addr: 0,
            written: 0,
            refused: false,
            staged: Vec::new(),
            area: None,
            mb_written: 0,
            mb_read_end: false,
            pwd_buf: Vec::new(),
            field: false,
            lpd: false,
        }
    }
}

impl Device for St25dv {
    fn class(&self) -> &'static DeviceClass {
        &ST25DV_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The two outward actions: joining the bus, and letting the reader door
        // reach this tag. Two-phase construction (`CLAUDE.md`) puts both here.
        if let Some(bus) = &self.bus {
            bus.attach(self.slave())?;
        }
        self.reader.bind(&self.shared);
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let drive = {
            let mut state = self.shared.state.lock();
            // A power-on reset restores every dynamic register from its static
            // image (§4.4) and clears the volatile ones. The EEPROM survives:
            // it is an EEPROM, and a board reset is not an erase. The tick is
            // not zeroed — `Machine::reset` does not rewind clock domains
            // (`ROADMAP.md` §4.2) — and neither `field` nor `lpd` is touched,
            // because both belong to something outside this device.
            state.gpo_dyn = state.sys[sys::GPO as usize];
            state.rf_mngt_dyn =
                state.sys[sys::RF_MNGT as usize] & (rf_bit::RF_DISABLE | rf_bit::RF_SLEEP);
            state.eh_en = state.sys[sys::EH_MODE as usize] & 1 == 0;
            state.sso = false;
            state.it_sts = 0;
            state.mb_ctrl = 0;
            state.mb_len = 0;
            state.rf_cfg_session = false;
            state.rf_user_session = 0;
            state.busy = false;
            state.busy_until = 0;
            state.wdg_armed = false;
            state.pulsing = false;
            state.rf_user = false;
            state.phase = Phase::Idle;
            state.addr = 0;
            state.written = 0;
            state.refused = false;
            state.staged.clear();
            state.area = None;
            state.mb_written = 0;
            state.mb_read_end = false;
            state.pwd_buf.clear();
            self.shared.publish(&state);
            self.shared.gpo_level(&state)
        };
        self.shared.refresh_gpo(drive);
        self.wires.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_bytes(&state.user)?;
        w.write_bytes(&state.sys)?;
        w.write_bytes(&state.ident)?;
        w.write_bytes(&state.pwd)?;
        for pwd in &state.rf_pwd {
            w.write_bytes(pwd)?;
        }
        w.write_u8(state.gpo_dyn)?;
        w.write_bool(state.eh_en)?;
        w.write_u8(state.rf_mngt_dyn)?;
        w.write_bool(state.sso)?;
        w.write_u8(state.it_sts)?;
        w.write_u8(state.mb_ctrl)?;
        w.write_u8(state.mb_len)?;
        w.write_bytes(&state.mailbox)?;
        w.write_bool(state.rf_cfg_session)?;
        w.write_u8(state.rf_user_session)?;
        w.write_u64(state.busy_until)?;
        w.write_bool(state.busy)?;
        w.write_u64(state.wdg_until)?;
        w.write_bool(state.wdg_armed)?;
        w.write_u64(state.pulse_until)?;
        w.write_bool(state.pulsing)?;
        w.write_bool(state.rf_user)?;
        w.write_u8(phase_code(state.phase))?;
        w.write_bool(state.e2)?;
        w.write_u16(state.addr)?;
        drop(state);
        // `field` and `lpd` are deliberately absent: both are levels something
        // else drives, and that something restores its own state and drives
        // them again (`ROADMAP.md` §4.5). So is the half-finished transaction
        // bookkeeping — `staged`, `pwd_buf` and the rest — because the bit
        // engine below carries where in the transfer the part is, and a
        // snapshot taken mid-command resumes there.
        self.wires.snapshot().write(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ticks = r.read_u64()?;
        let user = r.read_bytes()?.to_vec();
        let sys = r.read_bytes()?.to_vec();
        let ident = r.read_bytes()?.to_vec();
        let pwd = r.read_bytes()?.to_vec();
        let mut rf_pwd = [[0u8; 8]; 4];
        for slot in &mut rf_pwd {
            let bytes = r.read_bytes()?;
            if bytes.len() == 8 {
                slot.copy_from_slice(bytes);
            }
        }
        let gpo_dyn = r.read_u8()?;
        let eh_en = r.read_bool()?;
        let rf_mngt_dyn = r.read_u8()?;
        let sso = r.read_bool()?;
        let it_sts = r.read_u8()?;
        let mb_ctrl = r.read_u8()?;
        let mb_len = r.read_u8()?;
        let mailbox = r.read_bytes()?.to_vec();
        let rf_cfg_session = r.read_bool()?;
        let rf_user_session = r.read_u8()?;
        let busy_until = r.read_u64()?;
        let busy = r.read_bool()?;
        let wdg_until = r.read_u64()?;
        let wdg_armed = r.read_bool()?;
        let pulse_until = r.read_u64()?;
        let pulsing = r.read_bool()?;
        let rf_user = r.read_bool()?;
        let phase = phase_from_code(r.read_u8()?);
        let e2 = r.read_bool()?;
        let addr = r.read_u16()?;
        let bits = SlaveWiresState::read(r)?;

        let drive = {
            let mut state = self.shared.state.lock();
            // A snapshot is untrusted input: a length that does not match this
            // part's geometry is dropped rather than indexed with.
            if user.len() == state.user.len() {
                state.user = user;
            }
            if sys.len() == state.sys.len() {
                state.sys.copy_from_slice(&sys);
            }
            if ident.len() == state.ident.len() {
                state.ident.copy_from_slice(&ident);
            }
            if pwd.len() == state.pwd.len() {
                state.pwd.copy_from_slice(&pwd);
            }
            if mailbox.len() == state.mailbox.len() {
                state.mailbox = mailbox;
            }
            state.ticks = ticks;
            state.rf_pwd = rf_pwd;
            state.gpo_dyn = gpo_dyn;
            state.eh_en = eh_en;
            state.rf_mngt_dyn = rf_mngt_dyn;
            state.sso = sso;
            state.it_sts = it_sts;
            state.mb_ctrl = mb_ctrl;
            state.mb_len = mb_len;
            state.rf_cfg_session = rf_cfg_session;
            state.rf_user_session = rf_user_session;
            state.busy_until = busy_until;
            state.busy = busy;
            state.wdg_until = wdg_until;
            state.wdg_armed = wdg_armed;
            state.pulse_until = pulse_until;
            state.pulsing = pulsing;
            state.rf_user = rf_user;
            state.phase = phase;
            state.e2 = e2;
            state.addr = addr;
            state.written = 0;
            state.refused = false;
            state.staged.clear();
            state.area = None;
            state.mb_written = 0;
            state.mb_read_end = false;
            state.pwd_buf.clear();
            self.shared.publish(&state);
            self.shared.gpo_level(&state)
        };
        self.shared.refresh_gpo(drive);
        self.wires.restore(bits);
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        match port {
            line::SCL_NAME => Some(SinkPin {
                sink: self.wires.sink(line::SCL, sources),
                line: line::SCL,
            }),
            line::SDA_NAME => Some(SinkPin {
                sink: self.wires.sink(line::SDA, sources),
                line: line::SDA,
            }),
            pin::LPD => {
                let sink = Arc::new(LpdSink {
                    shared: Arc::clone(&self.shared),
                });
                // Kept, because a net refers to its sinks weakly.
                *self.lpd_pin.lock() = Some(Arc::clone(&sink));
                Some(SinkPin {
                    sink: sink as Arc<dyn WireSink>,
                    line: pin::LPD_LINE,
                })
            }
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            line::SCL_NAME => self.wires.connect(line::SCL, source),
            line::SDA_NAME => self.wires.connect(line::SDA, source),
            pin::GPO => *self.shared.gpo.lock() = Some(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "an ST25DV drives `{}` and `{}` — both open-drain — and `{}`. `{}` is an \
                         input.",
                        line::SCL_NAME,
                        line::SDA_NAME,
                        pin::GPO,
                        pin::LPD
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == pin::GPO {
            // A machine that wires GPO after a snapshot load has to be told
            // about a pulse that survived it.
            let drive = {
                let state = self.shared.state.lock();
                self.shared.gpo_level(&state)
            };
            self.shared.refresh_gpo(drive);
            return;
        }
        self.wires.announce();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes, and for three reasons: the internal write cycle of §6.4.3, the
    /// `IT_TIME` pulse of Eq. (1), and the mailbox watchdog of Table 17. The
    /// first is sampled — a master polls for its end — and the other two are
    /// scheduled, which is what [`Device::next_event_tick`] is for.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        St25dv::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for St25dv {}

/// A stable code for a phase, for the snapshot.
const fn phase_code(phase: Phase) -> u8 {
    match phase {
        Phase::Idle => 0,
        Phase::AddrHi => 1,
        Phase::AddrLo => 2,
        Phase::Writing => 3,
        Phase::Password => 4,
        Phase::Reading => 5,
        Phase::Dead => 6,
    }
}

/// The inverse. An unknown code loads as idle: a snapshot is untrusted input.
const fn phase_from_code(code: u8) -> Phase {
    match code {
        1 => Phase::AddrHi,
        2 => Phase::AddrLo,
        3 => Phase::Writing,
        4 => Phase::Password,
        5 => Phase::Reading,
        6 => Phase::Dead,
        _ => Phase::Idle,
    }
}

/// The `st.st25dv` device class.
pub static ST25DV_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "ST ST25DV04K/16K/64K dynamic NFC tag: dual-interface EEPROM, the fast transfer \
              mode mailbox, GPO with IT_TIME pulses, field detect and an RF reader door",
    properties: &[
        PropertySpec {
            name: "density",
            kind: ValueKind::Str,
            required: false,
            summary: "which part: \"4K\", \"16K\" or \"64K\" (default 64K)",
        },
        PropertySpec {
            name: "gpo",
            kind: ValueKind::Str,
            required: false,
            summary: "the GPO output stage: \"open-drain\" (-IE) or \"cmos\" (-JF), §5.2.1",
        },
        PropertySpec {
            name: "uid",
            kind: ValueKind::Uint,
            required: false,
            summary: "the five low serial bytes of the UID; ST fixes E0h 02h and the product code",
        },
        PropertySpec {
            name: "write-ticks",
            kind: ValueKind::Uint,
            required: false,
            summary: "tW per four-byte EEPROM page, in domain ticks (§6.4.2; default 5000)",
        },
        PropertySpec {
            name: "it-ticks",
            kind: ValueKind::Uint,
            required: false,
            summary: "the IT_TIME = 0 GPO pulse, in domain ticks (Eq. (1); default 301)",
        },
        PropertySpec {
            name: "wdg-ticks",
            kind: ValueKind::Uint,
            required: false,
            summary: "the mailbox watchdog's 30 ms unit, in domain ticks (Table 17; default 30000)",
        },
        PropertySpec {
            name: "password",
            kind: ValueKind::Uint,
            required: false,
            summary: "the factory I2C password as a 64-bit number (default 0, as delivered)",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "initial user memory; absent means all zero",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named I2C bus to hang off, for a transactional link",
        },
        PropertySpec {
            name: "reader",
            kind: ValueKind::Str,
            required: false,
            summary: "the name of the RF reader door this tag answers (default \"reader\")",
        },
    ],
    construct: |props| Ok(Box::new(St25dv::new(props)?)),
};

/// Add [`ST25DV_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&ST25DV_CLASS)
}

/// Bind [`ST25DV_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(St25dv::new(props)?)))
}

/// What the validator should know about `st.st25dv`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("density", ValueKind::Str).values(Density::NAMES))
        .prop(PropSchema::new("gpo", ValueKind::Str).values(GpoStyle::NAMES))
        .prop(PropSchema::new("uid", ValueKind::Uint))
        .prop(PropSchema::new("write-ticks", ValueKind::Uint))
        .prop(PropSchema::new("it-ticks", ValueKind::Uint))
        .prop(PropSchema::new("wdg-ticks", ValueKind::Uint))
        .prop(PropSchema::new("password", ValueKind::Uint))
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("reader", ValueKind::Str))
        // Both bus lines are open drain, so each is an input *and* an output.
        .port(line::SCL_NAME, PortDir::InOut)
        .port(line::SDA_NAME, PortDir::InOut)
        .port(pin::GPO, PortDir::Out)
        .port(pin::LPD, PortDir::In)
}
