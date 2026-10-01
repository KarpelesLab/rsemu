//! The navi's DC-DC/ADC monitor, as the SoC's HSPI channel 0 sees it.
//!
//! A part on the computer board answers the kernel's `DCDCAD` driver over
//! SPI: a version byte it checks before trusting anything else, and eight
//! 12-bit analogue readings it polls continuously. Nothing of the part itself
//! is modelled — not what it converts, nor how — only the conversation.
//!
//! # The protocol
//!
//! Full duplex, one frame per slave-select assertion. A command byte's answer
//! comes out on the bytes clocked after it:
//!
//! | Command | Answer |
//! | --- | --- |
//! | `0x17` | the version, one byte (the driver accepts 2 and 3) |
//! | `0x71`–`0x78` | channel 1–8's reading, two bytes, the 12 bits left-justified |
//! | `0xfb` | none; the next byte is a setting, taken and ignored |
//! | `0x7b` | two bytes, which the driver ignores |
//!
//! The driver polls all eight channels in one 18-byte frame — the eight
//! commands each followed by a byte of padding, then `0x17` — and checks the
//! version that ends it, so a frame that slipped is noticed.
//!
//! # Sources
//!
//! The navi's own kernel image, read as data: the order and content of the
//! transfers its `dcdcad_ltc_task` makes, and the checks it applies to what
//! comes back. What the channels measure is not known from that; each reading
//! is a property, mid-scale unless set.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::spi::{ChipSelect, Format, SpiBus, SpiSlave, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "navi.dcdcad";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

const CMD_VERSION: u8 = 0x17;
const CMD_SETTING: u8 = 0xfb;
const CMD_STATUS: u8 = 0x7b;

#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct State {
    /// Bytes owed to the master, oldest first.
    out: VecDeque<u8>,
    /// The next byte in is a setting's value, not a command.
    setting: bool,
}

struct Part {
    version: u8,
    readings: [u16; 8],
    state: Mutex<State>,
}

impl fmt::Debug for Part {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("navi.dcdcad")
            .field("version", &self.version)
            .finish_non_exhaustive()
    }
}

impl SpiSlave for Part {
    fn format(&self) -> Format {
        Format::DEFAULT
    }

    fn select(&self, _selected: bool) {
        // A frame starts clean either way.
        *self.state.lock() = State::default();
    }

    fn transfer(&self, mosi: u32) -> u32 {
        let byte = mosi as u8;
        let mut s = self.state.lock();
        let back = s.out.pop_front().unwrap_or(0);
        if s.setting {
            s.setting = false;
        } else {
            match byte {
                CMD_VERSION => s.out.push_back(self.version),
                0x71..=0x78 => {
                    let v = self.readings[usize::from(byte - 0x71)] << 4;
                    s.out.push_back((v >> 8) as u8);
                    s.out.push_back(v as u8);
                }
                CMD_SETTING => s.setting = true,
                CMD_STATUS => {
                    s.out.push_back(0);
                    s.out.push_back(0);
                }
                _ => {}
            }
        }
        u32::from(back)
    }
}

/// The monitor.
#[derive(Debug)]
pub struct Dcdcad {
    part: Arc<Part>,
    bus: Option<(Arc<SpiBus>, ChipSelect)>,
}

impl Dcdcad {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a malformed or unknown property.
    pub fn new(props: &Props) -> Result<Dcdcad> {
        let mut r = props.reader();
        let bus = r.require_str("bus")?.to_string();
        let cs = r.or_range("cs", 0u64, 0..=7)? as u8;
        let version = r.or_range("version", 2u64, 0..=0xff)? as u8;
        let mut readings = [0x800u16; 8];
        for (i, v) in readings.iter_mut().enumerate() {
            *v = r.or_range(&alloc::format!("ch{}", i + 1), 0x800u64, 0..=0xfff)? as u16;
        }
        r.finish()?;
        let bus = buses::attach(props, &bus)?;
        Ok(Dcdcad {
            part: Arc::new(Part::new(version, readings)),
            bus: Some((bus, ChipSelect(cs))),
        })
    }
}

impl Part {
    fn new(version: u8, readings: [u16; 8]) -> Part {
        Part {
            version,
            readings,
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
        }
    }
}

/// The `navi.dcdcad` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Alphard navi's DC-DC/ADC monitor: a version byte and eight 12-bit readings over SPI",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the SPI bus it sits on, by name",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "its chip select on that bus (default 0)",
        },
        PropertySpec {
            name: "version",
            kind: ValueKind::Uint,
            required: false,
            summary: "the version byte (default 2; the driver accepts 2 and 3)",
        },
        PropertySpec {
            name: "ch1",
            kind: ValueKind::Uint,
            required: false,
            summary: "channel 1's 12-bit reading (default 0x800)",
        },
        PropertySpec {
            name: "ch2",
            kind: ValueKind::Uint,
            required: false,
            summary: "channel 2's reading",
        },
        PropertySpec {
            name: "ch3",
            kind: ValueKind::Uint,
            required: false,
            summary: "channel 3's reading",
        },
        PropertySpec {
            name: "ch4",
            kind: ValueKind::Uint,
            required: false,
            summary: "channel 4's reading",
        },
        PropertySpec {
            name: "ch5",
            kind: ValueKind::Uint,
            required: false,
            summary: "channel 5's reading",
        },
        PropertySpec {
            name: "ch6",
            kind: ValueKind::Uint,
            required: false,
            summary: "channel 6's reading",
        },
        PropertySpec {
            name: "ch7",
            kind: ValueKind::Uint,
            required: false,
            summary: "channel 7's reading",
        },
        PropertySpec {
            name: "ch8",
            kind: ValueKind::Uint,
            required: false,
            summary: "channel 8's reading",
        },
    ],
    construct: |props| Ok(Box::new(Dcdcad::new(props)?)),
};

impl Device for Dcdcad {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        if let Some((bus, cs)) = &self.bus {
            bus.attach(*cs, Arc::clone(&self.part) as Arc<dyn SpiSlave>)?;
        }
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.part.state.lock() = State::default();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.part.state.lock().clone();
        let out: alloc::vec::Vec<u8> = s.out.iter().copied().collect();
        w.write_bytes(&out)?;
        w.write_bool(s.setting)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let out = r.read_bytes()?;
        if out.len() > 16 {
            return Err(Error::State(String::from(
                "navi.dcdcad owes at most a few bytes",
            )));
        }
        let s = State {
            out: out.iter().copied().collect(),
            setting: r.read_bool()?,
        };
        *self.part.state.lock() = s;
        Ok(())
    }
}

impl Instance for Dcdcad {}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Dcdcad::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    let mut s = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, 7))
        .prop(PropSchema::new("version", ValueKind::Uint).range(0, 0xff));
    for i in 1..=8 {
        s = s.prop(PropSchema::new(alloc::format!("ch{i}"), ValueKind::Uint).range(0, 0xfff));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn part() -> Part {
        Part::new(2, [0x123, 0x456, 0x789, 0xabc, 0xdef, 0x001, 0x800, 0xfff])
    }

    fn frame(p: &Part, out: &[u8]) -> alloc::vec::Vec<u8> {
        p.select(true);
        let back = out
            .iter()
            .map(|b| p.transfer(u32::from(*b)) as u8)
            .collect();
        p.select(false);
        back
    }

    #[test]
    fn the_version_comes_back_on_the_byte_after_its_command() {
        let p = part();
        assert_eq!(frame(&p, &[0x17, 0x00]), [0x00, 2]);
    }

    #[test]
    fn the_polling_frame_reads_all_eight_channels_and_ends_with_the_version() {
        let p = part();
        let mut out = alloc::vec::Vec::new();
        for ch in 0x71..=0x78u8 {
            out.push(ch);
            out.push(0);
        }
        out.extend([0x17, 0x00]);
        let back = frame(&p, &out);
        // The driver's reading: (back[2i+1] << 8 | back[2i+2]) >> 4.
        let readings: alloc::vec::Vec<u16> = (0..8)
            .map(|i| ((u16::from(back[2 * i + 1]) << 8) | u16::from(back[2 * i + 2])) >> 4)
            .collect();
        assert_eq!(
            readings,
            [0x123, 0x456, 0x789, 0xabc, 0xdef, 0x001, 0x800, 0xfff]
        );
        assert_eq!(back[17], 2, "the version closes the frame");
    }

    #[test]
    fn a_setting_byte_is_not_taken_for_a_command() {
        let p = part();
        // 0x17 as the setting's value must not queue a version.
        assert_eq!(frame(&p, &[0xfb, 0x17, 0x00]), [0, 0, 0]);
    }

    #[test]
    fn a_snapshot_round_trips() {
        let d = Dcdcad {
            part: Arc::new(part()),
            bus: None,
        };
        d.part.select(true);
        d.part.transfer(0x71);
        let save = |d: &Dcdcad| {
            let mut shape = MachineShape::new();
            shape.add_device("dcdc", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("dcdc", CLASS.name, CLASS.version).unwrap();
                d.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&d);
        let back = Dcdcad {
            part: Arc::new(part()),
            bus: None,
        };
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("dcdc", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = d.part.state.lock().clone();
        assert_eq!(*back.part.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
