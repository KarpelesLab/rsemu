//! The SNES work RAM: 128 KiB, and the B-bus port that reaches it.
//!
//! WRAM answers on the A-bus at banks `$7E–$7F` (all of it) and at
//! `$0000–$1FFF` of every system bank (the first 8 KiB, "LowRAM"). It is also
//! the only memory on the **B-bus**, through four registers:
//!
//! | B-bus | Register | |
//! | --- | --- | --- |
//! | `$80` | `WMDATA` | read or write the byte at `WMADD`, then increment it |
//! | `$81`–`$83` | `WMADDL`/`M`/`H` | the 17-bit address (bit 0 of `$83`) |
//!
//! That port is how a DMA channel moves a block *into* WRAM from the A-bus —
//! the B-bus side of a channel can only name a `$21xx` register. A DMA
//! between WRAM's own A-bus address and `WMDATA` does not work on the
//! hardware (the bus cannot drive both), and fullsnes says the data is
//! undefined; the model does the transfer anyway and documents it here
//! rather than guess at a value.
//!
//! Two regions: `ram` (128 KiB, offset = WRAM address; the board maps it at
//! `$7E0000` and its first 8 KiB again in each system bank) and `port`
//! (4 bytes at B-bus `$80`, which the board maps at `$2180` of the system
//! banks, where the B-bus lives on the A-bus). A debug read of `WMDATA` does
//! not advance the address.
//!
//! WRAM is **not cleared at reset** — its power-on contents are whatever the
//! cells held — and the model starts it at zero on a cold start and leaves it
//! alone on a warm one, as the console's reset button does.
//!
//! Sources: nocash fullsnes, "SNES Memory Work RAM Access"; the SNESdev wiki,
//! "WRAM". No emulator source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, RamStore, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::value::Width;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "snes.wram";

/// The 128 KiB array.
pub const RAM_REGION: &str = "ram";

/// The four B-bus registers `$2180`–`$2183`.
pub const PORT_REGION: &str = "port";

/// How much WRAM there is.
pub const WRAM_SIZE: u64 = 128 * 1024;

const STATE_VERSION: u32 = 1;

/// The `snes.wram` device.
pub struct Wram {
    port: Arc<Port>,
    ram_region: RegionRef,
    port_region: RegionRef,
}

impl fmt::Debug for Wram {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Wram")
            .field("wmadd", &self.port.addr.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

#[derive(Debug)]
struct Port {
    ram: Arc<RamStore>,
    /// `WMADD`, seventeen bits.
    addr: AtomicU32,
}

impl Port {
    fn bump(&self) -> u32 {
        // A single-writer increment: the B-bus port has one master at a time.
        let a = self.addr.load(Ordering::Relaxed);
        self.addr.store((a + 1) & 0x1_FFFF, Ordering::Relaxed);
        a
    }
}

impl MemOps for Port {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        match offset {
            0 => {
                let a = if attrs.debug {
                    self.addr.load(Ordering::Relaxed)
                } else {
                    self.bump()
                };
                *byte = self.ram.read_u8(u64::from(a))?;
                Ok(())
            }
            // WMADD is write-only: the read is open bus, which the CPU
            // supplies.
            _ => Err(BusError::Unassigned),
        }
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = u32::from(*value);
        let a = self.addr.load(Ordering::Relaxed);
        match offset {
            0 => {
                let at = self.bump();
                self.ram.write_u8(u64::from(at), *value)?;
            }
            1 => self.addr.store((a & !0xFF) | v, Ordering::Relaxed),
            2 => self.addr.store((a & !0xFF00) | (v << 8), Ordering::Relaxed),
            3 => self
                .addr
                .store((a & 0xFFFF) | ((v & 1) << 16), Ordering::Relaxed),
            _ => return Err(BusError::Unassigned),
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

impl Wram {
    /// A fresh WRAM, zeroed.
    #[must_use]
    pub fn new() -> Wram {
        let ram = Arc::new(RamStore::new(WRAM_SIZE));
        let port = Arc::new(Port {
            ram: Arc::clone(&ram),
            addr: AtomicU32::new(0),
        });
        let ram_region = Arc::new(MmioRegion::ram("snes.wram", ram));
        let port_region = Arc::new(MmioRegion::io(
            "snes.wram.port",
            4,
            Arc::clone(&port) as Arc<dyn MemOps>,
        ));
        Wram {
            port,
            ram_region,
            port_region,
        }
    }

    /// The current `WMADD`.
    #[must_use]
    pub fn port_address(&self) -> u32 {
        self.port.addr.load(Ordering::Relaxed)
    }
}

impl Default for Wram {
    fn default() -> Self {
        Wram::new()
    }
}

/// The `snes.wram` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "SNES work RAM: 128 KiB, LowRAM, and the WMDATA/WMADD port on the B-bus",
    properties: &[],
    construct: |props| Ok(Box::new(from_props(props)?) as Box<dyn Device>),
};

fn from_props(props: &Props) -> Result<Wram> {
    props.reader().finish()?;
    Ok(Wram::new())
}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for Wram {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | RAM_REGION => Some(Arc::clone(&self.ram_region)),
            PORT_REGION => Some(Arc::clone(&self.port_region)),
            _ => None,
        }
    }

    fn reset(&self, kind: ResetKind) {
        // The reset button reaches WMADD's flip-flops, not the RAM cells.
        self.port.addr.store(0, Ordering::Relaxed);
        if kind == ResetKind::Cold {
            let _ = self.port.ram.fill(0, WRAM_SIZE, 0);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u32(self.port.addr.load(Ordering::Relaxed))?;
        let mut bytes = vec![0u8; WRAM_SIZE as usize];
        self.port
            .ram
            .read_at(0, &mut bytes)
            .map_err(|e| Error::State(alloc::format!("WRAM: {e}")))?;
        w.write_bytes(&bytes)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let addr = r.read_u32()? & 0x1_FFFF;
        let bytes = r.read_bytes()?;
        if bytes.len() as u64 != WRAM_SIZE {
            return Err(Error::State(alloc::format!(
                "snapshot holds {} bytes of WRAM, not {WRAM_SIZE}",
                bytes.len()
            )));
        }
        self.port
            .ram
            .write_at(0, bytes)
            .map_err(|e| Error::State(alloc::format!("WRAM: {e}")))?;
        self.port.addr.store(addr, Ordering::Relaxed);
        Ok(())
    }
}

impl crate::machine::Instance for Wram {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(from_props(props)?)))
}

/// What the validator should know about `snes.wram`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    crate::machine::validate::ClassSchema::new(CLASS.name)
        .region("")
        .region(RAM_REGION)
        .region(PORT_REGION)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn port_write(w: &Wram, reg: u64, v: u8) {
        w.port.write(reg, &[v], MemAttrs::DEFAULT).unwrap();
    }

    fn port_read(w: &Wram, attrs: MemAttrs) -> u8 {
        let mut b = [0u8];
        w.port.read(0, &mut b, attrs).unwrap();
        b[0]
    }

    #[test]
    fn wmdata_walks_a_17_bit_address() {
        let w = Wram::new();
        port_write(&w, 1, 0xFF);
        port_write(&w, 2, 0xFF);
        port_write(&w, 3, 0x01);
        assert_eq!(w.port_address(), 0x1_FFFF);
        port_write(&w, 0, 0xAB);
        assert_eq!(w.port_address(), 0, "it wraps at 128 KiB");
        assert_eq!(w.port.ram.read_u8(0x1_FFFF).unwrap(), 0xAB);
        // Only bit 0 of WMADDH counts.
        port_write(&w, 3, 0xFE);
        assert_eq!(w.port_address(), 0);
    }

    #[test]
    fn a_debug_read_does_not_advance_the_address() {
        let w = Wram::new();
        w.port.ram.write_u8(0, 0x11).unwrap();
        w.port.ram.write_u8(1, 0x22).unwrap();
        assert_eq!(port_read(&w, MemAttrs::DEBUG), 0x11);
        assert_eq!(w.port_address(), 0);
        assert_eq!(port_read(&w, MemAttrs::DEFAULT), 0x11);
        assert_eq!(port_read(&w, MemAttrs::DEFAULT), 0x22);
        assert_eq!(w.port_address(), 2);
        // WMADD reads are open bus.
        let mut b = [0u8];
        assert_eq!(
            w.port.read(1, &mut b, MemAttrs::DEFAULT),
            Err(BusError::Unassigned)
        );
    }

    #[test]
    fn a_warm_reset_keeps_the_cells() {
        let w = Wram::new();
        w.port.ram.write_u8(0x100, 0x5A).unwrap();
        port_write(&w, 1, 0x34);
        Device::reset(&w, ResetKind::Warm);
        assert_eq!(w.port.ram.read_u8(0x100).unwrap(), 0x5A);
        assert_eq!(w.port_address(), 0);
        Device::reset(&w, ResetKind::Cold);
        assert_eq!(w.port.ram.read_u8(0x100).unwrap(), 0);
    }

    #[test]
    fn a_snapshot_round_trips() {
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
        let saved = Wram::new();
        saved.port.ram.write_u8(0x1_2345, 0x99).unwrap();
        port_write(&saved, 1, 0x77);
        let mut shape = MachineShape::new();
        shape.add_device("wram", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("wram", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let restored = Wram::new();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("wram", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();
        assert_eq!(restored.port.ram.read_u8(0x1_2345).unwrap(), 0x99);
        assert_eq!(restored.port_address(), 0x77);
    }
}
