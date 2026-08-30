//! The delta modulation channel — `$4010`–`$4013`.
//!
//! Sources: [NESdev APU DMC](https://www.nesdev.org/wiki/APU_DMC) for the
//! channel itself and [NESdev DMA](https://www.nesdev.org/wiki/DMA) for how its
//! sample fetch stalls the CPU.

use crate::core::error::Result;
use crate::core::state::{Sink, Source};

use super::frame::Timing;

/// DMC timer periods in **CPU cycles**, indexed by `$4010` bits 3-0.
const NTSC_RATES: [u16; 16] = [
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54,
];

/// DMC timer periods in CPU cycles for the RP2A07.
const PAL_RATES: [u16; 16] = [
    398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118, 98, 78, 66, 50,
];

/// The rate table for a console variant, in CPU cycles.
pub const fn rates(timing: Timing) -> [u16; 16] {
    match timing {
        Timing::Ntsc => NTSC_RATES,
        Timing::Pal => PAL_RATES,
    }
}

/// Which of the two documented DMC DMA flavours a request is.
///
/// They differ only in when the CPU is asked to halt — the wiki's *load* DMA
/// (scheduled by a `$4015` write that starts playback) aims at a get cycle,
/// the *reload* DMA (scheduled when the output unit empties the sample buffer)
/// at a put cycle — which is why load DMAs normally cost 3 CPU cycles and
/// reload DMAs 4. The APU decides the flavour; the cycle choreography belongs
/// to the CPU.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DmaKind {
    /// Scheduled by a `$4015` write that started sample playback.
    Load,
    /// Scheduled by the output unit emptying the sample buffer.
    Reload,
}

/// A sample fetch the DMC wants the CPU to perform.
///
/// The CPU polls [`super::Apu::dma_request`], runs the halt / dummy /
/// alignment / get sequence itself, and hands the byte back with
/// [`super::Apu::dma_complete`]. The `serial` makes the hand-back unambiguous:
/// if playback stops before the get cycle the request disappears, and a
/// completion quoting a stale serial is rejected rather than corrupting a
/// sample that has since restarted. That is the hook the "aborted DMA" case in
/// [NESdev DMA](https://www.nesdev.org/wiki/DMA) needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaRequest {
    /// Whether this is a load or a reload fetch.
    pub kind: DmaKind,
    /// The guest address to read, in the CPU's address space.
    pub addr: u16,
    /// Identifies this request for the lifetime of the machine.
    pub serial: u64,
}

/// A scheduled but unserviced fetch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Pending {
    kind: DmaKind,
    serial: u64,
}

/// The delta modulation channel: memory reader, sample buffer, timer, output
/// unit, and the DMC interrupt flag.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dmc {
    timing: Timing,
    /// `$4010` bit 7.
    irq_enabled: bool,
    /// `$4010` bit 6.
    loop_flag: bool,
    /// `$4010` bits 3-0.
    rate_index: u8,
    /// The 7-bit output level, also settable directly through `$4011`.
    output: u8,
    /// `$4012`, the sample address as written.
    addr_reg: u8,
    /// `$4013`, the sample length as written.
    len_reg: u8,
    /// The memory reader's address counter.
    cur_addr: u16,
    /// The memory reader's bytes-remaining counter.
    bytes_remaining: u16,
    /// The one-byte sample buffer, empty when `None`.
    buffer: Option<u8>,
    /// The output unit's 8-bit right shift register.
    shift: u8,
    /// The output unit's bits-remaining counter.
    bits_remaining: u8,
    /// The output unit's silence flag.
    silence: bool,
    /// The timer's down counter, in APU cycles.
    timer: u16,
    /// The DMC interrupt flag.
    irq: bool,
    /// A fetch waiting for the CPU.
    dma: Option<Pending>,
    /// Source of [`DmaRequest::serial`] values.
    next_serial: u64,
}

impl Dmc {
    /// A powered-on DMC: everything zero, output unit silent.
    pub const fn new(timing: Timing) -> Dmc {
        Dmc {
            timing,
            irq_enabled: false,
            loop_flag: false,
            rate_index: 0,
            output: 0,
            addr_reg: 0,
            len_reg: 0,
            cur_addr: 0,
            bytes_remaining: 0,
            buffer: None,
            shift: 0,
            bits_remaining: 8,
            silence: true,
            timer: 0,
            irq: false,
            dma: None,
            next_serial: 1,
        }
    }

    /// The 7-bit level this channel sends to the mixer.
    ///
    /// Sent whether or not the channel is enabled: the enable bit only controls
    /// automatic playback, not the DAC.
    #[inline]
    pub const fn output(&self) -> u8 {
        self.output
    }

    /// The DMC interrupt flag, which drives the CPU's IRQ line while set.
    #[inline]
    pub const fn irq(&self) -> bool {
        self.irq
    }

    /// Clear the DMC interrupt flag, as any `$4015` write does.
    pub fn clear_irq(&mut self) {
        self.irq = false;
    }

    /// Whether the memory reader still has bytes to fetch — `$4015` bit 4.
    #[inline]
    pub const fn active(&self) -> bool {
        self.bytes_remaining > 0
    }

    /// The memory reader's remaining byte count.
    #[inline]
    pub const fn bytes_remaining(&self) -> u16 {
        self.bytes_remaining
    }

    /// The timer period in CPU cycles, as the wiki's rate table gives it.
    #[inline]
    pub fn rate_cycles(&self) -> u16 {
        rates(self.timing)[usize::from(self.rate_index)]
    }

    /// The divider's reload value, in APU cycles.
    ///
    /// The rate table is in CPU cycles and every entry is even, so a rate of
    /// 428 is 214 APU cycles; a down counter reloaded with `P` has period
    /// `P + 1`, hence the `- 1`. The smallest entry is 50, so this cannot
    /// underflow.
    #[inline]
    fn reload(&self) -> u16 {
        self.rate_cycles() / 2 - 1
    }

    /// `$4010`: IRQ enable, loop flag, rate index.
    ///
    /// Clearing the IRQ enable bit clears the interrupt flag.
    pub fn write_control(&mut self, value: u8) {
        self.irq_enabled = value & 0x80 != 0;
        self.loop_flag = value & 0x40 != 0;
        self.rate_index = value & 0x0F;
        if !self.irq_enabled {
            self.irq = false;
        }
    }

    /// `$4011`: load the output level directly.
    ///
    /// The wiki records that a write landing on the same cycle as a timer clock
    /// "occasionally" fails to change the level properly. That is a race in the
    /// silicon with no documented rule, so it is not modelled: the write always
    /// takes effect here.
    pub fn write_output(&mut self, value: u8) {
        self.output = value & 0x7F;
    }

    /// `$4012`: sample address, `$C000 + A * 64`.
    pub fn write_address(&mut self, value: u8) {
        self.addr_reg = value;
    }

    /// `$4013`: sample length, `L * 16 + 1` bytes.
    pub fn write_length(&mut self, value: u8) {
        self.len_reg = value;
    }

    /// Point the memory reader at the start of the configured sample.
    fn restart(&mut self) {
        self.cur_addr = 0xC000u16.wrapping_add(u16::from(self.addr_reg) * 64);
        self.bytes_remaining = u16::from(self.len_reg) * 16 + 1;
    }

    /// Apply the `$4015` DMC enable bit.
    ///
    /// Clearing it zeroes the bytes-remaining counter — the channel then goes
    /// quiet once the sample buffer empties — and withdraws any fetch the CPU
    /// has not yet performed. Setting it restarts the sample only if the
    /// counter is already zero, and schedules a *load* fetch if the buffer is
    /// empty.
    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled {
            if self.bytes_remaining == 0 {
                self.restart();
            }
            self.schedule(DmaKind::Load);
        } else {
            self.bytes_remaining = 0;
            self.dma = None;
        }
    }

    /// Schedule a fetch if the memory reader wants one and none is outstanding.
    ///
    /// The wiki's condition verbatim: any time the sample buffer is empty and
    /// bytes remaining is not zero.
    fn schedule(&mut self, kind: DmaKind) {
        if self.dma.is_none() && self.buffer.is_none() && self.bytes_remaining > 0 {
            self.dma = Some(Pending {
                kind,
                serial: self.next_serial,
            });
            self.next_serial = self.next_serial.wrapping_add(1);
        }
    }

    /// The fetch the CPU should perform, if any.
    pub fn dma_request(&self) -> Option<DmaRequest> {
        self.dma.map(|p| DmaRequest {
            kind: p.kind,
            addr: self.cur_addr,
            serial: p.serial,
        })
    }

    /// Whether the request identified by `serial` is still outstanding.
    ///
    /// A CPU that has latched a request polls this to discover an aborted DMA:
    /// playback stopped between the halt and the get cycle, so the fetch never
    /// happens.
    pub fn dma_is_pending(&self, serial: u64) -> bool {
        self.dma.is_some_and(|p| p.serial == serial)
    }

    /// Deliver the byte the CPU fetched.
    ///
    /// Returns false — and changes nothing — if the request was withdrawn in
    /// the meantime.
    pub fn dma_complete(&mut self, serial: u64, byte: u8) -> bool {
        if !self.dma_is_pending(serial) {
            return false;
        }
        self.dma = None;
        self.buffer = Some(byte);
        self.cur_addr = if self.cur_addr == 0xFFFF {
            0x8000
        } else {
            self.cur_addr + 1
        };
        self.bytes_remaining -= 1;
        if self.bytes_remaining == 0 {
            if self.loop_flag {
                self.restart();
            } else if self.irq_enabled {
                self.irq = true;
            }
        }
        true
    }

    /// Clock the timer. Called once per APU cycle.
    pub fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.reload();
            self.clock_output();
        } else {
            self.timer -= 1;
        }
    }

    /// One output-unit clock: adjust the level, shift, count the bit.
    fn clock_output(&mut self) {
        if !self.silence {
            if self.shift & 1 != 0 {
                if self.output <= 125 {
                    self.output += 2;
                }
            } else if self.output >= 2 {
                self.output -= 2;
            }
        }
        self.shift >>= 1;
        self.bits_remaining -= 1;
        if self.bits_remaining == 0 {
            self.bits_remaining = 8;
            match self.buffer.take() {
                Some(byte) => {
                    self.silence = false;
                    self.shift = byte;
                }
                None => self.silence = true,
            }
            // Emptying the buffer is exactly the condition that schedules the
            // next fetch, and it is a reload rather than a load.
            self.schedule(DmaKind::Reload);
        }
    }

    /// A reset that leaves `$4010`–`$4013` alone.
    ///
    /// [NESdev CPU power up
    /// state](https://www.nesdev.org/wiki/CPU_power_up_state): a reset leaves
    /// the DMC's registers unchanged but ANDs the output level with 1.
    pub fn reset_warm(&mut self) {
        self.output &= 1;
        self.bytes_remaining = 0;
        self.buffer = None;
        self.silence = true;
        self.bits_remaining = 8;
        self.irq = false;
        self.dma = None;
    }

    /// Serialize architectural state.
    pub fn save(&self, w: &mut dyn Sink) -> Result<()> {
        w.write_bool(self.irq_enabled)?;
        w.write_bool(self.loop_flag)?;
        w.write_u8(self.rate_index)?;
        w.write_u8(self.output)?;
        w.write_u8(self.addr_reg)?;
        w.write_u8(self.len_reg)?;
        w.write_u16(self.cur_addr)?;
        w.write_u16(self.bytes_remaining)?;
        match self.buffer {
            Some(byte) => {
                w.write_bool(true)?;
                w.write_u8(byte)?;
            }
            None => {
                w.write_bool(false)?;
                w.write_u8(0)?;
            }
        }
        w.write_u8(self.shift)?;
        w.write_u8(self.bits_remaining)?;
        w.write_bool(self.silence)?;
        w.write_u16(self.timer)?;
        w.write_bool(self.irq)?;
        match self.dma {
            Some(p) => {
                w.write_u8(match p.kind {
                    DmaKind::Load => 1,
                    DmaKind::Reload => 2,
                })?;
                w.write_u64(p.serial)?;
            }
            None => {
                w.write_u8(0)?;
                w.write_u64(0)?;
            }
        }
        w.write_u64(self.next_serial)
    }

    /// Restore what [`Dmc::save`] wrote.
    pub fn load<'a>(&mut self, r: &mut dyn Source<'a>) -> Result<()> {
        self.irq_enabled = r.read_bool()?;
        self.loop_flag = r.read_bool()?;
        self.rate_index = r.read_u8()? & 0x0F;
        self.output = r.read_u8()? & 0x7F;
        self.addr_reg = r.read_u8()?;
        self.len_reg = r.read_u8()?;
        self.cur_addr = r.read_u16()?;
        self.bytes_remaining = r.read_u16()?;
        let filled = r.read_bool()?;
        let byte = r.read_u8()?;
        self.buffer = if filled { Some(byte) } else { None };
        self.shift = r.read_u8()?;
        self.bits_remaining = r.read_u8()?;
        self.silence = r.read_bool()?;
        self.timer = r.read_u16()?;
        self.irq = r.read_bool()?;
        let kind = r.read_u8()?;
        let serial = r.read_u64()?;
        self.dma = match kind {
            1 => Some(Pending {
                kind: DmaKind::Load,
                serial,
            }),
            2 => Some(Pending {
                kind: DmaKind::Reload,
                serial,
            }),
            _ => None,
        };
        self.next_serial = r.read_u64()?;
        Ok(())
    }
}
