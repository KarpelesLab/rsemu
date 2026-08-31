//! The delta modulation channel — `$4010`–`$4013`.
//!
//! Sources: [NESdev APU DMC](https://www.nesdev.org/wiki/APU_DMC) for the
//! channel itself and [NESdev DMA](https://www.nesdev.org/wiki/DMA) for how its
//! sample fetch stalls the CPU.

use crate::core::error::Result;
use crate::core::state::{Sink, Source};

use super::frame::Region;

/// CPU cycles between a `$4015` write that starts playback and the enable
/// reaching the DMA unit.
///
/// AccuracyCoin's subtests L, M and N write `$4015` two, one and zero cycles
/// before the DMC timer reaches zero and check that the resulting fetch is
/// delayed by one, two and three cycles respectively - three measurements of
/// one constant, which is what makes it a latch and not a fudge.
const ENABLE_LATCH_CYCLES: u64 = 3;

/// The shortest gap between one sample byte arriving and the next fetch being
/// allowed to halt the CPU.
///
/// "This is just another DMA test showing that the DMA cannot occur within 2
/// cycles of a previous DMC DMA" - AccuracyCoin's "Implicit DMA Abort",
/// subtest 4, sixteen measurements of it. Without the gap a reload scheduled
/// the moment the buffer empties halts on the very next cycle, which is a get,
/// and costs three cycles instead of four; with it the halt slips to the
/// following put and the whole answer key lines up.
const FETCH_SPACING_CYCLES: u64 = 3;

/// Cycles between the last sample byte arriving and the memory reader's
/// counter settling on "the sample has ended".
///
/// The byte lands on the DMA's get cycle; the counter that decides whether
/// another fetch is wanted takes a cycle more to say so. Everything downstream
/// of that is timed from here, and AccuracyCoin's "Implicit DMA Abort" measures
/// all of it forty-eight times over: a sample buffer that empties *before* the
/// counter settles schedules an ordinary four-cycle fetch, one that empties
/// within two cycles *after* it gets the one-cycle aborted DMA, and one later
/// than that gets nothing at all. One and three both miss.
///
/// Counted from the cycle the arbiter decides on, which is the one before the
/// get it is arranging - so this is the get plus one.
const IMPLICIT_STOP_DELAY: u64 = 2;

/// DMC timer periods in **CPU cycles**, indexed by `$4010` bits 3-0.
const NTSC_RATES: [u16; 16] = [
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54,
];

/// DMC timer periods in CPU cycles for the RP2A07.
///
/// A genuinely different table, listed separately by the wiki
/// ([NESdev APU DMC](https://www.nesdev.org/wiki/APU_DMC)) rather than derived
/// from the NTSC one.
const PAL_RATES: [u16; 16] = [
    398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118, 98, 78, 66, 50,
];

/// The rate table for a console variant, in CPU cycles.
///
/// Dendy shares NTSC's, for the reason
/// [`Region::four_step`](super::frame::Region::four_step) sets out: the UA6527P
/// is a 2A03 clone on a slower clock, not a chip with its own dividers.
pub const fn rates(region: Region) -> [u16; 16] {
    match region {
        Region::Ntsc | Region::Dendy => NTSC_RATES,
        Region::Pal => PAL_RATES,
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
    /// A reload that will never happen, and halts the CPU anyway.
    ///
    /// When playback stops during the APU cycle before a reload would have been
    /// scheduled, the DMA still starts — and is aborted after a single cycle
    /// ([NESdev DMA](https://www.nesdev.org/wiki/DMA)). One cycle of `/RDY`
    /// low, no dummy, no fetch. And unlike a real fetch, which a write cycle
    /// merely delays, an abort that meets a write cycle does not happen at all.
    Abort,
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
    /// The CPU cycle the fetch was scheduled on.
    pub at: u64,
    /// The earliest cycle the DMA unit may halt the CPU for this fetch,
    /// whatever its own phase rule says. Two things floor it.
    ///
    /// A `$4015` write that starts playback does not enable the channel on the
    /// cycle it lands: the enable latches a few cycles later, and a fetch the
    /// timer schedules in the interim is *ready* but cannot halt yet. "Per my
    /// current understanding, if a sample is playing, you disable the DMC, and
    /// the final bit is read from the buffer, the DMA will still attempt to run
    /// every cycle until the DMC is re-enabled. It just doesn't run until the
    /// DMA is enabled, 2 or 3 cycles after a write to `$4015`." (Its "Delta
    /// Modulation Channel" subtests L, M and N are one cycle apart on purpose.)
    ///
    /// And two fetches cannot be back to back: "the DMA cannot occur within 2
    /// cycles of a previous DMC DMA" ("Implicit DMA Abort", subtest 4).
    ///
    /// (AccuracyCoin.asm, MIT, (c) 2025 Chris Siebert.)
    pub not_before: u64,
}

/// A scheduled but unserviced fetch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Pending {
    kind: DmaKind,
    serial: u64,
    /// The CPU cycle the fetch was scheduled on.
    ///
    /// The DMA unit needs it: a *load* fetch does not halt the CPU until the
    /// get cycle of the second APU cycle after the write that scheduled it —
    /// the third or fourth CPU cycle, depending on the phase the write landed
    /// on (NESdev wiki, "DMA").
    at: u64,
}

/// The delta modulation channel: memory reader, sample buffer, timer, output
/// unit, and the DMC interrupt flag.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dmc {
    region: Region,
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
    /// The cycle the channel's enable latch settles on after a `$4015` write
    /// that started playback. See [`DmaRequest::not_before`].
    enabled_at: u64,
    /// The cycle the last sample byte arrived on.
    ///
    /// The next fetch may not halt the CPU until [`FETCH_SPACING_CYCLES`] after
    /// it.
    last_fetch_at: u64,
    /// The CPU cycle playback last stopped on, explicitly or implicitly.
    ///
    /// Kept so the output unit can tell whether the buffer emptied *just* after
    /// the stop, which is the window the aborted DMA lives in.
    stopped_at: Option<u64>,
    /// Source of [`DmaRequest::serial`] values.
    next_serial: u64,
}

impl Dmc {
    /// A powered-on DMC: everything zero, output unit silent.
    pub const fn new(region: Region) -> Dmc {
        Dmc {
            region,
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
            enabled_at: 0,
            last_fetch_at: 0,
            stopped_at: None,
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
        rates(self.region)[usize::from(self.rate_index)]
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
    pub fn set_enabled(&mut self, enabled: bool, now: u64) {
        if enabled {
            if self.bytes_remaining == 0 {
                // The channel was off, so this write starts it - and the enable
                // does not latch on the write's own cycle. See
                // [`DmaRequest::not_before`] and [`ENABLE_LATCH_CYCLES`].
                self.enabled_at = now + ENABLE_LATCH_CYCLES;
                self.restart();
            }
            self.schedule(DmaKind::Load, now);
        } else {
            // Playback stops, but a fetch that is already scheduled still
            // happens: the CPU has been told to halt and the DMA runs its full
            // length. What the byte is used for afterwards is another matter.
            self.bytes_remaining = 0;
            self.stopped_at = Some(now);
        }
    }

    /// Schedule a fetch if the memory reader wants one and none is outstanding.
    ///
    /// The wiki's condition verbatim: any time the sample buffer is empty and
    /// bytes remaining is not zero.
    fn schedule(&mut self, kind: DmaKind, now: u64) {
        // A sample that has *just* ended still looks like one with bytes left:
        // the memory reader's counter settles after the byte arrives, not with
        // it, so a buffer that empties inside that window schedules an ordinary
        // fetch rather than an abort. See [`IMPLICIT_STOP_DELAY`].
        let has_bytes = self.bytes_remaining > 0 || self.stopped_at.is_some_and(|at| now < at);
        if self.dma.is_none() && self.buffer.is_none() && has_bytes {
            self.dma = Some(Pending {
                kind,
                serial: self.next_serial,
                at: now,
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
            at: p.at,
            not_before: self
                .enabled_at
                .max(self.last_fetch_at + FETCH_SPACING_CYCLES),
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

    /// Withdraw a request the arbiter refused to take up.
    ///
    /// An aborted DMA whose halt attempt landed on a write cycle does not
    /// happen at all — the ordinary "wait and try again" rule does not apply to
    /// it — so the arbiter tells the channel to forget it.
    pub fn dma_withdraw(&mut self, serial: u64) {
        if self.dma_is_pending(serial) {
            self.dma = None;
        }
    }

    /// Deliver the byte the CPU fetched.
    ///
    /// Returns false — and changes nothing — if the request was withdrawn in
    /// the meantime.
    pub fn dma_complete(&mut self, serial: u64, byte: u8, now: u64) -> bool {
        if !self.dma_is_pending(serial) {
            return false;
        }
        self.dma = None;
        self.last_fetch_at = now + 1;
        self.buffer = Some(byte);
        self.cur_addr = if self.cur_addr == 0xFFFF {
            0x8000
        } else {
            self.cur_addr + 1
        };
        if self.bytes_remaining == 0 {
            // Playback was stopped while this fetch was in flight. The byte is
            // in the buffer and the memory reader has nothing left to count.
            return true;
        }
        self.bytes_remaining -= 1;
        if self.bytes_remaining == 0 {
            if self.loop_flag {
                self.restart();
            } else {
                // Playback has ended of its own accord. That counts as a stop
                // for the aborted-DMA window exactly as a `$4015` write does.
                self.stopped_at = Some(now + IMPLICIT_STOP_DELAY);
                if self.irq_enabled {
                    self.irq = true;
                }
            }
        }
        true
    }

    /// Clock the timer. Called once per APU cycle.
    pub fn tick_timer(&mut self, now: u64) {
        if self.timer == 0 {
            self.timer = self.reload();
            self.clock_output(now);
        } else {
            self.timer -= 1;
        }
    }

    /// One output-unit clock: adjust the level, shift, count the bit.
    fn clock_output(&mut self, now: u64) {
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
            self.schedule(DmaKind::Reload, now);
            // ...unless playback stopped a moment ago, in which case the DMA
            // starts and is aborted after one cycle. "This aborted DMA
            // schedules regardless of how playback was stopped, whether
            // explicitly or implicitly" (NESdev, "DMA").
            if self.dma.is_none()
                && self.bytes_remaining == 0
                && self.stopped_at.is_some_and(|at| at <= now && now - at <= 2)
            {
                self.dma = Some(Pending {
                    kind: DmaKind::Abort,
                    serial: self.next_serial,
                    at: now,
                });
                self.next_serial = self.next_serial.wrapping_add(1);
            }
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
                    DmaKind::Abort => 3,
                })?;
                w.write_u64(p.serial)?;
            }
            None => {
                w.write_u8(0)?;
                w.write_u64(0)?;
            }
        }
        w.write_u64(self.next_serial)?;
        // Appended: the cycle a pending fetch was scheduled on, which decides
        // the phase it may halt the CPU on.
        w.write_u64(self.dma.map_or(0, |p| p.at))?;
        w.write_u64(self.enabled_at)?;
        w.write_u64(self.last_fetch_at)
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
        self.next_serial = r.read_u64()?;
        // v2 appended the cycle a pending fetch was scheduled on. A v1 chunk
        // has no such field; zero is the right restore, because a fetch whose
        // schedule cycle is already past halts on the next legal phase.
        let at = r.read_u64().unwrap_or(0);
        self.enabled_at = r.read_u64().unwrap_or(0);
        self.last_fetch_at = r.read_u64().unwrap_or(0);
        self.dma = match kind {
            1 => Some(Pending {
                kind: DmaKind::Load,
                serial,
                at,
            }),
            2 => Some(Pending {
                kind: DmaKind::Reload,
                serial,
                at,
            }),
            3 => Some(Pending {
                kind: DmaKind::Abort,
                serial,
                at,
            }),
            _ => None,
        };
        Ok(())
    }
}
