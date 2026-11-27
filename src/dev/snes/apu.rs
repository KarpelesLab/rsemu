//! The S-SMP: the SNES audio unit's processor side.
//!
//! One device, `snes.apu`, that **contains** the SPC700 ([`crate::cpu::spc700`])
//! together with everything on its private bus: 64 KiB of audio RAM, the
//! sixteen registers at `$F0`–`$FF`, the three timers, the four communication
//! ports, the boot ROM overlay at `$FFC0`, and — through [`SDsp`] — the S-DSP.
//! Containment rather than an address space because every SMP cycle clocks
//! the timers and the DSP; an address space in that loop would buy nothing a
//! guest can observe and cost a dispatch per cycle.
//!
//! What the rest of the machine sees is one region and one clock:
//!
//! * **`cpuio`** — the S-CPU's side of the four ports, 64 bytes for the board
//!   to map at `$2140`–`$217F` on the B-bus (the four ports mirrored sixteen
//!   times).
//! * The device is **runnable** in its own clock domain, one tick per SMP
//!   cycle: 1.024 MHz, the audio unit's 24.576 MHz crystal divided by 24. The
//!   board declares that crystal; it is a second oscillator, independent of
//!   the S-CPU's (`ROADMAP.md` §4.2).
//!
//! A second region, **`aram`**, exposes the raw 64 KiB for a debugger; reads
//! and writes through it touch RAM only, never a register.
//!
//! # The ports
//!
//! Each port is two latches: one the S-CPU writes and the SMP reads at
//! `$F4`–`$F7`, and one the SMP writes and the S-CPU reads. Neither side ever
//! reads back what it wrote (fullsnes, *SNES APU Main CPU Communication
//! Port*). The latches are atomics, because the two processors run in
//! different scheduler slices and neither may take the other's lock. CONTROL
//! bits 4 and 5 clear the S-CPU-written latches of ports 0–1 and 2–3.
//!
//! # Registers (`$F0`–`$FF`)
//!
//! | Addr | Name | Here |
//! | --- | --- | --- |
//! | `$F0` | TEST | stored; bit 1 gates RAM writes, timers run while bit 3 is set and bit 0 clear; the wait-state bits are not modelled |
//! | `$F1` | CONTROL | timer enables (0→1 resets that timer), port clears, IPL ROM enable |
//! | `$F2` | DSPADDR | read/write |
//! | `$F3` | DSPDATA | [`SDsp::read`]/[`SDsp::write`]; reads at `DSPADDR ≥ $80` mirror `$00`–`$7F`, writes there are dropped |
//! | `$F4`–`$F7` | CPUIO0–3 | see above |
//! | `$F8`/`$F9` | AUXIO | plain storage |
//! | `$FA`–`$FC` | T0–T2 target | write-only, `$00` means 256 |
//! | `$FD`–`$FF` | T0–T2 out | 4-bit counter, cleared by a read — **not** by a debug read |
//!
//! Write-only registers read as zero. Writes to `$F0`–`$FF` also reach the RAM
//! underneath, and a write to `$FFC0`–`$FFFF` always lands in RAM whether or
//! not the ROM is mapped over it for reads.
//!
//! T0 and T1 count at 8 kHz (every 128 SMP cycles), T2 at 64 kHz (every 16).
//!
//! # The IPL ROM
//!
//! Nintendo's boot ROM is 64 bytes of copyrighted code and **is not shipped**.
//! [`CLEAN_IPL`] is a clean-room replacement written in SPC700 machine code
//! from the *protocol* the SNESdev wiki (*Booting the SPC700*) and fullsnes
//! (*SNES APU Boot ROM*) document — not from a disassembly of Nintendo's code.
//! Every byte is listed against its source line below. It implements:
//!
//! 1. `SP = $EF`, clear `$01`–`$EF`, post `$AA` / `$BB` on ports 0 / 1;
//! 2. wait for `$CC` on port 0; then, per block: the destination address
//!    from ports 2–3, the command from port 1, echo port 0;
//! 3. command 0 jumps to the address; otherwise bytes arrive as (data on
//!    port 1, index on port 0), each index echoed; any other port-0 value
//!    ends the block and starts the next one.
//!
//! Because it is a different program, code that reads the ROM's bytes or
//! jumps into its middle will not behave as on hardware. For that, the
//! optional **`ipl`** media property takes a real 64-byte dump.
//!
//! # Sources
//!
//! nocash *fullsnes* (*SNES APU* chapters: memory map, I/O ports, timers,
//! boot ROM), the SNESdev wiki (*S-SMP*, *Booting the SPC700*), and anomie's
//! *APU* document for the timer stages. No emulator source of any licence was
//! consulted.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use super::dsp_seam::{Aram, SDsp};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU8, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::cpu::spc700::{self, Bus, Core};

/// The name a `map` statement reaches the S-CPU's side of the ports by.
pub const CPUIO_REGION: &str = "cpuio";

/// The debugger's view of audio RAM.
pub const ARAM_REGION: &str = "aram";

/// The S-CPU window the ports occupy: `$2140`–`$217F`.
pub const CPUIO_SIZE: u64 = 0x40;

/// How many stereo frames the output ring holds before the oldest are dropped
/// — about half a second at 32 kHz.
pub const RING_FRAMES: usize = 16_384;

/// SMP cycles per output sample: 1.024 MHz / 32 kHz.
pub const SAMPLE_DIVISOR: u64 = 32;

/// Snapshot layout version.
const STATE_VERSION: u32 = 1;

/// The clean-room boot ROM, mapped at `$FFC0`–`$FFFF`.
///
/// Hand-assembled; each line is one instruction, its address, and the
/// protocol step it implements. Branch displacements count from the byte
/// after the branch.
pub const CLEAN_IPL: [u8; 64] = [
    0x20, //             FFC0  clrp              ; direct page = $00xx
    0xcd, 0xef, //       FFC1  mov x,#$ef
    0xbd, //             FFC3  mov sp,x          ; SP = $EF
    0xe8, 0x00, //       FFC4  mov a,#$00
    0xc6, //             FFC6  clr: mov (x),a    ; clear $EF..$01
    0x1d, //             FFC7  dec x
    0xd0, 0xfc, //       FFC8  bne clr           ; -> FFC6
    0x8f, 0xaa, 0xf4, // FFCA  mov $f4,#$aa      ; "ready" signature,
    0x8f, 0xbb, 0xf5, // FFCD  mov $f5,#$bb      ;  ports 0/1 = $AA/$BB
    0x78, 0xcc, 0xf4, // FFD0  w0: cmp $f4,#$cc  ; wait for the first kick
    0xd0, 0xfb, //       FFD3  bne w0            ; -> FFD0
    0xba, 0xf6, //       FFD5  blk: movw ya,$f6  ; destination from ports 2/3
    0xda, 0x00, //       FFD7  movw $00,ya       ; pointer at $00/$01
    0xba, 0xf4, //       FFD9  movw ya,$f4       ; A = kick, Y = command
    0xc4, 0xf4, //       FFDB  mov $f4,a         ; echo the kick
    0x5d, //             FFDD  mov x,a           ; X = value now stale on port 0
    0xdd, //             FFDE  mov a,y           ; Z = (command == 0)
    0xd0, 0x04, //       FFDF  bne xfer          ; -> FFE5
    0x5d, //             FFE1  mov x,a           ; X = 0
    0x1f, 0x00, 0x00, // FFE2  jmp [!$0000+x]    ; command 0: run it
    0x8d, 0x00, //       FFE5  xfer: mov y,#$00  ; index of the next byte
    0x3e, 0xf4, //       FFE7  w2: cmp x,$f4     ; port 0 still the stale value?
    0xf0, 0xfc, //       FFE9  beq w2            ; -> FFE7: keep waiting
    0x7e, 0xf4, //       FFEB  cmp y,$f4         ; changed: the next index?
    0xd0, 0xe6, //       FFED  bne blk           ; -> FFD5: no, a kick
    0xe4, 0xf5, //       FFEF  byte: mov a,$f5   ; the data byte
    0xd7, 0x00, //       FFF1  mov [$00]+y,a     ; store it
    0xcb, 0xf4, //       FFF3  mov $f4,y         ; echo the index
    0xdd, //             FFF5  mov a,y
    0x5d, //             FFF6  mov x,a           ; that index is now stale
    0xfc, //             FFF7  inc y
    0xd0, 0xed, //       FFF8  bne w2            ; -> FFE7
    0xab, 0x01, //       FFFA  inc $01           ; 256 bytes: next page
    0x2f, 0xe9, //       FFFC  bra w2            ; -> FFE7
    0xc0, 0xff, //       FFFE  reset vector      ; -> FFC0
];

/// A DSP that does nothing: registers read as written, no sound.
///
/// What the unit tests use, and what an embedder with no S-DSP gets.
#[derive(Debug, Clone)]
pub struct NullDsp {
    regs: [u8; 128],
}

impl Default for NullDsp {
    fn default() -> Self {
        NullDsp { regs: [0; 128] }
    }
}

impl SDsp for NullDsp {
    fn read(&self, reg: u8) -> u8 {
        self.regs[usize::from(reg & 0x7f)]
    }
    fn write(&mut self, reg: u8, value: u8) {
        self.regs[usize::from(reg & 0x7f)] = value;
    }
    fn clock(&mut self, _aram: &mut dyn Aram) -> Option<(i16, i16)> {
        None
    }
    fn reset(&mut self) {
        self.regs = [0; 128];
    }
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bytes(&self.regs)
    }
    fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        let b = r.read_bytes()?;
        if b.len() != 128 {
            return Err(Error::State(String::from(
                "NullDsp: register file is not 128 bytes",
            )));
        }
        self.regs.copy_from_slice(b);
        Ok(())
    }
}

/// The four ports' eight latches.
#[derive(Debug, Default)]
struct Ports {
    /// Written by the S-CPU, read by the SMP at `$F4`–`$F7`.
    to_smp: [AtomicU8; 4],
    /// Written by the SMP, read by the S-CPU.
    to_cpu: [AtomicU8; 4],
}

/// Audio RAM as the DSP sees it.
struct AramView<'a>(&'a mut [u8; 0x10000]);

impl Aram for AramView<'_> {
    fn read(&self, addr: u16) -> u8 {
        self.0[usize::from(addr)]
    }
    fn write(&mut self, addr: u16, value: u8) {
        self.0[usize::from(addr)] = value;
    }
}

/// The SMP's register file and timers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Io {
    test: u8,
    control: u8,
    dspaddr: u8,
    aux: [u8; 2],
    target: [u8; 3],
    /// The stage-2 counters, compared against `target`.
    stage: [u8; 3],
    /// The 4-bit outputs at `$FD`–`$FF`.
    out: [u8; 3],
    /// SMP cycles mod 128, for the 8 kHz and 64 kHz prescalers.
    prescale: u8,
}

impl Io {
    const fn power_on() -> Io {
        Io {
            test: 0x0a,
            control: 0xb0,
            dspaddr: 0,
            aux: [0; 2],
            target: [0; 3],
            stage: [0; 3],
            out: [0; 3],
            prescale: 0,
        }
    }

    fn timers_run(&self) -> bool {
        self.test & 0x08 != 0 && self.test & 0x01 == 0
    }

    /// One SMP cycle of the timer prescalers.
    fn tick(&mut self) {
        self.prescale = (self.prescale + 1) & 0x7f;
        if !self.timers_run() {
            return;
        }
        let fast = self.prescale & 0x0f == 0;
        let slow = self.prescale == 0;
        for t in 0..3 {
            let due = if t == 2 { fast } else { slow };
            if due && self.control & (1 << t) != 0 {
                // A target of $00 means 256: the u8 wraps to zero exactly then.
                self.stage[t] = self.stage[t].wrapping_add(1);
                if self.stage[t] == self.target[t] {
                    self.stage[t] = 0;
                    self.out[t] = (self.out[t] + 1) & 0x0f;
                }
            }
        }
    }
}

/// Everything the SMP mutates, behind one lock.
struct Inner {
    core: Core,
    aram: Box<[u8; 0x10000]>,
    io: Io,
    dsp: Box<dyn SDsp>,
    /// Cycles past the last budget, owed to the next.
    debt: u64,
}

impl fmt::Debug for Inner {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Inner")
            .field("core", &self.core)
            .field("io", &self.io)
            .field("dsp", &self.dsp)
            .finish_non_exhaustive()
    }
}

/// The SMP's view of its bus for one step.
struct SmpBus<'a> {
    aram: &'a mut [u8; 0x10000],
    io: &'a mut Io,
    dsp: &'a mut dyn SDsp,
    ports: &'a Ports,
    ipl: &'a [u8; 64],
    ring: &'a Mutex<VecDeque<(i16, i16)>>,
    recording: bool,
}

impl SmpBus<'_> {
    /// One SMP cycle of everything that is not the processor.
    fn clock(&mut self) {
        self.io.tick();
        if let Some(frame) = self.dsp.clock(&mut AramView(self.aram))
            && self.recording
        {
            let mut ring = self.ring.lock();
            if ring.len() >= RING_FRAMES {
                ring.pop_front();
            }
            ring.push_back(frame);
        }
    }

    /// A read as the SMP sees it. `debug` suppresses the counter clear.
    fn decode_read(&mut self, addr: u16, debug: bool) -> u8 {
        match addr {
            0x00f0..=0x00ff => {
                let io = &mut *self.io;
                match addr {
                    0xf2 => io.dspaddr,
                    0xf3 => self.dsp.read(io.dspaddr & 0x7f),
                    0xf4..=0xf7 => {
                        self.ports.to_smp[usize::from(addr - 0xf4)].load(Ordering::Acquire)
                    }
                    0xf8 | 0xf9 => io.aux[usize::from(addr - 0xf8)],
                    0xfd..=0xff => {
                        let t = usize::from(addr - 0xfd);
                        let v = io.out[t];
                        if !debug {
                            io.out[t] = 0;
                        }
                        v
                    }
                    // TEST, CONTROL and the targets are write-only.
                    _ => 0,
                }
            }
            0xffc0..=0xffff if self.io.control & 0x80 != 0 => self.ipl[usize::from(addr - 0xffc0)],
            _ => self.aram[usize::from(addr)],
        }
    }

    fn decode_write(&mut self, addr: u16, value: u8) {
        if self.io.test & 0x02 != 0 {
            self.aram[usize::from(addr)] = value;
        }
        if !(0xf0..=0xff).contains(&addr) {
            return;
        }
        let io = &mut *self.io;
        match addr {
            0xf0 => io.test = value,
            0xf1 => {
                for t in 0..3 {
                    let bit = 1 << t;
                    if value & bit != 0 && io.control & bit == 0 {
                        io.stage[t] = 0;
                        io.out[t] = 0;
                    }
                }
                if value & 0x10 != 0 {
                    self.ports.to_smp[0].store(0, Ordering::Release);
                    self.ports.to_smp[1].store(0, Ordering::Release);
                }
                if value & 0x20 != 0 {
                    self.ports.to_smp[2].store(0, Ordering::Release);
                    self.ports.to_smp[3].store(0, Ordering::Release);
                }
                io.control = value;
            }
            0xf2 => io.dspaddr = value,
            0xf3 => {
                if io.dspaddr < 0x80 {
                    self.dsp.write(io.dspaddr, value);
                }
            }
            0xf4..=0xf7 => {
                self.ports.to_cpu[usize::from(addr - 0xf4)].store(value, Ordering::Release)
            }
            0xf8 | 0xf9 => io.aux[usize::from(addr - 0xf8)] = value,
            0xfa..=0xfc => io.target[usize::from(addr - 0xfa)] = value,
            // The counters are read-only.
            _ => {}
        }
    }
}

impl Bus for SmpBus<'_> {
    fn read(&mut self, addr: u16) -> u8 {
        let v = self.decode_read(addr, false);
        self.clock();
        v
    }

    fn write(&mut self, addr: u16, value: u8) {
        self.decode_write(addr, value);
        self.clock();
    }

    fn idle(&mut self) {
        self.clock();
    }
}

/// The S-CPU's side of the ports.
#[derive(Debug)]
struct CpuPort {
    ports: Arc<Ports>,
}

impl MemOps for CpuPort {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        // No side effects on this side, so a debug read is an ordinary one.
        for (i, b) in dst.iter_mut().enumerate() {
            *b = self.ports.to_cpu[((offset + i as u64) & 3) as usize].load(Ordering::Acquire);
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A debugger poking the ports would be a message to the SMP.
            return Ok(());
        }
        for (i, b) in src.iter().enumerate() {
            self.ports.to_smp[((offset + i as u64) & 3) as usize].store(*b, Ordering::Release);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U16)
    }
}

/// The raw audio RAM, for a debugger.
#[derive(Debug)]
struct AramPort {
    inner: Arc<Mutex<Inner>>,
}

impl MemOps for AramPort {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let inner = self.inner.lock();
        for (i, b) in dst.iter_mut().enumerate() {
            *b = inner.aram[((offset + i as u64) & 0xffff) as usize];
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let mut inner = self.inner.lock();
        for (i, b) in src.iter().enumerate() {
            inner.aram[((offset + i as u64) & 0xffff) as usize] = *b;
        }
        Ok(())
    }
}

/// The `snes.apu` device: the S-SMP with its SPC700, ARAM, timers and ports.
pub struct SnesApu {
    ports: Arc<Ports>,
    inner: Arc<Mutex<Inner>>,
    ipl: [u8; 64],
    ring: Mutex<VecDeque<(i16, i16)>>,
    recording: AtomicBool,
    cpuio: RegionRef,
    aram_region: RegionRef,
}

impl fmt::Debug for SnesApu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SnesApu")
            .field("inner", &self.inner)
            .finish_non_exhaustive()
    }
}

impl SnesApu {
    /// A unit at power-on around `dsp`, booting from [`CLEAN_IPL`].
    #[must_use]
    pub fn new(dsp: Box<dyn SDsp>) -> SnesApu {
        SnesApu::with_ipl(dsp, CLEAN_IPL)
    }

    /// The same, booting from a supplied 64-byte IPL image.
    #[must_use]
    pub fn with_ipl(dsp: Box<dyn SDsp>, ipl: [u8; 64]) -> SnesApu {
        let ports = Arc::new(Ports::default());
        let inner = Arc::new(Mutex::with_rank(
            LockRank::DEVICE,
            Inner {
                core: Core::new(),
                aram: Box::new([0; 0x10000]),
                io: Io::power_on(),
                dsp,
                debt: 0,
            },
        ));
        let cpuio = Arc::new(MmioRegion::io(
            "snes.apu.cpuio",
            CPUIO_SIZE,
            Arc::new(CpuPort {
                ports: Arc::clone(&ports),
            }) as Arc<dyn MemOps>,
        ));
        let aram_region = Arc::new(MmioRegion::io(
            "snes.apu.aram",
            0x1_0000,
            Arc::new(AramPort {
                inner: Arc::clone(&inner),
            }) as Arc<dyn MemOps>,
        ));
        SnesApu {
            ports,
            inner,
            ipl,
            ring: Mutex::with_rank(LockRank::LEAF, VecDeque::new()),
            recording: AtomicBool::new(false),
            cpuio,
            aram_region,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// An unknown property, or an `ipl` image that is not exactly 64 bytes.
    pub fn from_props(props: &Props) -> Result<SnesApu> {
        let mut r = props.reader();
        let ipl = match r.optional_media("ipl")? {
            Some(m) => {
                let bytes = m.bytes();
                let image: [u8; 64] = bytes.try_into().map_err(|_| Error::Config {
                    at: String::from("ipl"),
                    message: alloc::format!(
                        "an SPC700 boot ROM is exactly 64 bytes; `{}` is {}",
                        m.name(),
                        bytes.len()
                    ),
                })?;
                image
            }
            None => CLEAN_IPL,
        };
        let record = r.or("record", false)?;
        r.finish()?;
        let apu = SnesApu::with_ipl(default_dsp(), ipl);
        apu.set_recording(record);
        Ok(apu)
    }

    /// Whether output frames are kept for a host to drain.
    pub fn set_recording(&self, on: bool) {
        self.recording.store(on, Ordering::Relaxed);
    }

    /// Whether output frames are kept.
    #[must_use]
    pub fn recording(&self) -> bool {
        self.recording.load(Ordering::Relaxed)
    }

    /// Drain the output ring: 32 kHz stereo frames, oldest first.
    pub fn take_samples(&self) -> Vec<(i16, i16)> {
        self.ring.lock().drain(..).collect()
    }

    /// The S-CPU reading port `n` (`$2140 + n`).
    #[must_use]
    pub fn cpu_read(&self, port: u8) -> u8 {
        self.ports.to_cpu[usize::from(port & 3)].load(Ordering::Acquire)
    }

    /// The S-CPU writing port `n`.
    pub fn cpu_write(&self, port: u8, value: u8) {
        self.ports.to_smp[usize::from(port & 3)].store(value, Ordering::Release);
    }

    /// The SPC700's register file.
    #[must_use]
    pub fn regs(&self) -> spc700::Regs {
        self.inner.lock().core.regs
    }

    /// SMP cycles since power-on.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.inner.lock().core.cycles
    }

    /// Read the SMP's address space **without side effects** — a debugger,
    /// a disassembler. A timer counter read here is not cleared.
    #[must_use]
    pub fn peek(&self, addr: u16) -> u8 {
        self.with_bus(|bus| bus.decode_read(addr, true))
    }

    /// Read as the SPC700 would, side effects included (one cycle is *not*
    /// charged: nothing is clocked).
    pub fn smp_read(&self, addr: u16) -> u8 {
        self.with_bus(|bus| bus.decode_read(addr, false))
    }

    /// Write as the SPC700 would, side effects included, without clocking.
    pub fn smp_write(&self, addr: u16, value: u8) {
        self.with_bus(|bus| bus.decode_write(addr, value));
    }

    /// Raw audio RAM, under any overlay.
    #[must_use]
    pub fn aram(&self, addr: u16) -> u8 {
        self.inner.lock().aram[usize::from(addr)]
    }

    /// Disassemble `count` instructions at `pc` with side-effect-free reads.
    #[must_use]
    pub fn disassemble(&self, pc: u16, count: usize) -> Vec<spc700::disasm::Disassembled> {
        spc700::disasm::disassemble_run(pc, count, |a| Some(self.peek(a)))
    }

    fn with_bus<T>(&self, f: impl FnOnce(&mut SmpBus<'_>) -> T) -> T {
        let mut inner = self.inner.lock();
        let Inner { aram, io, dsp, .. } = &mut *inner;
        let mut bus = SmpBus {
            aram,
            io,
            dsp: dsp.as_mut(),
            ports: &self.ports,
            ipl: &self.ipl,
            ring: &self.ring,
            recording: self.recording.load(Ordering::Relaxed),
        };
        f(&mut bus)
    }

    /// Execute one SPC700 instruction and return its cycles.
    pub fn step(&self) -> u64 {
        let mut inner = self.inner.lock();
        let Inner {
            core,
            aram,
            io,
            dsp,
            ..
        } = &mut *inner;
        let mut bus = SmpBus {
            aram,
            io,
            dsp: dsp.as_mut(),
            ports: &self.ports,
            ipl: &self.ipl,
            ring: &self.ring,
            recording: self.recording.load(Ordering::Relaxed),
        };
        core.step(&mut bus)
    }

    /// Run a scheduler budget of SMP cycles, carrying the overshoot of the
    /// last instruction into the next budget.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.inner.lock().debt;
        if owed >= ticks {
            self.inner.lock().debt = owed - ticks;
            return ticks;
        }
        let allowance = ticks - owed;
        let mut used = 0;
        while used < allowance {
            used += self.step();
        }
        self.inner.lock().debt = used - allowance;
        ticks
    }

    fn reset_inner(&self, cold: bool) {
        let mut inner = self.inner.lock();
        inner.core = Core::new();
        inner.io = Io::power_on();
        inner.debt = 0;
        inner.dsp.reset();
        if cold {
            inner.aram.fill(0);
        }
        drop(inner);
        for p in self.ports.to_smp.iter().chain(&self.ports.to_cpu) {
            p.store(0, Ordering::Release);
        }
    }
}

/// The S-DSP a described machine gets.
fn default_dsp() -> Box<dyn SDsp> {
    Box::new(super::dsp::SDspChip::new())
}

/// The `snes.apu` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "snes.apu",
    version: 1,
    summary: "SNES S-SMP: SPC700, 64 KiB audio RAM, timers, CPU ports, boot ROM, S-DSP",
    properties: &[
        PropertySpec {
            name: "ipl",
            kind: ValueKind::Media,
            required: false,
            summary: "a real 64-byte SPC700 boot ROM; the default is a clean-room one",
        },
        PropertySpec {
            name: "record",
            kind: ValueKind::Bool,
            required: false,
            summary: "keep 32 kHz output frames in a ring for a host audio sink",
        },
    ],
    construct: |props| Ok(Box::new(SnesApu::from_props(props)?)),
};

/// Add `snes.apu` to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

/// Bind `snes.apu` into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(SnesApu::from_props(props)?))
    })
}

/// What the validator should know about `snes.apu`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("ipl", ValueKind::Media))
        .prop(PropSchema::new("record", ValueKind::Bool))
        .region(CPUIO_REGION)
        .region(ARAM_REGION)
}

impl crate::machine::Instance for SnesApu {}

impl Device for SnesApu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            CPUIO_REGION => Some(Arc::clone(&self.cpuio)),
            ARAM_REGION => Some(Arc::clone(&self.aram_region)),
            _ => None,
        }
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, kind: ResetKind) {
        self.reset_inner(kind == ResetKind::Cold);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let inner = self.inner.lock();
        w.write_u32(STATE_VERSION)?;
        spc700::save_core(&inner.core, w)?;
        w.write_u64(inner.debt)?;
        w.write_bytes(&inner.aram[..])?;
        let io = &inner.io;
        w.write_bytes(&[
            io.test,
            io.control,
            io.dspaddr,
            io.aux[0],
            io.aux[1],
            io.target[0],
            io.target[1],
            io.target[2],
            io.stage[0],
            io.stage[1],
            io.stage[2],
            io.out[0],
            io.out[1],
            io.out[2],
            io.prescale,
        ])?;
        for p in self.ports.to_smp.iter().chain(&self.ports.to_cpu) {
            w.write_u8(p.load(Ordering::Acquire))?;
        }
        inner.dsp.save(w)?;
        // The output ring is not saved: it is audio on its way to a host, and
        // restoring it would replay what has already been heard.
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let version = r.read_u32()?;
        if version != STATE_VERSION {
            return Err(Error::State(alloc::format!(
                "snes.apu snapshot is version {version}, this build reads {STATE_VERSION}"
            )));
        }
        let core = spc700::load_core(r)?;
        let debt = r.read_u64()?;
        let aram = r.read_bytes()?;
        if aram.len() != 0x10000 {
            return Err(Error::State(String::from(
                "snes.apu: audio RAM is not 64 KiB",
            )));
        }
        let b = r.read_bytes()?;
        let [
            test,
            control,
            dspaddr,
            a0,
            a1,
            t0,
            t1,
            t2,
            s0,
            s1,
            s2,
            o0,
            o1,
            o2,
            prescale,
        ] = *b
        else {
            return Err(Error::State(String::from(
                "snes.apu: register block has the wrong size",
            )));
        };
        // The timer prescaler counts SMP cycles modulo 128, and `Io::tick`
        // advances it with `(prescale + 1) & 0x7f` — which overflows before
        // the mask for the one value a `u8` holds that the counter never
        // does. A snapshot is a file somebody can be handed; a prescaler
        // outside its modulus is refused rather than believed.
        if prescale > 0x7f {
            return Err(Error::State(alloc::format!(
                "snes.apu: the timer prescaler is {prescale}, and it counts modulo 128"
            )));
        }
        let mut port = [0u8; 8];
        for p in &mut port {
            *p = r.read_u8()?;
        }
        let mut inner = self.inner.lock();
        inner.dsp.load(r)?;
        inner.core = core;
        inner.debt = debt;
        inner.aram.copy_from_slice(aram);
        inner.io = Io {
            test,
            control,
            dspaddr,
            aux: [a0, a1],
            target: [t0, t1, t2],
            stage: [s0, s1, s2],
            out: [o0, o1, o2],
            prescale,
        };
        drop(inner);
        for (latch, v) in self.ports.to_smp.iter().chain(&self.ports.to_cpu).zip(port) {
            latch.store(v, Ordering::Release);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Media;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn apu() -> SnesApu {
        SnesApu::new(Box::new(NullDsp::default()))
    }

    /// Run the SMP until `cond` holds, panicking after a generous bound.
    fn until(apu: &SnesApu, what: &str, cond: impl Fn(&SnesApu) -> bool) {
        for _ in 0..10_000 {
            if cond(apu) {
                return;
            }
            apu.run_budget(32);
        }
        panic!("timed out waiting for {what}; SMP at {}", apu.regs());
    }

    /// The S-CPU side of the documented upload protocol (SNESdev, *Booting
    /// the SPC700*), written independently of the IPL it talks to.
    fn upload(apu: &SnesApu, blocks: &[(u16, &[u8])], entry: u16) {
        until(apu, "$BBAA", |a| {
            a.cpu_read(0) == 0xaa && a.cpu_read(1) == 0xbb
        });
        let mut kick = 0xccu8;
        for (addr, data) in blocks {
            assert!(!data.is_empty(), "the protocol cannot send an empty block");
            apu.cpu_write(2, *addr as u8);
            apu.cpu_write(3, (*addr >> 8) as u8);
            apu.cpu_write(1, 1);
            apu.cpu_write(0, kick);
            until(apu, "the kick echo", |a| a.cpu_read(0) == kick);
            for (i, b) in data.iter().enumerate() {
                apu.cpu_write(1, *b);
                apu.cpu_write(0, i as u8);
                until(apu, "a byte echo", |a| a.cpu_read(0) == i as u8);
            }
            // The last index plus two, never zero.
            kick = ((data.len() - 1) as u8).wrapping_add(2);
            if kick == 0 {
                kick = 1;
            }
        }
        apu.cpu_write(2, entry as u8);
        apu.cpu_write(3, (entry >> 8) as u8);
        apu.cpu_write(1, 0);
        apu.cpu_write(0, kick);
        until(apu, "the final echo", |a| a.cpu_read(0) == kick);
    }

    #[test]
    fn the_clean_ipl_posts_its_signature() {
        let apu = apu();
        until(&apu, "$AA/$BB", |a| {
            a.cpu_read(0) == 0xaa && a.cpu_read(1) == 0xbb
        });
        assert_eq!(apu.regs().sp, 0xef);
        assert_eq!(apu.peek(0xfffe), 0xc0, "reset vector from the overlay");
        assert_eq!(alloc::format!("{}", apu.disassemble(0xffc0, 1)[0]), "CLRP");
    }

    #[test]
    fn a_host_uploads_a_program_through_the_ports_and_it_runs() {
        let apu = apu();
        // A data block longer than a page, so the IPL's pointer carry runs.
        let data: Vec<u8> = (0..300u32).map(|i| (i * 7) as u8).collect();
        // MOV A,!$042B ; MOV $F7,A ; MOV $F6,#$77 ; BRA $
        let program = [0xe5, 0x2b, 0x04, 0xc4, 0xf7, 0x8f, 0x77, 0xf6, 0x2f, 0xfe];
        upload(&apu, &[(0x0300, &data), (0x0200, &program)], 0x0200);
        apu.run_budget(200);
        for (i, b) in data.iter().enumerate() {
            assert_eq!(apu.aram(0x0300 + i as u16), *b, "byte {i}");
        }
        assert_eq!(apu.cpu_read(2), 0x77);
        assert_eq!(apu.cpu_read(3), data[0x12b], "the program read its data");
        assert_eq!(apu.regs().pc, 0x0208);
    }

    #[test]
    fn the_ports_are_two_latches_each_and_mirror_across_the_window() {
        let apu = apu();
        let ops = CpuPort {
            ports: Arc::clone(&apu.ports),
        };
        ops.write(0x3d, &[0x5a], MemAttrs::DEFAULT).unwrap();
        assert_eq!(apu.smp_read(0xf5), 0x5a, "$217D is port 1");
        apu.smp_write(0xf5, 0xa5);
        let mut b = [0u8];
        ops.read(0x01, &mut b, MemAttrs::DEFAULT).unwrap();
        assert_eq!(b[0], 0xa5);
        assert_eq!(
            apu.smp_read(0xf5),
            0x5a,
            "neither side reads back its own write"
        );
        ops.write(0x00, &[0x11], MemAttrs::DEBUG).unwrap();
        assert_eq!(apu.smp_read(0xf4), 0, "a debug write sends nothing");
    }

    #[test]
    fn control_clears_the_input_latches_and_unmaps_the_rom() {
        let apu = apu();
        for p in 0..4 {
            apu.cpu_write(p, 0x10 + p);
        }
        apu.smp_write(0xf1, 0x10);
        assert_eq!((apu.smp_read(0xf4), apu.smp_read(0xf5)), (0, 0));
        assert_eq!((apu.smp_read(0xf6), apu.smp_read(0xf7)), (0x12, 0x13));
        apu.smp_write(0xf1, 0x20);
        assert_eq!(apu.smp_read(0xf6), 0);
        // Bit 7 is now clear: reads at $FFC0 see RAM.
        apu.smp_write(0xffc0, 0x42);
        assert_eq!(apu.peek(0xffc0), 0x42);
        apu.smp_write(0xf1, 0x80);
        assert_eq!(apu.peek(0xffc0), CLEAN_IPL[0], "the ROM shadows RAM");
        apu.smp_write(0xffc1, 0x43);
        assert_eq!(apu.aram(0xffc1), 0x43, "writes under the ROM reach RAM");
        assert_eq!(apu.peek(0xffc1), CLEAN_IPL[1]);
    }

    #[test]
    fn timers_count_at_their_prescaler_and_reads_clear_them() {
        let apu = apu();
        apu.smp_write(0xfc, 4); // T2: every 4 x 16 cycles
        apu.smp_write(0xfa, 2); // T0: every 2 x 128 cycles
        apu.smp_write(0xf1, 0x85);
        apu.inner.lock().io.prescale = 0;
        let start = apu.cycles();
        while apu.cycles() - start < 1000 {
            apu.step();
        }
        let elapsed = apu.cycles() - start;
        assert_eq!(u64::from(apu.peek(0xff)), (elapsed / 64) & 15);
        assert_eq!(u64::from(apu.peek(0xfd)), elapsed / 256);
        assert_eq!(apu.peek(0xfe), 0, "T1 is not enabled");
        // A debug read leaves the counter alone; a real one clears it.
        let t2 = apu.peek(0xff);
        assert_ne!(t2, 0);
        assert_eq!(apu.peek(0xff), t2);
        assert_eq!(apu.smp_read(0xff), t2);
        assert_eq!(apu.peek(0xff), 0);
        // Rewriting an already-set enable does not restart the timer...
        let out0 = apu.peek(0xfd);
        apu.smp_write(0xf1, 0x85);
        assert_eq!(apu.peek(0xfd), out0);
        // ...but a 0 -> 1 transition does.
        apu.smp_write(0xf1, 0x84);
        apu.smp_write(0xf1, 0x85);
        assert_eq!(apu.inner.lock().io.stage[0], 0);
        assert_eq!(apu.peek(0xfd), 0);
        // Targets and CONTROL are write-only.
        assert_eq!((apu.peek(0xfa), apu.peek(0xf1)), (0, 0));
    }

    #[test]
    fn dspaddr_mirrors_reads_and_drops_writes_above_7f() {
        let apu = apu();
        apu.smp_write(0xf2, 0x0c);
        apu.smp_write(0xf3, 0x7f);
        apu.smp_write(0xf2, 0x8c);
        apu.smp_write(0xf3, 0x01);
        assert_eq!(
            apu.smp_read(0xf3),
            0x7f,
            "$8C reads $0C, and the write was dropped"
        );
        assert_eq!(apu.smp_read(0xf2), 0x8c);
    }

    #[test]
    fn test_register_bit_1_gates_ram_writes() {
        let apu = apu();
        apu.smp_write(0x1234, 1);
        apu.smp_write(0xf0, 0x08);
        apu.smp_write(0x1234, 2);
        assert_eq!(apu.aram(0x1234), 1);
    }

    fn image(apu: &SnesApu) -> Vec<u8> {
        let mut w = StateWriter::new(MachineShape::new());
        {
            let mut c = w.chunk("apu", CLASS.name, CLASS.version).unwrap();
            Device::save(apu, &mut c).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trip_reproduces_the_state_exactly() {
        let a = apu();
        a.smp_write(0xfc, 3);
        a.smp_write(0xf1, 0x84);
        a.cpu_write(2, 0x99);
        a.run_budget(5000);
        let first = image(&a);
        let b = apu();
        let reader = StateReader::new(&first).unwrap();
        let chunk = reader
            .load("apu", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        Device::load(&b, &mut chunk.reader()).unwrap();
        assert_eq!(image(&b), first, "identical state image");
        a.run_budget(3000);
        b.run_budget(3000);
        assert_eq!(image(&a), image(&b), "and identical futures");
        assert_eq!(b.smp_read(0xf6), 0x99);
    }

    #[test]
    fn a_snapshot_with_the_timer_prescaler_outside_its_modulus_is_refused() {
        // The prescaler counts SMP cycles modulo 128 and `Io::tick` steps it
        // with `(prescale + 1) & 0x7f`, which overflows before the mask for
        // 255. With the null DSP the chunk ends: the fifteen-byte register
        // block (the prescaler last), the eight port latches, and the DSP's
        // 128 registers behind their length.
        let a = apu();
        a.run_budget(5000);
        let first = image(&a);
        let reader = StateReader::new(&first).unwrap();
        let (_, _, payload) = reader.load_raw("apu").unwrap();
        let prescale_at = payload.len() - (8 + 128) - 8 - 1;
        assert!(payload[prescale_at] <= 0x7f);

        let try_load = |value: u8| {
            let mut payload = payload.to_vec();
            payload[prescale_at] = value;
            let mut shape = MachineShape::new();
            shape.add_device("apu", CLASS.name).unwrap();
            let mut w = StateWriter::new(shape);
            w.raw_chunk("apu", CLASS.name, CLASS.version, &payload)
                .unwrap();
            let bytes = w.to_vec().unwrap();
            let b = apu();
            let reader = StateReader::new(&bytes).unwrap();
            let chunk = reader
                .load("apu", CLASS.name, CLASS.version, &Migrations::new())
                .unwrap();
            let result = Device::load(&b, &mut chunk.reader());
            if result.is_ok() {
                b.run_budget(3000);
            }
            result
        };
        try_load(0x7f).expect("the last count of the modulus");
        assert!(try_load(0x80).is_err());
        assert!(try_load(0xff).is_err(), "255 + 1 is the overflow");
    }

    #[test]
    fn the_real_dsp_round_trips_too() {
        let a = SnesApu::new(default_dsp());
        a.smp_write(0xf2, 0x0c);
        a.smp_write(0xf3, 0x40);
        a.run_budget(4000);
        let first = image(&a);
        let b = SnesApu::new(default_dsp());
        let reader = StateReader::new(&first).unwrap();
        let chunk = reader
            .load("apu", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        Device::load(&b, &mut chunk.reader()).unwrap();
        assert_eq!(image(&b), first);
    }

    #[test]
    fn recording_collects_one_frame_per_32_cycles() {
        let a = SnesApu::new(default_dsp());
        a.set_recording(true);
        a.run_budget(32 * 100);
        let n = a.take_samples().len() as i64;
        assert!((n - 100).abs() <= 1, "{n} frames for 3200 cycles");
    }

    #[test]
    fn the_ipl_property_takes_exactly_64_bytes() {
        let ok = Props::new().with("ipl", Media::new("ipl.bin", alloc::vec![0u8; 64]));
        assert!(SnesApu::from_props(&ok).is_ok());
        let bad = Props::new().with("ipl", Media::new("ipl.bin", alloc::vec![0u8; 63]));
        assert!(SnesApu::from_props(&bad).is_err());
        assert!(SnesApu::from_props(&Props::new().with("recrod", true)).is_err());
    }

    #[test]
    fn a_cold_reset_reboots_and_clears_ram() {
        let apu = apu();
        apu.smp_write(0x0300, 9);
        apu.run_budget(2000);
        Device::reset(&apu, ResetKind::Warm);
        assert_eq!(apu.aram(0x0300), 9, "a warm reset keeps audio RAM");
        Device::reset(&apu, ResetKind::Cold);
        assert_eq!(apu.aram(0x0300), 0);
        until(&apu, "the signature again", |a| a.cpu_read(0) == 0xaa);
    }
}
