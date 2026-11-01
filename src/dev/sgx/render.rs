//! Drawing a TA/3D command: the SGX's vertex data master, tile accelerator
//! and 3D pipeline, modelled functionally.
//!
//! The stand-in completes a TA kick's sync objects as soon as it takes the
//! command. Before it does, [`render_ta`] performs the work the command
//! describes, so the pixels are in memory when the host is told they are.
//!
//! # Sources
//!
//! As for the rest of this device: the navi's own binaries and memory, never
//! driver source. Every structure below was read off TA commands, PDS
//! programs and USSE programs in snapshots of a running navi, following the
//! data from one to the next and checking each reading against what the
//! program must compute — a full-screen quad's clip coordinates, a texture
//! descriptor whose third word is a mapped surface. What a guess is, the
//! comment says (`[G]`); `[strong]` means several independent uses agree.
//!
//! # What a TA command describes
//!
//! * `+0x248`: the **control stream** (below), read until its `0xc0000000`.
//! * `+0x38`: the render's register block; `+0x80` there is the **pixel
//!   event** PDS program and `+0x84` its data rows. Its end-of-tile branch
//!   starts a USSE program that loads the render target's description into
//!   `r0..r5` and sends it to the pixel back end with `emit158`: `r1` the
//!   address, `r2 + 1` the stride in 8-byte units, `r5` the bounds
//!   (`(height - 1) << 12 | (width - 1)`), `r0` the format \[strong\].
//!
//! # The control stream
//!
//! Words, the top nibble a type \[strong\]:
//!
//! | Type | Words | Meaning |
//! |---|---|---|
//! | 4, 6 | 2 | a state PDS program: address `PDS(w0)`, data rows `w1 >> 27` |
//! | 8, 9 | 6 | a draw: `w0[15:0]` indices, `w1` the u16 index list, `w4`/`w5` the vertex PDS program and its rows (`w5[3:0]`) |
//! | `c` | 1 | end of this command's stream |
//!
//! `PDS(w)` is `0xf000_0000 | (w & 0xff_ffff) << 4` \[strong\]: every PDS
//! pointer seen, vertex- and pixel-side, resolves that way. A 4-index draw of
//! type 9 is a fan, type 8 a triangle list \[G\].
//!
//! State programs send their words to the tile accelerator as one `emit159`
//! burst from `o0`. A full state block (word 3 non-zero) names the pixel
//! programs: word 3 the per-draw (secondary) PDS program, word 5 the
//! per-pixel (primary) one, with `(w5 >> 28) + 1` data rows. Word 0 bit 7
//! puts two more words after word 5, and bit 9 then six viewport floats —
//! x scale, x offset, y offset, y scale, then z \[G\]. Other state bursts
//! (control words such as `0x800`) are ignored.
//!
//! # Vertices
//!
//! The vertex PDS program runs once per index (`IR0` = the index), DMAs the
//! vertex's attributes and starts the vertex shader; the shader's `emit160`
//! burst from `o0` is the vertex: `o0..o3` the clip position, then the
//! varyings, `o4` and `o5` the first texture coordinate \[G\].
//!
//! # Pixels
//!
//! Each covered pixel runs the primary PDS program. Its texture DOUT reads
//! the texel at the interpolated coordinate (nearest) and returns it in
//! `pa0..pa1` as four f16 lanes in memory byte order, which the UI's pixel
//! shaders convert to unorm8 with `pck` \[G\]; its USSE task runs with `o0`
//! holding the pixel's current colour, and leaves the new one there — the
//! UI's shaders blend in the shader (`blend2 o0, pa0, o0, …`).
//!
//! The tile buffer starts as the render target's current contents: the
//! background object of every command seen reloads the target \[G\]. At the
//! end the buffer is written back in the target's format, taken as 32 bits
//! a pixel \[G\].

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use super::pds::{Dout, DoutType, Pds, PdsBus};
use super::raster::{self, Rect, Vertex};
use super::usse::{self, Emit, Size, Usse, UsseBus};
use crate::core::error::BusError;
use crate::core::space::MemResult;

/// TA command fields.
const CMD_REGS: u32 = 0x38;
const CMD_STREAM: u32 = 0x248;
/// Render register block fields.
const REGS_EVENT_PDS: u32 = 0x80;
const REGS_EVENT_ROWS: u32 = 0x84;

/// `emit` forms (`usse::Emit::form`).
const FORM_PBE: u8 = usse::EMIT_158;

/// Budgets: a runaway program stops rather than hanging the device.
const PDS_BUDGET: u64 = 4096;
const USSE_BUDGET: u64 = 1 << 16;
const STREAM_WORDS: u32 = 1 << 16;
/// The largest target this draws, in pixels.
const MAX_PIXELS: usize = 2048 * 2048;

/// What drawing a command did, for the log.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Summary {
    /// The render target's address.
    pub target: u32,
    /// Its size.
    pub width: u32,
    /// Its size.
    pub height: u32,
    /// Draws executed.
    pub draws: u32,
    /// Pixels shaded.
    pub pixels: u64,
    /// Programs that stopped on something other than their end.
    pub faults: u32,
    /// The first of them: the program's address and why it stopped.
    pub first_fault: Option<Fault>,
}

/// Why a program stopped early.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    /// The PDS program at `at` stopped.
    Pds {
        /// The program.
        at: u32,
        /// Why.
        stop: super::pds::Stop,
    },
    /// The USSE program started at `at` stopped.
    Usse {
        /// The program's first instruction.
        at: u32,
        /// Why.
        stop: usse::Stop,
    },
}

/// GPU-virtual memory as one context sees it, through its MMU.
pub trait GpuMemory {
    /// The word at `va`, if it is mapped.
    fn read(&self, va: u32) -> Option<u32>;
    /// Writes the word at `va`, if it is mapped.
    fn write(&self, va: u32, value: u32);
}

/// The PDS address a stream or state word names.
fn pds_address(w: u32) -> u32 {
    0xf000_0000 | (w & 0x00ff_ffff) << 4
}

/// Converts a value in `0.0..=1.0` to an f16 bit pattern.
fn f16_unorm(v: f32) -> u32 {
    if v <= 0.0 {
        return 0;
    }
    if v >= 1.0 {
        return 0x3c00;
    }
    let b = v.to_bits();
    let exp = ((b >> 23) & 0xff) as i32 - 127 + 15;
    if exp <= 0 {
        return 0;
    }
    let man = b & 0x7f_ffff;
    let mut h = (exp as u32) << 10 | man >> 13;
    let rem = man & 0x1fff;
    if rem > 0x1000 || (rem == 0x1000 && h & 1 == 1) {
        h += 1;
    }
    h
}

/// The engine-facing side of a program run: memory through the context's
/// MMU, and whatever the programs send out.
struct Bus<'a> {
    gpu: &'a dyn GpuMemory,
    primary: [u32; usse::BANK_SIZE],
    secondary: &'a mut [u32; usse::BANK_SIZE],
    /// DMAs fill the secondary attributes too (state programs).
    to_secondary: bool,
    /// The answer to every PDS condition: true for vertex programs (their
    /// final `DOUTU` is conditional), false for the pixel event program,
    /// whose code 3/4 branches lead away from the end-of-tile work \[G\].
    cond: bool,
    tasks: Vec<u32>,
    emits: Vec<(Emit, [u32; 32])>,
    tex_coord: [f32; 2],
}

impl PdsBus for Bus<'_> {
    fn load(&mut self, addr: u32) -> MemResult<u32> {
        self.gpu.read(addr).ok_or(BusError::Unassigned)
    }

    fn dout(&mut self, out: &Dout) -> MemResult {
        let [w0, w1, w2, _] = out.words;
        match out.kind {
            DoutType::DMA => {
                let first = w1 >> 8 & 0xfff;
                for i in 0..=(w1 & 0xff) {
                    let v = PdsBus::load(self, w0.wrapping_add(4 * i))?;
                    let r = (first + i) as usize;
                    if let Some(p) = self.primary.get_mut(r) {
                        *p = v;
                    }
                    if self.to_secondary
                        && let Some(s) = self.secondary.get_mut(r)
                    {
                        *s = v;
                    }
                }
            }
            DoutType::ATTR => {
                if let Some(p) = self.primary.get_mut((w1 >> 8 & 0xfff) as usize) {
                    *p = w0;
                }
            }
            DoutType::USSE => self.tasks.push(w0),
            DoutType::TEXTURE => self.texture(w1, w2),
            // Iterators: the coordinate comes from the rasteriser.
            _ => {}
        }
        Ok(())
    }

    fn accepts_dout(&self, _kind: DoutType) -> bool {
        true
    }

    fn condition(&mut self, _cc: u8, _pc: u32) -> Option<bool> {
        Some(self.cond)
    }
}

impl Bus<'_> {
    /// A texture fetch: `size` is `format[31:24] (width - 1)[23:12]
    /// (height - 1)[11:0]`, `addr` the first texel; 32-bit texels, the row
    /// stride the width \[G\].
    fn texture(&mut self, size: u32, addr: u32) {
        let (w, h) = (((size >> 12) & 0xfff) + 1, (size & 0xfff) + 1);
        let pick = |c: f32, n: u32| -> u32 {
            let v = c * n as f32;
            if v.is_nan() || v < 0.0 {
                0
            } else {
                (v as u32).min(n - 1)
            }
        };
        let (x, y) = (pick(self.tex_coord[0], w), pick(self.tex_coord[1], h));
        let t = self
            .gpu
            .read(addr.wrapping_add(4 * (y * w + x)))
            .unwrap_or(0);
        let lane = |k: u32| f16_unorm(((t >> (8 * k)) & 0xff) as f32 / 255.0);
        self.primary[0] = lane(0) | lane(1) << 16;
        self.primary[1] = lane(2) | lane(3) << 16;
    }
}

impl UsseBus for Bus<'_> {
    fn load(&mut self, addr: u32, size: Size) -> MemResult<u32> {
        let w = self.gpu.read(addr & !3).ok_or(BusError::Unassigned)?;
        let sh = (addr & 3) * 8;
        Ok(match size {
            Size::Byte => w >> sh & 0xff,
            Size::Word => w >> sh & 0xffff,
            Size::Dword => w,
        })
    }

    fn store(&mut self, addr: u32, size: Size, value: u32) -> MemResult {
        if size != Size::Dword || addr & 3 != 0 {
            return Err(BusError::BadAccess);
        }
        self.gpu.write(addr, value);
        Ok(())
    }

    fn reg_read(&mut self, _index: u32) -> u32 {
        0
    }

    fn reg_write(&mut self, _index: u32, _value: u32) {}

    fn emit(&mut self, emit: &Emit) -> MemResult {
        self.emits.push((*emit, [0; 32]));
        Ok(())
    }

    fn emit_from(&mut self, emit: &Emit, u: &Usse) -> MemResult {
        // The bursts are the outputs from o0 on \[G\].
        let mut regs = [0u32; 32];
        regs.copy_from_slice(&u.output[..32]);
        self.emits.push((*emit, regs));
        Ok(())
    }
}

/// One drawing session: a command's memory view and running state.
struct Renderer<'a> {
    gpu: &'a dyn GpuMemory,
    code_bases: [u32; 16],
    secondary: Box<[u32; usse::BANK_SIZE]>,
    summary: Summary,
}

/// What a program run left behind.
struct Ran {
    emits: Vec<(Emit, [u32; 32])>,
    ok: bool,
}

impl Renderer<'_> {
    /// Runs a PDS program and the USSE tasks it starts. `o` is the first
    /// four output registers, in and out.
    fn run(&mut self, at: u32, rows: u32, ir: [u32; 3], kind: Kind, o: &mut [u32; 4]) -> Ran {
        let mut bus = Bus {
            gpu: self.gpu,
            primary: [0; usse::BANK_SIZE],
            secondary: &mut self.secondary,
            to_secondary: kind == Kind::State,
            cond: kind == Kind::Vertex,
            tasks: Vec::new(),
            emits: Vec::new(),
            tex_coord: [0.0; 2],
        };
        if let Kind::Pixel(s, t) = kind {
            bus.tex_coord = [s, t];
        }
        let mut pds = Pds::default();
        let stop = match pds.launch(&mut bus, at, rows, ir) {
            Ok(()) => pds.run(&mut bus, PDS_BUDGET),
            Err(stop) => stop,
        };
        let mut fault = match stop {
            super::pds::Stop::Halt { .. } => None,
            stop => Some(Fault::Pds { at, stop }),
        };
        for w0 in core::mem::take(&mut bus.tasks) {
            let base = self.code_bases[(w0 & 0xf) as usize];
            let mut u = Box::new(Usse::new(base, w0 >> 4 & 0xf_ffff));
            u.primary = bus.primary;
            u.secondary = *bus.secondary;
            u.output[..4].copy_from_slice(o);
            let mut budget = USSE_BUDGET;
            let start = u.address_of(u.pc);
            loop {
                let before = u.retired;
                let stop = u.run(&mut bus, budget);
                budget = budget.saturating_sub(u.retired - before);
                match stop {
                    usse::Stop::Phase { next: Some(n), .. } if n != 0 && budget > 0 => u.pc = n,
                    usse::Stop::Phase { next: None, .. } if budget > 0 => {}
                    usse::Stop::Phase { .. } | usse::Stop::End { .. } => break,
                    stop => {
                        fault = fault.or(Some(Fault::Usse { at: start, stop }));
                        break;
                    }
                }
            }
            o.copy_from_slice(&u.output[..4]);
        }
        if fault.is_some() {
            self.summary.faults += 1;
            self.summary.first_fault = self.summary.first_fault.or(fault);
        }
        Ran {
            emits: bus.emits,
            ok: fault.is_none(),
        }
    }
}

/// Which kind of program a run is, for what its DOUTs and conditions mean.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Kind {
    /// A state program from the stream.
    State,
    /// A vertex program.
    Vertex,
    /// The pixel event program.
    Event,
    /// A pixel's program, with its texture coordinate.
    Pixel(f32, f32),
}

/// The render target, from the pixel back end's `emit158`.
#[derive(Clone, Copy, Debug)]
struct Target {
    addr: u32,
    stride: u32,
    width: u32,
    height: u32,
}

/// Draws the TA command at `cmd` (a kernel virtual address readable through
/// `gpu`, the issuing context's MMU), with the USE code bases the host
/// programmed. Returns what it did, or `None` if the command did not
/// describe a render this model understands.
///
/// Public so that a command can be replayed against a snapshot's memory,
/// outside a running machine.
pub fn render_ta(gpu: &dyn GpuMemory, cmd: u32, code_bases: [u32; 16]) -> Option<Summary> {
    let mut r = Renderer {
        gpu,
        code_bases,
        secondary: Box::new([0; usse::BANK_SIZE]),
        summary: Summary::default(),
    };
    let regs = gpu.read(cmd + CMD_REGS)?;
    let stream = gpu.read(cmd + CMD_STREAM)?;
    let target = r.target(regs)?;
    r.summary.target = target.addr;
    r.summary.width = target.width;
    r.summary.height = target.height;
    let (w, h) = (target.width as usize, target.height as usize);
    if w * h > MAX_PIXELS {
        return None;
    }
    let pixel = |x: usize, y: usize| target.addr + y as u32 * target.stride + 4 * x as u32;
    let mut tile: Vec<u32> = vec![0; w * h];
    for (i, t) in tile.iter_mut().enumerate() {
        *t = gpu.read(pixel(i % w, i / w)).unwrap_or(0);
    }
    r.walk(stream, &mut tile, w, h);
    for (i, t) in tile.iter().enumerate() {
        gpu.write(pixel(i % w, i / w), *t);
    }
    Some(r.summary)
}

impl Renderer<'_> {
    /// Runs the pixel event program and reads the target off its `emit158`.
    fn target(&mut self, regs: u32) -> Option<Target> {
        let at = self.gpu.read(regs + REGS_EVENT_PDS)?;
        let rows = self.gpu.read(regs + REGS_EVENT_ROWS)?;
        let ran = self.run(at, rows, [0; 3], Kind::Event, &mut [0; 4]);
        let (e, _) = ran.emits.iter().find(|(e, _)| e.form == FORM_PBE)?;
        let [[_format, addr], [stride, _], [_, bounds]] = e.sources;
        Some(Target {
            addr,
            stride: (stride + 1) * 8,
            width: (bounds & 0xfff) + 1,
            height: (bounds >> 12 & 0xfff) + 1,
        })
    }

    /// Walks the control stream at `at`, drawing into `tile`.
    fn walk(&mut self, mut at: u32, tile: &mut [u32], w: usize, h: usize) {
        let mut state = [0u32; 32];
        // [G] Until a state block sets one: the full target.
        let mut viewport = [
            w as f32 / 2.0,
            w as f32 / 2.0,
            h as f32 / 2.0,
            -(h as f32) / 2.0,
            0.5,
            0.5,
        ];
        let end = at.wrapping_add(4 * STREAM_WORDS);
        while at != end {
            let Some(w0) = self.gpu.read(at) else { return };
            match w0 >> 28 {
                4 | 6 => {
                    let w1 = self.gpu.read(at + 4).unwrap_or(0);
                    let ran = self.run(pds_address(w0), w1 >> 27, [0; 3], Kind::State, &mut [0; 4]);
                    for (e, regs) in &ran.emits {
                        if e.form != FORM_PBE && regs[3] != 0 {
                            state = *regs;
                            let mut k = 6;
                            if state[0] & 0x80 != 0 {
                                k += 2;
                            }
                            if state[0] & 0x200 != 0 {
                                for (j, v) in viewport.iter_mut().enumerate() {
                                    *v = f32::from_bits(state[k + j]);
                                }
                            }
                        }
                    }
                    at = at.wrapping_add(8);
                }
                8 | 9 => {
                    let mut d = [0u32; 6];
                    for (i, v) in d.iter_mut().enumerate() {
                        *v = self.gpu.read(at + 4 * i as u32).unwrap_or(0);
                    }
                    self.draw(&d, &state, &viewport, tile, w, h);
                    at = at.wrapping_add(24);
                }
                0xc => return,
                _ => at = at.wrapping_add(4),
            }
        }
    }

    /// One draw.
    fn draw(
        &mut self,
        d: &[u32; 6],
        state: &[u32; 32],
        vp: &[f32; 6],
        tile: &mut [u32],
        w: usize,
        h: usize,
    ) {
        self.summary.draws += 1;
        let count = (d[0] & 0xffff).min(4096);
        let (vpds, vrows) = (pds_address(d[4]), d[5] & 0xf);
        let mut verts = Vec::with_capacity(count as usize);
        for i in 0..count {
            let a = d[1].wrapping_add(2 * i);
            let word = self.gpu.read(a & !3).unwrap_or(0);
            let index = if a & 2 != 0 {
                word >> 16
            } else {
                word & 0xffff
            };
            let ran = self.run(vpds, vrows, [index, 0, 0], Kind::Vertex, &mut [0; 4]);
            let Some((_, o)) = ran.emits.iter().find(|(e, _)| e.form != FORM_PBE) else {
                verts.push(None);
                continue;
            };
            let f = |k: usize| f32::from_bits(o[k]);
            let cw = f(3);
            if !ran.ok || cw == 0.0 || cw.is_nan() {
                verts.push(None);
                continue;
            }
            let mut v = Vertex {
                x: f(0) / cw * vp[0] + vp[1],
                y: f(1) / cw * vp[3] + vp[2],
                z: f(2) / cw * vp[4] + vp[5],
                inv_w: 1.0 / cw,
                ..Vertex::default()
            };
            for (k, a) in v.attrs.iter_mut().enumerate().take(8) {
                *a = f(4 + k);
            }
            verts.push(Some(v));
        }
        let tris: Vec<[Option<Vertex>; 3]> = if d[0] >> 28 == 9 {
            (1..verts.len().saturating_sub(1))
                .map(|i| [verts[0], verts[i], verts[i + 1]])
                .collect()
        } else {
            verts.as_chunks::<3>().0.to_vec()
        };
        let (ppds, prows) = (pds_address(state[5]), (state[5] >> 28) + 1);
        if state[3] == 0 {
            return;
        }
        let clip = Rect {
            x0: 0,
            y0: 0,
            x1: w as i32,
            y1: h as i32,
        };
        for t in tris {
            let [Some(a), Some(b), Some(c)] = t else {
                continue;
            };
            raster::triangle(&[a, b, c], 8, clip, |fr| {
                let i = fr.y as usize * w + fr.x as usize;
                let mut o = [tile[i], 0, 0, 0];
                self.run(
                    ppds,
                    prows,
                    [0; 3],
                    Kind::Pixel(fr.attrs[0], fr.attrs[1]),
                    &mut o,
                );
                tile[i] = o[0];
                self.summary.pixels += 1;
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pds_pointers_resolve_into_the_pds_heap() {
        // A vertex-side stream word, and a pixel state word whose bits 27:24
        // are not part of the address.
        assert_eq!(pds_address(0x4f24_0e00), 0xf240_e000);
        assert_eq!(pds_address(0x1703_c80c), 0xf03c_80c0);
        assert_eq!(pds_address(0x0f01_c618), 0xf01c_6180);
    }

    #[test]
    fn unorm_values_become_f16() {
        assert_eq!(f16_unorm(0.0), 0);
        assert_eq!(f16_unorm(1.0), 0x3c00);
        assert_eq!(f16_unorm(0.5), 0x3800);
        // Every unorm8 level survives the trip through f16.
        for v in 0..=255u32 {
            let h = f16_unorm(v as f32 / 255.0);
            let (e, m) = ((h >> 10) as i32, h & 0x3ff);
            let back = if h == 0 {
                0.0
            } else {
                (1.0 + m as f32 / 1024.0) * libm_pow2(e - 15)
            };
            assert_eq!((back * 255.0 + 0.5) as u32, v);
        }
    }

    fn libm_pow2(e: i32) -> f32 {
        f32::from_bits(((e + 127) as u32) << 23)
    }
}
