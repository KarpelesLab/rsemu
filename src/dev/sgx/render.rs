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
use alloc::format;
use alloc::string::String;
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
    /// The pixel's interpolated varyings (vertex outputs `o4` on).
    varyings: [f32; 8],
    /// The next primary attribute an iterator or texture fetch fills.
    next_attr: usize,
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
            DoutType::TEXTURE => self.texture([w0, w1, w2, out.words[3]]),
            // [G] The iterators load the varyings as F16 lanes: the first
            // (o4..o7) into pa0..pa1, the second (o8..o11) into pa2..pa3. A
            // texture DOUT after them replaces pa0..pa1 with the texel. The
            // UI's untextured shader packs pa0 as its colour; the textured
            // one scales the texel's alpha by pa2's.
            DoutType::ITER => self.iterate(w0),
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
    /// The primary attributes iterators and texture fetches fill are
    /// allocated in issue order from `pa0` \[G\], which every pixel program
    /// in the UI agrees with:
    ///
    /// * an iterator whose word has a non-zero top byte (`0x2fc0000f`,
    ///   `0x2fc01900`, `0x1fc01900`) writes a varying as four F16 lanes, two
    ///   registers: the first varying (`o4..o7`) when the word iterates
    ///   nothing else (`[11:8]` clear), else the second (`o8..o11`) — the
    ///   first being the texture coordinate. The transfer queue's
    ///   `0x0000f800` only feeds the coordinate and writes nothing;
    /// * a texture fetch writes after it.
    ///
    /// So the untextured shader packs its colour from `pa0`, the glyph and
    /// icon shaders take the vertex colour from `pa0` and the mask's alpha
    /// from `pa2.w`, the compositor blends the layer it samples from `pa2`,
    /// and the transfer queue's copy moves the texel from `pa0`.
    fn iterate(&mut self, word: u32) {
        if word >> 24 == 0 {
            return;
        }
        let from = if word & 0xf00 == 0 { 0 } else { 4 };
        let v = self.varyings;
        let h = |k: usize| f16_unorm(v[from + k]);
        self.attr(h(0) | h(1) << 16);
        self.attr(h(2) | h(3) << 16);
        // [G] The same varying, as four unorm8 bytes, at pa64: the UI's
        // texturing shaders modulate the texel by it with the byte-wise
        // blend (`blend o0, pa2, pa64, src2-colour, zero`), the vertex colour
        // of GL's MODULATE; the compositor's is white, a plain copy.
        let byte = |k: usize| {
            let c = v[from + k];
            if c.is_nan() || c <= 0.0 {
                0
            } else if c >= 1.0 {
                255
            } else {
                (c * 255.0 + 0.5) as u32
            }
        };
        self.primary[64] = byte(0) | byte(1) << 8 | byte(2) << 16 | byte(3) << 24;
    }

    fn attr(&mut self, v: u32) {
        if let Some(p) = self.primary.get_mut(self.next_attr) {
            *p = v;
        }
        self.next_attr += 1;
    }

    /// A texture fetch from the four-word descriptor (`T0..T3`, see
    /// [`Texture::decode`]), at the pixel's interpolated coordinate, nearest
    /// texel. With `T3[31:28]` = `0xe` (the 8-bit masks the UI's glyphs and
    /// icons use) it returns four F16 lanes, two registers, which those
    /// shaders scale in F16; otherwise the raw texel, one register, which
    /// the compositor's byte-wise `blend` and the transfer queue's copy use
    /// as it is \[G\].
    fn texture(&mut self, t: [u32; 4]) {
        let tex = Texture::decode(t);
        let pick = |c: f32, n: u32| -> u32 {
            let v = c * n as f32;
            if v.is_nan() || v < 0.0 {
                0
            } else {
                (v as u32).min(n.max(1) - 1)
            }
        };
        let (x, y) = (
            pick(self.tex_coord[0], tex.width),
            pick(self.tex_coord[1], tex.height),
        );
        let v = tex.surface.read(self.gpu, x, y);
        if t[3] >> 28 != 0xe {
            self.attr(v);
            return;
        }
        let lanes: [u32; 4] = match tex.surface.bytes {
            // [G] A one-channel texel feeds every lane.
            1 => [v; 4],
            _ => core::array::from_fn(|k| v >> (8 * k) & 0xff),
        };
        let lane = |k: usize| f16_unorm(lanes[k] as f32 / 255.0);
        self.attr(lane(0) | lane(1) << 16);
        self.attr(lane(2) | lane(3) << 16);
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
    /// Lines describing the work, when the caller asked for them.
    trace: Option<Vec<String>>,
}

impl Renderer<'_> {
    fn note(&mut self, f: impl FnOnce() -> String) {
        if let Some(t) = &mut self.trace {
            t.push(f());
        }
    }
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
            varyings: [0.0; 8],
            next_attr: 0,
        };
        if let Kind::Pixel(v) = kind {
            bus.tex_coord = [v[0], v[1]];
            bus.varyings = v;
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
        let emits = bus.emits;
        if let Some(f) = fault {
            self.note(|| format!("  fault {f:x?}"));
            self.summary.faults += 1;
            self.summary.first_fault = self.summary.first_fault.or(fault);
        }
        Ran {
            emits,
            ok: fault.is_none(),
        }
    }
}

/// A state block as the tile accelerator receives it \[G\]: word 0 a
/// header whose bits say which words follow —
///
/// * bit 0, bit 1: one word each (`0x01d00200`, `0x0000f000` in the UI);
/// * always: the per-draw PDS program, a word of its size, and the
///   per-pixel PDS program (`(w >> 28) + 1` data rows);
/// * bit 7: two more words;
/// * bit 9: six viewport floats: x scale, x offset, y offset, y scale, z
///   scale, z offset.
///
/// Read off the four header forms the UI sends (`0xdfc3`, `0xd4c3`,
/// `0xc440`, `0x0040`), whose per-pixel program and viewport only land on
/// sensible values this way. Bit 6 is set in every state block and in none
/// of the other words the state helpers emit (`0x800`, `0x2000`).
#[derive(Clone, Copy, Debug, PartialEq)]
struct StateBlock {
    pixel_pds: u32,
    viewport: Option<[f32; 6]>,
}

impl StateBlock {
    fn parse(w: &[u32; 32]) -> Option<StateBlock> {
        let hdr = w[0];
        if hdr & 0x40 == 0 || hdr >> 16 != 0 {
            return None;
        }
        let mut k = 1 + (hdr & 1) as usize + (hdr >> 1 & 1) as usize;
        let pixel_pds = w[k + 2];
        k += 3;
        if hdr & 0x80 != 0 {
            k += 2;
        }
        let viewport =
            (hdr & 0x200 != 0).then(|| core::array::from_fn(|j| f32::from_bits(w[k + j])));
        Some(StateBlock {
            pixel_pds,
            viewport,
        })
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
    Pixel([f32; 8]),
}

/// The render target, from the pixel back end's `emit158`.
#[derive(Clone, Copy, Debug)]
struct Target {
    surface: Surface,
    /// The first pixel the render covers.
    x0: u32,
    /// The first pixel the render covers.
    y0: u32,
    /// One past the last.
    x1: u32,
    /// One past the last.
    y1: u32,
}

impl Target {
    /// The pixel back end's words, `emit158 #0, r0, r2, r4` \[strong\]:
    ///
    /// * `r0`: the format in `[31:28]` (8: 8 bits a pixel; 0: 32; 3–6: 16
    ///   \[G\]) and the layout in `[27:26]` (0 strided, 1 tiled, 2
    ///   twiddled);
    /// * `r1`: the address (`[1:0]` a rotation, not modelled);
    /// * `r2[13:0] + 1`: half the stride, in pixels;
    /// * `r4`: the first pixel, `x[11:0] y[23:12]`, and for a twiddled
    ///   surface `log2(w/16)[27:24] log2(h/16)[31:28]`;
    /// * `r5`: the last pixel, inclusive, the same way.
    fn from_pbe([[r0, r1], [r2, _], [r4, r5]]: [[u32; 2]; 3]) -> Target {
        let format = r0 >> 28;
        Target {
            surface: Surface {
                addr: r1 & !3,
                layout: match r0 >> 26 & 3 {
                    1 => Layout::Tiled,
                    2 => Layout::Twiddled,
                    _ => Layout::Strided,
                },
                bytes: format_bytes(format == 8, (3..=6).contains(&format)),
                stride: ((r2 & 0x3fff) + 1) * 2,
                log2: ((r4 >> 24 & 0xf) + 4, (r4 >> 28 & 0xf) + 4),
            },
            x0: r4 & 0xfff,
            y0: r4 >> 12 & 0xfff,
            x1: (r5 & 0xfff) + 1,
            y1: (r5 >> 12 & 0xfff) + 1,
        }
    }
}

/// How a surface's pixels are laid out in memory.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Layout {
    /// Rows of `stride` pixels.
    Strided,
    /// 32 × 32 tiles, row after row of them, `stride` pixels a row of
    /// tiles; within a tile, rows of 32 \[G\].
    Tiled,
    /// Morton order: x and y bits interleaved, y the lower, over the
    /// smaller dimension, the larger's remaining bits above \[G\].
    Twiddled,
}

/// A surface in GPU memory: where pixel `(x, y)` is.
#[derive(Clone, Copy, Debug)]
struct Surface {
    addr: u32,
    layout: Layout,
    /// Bytes a pixel: 1, 2 or 4.
    bytes: u32,
    /// Pixels a row (strided and tiled).
    stride: u32,
    /// log2 of the width and height (twiddled).
    log2: (u32, u32),
}

impl Surface {
    fn offset(&self, x: u32, y: u32) -> u32 {
        let index = match self.layout {
            Layout::Strided => y.wrapping_mul(self.stride).wrapping_add(x),
            Layout::Tiled => {
                let per_row = self.stride.div_ceil(32);
                ((y / 32) * per_row + x / 32) * 1024 + (y % 32) * 32 + x % 32
            }
            Layout::Twiddled => twiddle(x, y, self.log2),
        };
        self.addr.wrapping_add(index.wrapping_mul(self.bytes))
    }

    fn read(&self, gpu: &dyn GpuMemory, x: u32, y: u32) -> u32 {
        let at = self.offset(x, y);
        let word = gpu.read(at & !3).unwrap_or(0);
        match self.bytes {
            1 => word >> (8 * (at & 3)) & 0xff,
            2 => word >> (8 * (at & 2)) & 0xffff,
            _ => word,
        }
    }

    fn write(&self, gpu: &dyn GpuMemory, x: u32, y: u32, v: u32) {
        let at = self.offset(x, y);
        if self.bytes == 4 {
            gpu.write(at, v);
            return;
        }
        let (shift, mask) = match self.bytes {
            1 => (8 * (at & 3), 0xff),
            _ => (8 * (at & 2), 0xffff),
        };
        let word = gpu.read(at & !3).unwrap_or(0);
        gpu.write(at & !3, word & !(mask << shift) | (v & mask) << shift);
    }
}

/// The Morton index of `(x, y)` in a `2^lw × 2^lh` surface \[G\].
fn twiddle(x: u32, y: u32, (lw, lh): (u32, u32)) -> u32 {
    let m = lw.min(lh);
    let mut index = 0;
    for k in 0..m {
        index |= (y >> k & 1) << (2 * k) | (x >> k & 1) << (2 * k + 1);
    }
    let rest = if lw > lh { x >> m } else { y >> m };
    index | rest << (2 * m)
}

/// Bytes a pixel for a format field \[G beyond 8 and 32 bits\].
fn format_bytes(eight: bool, sixteen: bool) -> u32 {
    if eight {
        1
    } else if sixteen {
        2
    } else {
        4
    }
}

/// A texture, from its four descriptor words:
///
/// * `T1[31:30]` the layout — 0 twiddled with `log2(w)` in `[19:16]` and
///   `log2(h)` in `[3:0]`; otherwise `(w − 1)[23:12]` and `(h − 1)[11:0]`,
///   and 1 strided at the width \[G\], 2 tiled (rows of tiles the width
///   rounded up to 32), 3 strided at `T0`'s stride;
/// * `T1[28:24]` the format: 0 is 8 bits a texel, `0x0c` 32, `0x02..0x05`
///   16 \[G\];
/// * `T2` the address; `T3` format bits not modelled;
/// * `T0` the sampler and, for layout 3, the stride: `v = bytes/4 − 1`,
///   `v[2:0]` in `[11:9]`, `v[12:3]` in `[26:17]`, `v[14:13]` in `[2:1]`.
///
/// \[strong\]: the transfer queue's writer in `libsrv_um`, checked against
/// strides of 356, 2048 and 4096 bytes and every texture the UI samples.
#[derive(Clone, Copy, Debug)]
struct Texture {
    surface: Surface,
    width: u32,
    height: u32,
}

impl Texture {
    fn decode([t0, t1, t2, _]: [u32; 4]) -> Texture {
        let format = t1 >> 24 & 0x1f;
        let bytes = format_bytes(format == 0, (2..=5).contains(&format));
        let layout = t1 >> 30;
        let (width, height, log2) = if layout == 0 {
            let (lw, lh) = (t1 >> 16 & 0xf, t1 & 0xf);
            (1 << lw, 1 << lh, (lw, lh))
        } else {
            ((t1 >> 12 & 0xfff) + 1, (t1 & 0xfff) + 1, (0, 0))
        };
        let stride = match layout {
            3 => {
                let v = (t0 >> 9 & 7) | (t0 >> 17 & 0x3ff) << 3 | (t0 >> 1 & 3) << 13;
                (v + 1) * 4 / bytes
            }
            2 => width.next_multiple_of(32),
            _ => width,
        };
        Texture {
            surface: Surface {
                addr: t2 & !3,
                layout: match layout {
                    0 => Layout::Twiddled,
                    2 => Layout::Tiled,
                    _ => Layout::Strided,
                },
                bytes,
                stride,
                log2,
            },
            width,
            height,
        }
    }
}

/// Draws the TA command at `cmd` (a kernel virtual address readable through
/// `gpu`, the issuing context's MMU), with the USE code bases the host
/// programmed. Returns what it did, or `None` if the command did not
/// describe a render this model understands.
///
/// Public so that a command can be replayed against a snapshot's memory,
/// outside a running machine.
pub fn render_ta(gpu: &dyn GpuMemory, cmd: u32, code_bases: [u32; 16]) -> Option<Summary> {
    render(gpu, cmd, code_bases, None).0
}

/// [`render_ta`], also describing the work line by line: each state block,
/// each draw's vertices and pixel program, and the first pixel's outcome.
pub fn render_ta_traced(
    gpu: &dyn GpuMemory,
    cmd: u32,
    code_bases: [u32; 16],
) -> (Option<Summary>, Vec<String>) {
    let (s, t) = render(gpu, cmd, code_bases, Some(Vec::new()));
    (s, t.unwrap_or_default())
}

/// Performs the transfer (blit) command at `cmd`: a render with no tile
/// accelerator, whose one rectangle samples the source and whose pixel back
/// end writes the destination \[strong\]:
///
/// * `+0x00[24:0] << 4`, plus `+0x18 & 0xfff0_0000`, is a short ISP stream:
///   the per-draw PDS program, a control word, the per-pixel PDS program
///   (its texture DOUT is the source), then for each source the `(u, v)` of
///   three vertices, `2·sources, 0`, and three vertices `x << 16 | y`, z —
///   `(c + 1024) << 4` fixed point, or `(c + 2048) << 3` when `+0x3c` is
///   set — in the order `(x0, y0) (x1, y0) (x0, y1)`;
/// * `+0x24`/`+0x28` the pixel-event program and its rows, whose end-of-tile
///   branch emits the destination as for a TA render.
///
/// Read off `libsrv_um`'s transfer-queue writer and the commands in a
/// snapshot. Each destination pixel inside the rectangle runs the per-pixel
/// program with `(u, v)` interpolated linearly and `o0` the destination's
/// current value.
pub fn render_transfer(gpu: &dyn GpuMemory, cmd: u32, code_bases: [u32; 16]) -> Option<Summary> {
    let mut r = Renderer {
        gpu,
        code_bases,
        secondary: Box::new([0; usse::BANK_SIZE]),
        summary: Summary::default(),
        trace: None,
    };
    r.transfer(cmd)
}

impl Renderer<'_> {
    fn transfer(&mut self, cmd: u32) -> Option<Summary> {
        let g = self.gpu;
        let word = |o: u32| g.read(cmd.wrapping_add(o));
        let isp = (word(0x18)? & 0xfff0_0000).wrapping_add((word(0)? & 0x01ff_ffff) << 4);
        let fine = word(0x3c)? != 0;
        let ran = self.run(word(0x24)?, word(0x28)?, [0; 3], Kind::Event, &mut [0; 4]);
        let target = Target::from_pbe(
            ran.emits
                .iter()
                .find(|(e, _)| e.form == FORM_PBE)?
                .0
                .sources,
        );
        let s = |i: u32| g.read(isp.wrapping_add(4 * i)).unwrap_or(0);
        let f = |i: u32| f32::from_bits(s(i));
        let (secondary, primary) = (s(0), s(2));
        // The count follows the coordinates; a source's own (0, 0) would also
        // read as "no sources", so the largest consistent count wins.
        let sources = (0..=3)
            .rev()
            .find(|&n| s(3 + 6 * n) == 2 * n && s(4 + 6 * n) == 0)?;
        let v = 3 + 6 * sources + 2;
        let at = |c: u32| {
            if fine {
                c as f32 / 8.0 - 2048.0
            } else {
                c as f32 / 16.0 - 1024.0
            }
        };
        let (x0, y0) = (at(s(v) >> 16), at(s(v) & 0xffff));
        let (x1, y1) = (at(s(v + 2) >> 16), at(s(v + 4) & 0xffff));
        let (u0, v0, u1, v1) = (f(3), f(4), f(5), f(8));
        self.run(
            pds_address(secondary),
            secondary >> 27,
            [0; 3],
            Kind::State,
            &mut [0; 4],
        );
        let clamp = |c: f32, lo: u32, hi: u32| -> u32 {
            if c.is_nan() || c < lo as f32 {
                lo
            } else {
                (c as u32).clamp(lo, hi)
            }
        };
        let (px0, px1) = (
            clamp(x0, target.x0, target.x1),
            clamp(x1, target.x0, target.x1),
        );
        let (py0, py1) = (
            clamp(y0, target.y0, target.y1),
            clamp(y1, target.y0, target.y1),
        );
        if (px1 - px0) as usize * (py1 - py0) as usize > MAX_PIXELS {
            return None;
        }
        let surface = target.surface;
        let (pds, rows) = (pds_address(primary), primary >> 27);
        for y in py0..py1 {
            let ty = (y as f32 + 0.5 - y0) / (y1 - y0);
            for x in px0..px1 {
                let tx = (x as f32 + 0.5 - x0) / (x1 - x0);
                let mut o = [surface.read(g, x, y), 0, 0, 0];
                let (u, v) = (u0 + (u1 - u0) * tx, v0 + (v1 - v0) * ty);
                self.run(
                    pds,
                    rows,
                    [0; 3],
                    Kind::Pixel([u, v, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                    &mut o,
                );
                surface.write(g, x, y, o[0]);
                self.summary.pixels += 1;
            }
        }
        self.summary.target = surface.addr;
        self.summary.width = px1 - px0;
        self.summary.height = py1 - py0;
        self.summary.draws = 1;
        Some(self.summary)
    }
}

fn render(
    gpu: &dyn GpuMemory,
    cmd: u32,
    code_bases: [u32; 16],
    trace: Option<Vec<String>>,
) -> (Option<Summary>, Option<Vec<String>>) {
    let mut r = Renderer {
        gpu,
        code_bases,
        secondary: Box::new([0; usse::BANK_SIZE]),
        summary: Summary::default(),
        trace,
    };
    let s = r.command(cmd);
    (s, r.trace)
}

impl Renderer<'_> {
    fn command(&mut self, cmd: u32) -> Option<Summary> {
        let gpu = self.gpu;
        let r = self;
        let regs = gpu.read(cmd + CMD_REGS)?;
        let stream = gpu.read(cmd + CMD_STREAM)?;
        let target = r.target(regs)?;
        r.summary.target = target.surface.addr;
        r.summary.width = target.x1;
        r.summary.height = target.y1;
        // The tile buffer spans the target from its origin, so a pixel's
        // index is its position.
        let (w, h) = (target.x1 as usize, target.y1 as usize);
        if w * h > MAX_PIXELS {
            return None;
        }
        let surface = target.surface;
        let mut tile: Vec<u32> = vec![0; w * h];
        for (i, t) in tile.iter_mut().enumerate() {
            *t = surface.read(gpu, (i % w) as u32, (i / w) as u32);
        }
        r.walk(stream, &mut tile, w, h);
        for (i, t) in tile.iter().enumerate() {
            let (x, y) = ((i % w) as u32, (i / w) as u32);
            if x >= target.x0 && y >= target.y0 {
                surface.write(gpu, x, y, *t);
            }
        }
        Some(r.summary)
    }
}

impl Renderer<'_> {
    /// Runs the pixel event program and reads the target off its `emit158`.
    fn target(&mut self, regs: u32) -> Option<Target> {
        let at = self.gpu.read(regs + REGS_EVENT_PDS)?;
        let rows = self.gpu.read(regs + REGS_EVENT_ROWS)?;
        let ran = self.run(at, rows, [0; 3], Kind::Event, &mut [0; 4]);
        let (e, _) = ran.emits.iter().find(|(e, _)| e.form == FORM_PBE)?;
        Some(Target::from_pbe(e.sources))
    }

    /// Walks the control stream at `at`, drawing into `tile`.
    fn walk(&mut self, mut at: u32, tile: &mut [u32], w: usize, h: usize) {
        let mut state = 0u32;
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
                        if e.form != FORM_PBE
                            && let Some(block) = StateBlock::parse(regs)
                        {
                            self.note(|| format!("state {:08x?} -> {block:x?}", &regs[..12]));
                            state = block.pixel_pds;
                            if let Some(vp) = block.viewport {
                                viewport = vp;
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
                    self.draw(&d, state, &viewport, tile, w, h);
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
        state: u32,
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
            if !ran.ok || cw.is_nan() {
                verts.push(None);
                continue;
            }
            // [G] A w of 0 is a vertex already in window coordinates: each
            // UI scene opens with one triangle at (0, 0), (1600, 0),
            // (0, 960), z 1, w 0, under a state block without a viewport,
            // that clears the 800 x 480 target.
            let mut v = if cw == 0.0 {
                Vertex {
                    x: f(0),
                    y: f(1),
                    z: f(2),
                    ..Vertex::default()
                }
            } else {
                Vertex {
                    x: f(0) / cw * vp[0] + vp[1],
                    y: f(1) / cw * vp[3] + vp[2],
                    z: f(2) / cw * vp[4] + vp[5],
                    inv_w: 1.0 / cw,
                    ..Vertex::default()
                }
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
        let (ppds, prows) = (pds_address(state), state >> 27);
        self.note(|| {
            format!(
                "draw type {} n {count} vpds {vpds:08x} ppds {ppds:08x}/{prows} verts {:?}",
                d[0] >> 28,
                verts
                    .iter()
                    .take(4)
                    .map(|v| v.map(|v| (v.x, v.y, v.attrs[..8].to_vec())))
                    .collect::<Vec<_>>()
            )
        });
        if state == 0 {
            return;
        }
        let mut first = true;
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
                    Kind::Pixel(core::array::from_fn(|k| fr.attrs[k])),
                    &mut o,
                );
                if first {
                    first = false;
                    let before = tile[i];
                    let f = self.summary.first_fault;
                    self.note(|| {
                        format!(
                            "  pixel ({}, {}) {before:08x} -> {:08x} (faults so far {f:x?})",
                            fr.x, fr.y, o[0]
                        )
                    });
                }
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

    fn block(words: &[u32]) -> [u32; 32] {
        let mut w = [0; 32];
        w[..words.len()].copy_from_slice(words);
        w
    }

    #[test]
    fn state_blocks_place_the_pixel_program_by_their_header() {
        // The four header forms the UI sends, as captured.
        let full = block(&[
            0xdfc3,
            0x01d0_0200,
            0xf000,
            0x0f03_c800,
            0x0801_e000,
            0x1703_c802,
            0x8000_0018,
            0xe,
            0x43c8_0000,
            0x43c8_0000,
            0x4370_0000,
            0xc370_0000,
            0x3f00_0000,
            0x3f00_0000,
        ]);
        let b = StateBlock::parse(&full).expect("a state block");
        assert_eq!(b.pixel_pds, 0x1703_c802);
        assert_eq!(b.viewport, Some([400.0, 400.0, 240.0, -240.0, 0.5, 0.5]));
        let no_vp = block(&[
            0xd4c3,
            0x01d0_0200,
            0xf000,
            0x0f03_c800,
            0x0801_e000,
            0x0f01_c618,
        ]);
        assert_eq!(
            StateBlock::parse(&no_vp).map(|b| (b.pixel_pds, b.viewport)),
            Some((0x0f01_c618, None))
        );
        let short = block(&[0xc440, 0x0f03_c800, 0x0801_e000, 0x1703_c81e, 0x0c00_1000]);
        assert_eq!(
            StateBlock::parse(&short).map(|b| b.pixel_pds),
            Some(0x1703_c81e)
        );
        let shortest = block(&[0x40, 0x0f03_c800, 0x0801_e000, 0x1703_c82a]);
        assert_eq!(
            StateBlock::parse(&shortest).map(|b| b.pixel_pds),
            Some(0x1703_c82a)
        );
        // The control words the same helpers emit are not state blocks.
        assert_eq!(StateBlock::parse(&block(&[0x800])), None);
        assert_eq!(StateBlock::parse(&block(&[0x2000, 0x0018_000e])), None);
    }

    #[test]
    fn texture_words_give_layout_size_and_stride() {
        // A strided 8-bit staging source: v = 88 is a 356-byte row.
        let t = Texture::decode([0x0016_0000, 0xc016_0020, 0x0010_2de4, 0]);
        assert_eq!(t.surface.layout, Layout::Strided);
        assert_eq!((t.surface.bytes, t.surface.stride), (1, 356));
        assert_eq!((t.width, t.height), (0x161, 0x21));
        // 4096 bytes, the high stride bits.
        assert_eq!(
            Texture::decode([0x00fe_0e00, 0xc0ff_f006, 0, 0])
                .surface
                .stride,
            4096
        );
        // A tiled 32-bit texture: rows of tiles the width rounded up to 32.
        let t = Texture::decode([0x03fe_0090, 0x8c01_f011, 0x1191_a000, 0]);
        assert_eq!((t.surface.layout, t.surface.bytes), (Layout::Tiled, 4));
        assert_eq!((t.width, t.height, t.surface.stride), (32, 18, 32));
        // A twiddled 8-bit atlas, log2 sizes.
        let t = Texture::decode([0x03fe_0090, 0x0008_0006, 0x1191_6000, 0xe000_0000]);
        assert_eq!(
            (t.surface.layout, t.width, t.height, t.surface.bytes),
            (Layout::Twiddled, 256, 64, 1)
        );
    }

    #[test]
    fn pixel_back_end_words_give_the_destination() {
        // The UI's display buffer: 32 bits, strided, 800 pixels a row.
        let t = Target::from_pbe([[0x0011_0000, 0x1079_7800], [0x18f, 0], [0, 0x001d_f31f]]);
        assert_eq!(
            (t.surface.bytes, t.surface.stride, t.surface.layout),
            (4, 800, Layout::Strided)
        );
        assert_eq!((t.x0, t.y0, t.x1, t.y1), (0, 0, 800, 480));
        assert_eq!(t.surface.offset(1, 1), 0x1079_7800 + 3200 + 4);
        // A tiled 8-bit texture upload, 480-pixel rows of tiles.
        let t = Target::from_pbe([[0x8401_0000, 0x11a4_0000], [0xef, 0], [0, 0x0002_0160]]);
        assert_eq!(
            (t.surface.bytes, t.surface.stride, t.surface.layout),
            (1, 480, Layout::Tiled)
        );
        assert_eq!(t.surface.offset(33, 1), 0x11a4_0000 + 1024 + 32 + 1);
    }

    #[test]
    fn twiddling_interleaves_the_smaller_dimension() {
        assert_eq!(twiddle(0, 1, (2, 2)), 1);
        assert_eq!(twiddle(1, 0, (2, 2)), 2);
        assert_eq!(twiddle(3, 3, (2, 2)), 15);
        // 8 x 2: one interleaved bit pair, then x's remaining bits.
        assert_eq!(twiddle(2, 0, (3, 1)), 4);
        assert_eq!(twiddle(7, 1, (3, 1)), 15);
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
