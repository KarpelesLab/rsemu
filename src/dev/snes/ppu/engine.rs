//! The S-PPU's state and its beam: memories, the register file, and the walk
//! from one timing boundary to the next.

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{Error, Result};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};

use super::regs::Regs;
use super::timing::{
    Frame, HBLANK_END_DOT, HBLANK_START_DOT, RENDER_DOT, Region, VBLANK_LINE, VBLANK_LINE_OVERSCAN,
    dot_at, dot_start,
};

/// Words of video RAM: 64 KiB, addressed as 32 K sixteen-bit words.
pub const VRAM_WORDS: usize = 0x8000;
/// CGRAM entries: 256 fifteen-bit colours, 512 bytes.
pub const CGRAM_ENTRIES: usize = 256;
/// OAM bytes: a 512-byte low table and a 32-byte high table.
pub const OAM_LEN: usize = 544;

/// Framebuffer width: always the hires width. A low-resolution line is
/// stored pixel-doubled, so a frame that mixes the two needs no rescaling.
pub const FB_WIDTH: usize = 512;
/// Framebuffer height: always the interlaced overscan height. A
/// non-interlaced line is stored on both rows of its pair.
pub const FB_HEIGHT: usize = 478;
/// Entries in the framebuffer.
pub const FB_LEN: usize = FB_WIDTH * FB_HEIGHT;

/// The snapshot encoding. Bump with the layout, never on its own.
pub(super) const STATE_VERSION: u32 = 1;

/// One value the snapshot carries, encoded one canonical way.
///
/// Lets [`Regs`] list its fields once for both directions, so save and load
/// cannot drift apart field by field.
pub(super) trait Field: Sized {
    fn put(&self, w: &mut ChunkWriter<'_>) -> Result<()>;
    fn get(r: &mut ChunkReader<'_>) -> Result<Self>;
}

macro_rules! field_int {
    ($($t:ty => $w:ident, $r:ident;)*) => {$(
        impl Field for $t {
            fn put(&self, w: &mut ChunkWriter<'_>) -> Result<()> { w.$w(*self) }
            fn get(r: &mut ChunkReader<'_>) -> Result<Self> { r.$r() }
        }
    )*};
}

field_int! {
    u8 => write_u8, read_u8;
    u16 => write_u16, read_u16;
    u32 => write_u32, read_u32;
    u64 => write_u64, read_u64;
    bool => write_bool, read_bool;
}

impl<T: Field + Copy + Default, const N: usize> Field for [T; N] {
    fn put(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        for v in self {
            v.put(w)?;
        }
        Ok(())
    }
    fn get(r: &mut ChunkReader<'_>) -> Result<Self> {
        let mut out = [T::default(); N];
        for v in &mut out {
            *v = T::get(r)?;
        }
        Ok(out)
    }
}

/// Everything the chip pair remembers, plus the beam.
pub struct Engine {
    /// Video RAM, by word.
    pub vram: Vec<u16>,
    /// Colour RAM: `0bbbbbgggggrrrrr` per entry.
    pub cgram: Vec<u16>,
    /// Object attribute memory.
    pub oam: Vec<u8>,
    /// Every register and access latch.
    pub r: Regs,

    pub(super) region: Region,
    pub(super) ppu1_version: u8,
    pub(super) ppu2_version: u8,

    /// Master clocks since construction. The device's own tick.
    pub master: u64,
    /// Master clocks into the current line.
    pub line_pos: u32,
    /// The V counter.
    pub v: u16,
    /// The interlace field, `STAT78` bit 7. Toggles at the start of every frame.
    pub field: bool,
    /// Frames completed — counted at the start of vertical blank, which is when
    /// the picture is whole.
    pub frame: u64,
    /// The vertical-blank level.
    pub vblank: bool,
    /// The horizontal-blank level.
    pub hblank: bool,
    /// `SETINI` interlace as latched at the start of this frame.
    pub interlace_frame: bool,
    /// Lines in this frame, fixed at its start.
    pub lines_in_frame: u16,
    /// The first line of vertical blank this frame: 225, or 240 when overscan
    /// was on at line 225.
    pub vblank_line: u16,
    /// The `EXTLATCH` input — `WRIO` bit 7 and controller port 2 pin 6.
    pub extlatch: bool,

    /// Whether any line of the frame being drawn was 512 wide.
    pub(super) hires_seen: bool,
    /// The completed frame's output width: 256 or 512.
    pub out_width: u16,
    /// The completed frame's output height: 224, 239, 448 or 478.
    pub out_height: u16,
    /// RGB555 with `INIDISP` brightness applied, [`FB_WIDTH`] × [`FB_HEIGHT`].
    pub fb: Vec<u16>,
}

impl fmt::Debug for Engine {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Engine")
            .field("region", &self.region)
            .field("v", &self.v)
            .field("h", &self.h_dot())
            .field("field", &self.field)
            .field("frame", &self.frame)
            .field("regs", &self.r)
            .finish_non_exhaustive()
    }
}

impl Engine {
    /// A chip pair at power-on.
    pub fn new(region: Region, ppu1_version: u8, ppu2_version: u8) -> Engine {
        let mut e = Engine {
            vram: vec![0; VRAM_WORDS],
            cgram: vec![0; CGRAM_ENTRIES],
            oam: vec![0; OAM_LEN],
            r: Regs::power_on(),
            region,
            ppu1_version,
            ppu2_version,
            master: 0,
            line_pos: 0,
            v: 0,
            field: false,
            frame: 0,
            vblank: false,
            hblank: true,
            interlace_frame: false,
            lines_in_frame: region.base_lines(),
            vblank_line: VBLANK_LINE,
            extlatch: false,
            hires_seen: false,
            out_width: 256,
            out_height: 224,
            fb: vec![0; FB_LEN],
        };
        e.begin_frame_geometry();
        e
    }

    /// Back to the power-on register state. The master counter carries on:
    /// the scheduler owns time, and a chip that rewound it would be told to
    /// replay every clock since.
    ///
    /// A **cold** reset clears the memories too; a warm one keeps them, which
    /// is what the reset button does to SRAM chips that never lose power.
    pub fn reset(&mut self, cold: bool) {
        let master = self.master;
        let (vram, cgram, oam, fb) = if cold {
            (
                vec![0; VRAM_WORDS],
                vec![0; CGRAM_ENTRIES],
                vec![0; OAM_LEN],
                vec![0; FB_LEN],
            )
        } else {
            (
                core::mem::take(&mut self.vram),
                core::mem::take(&mut self.cgram),
                core::mem::take(&mut self.oam),
                core::mem::take(&mut self.fb),
            )
        };
        let extlatch = self.extlatch;
        *self = Engine::new(self.region, self.ppu1_version, self.ppu2_version);
        self.vram = vram;
        self.cgram = cgram;
        self.oam = oam;
        self.fb = fb;
        self.master = master;
        self.extlatch = extlatch;
    }

    /// The region this chip was built for.
    pub fn region(&self) -> Region {
        self.region
    }

    // -- the beam -------------------------------------------------------------

    fn long_dots(&self) -> bool {
        self.region
            .has_long_dots(self.v, self.field, self.interlace_frame)
    }

    /// Master clocks in the current line.
    pub fn line_length(&self) -> u32 {
        self.region
            .line_length(self.v, self.field, self.interlace_frame)
    }

    /// The H counter: which dot the beam is in.
    pub fn h_dot(&self) -> u16 {
        dot_at(self.line_pos, self.long_dots())
    }

    /// Whether the chip is in forced blank.
    pub fn forced_blank(&self) -> bool {
        self.r.inidisp & 0x80 != 0
    }

    /// The next position in this line at which something happens.
    fn next_boundary(&self) -> u32 {
        // None of the three in-line boundaries is past a long dot, so they are
        // the same in a short line as in an ordinary one.
        for dot in [HBLANK_END_DOT, RENDER_DOT, HBLANK_START_DOT] {
            let pos = dot_start(dot, true);
            if pos > self.line_pos {
                return pos;
            }
        }
        self.line_length()
    }

    /// The master tick of the next boundary; always in the future.
    pub fn next_event(&self) -> u64 {
        self.master + u64::from(self.next_boundary() - self.line_pos)
    }

    /// Run until `target` master clocks have elapsed, or until a boundary
    /// moves one of the output levels — whichever comes first — so the caller
    /// can drive the wire before going on. Returns whether `target` was
    /// reached.
    pub fn run_to(&mut self, target: u64) -> bool {
        while self.master < target {
            let boundary = self.next_boundary();
            let step = (target - self.master).min(u64::from(boundary - self.line_pos));
            self.master += step;
            // `step` is at most a line, so this cannot truncate.
            self.line_pos += step as u32;
            if self.line_pos == boundary && self.boundary(boundary) {
                return self.master >= target;
            }
        }
        true
    }

    /// Handle the boundary at `pos`. Returns whether a level moved.
    fn boundary(&mut self, pos: u32) -> bool {
        if pos == self.line_length() {
            self.line_pos = 0;
            self.v += 1;
            if self.v >= self.lines_in_frame {
                self.v = 0;
            }
            return self.line_start();
        }
        if pos == dot_start(HBLANK_END_DOT, true) {
            self.hblank = false;
            return true;
        }
        if pos == dot_start(RENDER_DOT, true) {
            if self.v >= 1 && self.v < self.vblank_line {
                self.render_line(self.v);
            }
            return false;
        }
        if pos == dot_start(HBLANK_START_DOT, true) {
            self.hblank = true;
            return true;
        }
        false
    }

    /// The frame-start latches: field, interlace, the frame's length.
    fn begin_frame_geometry(&mut self) {
        self.interlace_frame = self.r.setini & 0x01 != 0;
        self.lines_in_frame = self
            .region
            .lines_per_frame(self.interlace_frame, self.field);
        self.vblank_line = VBLANK_LINE;
    }

    /// Dot 0 of a line. Returns whether a level moved.
    fn line_start(&mut self) -> bool {
        let mut moved = false;
        if self.v == 0 {
            self.field = !self.field;
            self.begin_frame_geometry();
            self.vblank = false;
            moved = true;
            // fullsnes, STAT77: the overflow flags are reset at the end of
            // vertical blank, but not while forced blank holds.
            if !self.forced_blank() {
                self.r.time_over = false;
                self.r.range_over = false;
            }
            self.hires_seen = false;
        }
        if self.v == VBLANK_LINE && self.r.setini & 0x04 != 0 {
            self.vblank_line = VBLANK_LINE_OVERSCAN;
        }
        if self.v == self.vblank_line {
            self.vblank = true;
            moved = true;
            // fullsnes, OAMADD: reloaded "at begin of Vblank … but only if not
            // in Forced Blank mode".
            if !self.forced_blank() {
                self.r.oam_addr = self.r.oam_reload << 1;
            }
            self.frame += 1;
            self.out_width = if self.hires_seen { 512 } else { 256 };
            let lines = self.vblank_line - 1;
            self.out_height = if self.interlace_frame {
                lines * 2
            } else {
                lines
            };
        }
        moved
    }

    /// The frame the beam is in, pinned to the master-clock timeline.
    ///
    /// Derived, not stored: the frame began `line_pos` clocks and `v` whole
    /// lines ago.
    pub fn beam_frame(&self) -> Frame {
        let mut frame = Frame {
            region: self.region,
            start: 0,
            field: self.field,
            interlace: self.interlace_frame,
            lines: self.lines_in_frame,
        };
        let into = frame.line_start(self.v) + u64::from(self.line_pos);
        // Saturating only so a snapshot that names a beam position its master
        // count is too small for cannot panic; no run produces one.
        frame.start = self.master.saturating_sub(into);
        frame
    }

    /// The interlace setting the next frame will latch if `SETINI` is left
    /// alone.
    pub fn interlace_next(&self) -> bool {
        self.r.setini & 0x01 != 0
    }

    /// Master clocks from where the beam stands until it next reaches the
    /// start of dot `h` of line `v`, or `None` if no such position comes
    /// within the next two frames (a line or dot that does not exist).
    ///
    /// A projection: it assumes `SETINI` keeps its current interlace and
    /// overscan settings, which is what the S-CPU's H/V timer must assume too.
    pub fn master_until(&self, v: u16, h: u16) -> Option<u64> {
        let interlace_next = self.r.setini & 0x01 != 0;
        let (mut line, mut field, mut interlace) = (self.v, self.field, self.interlace_frame);
        let mut lines = self.lines_in_frame;
        let mut acc: u64 = 0;
        let mut pos = self.line_pos;
        for _ in 0..(2 * 314 + 2) {
            let long = self.region.has_long_dots(line, field, interlace);
            let len = self.region.line_length(line, field, interlace);
            if line == v && h < self.region.dots_in_line(line, field, interlace) {
                let target = dot_start(h, long);
                if target >= pos {
                    return Some(acc + u64::from(target - pos));
                }
            }
            acc += u64::from(len - pos);
            pos = 0;
            line += 1;
            if line >= lines {
                line = 0;
                field = !field;
                interlace = interlace_next;
                lines = self.region.lines_per_frame(interlace, field);
            }
        }
        None
    }

    // -- snapshots ------------------------------------------------------------

    /// Write the whole state.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u32(STATE_VERSION)?;
        w.write_bytes(&words_to_bytes(&self.vram))?;
        w.write_bytes(&words_to_bytes(&self.cgram))?;
        w.write_bytes(&self.oam)?;
        self.r.save(w)?;
        w.write_u64(self.master)?;
        w.write_u32(self.line_pos)?;
        w.write_u16(self.v)?;
        w.write_bool(self.field)?;
        w.write_u64(self.frame)?;
        w.write_bool(self.vblank)?;
        w.write_bool(self.hblank)?;
        w.write_bool(self.interlace_frame)?;
        w.write_u16(self.lines_in_frame)?;
        w.write_u16(self.vblank_line)?;
        w.write_bool(self.extlatch)?;
        w.write_bool(self.hires_seen)?;
        w.write_u16(self.out_width)?;
        w.write_u16(self.out_height)?;
        // The framebuffer is not derived state: a snapshot taken mid-frame
        // must show the lines already drawn.
        w.write_bytes(&words_to_bytes(&self.fb))?;
        Ok(())
    }

    /// Read the whole state, all or nothing.
    pub fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        let version = r.read_u32()?;
        if version != STATE_VERSION {
            return Err(Error::State(alloc::format!(
                "the S-PPU's snapshot is version {version}, this build writes {STATE_VERSION}"
            )));
        }
        let vram = bytes_to_words(r.read_bytes()?, VRAM_WORDS)?;
        let cgram = bytes_to_words(r.read_bytes()?, CGRAM_ENTRIES)?;
        let oam = r.read_bytes()?;
        if oam.len() != OAM_LEN {
            return Err(bad_size());
        }
        let oam = oam.to_vec();
        let regs = Regs::load(r)?;
        let master = r.read_u64()?;
        let line_pos = r.read_u32()?;
        let v = r.read_u16()?;
        let field = r.read_bool()?;
        let frame = r.read_u64()?;
        let vblank = r.read_bool()?;
        let hblank = r.read_bool()?;
        let interlace_frame = r.read_bool()?;
        let lines_in_frame = r.read_u16()?;
        let vblank_line = r.read_u16()?;
        let extlatch = r.read_bool()?;
        let hires_seen = r.read_bool()?;
        let out_width = r.read_u16()?;
        let out_height = r.read_u16()?;
        let fb = bytes_to_words(r.read_bytes()?, FB_LEN)?;
        let len = self.region.line_length(v, field, interlace_frame);
        if v >= lines_in_frame || line_pos >= len || lines_in_frame > 313 {
            return Err(Error::State(String::from(
                "the S-PPU's snapshot puts the beam outside its frame",
            )));
        }
        self.vram = vram;
        self.cgram = cgram;
        self.oam = oam;
        self.r = regs;
        self.master = master;
        self.line_pos = line_pos;
        self.v = v;
        self.field = field;
        self.frame = frame;
        self.vblank = vblank;
        self.hblank = hblank;
        self.interlace_frame = interlace_frame;
        self.lines_in_frame = lines_in_frame;
        self.vblank_line = vblank_line;
        self.extlatch = extlatch;
        self.hires_seen = hires_seen;
        self.out_width = out_width;
        self.out_height = out_height;
        self.fb = fb;
        Ok(())
    }
}

fn bad_size() -> Error {
    Error::State(String::from(
        "the S-PPU's snapshot has the wrong memory sizes",
    ))
}

fn words_to_bytes(words: &[u16]) -> Vec<u8> {
    let mut out = Vec::with_capacity(words.len() * 2);
    for w in words {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}

fn bytes_to_words(bytes: &[u8], words: usize) -> Result<Vec<u16>> {
    if bytes.len() != words * 2 {
        return Err(bad_size());
    }
    Ok(bytes
        .as_chunks::<2>()
        .0
        .iter()
        .map(|c| u16::from_le_bytes(*c))
        .collect())
}
