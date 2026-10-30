//! The XSAVE feature set: `XCR0`, `XGETBV`/`XSETBV`, and the instructions
//! that save and restore processor state by **component**.
//!
//! # Why it exists
//!
//! `FXSAVE` has a fixed 512-byte image, and every register file added after
//! SSE would have needed another instruction and another operating-system
//! patch. XSAVE replaces that with a bitmap: processor state is divided into
//! numbered *state components* — 0 is the x87 unit, 1 the `XMM` registers,
//! 2 the upper halves of the `YMM` registers, and so on — and an operating
//! system says once, through `XCR0`, which components it is prepared to
//! manage. Each save or restore then names a subset in `EDX:EAX`, and the
//! instruction operates on the **requested-feature bitmap**
//! `RFBM = XCR0 & EDX:EAX` (*Intel SDM* volume 1 §13.1, §13.3).
//!
//! # The area
//!
//! | bytes | what |
//! | --- | --- |
//! | 0-511 | the legacy region: `FXSAVE`'s image, byte for byte — component 0 is bytes 0-23 and 32-159, component 1 is the `XMM` registers from 160, and `MXCSR` sits at 24 |
//! | 512-575 | the header: `XSTATE_BV` at 512, `XCOMP_BV` at 520, the rest reserved |
//! | 576- | the extended region: components 2 and up, at the offsets `CPUID` leaf `0Dh` reports |
//!
//! The legacy region is written by the same code `FXSAVE` uses, split at the
//! component boundaries (`fpexec`'s `store_x87_image` and its siblings), so
//! the two images agree byte for byte by construction rather than by care.
//!
//! `XSTATE_BV` says, per component, whether the area holds that component's
//! state (1) or whether the component was in its **initial configuration**
//! (0) — in which case `XRSTOR` does not read the area at all but puts the
//! component back to its init values. That is the "init optimisation", and it
//! is what makes the next point matter.
//!
//! # `XINUSE`: what "in use" means here
//!
//! The architecture tracks, per component, a bit `XINUSE[i]` that is clear
//! only when the component is known to be in its initial configuration; it is
//! always permitted to be **set** even when the component happens to be at
//! its init values (*Intel SDM* volume 1 §13.6). This core computes it rather
//! than tracking it: a component is in use exactly when its state differs from
//! the init configuration — the x87 unit as `FNINIT` leaves it with every
//! register zero and both pointers clear, the `XMM` registers all zero (and
//! not `MXCSR`, which §13.6 excludes from `XINUSE[1]`). That is the most
//! precise answer the definition allows, costs one comparison per component
//! per save, and needs no bookkeeping on the hundreds of instructions that
//! would otherwise have to set a bit. `XGETBV` with `ECX = 1` returns
//! `XCR0 & XINUSE`, and `CPUID.(EAX=0Dh,ECX=1):EAX[2]` says so.
//!
//! # `XSAVEOPT` and `XSAVEC`
//!
//! `XSAVEOPT` is permitted two optimisations over `XSAVE`: skipping a
//! component that is in its init configuration, and skipping one that has not
//! been modified since the last `XRSTOR` from the same area (§13.9). Both are
//! permissions, not obligations — the area's `XSTATE_BV` is what a restore
//! reads, and it is the same either way — so `XSAVEOPT` here **is** `XSAVE`.
//! A guest cannot tell, except by timing or by reading bytes the architecture
//! says are unspecified.
//!
//! `XSAVEC` writes the **compacted** format (§13.10): the extended components
//! packed one after another from byte 576 in component order, only the
//! requested ones present, `XCOMP_BV` recording which, and only the components
//! in use actually written. `XRSTOR` reads either format, told apart by
//! `XCOMP_BV[63]` (§13.8).
//!
//! `XSAVES` and `XRSTORS` are not implemented: their only purpose beyond
//! `XSAVEC` is *supervisor* state components selected by `IA32_XSS`, and this
//! core has none. `CPUID` reports both absent, so a guest never tries.
//!
//! # Adding a component
//!
//! [`COMPONENTS`] is the one table: a row per state component above the
//! legacy region, with its size, its standard-format offset, its alignment in
//! the compacted format, and four hooks — whether this instance has it,
//! whether it is in use, and how to copy it out of and into the processor.
//! `CPUID` leaf `0Dh`, the area sizes, `XSETBV`'s validity check and every
//! instruction here iterate it; nothing else needs to learn the component's
//! number.
//!
//! # Sources
//!
//! *Intel SDM* volume 1 chapter 13 ("Managing State Using the XSAVE Feature
//! Set"); volume 2's pages for `XGETBV`, `XSETBV`, `XSAVE`, `XSAVEOPT`,
//! `XSAVEC` and `XRSTOR` (also at felixcloutier.com/x86); volume 2 `CPUID`
//! leaf `0Dh`; volume 3 §2.5 and §2.6 for `CR4.OSXSAVE` and `XCR0`.

use super::Features;
use super::exec::{Ex, Exec, Fault, State, VEC_NM, VEC_UD};
use super::fpu::{X87, mxcsr};
use super::isa::{Fields, Op};
use super::prot::{cr0, cr4, xcr0};

/// Where the header starts: straight after the legacy region.
pub(super) const HEADER: u64 = 512;
/// Where the extended region starts: after the 64-byte header.
pub(super) const EXTENDED: u32 = 576;
/// `XCOMP_BV[63]`: the area is in the compacted format.
const COMPACTED: u64 = 1 << 63;

/// One state component above the legacy region.
///
/// The hooks are plain function pointers over [`State`] rather than methods
/// on [`Exec`] so that a component's whole description is this one row:
/// where it lives in the area, and how it moves between the area and the
/// processor. Its storage is wherever the component's own feature keeps it.
#[derive(Debug, Clone, Copy)]
pub(super) struct Component {
    /// The component number, which is also its `XCR0` bit.
    pub bit: u8,
    /// Its size in bytes — a multiple of eight, so the hooks move it as
    /// quadwords. `CPUID.(EAX=0Dh,ECX=bit):EAX`.
    pub size: u32,
    /// Its offset in the **standard** format. `CPUID.(EAX=0Dh,ECX=bit):EBX`.
    pub offset: u32,
    /// Whether the **compacted** format aligns it to 64 bytes.
    /// `CPUID.(EAX=0Dh,ECX=bit):ECX[1]`.
    pub align64: bool,
    /// Whether an instance with these features has the component at all.
    pub present: fn(Features) -> bool,
    /// Whether it differs from its initial configuration — `XINUSE[bit]`.
    pub in_use: fn(&State) -> bool,
    /// Copy it out, `size / 8` quadwords in area order.
    pub save: fn(&State, &mut [u64]),
    /// Copy it in from `size / 8` quadwords, or — given `None` — put it in its
    /// initial configuration.
    pub load: fn(&mut State, Option<&[u64]>),
}

/// The state components above the legacy region, in component order.
///
/// Empty until AVX: components 0 and 1 are the legacy region, which has its
/// own fixed layout and is handled apart. AVX's row is component 2, the
/// upper 128 bits of the sixteen `YMM` registers — 256 bytes at offset 576,
/// not 64-byte aligned when compacted (*Intel SDM* volume 1 §13.5.3).
pub(super) static COMPONENTS: &[Component] = &[];

/// The standard-format size of an area holding `mask`'s components.
///
/// Never less than [`EXTENDED`]: the legacy region and the header are there
/// whatever is requested.
fn standard_size(mask: u64) -> u32 {
    COMPONENTS
        .iter()
        .filter(|c| mask & (1 << c.bit) != 0)
        .map(|c| c.offset + c.size)
        .fold(EXTENDED, u32::max)
}

/// The compacted-format size of an area holding `mask`'s components.
fn compacted_size(mask: u64) -> u32 {
    let mut at = EXTENDED;
    for c in COMPONENTS.iter().filter(|c| mask & (1 << c.bit) != 0) {
        if c.align64 {
            at = at.next_multiple_of(64);
        }
        at += c.size;
    }
    at
}

/// Which of the four instructions that write an area this is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Format {
    /// `XSAVE` and `XSAVEOPT`: every requested component at its fixed offset.
    Standard,
    /// `XSAVEC`: the requested components packed, the ones in use written.
    Compacted,
}

impl Exec<'_> {
    /// The `XCR0` bits this instance supports: x87 and SSE, plus every
    /// [`COMPONENTS`] row this instance's features include — or nothing at
    /// all on a part without XSAVE.
    pub(super) fn xcr0_supported(&self) -> u64 {
        let features = self.cfg.features;
        if !features.xsave {
            return 0;
        }
        COMPONENTS
            .iter()
            .filter(|c| (c.present)(features))
            .fold(xcr0::X87 | xcr0::SSE, |m, c| m | (1 << c.bit))
    }

    /// `XINUSE`, restricted to `XCR0`. See the module documentation for why
    /// it is computed rather than tracked.
    fn xinuse(&self) -> u64 {
        let state = &*self.state;
        let mut used = 0;
        if state.x87 != X87::new() {
            used |= xcr0::X87;
        }
        if state.sse.xmm != [[0; 2]; 16] {
            used |= xcr0::SSE;
        }
        for c in COMPONENTS {
            if (c.in_use)(state) {
                used |= 1 << c.bit;
            }
        }
        used & state.sys.xcr0
    }

    /// `CPUID` leaf `0Dh`, sub-leaf `sub`: what the XSAVE feature set
    /// supports and how big its areas are. Zero on a part without it.
    ///
    /// *Intel SDM* volume 2 `CPUID`, "Processor Extended State Enumeration".
    pub(super) fn cpuid_xsave(&self, sub: u32) -> (u32, u32, u32, u32) {
        let supported = self.xcr0_supported();
        if supported == 0 {
            return (0, 0, 0, 0);
        }
        let enabled = self.state.sys.xcr0;
        match sub {
            // EDX:EAX the `XCR0` bits that may be set; EBX the area size for
            // what *is* set, which is what an operating system allocates per
            // task; ECX the size were everything set.
            0 => (
                supported as u32,
                standard_size(enabled),
                standard_size(supported),
                (supported >> 32) as u32,
            ),
            // EAX: bit 0 `XSAVEOPT`, bit 1 `XSAVEC` and the compacted form of
            // `XRSTOR`, bit 2 `XGETBV` with `ECX = 1`. Bit 3, `XSAVES` and
            // `IA32_XSS`, stays clear. EBX is the compacted size for
            // `XCR0 | IA32_XSS`, and `IA32_XSS` is zero here; ECX:EDX, the
            // supervisor components `IA32_XSS` may name, are none.
            1 => (0b111, compacted_size(enabled), 0, 0),
            // One sub-leaf per component: size, standard offset, and in ECX
            // bit 0 "supervisor" (never, here) and bit 1 "64-byte aligned when
            // compacted". Components 0 and 1 are the legacy region and their
            // sub-leaves are reserved, which the table's absence of rows for
            // them gives for free.
            _ => COMPONENTS
                .iter()
                .find(|c| u32::from(c.bit) == sub && supported & (1 << c.bit) != 0)
                .map_or((0, 0, 0, 0), |c| {
                    (c.size, c.offset, u32::from(c.align64) << 1, 0)
                }),
        }
    }

    /// The gate every instruction here passes first: the feature present
    /// and the operating system's `CR4.OSXSAVE` set, or `#UD`.
    fn require_osxsave(&self) -> Ex<()> {
        if !self.cfg.features.xsave || self.state.sys.cr4 & cr4::OSXSAVE == 0 {
            return Err(Fault::bare(VEC_UD));
        }
        Ok(())
    }

    /// `EDX:EAX` as one 64-bit value: the low doubleword of each, whatever
    /// the mode, as every instruction here reads them.
    fn edx_eax(&self) -> u64 {
        let regs = &self.state.regs;
        ((regs.rdx & 0xffff_ffff) << 32) | (regs.rax & 0xffff_ffff)
    }

    /// `XGETBV`: read the extended control register `ECX` names.
    ///
    /// Any privilege level. `ECX = 0` is `XCR0`; `ECX = 1` is
    /// `XCR0 & XINUSE`, which `CPUID` leaf `0Dh` sub-leaf 1 advertises; any
    /// other is `#GP(0)`. Both halves of the result are 32-bit writes, so each
    /// zero-extends in 64-bit mode (*Intel SDM* volume 2, `XGETBV`).
    pub(super) fn xgetbv(&mut self) -> Ex<()> {
        self.require_osxsave()?;
        let value = match self.state.regs.rcx as u32 {
            0 => self.state.sys.xcr0,
            1 => self.xinuse(),
            _ => return Err(Fault::gp(0)),
        };
        self.state.regs.set_dword(0, value as u32);
        self.state.regs.set_dword(2, (value >> 32) as u32);
        Ok(())
    }

    /// `XSETBV`: write `XCR0`.
    ///
    /// `#GP(0)` unless at `CPL` 0 with `ECX = 0`, and unless the value is one
    /// a processor could hold: only supported bits, bit 0 (x87) always set,
    /// and bit 2 (AVX) only beside bit 1 (SSE), because the upper halves of
    /// the `YMM` registers mean nothing without the lower (*Intel SDM*
    /// volume 2, `XSETBV`; volume 1 §13.3).
    pub(super) fn xsetbv(&mut self) -> Ex<()> {
        self.require_osxsave()?;
        if self.cpl() != 0 || self.state.regs.rcx as u32 != 0 {
            return Err(Fault::gp(0));
        }
        let value = self.edx_eax();
        if value & !self.xcr0_supported() != 0
            || value & xcr0::X87 == 0
            || (value & xcr0::AVX != 0 && value & xcr0::SSE == 0)
        {
            return Err(Fault::gp(0));
        }
        self.state.sys.xcr0 = value;
        Ok(())
    }

    /// `XSAVE`, `XSAVEOPT`, `XSAVEC` and `XRSTOR`, and their `REX.W` forms.
    ///
    /// The gates, in the order the instruction pages list them: `#UD` without
    /// the feature or `CR4.OSXSAVE`; `#NM` with `CR0.TS` set — so a lazy
    /// context switch catches these exactly as it catches `FXSAVE`, while
    /// `CR0.EM` is *not* consulted; then `#GP(0)` for an area that is not
    /// 64-byte aligned.
    pub(super) fn xsave_instruction(&mut self, f: &Fields) -> Ex<()> {
        self.require_osxsave()?;
        if self.state.sys.cr0 & cr0::TS != 0 {
            return Err(Fault::bare(VEC_NM));
        }
        let (sr, off) = self.ea();
        if !off.is_multiple_of(64) {
            return Err(Fault::gp(0));
        }
        match f.insn.op {
            Op::XRSTOR => self.xrstor(f, sr, off),
            Op::XSAVEC => self.xsave(f, sr, off, Format::Compacted),
            _ => self.xsave(f, sr, off, Format::Standard),
        }
    }

    /// How many `XMM` registers the SSE component covers in this mode:
    /// sixteen in 64-bit mode and eight anywhere else, where the top eight
    /// are not part of the architectural state (*Intel SDM* volume 1
    /// §13.5.2). Unlike `FXSAVE`, which writes all sixteen regardless, an
    /// `XSAVE` outside 64-bit mode leaves bytes 288-415 alone.
    fn xmm_count(&self) -> u8 {
        if self.sixty_four() { 16 } else { 8 }
    }

    /// Save the requested components.
    ///
    /// The standard form's header rule is that `XSTATE_BV` is rewritten only
    /// for the requested components — `(old & !RFBM) | (XINUSE & RFBM)` — and
    /// nothing else in the header is written, `XCOMP_BV` included. The
    /// compacted form writes both fields outright, and saves only what is in
    /// use, with `MXCSR` at a value other than its default counting as the
    /// SSE component being in use (the `XSAVEC` instruction page).
    fn xsave(&mut self, f: &Fields, sr: u8, off: u64, format: Format) -> Ex<()> {
        let wide = f.rex_w();
        let rfbm = self.state.sys.xcr0 & self.edx_eax();
        let inuse = self.xinuse();
        let count = self.xmm_count();
        let header = off.wrapping_add(HEADER);
        // The extended components' images, gathered before anything is
        // written so the one borrow of `State` the hooks need is not held
        // across the bus.
        let mut extended = alloc::vec::Vec::new();
        let save = match format {
            Format::Standard => rfbm,
            Format::Compacted => {
                let mut save = rfbm & inuse;
                if rfbm & xcr0::SSE != 0 && self.state.sse.mxcsr != mxcsr::RESET {
                    save |= xcr0::SSE;
                }
                save
            }
        };
        match format {
            Format::Standard => {
                // Read first: a header that is not writable faults before a
                // byte of the area has changed.
                let old = self.read_mem(sr, header, 8)?;
                if save & xcr0::X87 != 0 {
                    self.store_x87_image(sr, off, wide)?;
                }
                if save & xcr0::SSE != 0 {
                    self.store_xmm_image(sr, off, count)?;
                }
                // `MXCSR` belongs to whichever of SSE and AVX is asked for:
                // either kind of vector instruction depends on it.
                if save & (xcr0::SSE | xcr0::AVX) != 0 {
                    self.store_mxcsr_image(sr, off)?;
                }
                for c in COMPONENTS.iter().filter(|c| save & (1 << c.bit) != 0) {
                    extended.push((u64::from(c.offset), *c));
                }
                self.store_components(sr, off, &extended)?;
                let bv = (old & !rfbm) | (inuse & rfbm);
                self.write_mem(sr, header, 8, bv)?;
            }
            Format::Compacted => {
                if save & xcr0::X87 != 0 {
                    self.store_x87_image(sr, off, wide)?;
                }
                if save & xcr0::SSE != 0 {
                    self.store_mxcsr_image(sr, off)?;
                    self.store_xmm_image(sr, off, count)?;
                }
                let mut at = EXTENDED;
                for c in COMPONENTS.iter().filter(|c| rfbm & (1 << c.bit) != 0) {
                    if c.align64 {
                        at = at.next_multiple_of(64);
                    }
                    if save & (1 << c.bit) != 0 {
                        extended.push((u64::from(at), *c));
                    }
                    at += c.size;
                }
                self.store_components(sr, off, &extended)?;
                self.write_mem(sr, header, 8, save)?;
                self.write_mem(sr, header.wrapping_add(8), 8, rfbm | COMPACTED)?;
            }
        }
        Ok(())
    }

    /// Write each `(offset, component)` from the processor into the area.
    fn store_components(&mut self, sr: u8, off: u64, list: &[(u64, Component)]) -> Ex<()> {
        for &(at, c) in list {
            let mut words = alloc::vec![0u64; (c.size / 8) as usize];
            (c.save)(&*self.state, &mut words);
            for (i, word) in words.into_iter().enumerate() {
                let to = off.wrapping_add(at).wrapping_add(8 * i as u64);
                self.write_mem(sr, to, 8, word)?;
            }
        }
        Ok(())
    }

    /// Read one component's image out of the area.
    fn load_component(&mut self, sr: u8, at: u64, c: Component) -> Ex<alloc::vec::Vec<u64>> {
        let mut words = alloc::vec![0u64; (c.size / 8) as usize];
        for (i, word) in words.iter_mut().enumerate() {
            *word = self.read_mem(sr, at.wrapping_add(8 * i as u64), 8)?;
        }
        Ok(words)
    }

    /// `XRSTOR`, in whichever format the header says the area is in.
    ///
    /// Validation first, all of it `#GP(0)` (*Intel SDM* volume 1 §13.8):
    ///
    /// - **standard** (`XCOMP_BV[63]` clear): `XSTATE_BV` may name only
    ///   components `XCR0` enables, and header bytes 8-23 — `XCOMP_BV` and
    ///   the quadword after it — must be zero. Bytes 24-63 are not checked.
    /// - **compacted**: `XCOMP_BV` may name only enabled components,
    ///   `XSTATE_BV` only components `XCOMP_BV` names, and header bytes 16-63
    ///   must be zero.
    ///
    /// Then every requested component is either loaded from the area (its
    /// `XSTATE_BV` bit set) or put in its initial configuration (clear), and
    /// nothing is committed until every read has succeeded — a fault leaves
    /// the processor as it was. A reserved bit in an `MXCSR` that is going to
    /// be loaded is `#GP(0)` as for `FXRSTOR`.
    #[allow(clippy::too_many_lines)]
    fn xrstor(&mut self, f: &Fields, sr: u8, off: u64) -> Ex<()> {
        let wide = f.rex_w();
        let enabled = self.state.sys.xcr0;
        let rfbm = enabled & self.edx_eax();
        let count = self.xmm_count();
        let header = off.wrapping_add(HEADER);
        let xstate_bv = self.read_mem(sr, header, 8)?;
        let xcomp_bv = self.read_mem(sr, header.wrapping_add(8), 8)?;
        let compacted = xcomp_bv & COMPACTED != 0;
        if compacted {
            let mut reserved = 0;
            for i in 2..8u64 {
                reserved |= self.read_mem(sr, header.wrapping_add(8 * i), 8)?;
            }
            let named = xcomp_bv & !COMPACTED;
            if named & !enabled != 0 || xstate_bv & !named != 0 || reserved != 0 {
                return Err(Fault::gp(0));
            }
        } else {
            let reserved = self.read_mem(sr, header.wrapping_add(16), 8)?;
            if xstate_bv & !enabled != 0 || xcomp_bv != 0 || reserved != 0 {
                return Err(Fault::gp(0));
            }
        }

        // Gather.
        let mut x87 = None;
        if rfbm & xcr0::X87 != 0 {
            x87 = Some(if xstate_bv & xcr0::X87 != 0 {
                self.load_x87_image(sr, off, wide)?
            } else {
                X87::new()
            });
        }
        let mut xmm = None;
        let mut new_mxcsr = None;
        if rfbm & xcr0::SSE != 0 {
            if xstate_bv & xcr0::SSE != 0 {
                xmm = Some(self.load_xmm_image(sr, off, count)?);
            } else {
                let mut regs = self.state.sse.xmm;
                for reg in regs.iter_mut().take(usize::from(count)) {
                    *reg = [0; 2];
                }
                xmm = Some(regs);
            }
        }
        if compacted {
            // The compacted form treats `MXCSR` as part of the SSE component
            // outright: loaded with it, or reset to its default with it.
            if rfbm & xcr0::SSE != 0 {
                new_mxcsr = Some(if xstate_bv & xcr0::SSE != 0 {
                    self.load_mxcsr_image(sr, off)?
                } else {
                    mxcsr::RESET
                });
            }
        } else if rfbm & (xcr0::SSE | xcr0::AVX) != 0 {
            // The standard form loads it whenever either vector component is
            // requested, `XSTATE_BV` notwithstanding.
            new_mxcsr = Some(self.load_mxcsr_image(sr, off)?);
        }
        let mut extended = alloc::vec::Vec::new();
        let mut at = EXTENDED;
        for c in COMPONENTS {
            let bit = 1u64 << c.bit;
            let place = if compacted {
                if xcomp_bv & bit == 0 {
                    None
                } else {
                    if c.align64 {
                        at = at.next_multiple_of(64);
                    }
                    let here = at;
                    at += c.size;
                    Some(here)
                }
            } else {
                Some(c.offset)
            };
            if rfbm & bit == 0 {
                continue;
            }
            let image = match place {
                Some(pos) if xstate_bv & bit != 0 => {
                    Some(self.load_component(sr, off.wrapping_add(u64::from(pos)), *c)?)
                }
                _ => None,
            };
            extended.push((*c, image));
        }

        // Commit.
        if let Some(x) = x87 {
            self.state.x87 = x;
            self.x87_resummarise();
        }
        if let Some(regs) = xmm {
            self.state.sse.xmm = regs;
        }
        if let Some(value) = new_mxcsr {
            self.state.sse.mxcsr = value;
        }
        for (c, image) in extended {
            (c.load)(&mut *self.state, image.as_deref());
        }
        Ok(())
    }
}
