//! The differential harness: the x86 lifter against the x86 interpreter,
//! forever.
//!
//! CLAUDE.md, "CPU cores": *the IR frontend comes later and is differentially
//! tested against the interpreter forever. **The interpreter is the oracle.***
//! This module is that, for [`lift`].
//!
//! # The comparison
//!
//! One program, two machines built the same way, and everything either of them
//! can be seen to do:
//!
//! | | oracle | subject |
//! | --- | --- | --- |
//! | engine | [`X86::step`], `insns` times | [`lift`] → [`verify`] → [`Interp`] |
//! | registers | `RAX`..`RDI`, `R8`..`R15` and the program counter, all sixty-four bits | the slots the block materialized |
//! | **flags** | `EFLAGS`, every bit | the six flag slots plus `EFLAGS_REST`, reassembled |
//! | ticks | `X86::cycles` | the sum of the charges the block made |
//! | the static column | — | [`InsnStart::ticks`] at the exit, plus what the accesses spent |
//! | memory | the RAM it wrote | the RAM it wrote |
//! | faults | whether a trap was taken, where, and in what state | whether the block faulted, where, and in what state |
//!
//! The flags row is the one this harness exists for. Six flags are written by
//! nearly every arithmetic instruction and read by almost none, the lifter
//! elides the ones it can prove unobservable ([`lift::Flags::Elide`]), and
//! dead-code elimination then removes the arithmetic behind them. Every one of
//! those steps is a place to be wrong in a way no register comparison notices,
//! so `EFLAGS` is compared **whole**, at the end of every case and at every
//! fault.
//!
//! # How a trap is detected, and why nothing is delivered
//!
//! The oracle's `IDTR` limit is **zero**, so the first exception cannot read
//! its gate, escalates to `#DF`, fails again, and shuts the processor down —
//! which is what [`X86::is_halted`] then reports. That is deliberate rather
//! than lazy, and it buys three things a working interrupt descriptor table
//! would cost:
//!
//! * **The architectural state at the fault is what is left behind.**
//!   `Exec::deliver` restores its pre-instruction register snapshot before
//!   each retry, and `Exec::step` took that snapshot before decoding — so a
//!   faulting instruction is architecturally as if it had never started, and
//!   `X86::regs` afterwards *is* the state to compare against. The same thing
//!   the IR gives for free through lazy publication (`ir::interp`).
//! * **No ticks are charged for the delivery.** `protected_interrupt` rejects
//!   a vector past the table's limit before it reads anything, so the cycle
//!   counter still holds exactly what the faulting instruction spent — which
//!   is what makes the tick column comparable at a fault rather than only at
//!   an exit.
//! * **No memory is written.** A real gate would push a trap frame onto the
//!   stack, and the byte-for-byte RAM comparison would then have to know
//!   about it.
//!
//! # The machine, and the second one beside it
//!
//! A 386 in 32-bit protected mode with paging off — the world
//! [`lift::World::of`] accepts, and the one `pc-at` firmware and FreeDOS run
//! in. `CS` is flat, because a computed near transfer would otherwise need a
//! conditional `#GP` the IR cannot express; the five data segments have a base
//! of [`BASE`] and a limit of `RAM_SIZE - 1`, which is what gives this harness
//! a **fault to compare**: an offset past the end of RAM raises `#GP`, or
//! `#SS` through the stack, before anything reaches the bus. Every offset
//! inside the limit lands inside mapped RAM, so no access ever reaches an
//! unassigned address and the interpreter's open-bus path is never a source of
//! disagreement.
//!
//! [`Case::paged`] is the second machine, and it is a **world** rather than a
//! policy: `CR0.PG` set, on the one part whose translation buffers are split.
//! The guest's linear addresses are exactly the same, and every one of them
//! now names a different physical page — deliberately, because an identity map
//! would leave every linear-versus-physical confusion invisible, and that is
//! the class of bug paging adds. Four more pages hold the page directory and
//! the page table, and `memory` compares those along with the rest, so the
//! **accessed and dirty bits both engines wrote are a compared column** rather
//! than an assumption.
//!
//! [`Case::compat`] is the **fourth**, and it is neither of the two it sits
//! between: `EFER.LMA` set, so the walk is IA-32e's four levels of eight-byte
//! entries and the interrupt structures are long mode's, under a **32-bit**
//! code segment. [`lift::World::of`] asks `Sys::sixty_four`, which is `LMA`
//! *and* `CS.L`, so it has accepted this world all along and nothing generated
//! had ever put a case in it. Three things separate it from the two it looks
//! like: the decoder is driven at [`Bits::B32`], so it runs [`synthesize`]'s
//! corpus and not [`synthesize64`]'s; segmentation is back in force for the
//! data registers, where 64-bit mode defines base, limit and type away; and
//! `cpu::x86::engine`'s `narrow_state_is_clean` is a check that can fire
//! rather than a formality, because this is exactly the state its
//! documentation names — a 64-bit kernel's leftovers still in the register
//! file when a 32-bit code segment starts executing. A 64-bit kernel is in it
//! whenever it runs a 32-bit program, so it is reachable on `pc64` and
//! `q35-linux` rather than merely legal.
//!
//! [`Case::long`] is the third, and it is the second plus what long mode
//! requires: `CR4.PAE`, `EFER.LME` with the `LMA` the processor sets, and a
//! code segment with its `L` bit set. The walk becomes four levels of
//! eight-byte entries, `CS`, `DS`, `ES` and `SS` lose their bases by
//! definition, and — the part that makes it a test rather than a re-run — the
//! data window moves to [`HIGH`], two and a half tebibytes up, with a
//! different index at each of the top three levels of the walk. **Every
//! address the corpus computes there is one only sixty-four bits can hold**,
//! which is what separates a correct 64-bit address computation from one that
//! masks its answer to thirty-two: a case whose whole window fits below 2^32
//! cannot tell them apart, and the mask survived deliberate injection until
//! the window moved.
//!
//! The code stays at [`BASE`], so the four guest frames are mapped **twice**,
//! and that is deliberate as well: two linear pages naming one physical page
//! is exactly the arrangement the in-block store guard cannot see, and
//! `a_store_through_the_other_mapping_of_the_code_page_is_honoured` drives a
//! store through the far alias into the running block's own frame.
//!
//! # Why the host here re-implements the memory path, and where it stops
//!
//! [`IrHost::load`] and [`IrHost::store`] are where a lifted block meets guest
//! memory, and the interpreter's own path through them is private to a step in
//! progress. So this module's host performs the access itself, in the same
//! shape `Exec` does: the segment check `prot::Exec::seg_linear` makes, then
//! one bus transaction charged at the part's [`Variant::bus_clocks`]. Only the
//! *limit* half of that check is implemented, and deliberately: every segment
//! this machine builds is present, readable and writable, so a permission
//! check could never fire — and a check that cannot fire is one nobody would
//! notice going wrong.
//!
//! That is a second implementation of a rule, which is normally the thing to
//! avoid — but it is the *host's* rule rather than the frontend's, and the
//! frontend is what is under test. The lifter's contribution is the
//! [`MemOp`]'s size and its `SegId`, and a wrong one of those diverges here
//! immediately.
//!
//! **Under paging it stops being defensible, so it stops.** A paged access has
//! a translation in front of it, and that translation has a tick cost that
//! depends on a buffer hit, an accessed bit, a dirty bit written by a
//! fall-through walk on the first store to a page, and a per-byte split when
//! the access crosses a page boundary. A second implementation of *that* would
//! be a job rather than a line, and every one of its mistakes would be
//! reported as a divergence in the lifter. So a paged host owns an `Mmu` and
//! calls `Exec::read_mem` and `Exec::write_mem` — the functions `Exec::step`
//! itself calls — which is the answer `cpu::riscv::engine` reached for the same
//! reason.
//!
//! It also owes the entry fetch translation, on every execution of every
//! block: see `Host::enter`, and [`lift`]'s module docs for why getting that
//! wrong looks like a working JIT with a short clock.
//!
//! # Two harnesses, not one
//!
//! [`compare`] runs **one block**, freshly lifted, and stops. That is right
//! for testing a frontend and blind to everything the translation runtime
//! does, so [`compare_cached`] (with the `jit` feature) is the second harness:
//! the same oracle, the same columns, but many blocks through
//! `jit::Dispatcher` — served from a block cache, chained exit to successor,
//! invalidated when the guest writes into a translated page, and with the
//! instruction bytes coming out of **guest RAM** rather than out of
//! [`Case::program`]. That last difference is what makes self-modifying code
//! testable at all, and on x86 it is not optional: the architecture makes a
//! coherent instruction cache a guarantee rather than a courtesy, so a store
//! into a running block's own page has to be honoured before the next
//! instruction — which is exactly what [`lift::Smc::Guard`] emits and what
//! `a_store_into_the_running_blocks_own_page_is_honoured_immediately` checks.
//!
//! # What breaking it deliberately caught
//!
//! A harness nobody has watched fail is a harness that passes. Twenty bugs
//! were injected into [`lift`] one at a time and the suite run
//! against each; **nineteen** were caught, and the twentieth is written up
//! below because it is a finding rather than a gap.
//!
//! Six of them are flags — the auxiliary carry taken from bit 3 instead of
//! bit 4, the parity inverted, `SHL`'s carry read off the result rather than
//! off the bit above the operand's width, `INC` clobbering the carry it must
//! preserve, `AND` leaving the auxiliary carry alone, and a multiply's
//! undefined four taken from the low half of the product. Four are the
//! translation's own machinery: a boundary eliding a flag at an instruction
//! that *can* fault, the exit boundary eliding flags at all, a trace's side
//! exit inverted, and a block reporting its program-order successor rather
//! than the transfer's target. Three are arithmetic: subtract-with-borrow's
//! carry ignoring the borrow in, a 16-bit register write clobbering the upper
//! half, and the effective address widened and never masked. Three are the
//! rules that decide what may be *removed* or *deferred*: a guest load made
//! eliminable, the instruction's own clocks left out of the charge, and the
//! effective address computed after the instruction had already moved a
//! register. And two are self-modifying code: the in-block page guard removed
//! outright, and a `CALL`'s guard resuming after the call rather than at its
//! target.
//!
//! Three of those needed a case the generated corpus does not reach, and each
//! got one: `a_call_that_rewrites_its_own_target_resumes_at_the_target`,
//! `a_compare_against_memory_still_makes_its_bus_cycle` — for which
//! [`synthesize`] also grew three memory-comparison forms, because a load
//! whose only consumer is a flag is the shape that makes [`MemOp`]'s
//! `volatile` load-bearing — and
//! `a_pop_into_a_stack_relative_address_uses_the_stack_pointer_it_started_with`,
//! whose first draft wrote the same zero to both the right address and the
//! wrong one and caught nothing.
//!
//! # What the corpus reaches, and the three things widening it found
//!
//! [`synthesize`] and [`synthesize64`] are the coverage claim, so what they do
//! *not* draw is a gap with no symptom: a form nobody generates is a path
//! nobody compares, and every sweep still passes. Both were widened this
//! round, from fifty-six forms to ninety-eight and from sixty-two to
//! ninety-seven, and what went in is what an audit of [`lift`] against them
//! said was unreachable:
//!
//! * **a 16-bit operand size**, which is a whole operand *width* the lifter
//!   has always had and neither corpus had ever written — `set_szp` at bit
//!   fifteen, `add`'s carry and `sub`'s borrow at sixteen, a rotate's overflow
//!   off bits fifteen and fourteen, `MUL` into `DX:AX`, `CBW`'s half-width
//!   arm, and a word write that must **preserve** the doubleword above it
//!   where a doubleword write zero-extends;
//! * **`AH`, `CH`, `DH` and `BH`** — byte register numbers four to seven,
//!   which are the *top* halves of the first four registers and are why
//!   [`Opcode::EXTRACT`](crate::ir::Opcode::EXTRACT)'s documentation names
//!   x86. One form reached them;
//! * **`Lifter::ea`'s SIB arm** — a base, a scaled index, "no index" and "no
//!   base" — which nothing generated at all, because every memory form was
//!   `mod=01` with a base of zero to three. The long-mode draw has to *make*
//!   an index register, because both windows are two and a half tebibytes up
//!   and two pointers added together are nowhere;
//! * **`mod=10`'s 32-bit displacement, `mod=00`'s bare one, and the direct
//!   offsets** (`A0`-`A3`), the last of which is the only encoding whose
//!   immediate is an *address* — a `moffs64` in long mode, and the only reason
//!   `Lifter::ea`'s `None` arm exists;
//! * **`MOVZX`/`MOVSX` from a word**, the other arm of `Plan::MovX`'s
//!   `src_size`;
//! * **the accumulator-immediate forms and group `80`**, which are second and
//!   third encodings of operations already covered and the only ones that
//!   reach `Arg::Al` and `Arg::Ax`;
//! * **`PUSH imm`, `LEAVE`, `CALL rel`, `JMP rel` and the near `Jcc`**, which
//!   hand-written cases reached and the generator did not — including the
//!   merged `CALL` whose self-modifying-code exit resumes at the call's
//!   *target*.
//!
//! Three things came out of it, and none of them is a lifter bug:
//!
//! 1. **`CMOVcc` had never been generated in a 32-bit world at all.** It is a
//!    property of the *instance* — `Exec` raises `#UD` for one on a part
//!    without the feature and [`lift`] refuses it there — and the unpaged
//!    sweep ran a 386, which has it clear. The form was drawn six hundred
//!    times a sweep and lifted nothing every time. Half the unpaged cases now
//!    run on a part that has it.
//! 2. **[`synthesize64`]'s byte shift had no immediate.** `C0 /n` takes an
//!    `ib` and the arm did not push one, so every draw of it swallowed the
//!    first byte of the instruction after it and shifted the rest of the
//!    program along. Both engines still agreed, because both decode the same
//!    bytes; what was lost is that the form after it was not the form the
//!    generator drew.
//! 3. Neither of those has a symptom a sweep can report, which is why
//!    `every_generated_form_is_one_the_frontend_lifts` is now a test: it
//!    checks **form by form** that what the generator can draw is something
//!    the frontend lifts, in both of the parts the sweeps run. A form that
//!    quietly leaves the subset fails it, where the thresholds — counted over
//!    whole programs — would not notice.
//!
//! ## The mutation that survived, and why the shape test is the answer
//!
//! The one that survived: making a `CL` shift claim it writes its flags
//! **unconditionally**, so the boundary before it elides them. That is a wrong
//! statement about the instruction and it is not an observable one, for a
//! reason worth writing down — a `CL` shift with a memory destination is
//! outside the subset, so such a shift cannot fault, so nothing can look at
//! the flags the boundary dropped. It is asserted by shape instead, in
//! `lift`'s `a_shift_by_cl_elides_nothing_because_it_may_write_nothing`, and
//! it becomes differentially observable the day that form is lifted.
//!
//! # What this harness deliberately does not cover
//!
//! * **What a trap does next.** The state *at* the fault is compared in full;
//!   vectoring is the interpreter's job and a lifted block hands the fault
//!   back rather than delivering it.
//! * **Anything outside the lifted subset**, which ends the block by
//!   construction — so it is not skipped, it simply is not reached.
//! * **Real mode, compatibility mode's 16-bit code segments, the segment loads
//!   and paging on a part with one translation buffer**, none of which
//!   [`lift::World::of`] accepts. Paging on a part with two is covered — see
//!   [`Case::paged`] — and so is long mode, which this list named for four
//!   rounds; see [`Case::long`]. Compatibility mode's **32-bit** code segments
//!   are covered too, and are a world rather than a sixth policy; see
//!   [`Case::compat`].
//! * **The three computed near transfers in long mode.** `RET`, `JMP r/m` and
//!   `CALL r/m` end a block there rather than being lifted, because a
//!   non-canonical target is a `#GP` at the transfer that a block cannot
//!   deliver — so they are not skipped, they are the interpreter's.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

#[cfg(any(feature = "jit", test))]
use crate::core::device::DebugTranslation;
use crate::core::error::BusError;
use crate::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region};
use crate::ir::{InsnStart, Interp, IrHost, MemOp, Outcome, RegSlot, verify};

use super::exec::{Exec, State};
use super::isa::{Bits, seg};
use super::lift::{
    self, ARITH_MASK, EFLAGS_REST, FLAG_BITS, FLAG_SLOTS, Flags, Origin, RIP, SLOT_COUNT, Shape,
    Smc, World, r_slot,
};
#[cfg(any(feature = "jit", test))]
use super::paging::debug_translate;
use super::paging::{Access, pte};
use super::prot::{SegReg, Sys, ar, cr0, cr4, efer};
use super::{Config, Lines, Regs, Variant, X86, flags};

#[cfg(feature = "jit")]
use crate::core::value::Width;
#[cfg(feature = "jit")]
use crate::ir::AccessKind;
#[cfg(feature = "jit")]
use crate::jit::{
    BlockCache, Context as TlbContext, DirtyPages, Dispatcher, Entry, Epoch, FastMem, Frontend,
    Stop, StoreLog, Tlb, Translation,
};

/// Where a case's RAM is mapped, and where its program starts.
///
/// One megabyte: page-aligned, above everything a PC's low memory would hold,
/// and far enough from zero that a small negative displacement off a seeded
/// pointer does not wrap into nothing.
pub const BASE: u64 = 0x0010_0000;

/// How much RAM a case gets: four pages.
///
/// The first page holds the program — a block is bounded by its page, so it
/// can hold more instructions than [`lift::MAX_INSNS`] will ever read — and
/// the rest is the data window that loads and stores are aimed at. It is also
/// the data segments' limit, which is what makes an access past it a `#GP`.
pub const RAM_SIZE: u64 = 4 * 4096;

/// Where the data window starts, as an offset from [`BASE`].
pub const DATA: u64 = 4096;

/// Where the stack pointer starts, as an offset from [`BASE`].
///
/// In the middle of the data window with room on both sides, so a run of
/// pushes reaches neither the code page below it nor the segment limit above.
pub const STACK: u64 = DATA + 0x800;

/// How much physical RAM a **paged** case gets: eight pages.
///
/// Four of them are the guest's — the same four [`RAM_SIZE`] describes, at the
/// same *linear* addresses — and the other four hold the translation
/// structures and the gap that keeps linear and physical from being the same
/// number by accident. Everything in the region is compared byte for byte,
/// which is what makes the accessed and dirty bits a compared column rather
/// than an assumption.
pub const PAGED_RAM_SIZE: u64 = 8 * 4096;

/// How much physical RAM a **long-mode** case gets: twelve pages.
///
/// Eight of them are laid out as [`PAGED_RAM_SIZE`] describes — four tables
/// and the four guest pages — and three more hold a second chain of tables
/// that maps the same four frames a second time, at [`HIGH`]. The twelfth is
/// slack, so the number is a round one.
pub const LONG_RAM_SIZE: u64 = 12 * 4096;

/// Where a long-mode case's data window is, **linearly**.
///
/// Two and a half tebibytes up, with a different index at each of the top
/// three levels of the walk, which is the whole point: a 32-bit case cannot
/// tell a correct 64-bit address computation from one that masks its answer to
/// thirty-two bits, because every address it forms fits in thirty-two anyway.
/// So a long-mode case puts its pointer registers and its stack here and every
/// address the corpus computes is one only sixty-four bits can hold.
///
/// The code stays at [`BASE`] — a block is bounded by its page and the entry
/// translation is what names it, so moving the program would test the same
/// thing twice — which makes the two windows **aliases of one set of physical
/// frames**. That is deliberate too: two linear pages naming one physical page
/// is exactly the arrangement the in-block store guard could not see, and the
/// reason [`Smc::Guard`] is refused under paging.
pub const HIGH: u64 = 0x0000_0280_c040_0000;

/// Where the high window's page-directory-pointer table sits, physically.
pub const PDPT_HIGH: u64 = BASE + 0x8000;
/// Where the high window's page directory sits, physically.
pub const PDIR_HIGH: u64 = BASE + 0x9000;
/// Where the high window's page table sits, physically.
pub const PTAB_HIGH: u64 = BASE + 0xa000;

/// Where the page directory sits, physically, in a legacy two-level case.
pub const PDIR: u64 = BASE;

/// Where the one page table sits, physically, in a legacy two-level case.
pub const PTAB: u64 = BASE + 0x1000;

/// Where the four-level walk's top table sits, physically.
///
/// IA-32e paging is four levels of 8-byte entries and long mode requires it
/// (*Intel SDM* volume 3 §9.8.5), so a 64-bit case needs four tables where a
/// legacy one needs two. They occupy the four physical pages below
/// [`PAGED_PROGRAM`] — the same four the legacy layout uses two of and leaves
/// two of as the gap that keeps linear and physical from coinciding.
pub const PML4: u64 = BASE;
/// Where the page-directory-pointer table sits, physically.
pub const PDPT: u64 = BASE + 0x1000;
/// Where the page directory sits, physically, in a four-level case.
pub const PDIR64: u64 = BASE + 0x2000;
/// Where the one page table sits, physically, in a four-level case.
pub const PTAB64: u64 = BASE + 0x3000;

/// The physical page linear [`BASE`] is mapped to in a paged case.
///
/// Deliberately **not** [`BASE`]: an identity map would leave every
/// linear-versus-physical confusion invisible, which is the whole class of bug
/// paging adds. The four guest pages sit at `PAGED_PROGRAM ..
/// PAGED_PROGRAM + RAM_SIZE`.
pub const PAGED_PROGRAM: u64 = BASE + 0x4000;

/// The selector the flat code segment is loaded from.
const CODE_SEL: u16 = 0x08;
/// The selector the data segments are loaded from.
const DATA_SEL: u16 = 0x10;

/// A 32-bit ring-0 code segment: present, readable, executable, `D` set.
const CODE32: u32 = ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::DB;
/// A 32-bit ring-0 data segment: present, writable, `B` set.
const DATA32: u32 = ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::DB;
/// A 64-bit ring-0 code segment: `L` set and `D` **clear**, which is the one
/// combination that means 64-bit mode — `L` with `D` is reserved.
const CODE64: u32 = ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::L | ar::GRANULAR;

/// One differential case: a program, the state it starts with, and the three
/// lifter policies it is lifted under.
#[derive(Debug, Clone)]
pub struct Case {
    /// Which part. Must be one [`World::of`] accepts.
    pub variant: Variant,
    /// The instruction bytes, loaded at [`BASE`] and entered at `CS:BASE`.
    pub program: Vec<u8>,
    /// The initial `RAX`..`RDI` then `R8`..`R15`, in ModRM order. The stack
    /// pointer is overwritten with [`STACK`] unless [`Case::keep_esp`] is set,
    /// because a random stack pointer makes every push a fault and measures
    /// the trap path instead of the lifter.
    ///
    /// Sixteen and sixty-four bits wide whatever the case, because the *host*
    /// is: a 32-bit case simply never binds the top eight, and both engines
    /// start them from the same numbers so they stay equal by not being
    /// touched.
    pub regs: [u64; 16],
    /// Which of [`Case::regs`] hold an **offset into the data window** rather
    /// than a value, as a bitmask over the register numbers.
    ///
    /// The window is reached through a segment with a base of [`BASE`] below
    /// long mode and through a flat address space in it — 64-bit mode gives
    /// `DS`, `ES` and `SS` a base of zero by definition — so the same case
    /// needs a different number in a pointer register depending on the world.
    /// Recording *which* registers are pointers is how one seeded case can be
    /// run in both, and it is why [`Case::with_reg`] clears the bit it writes.
    pub pointers: u16,
    /// Whether [`Case::regs`]'s stack pointer is used as given.
    pub keep_esp: bool,
    /// The initial `EFLAGS`, before normalisation.
    pub eflags: u32,
    /// How much the lifter may swallow into one block.
    pub shape: Shape,
    /// What a store does to the block it is in.
    pub smc: Smc,
    /// Whether a boundary names every flag.
    ///
    /// Every one of these three is a separate frontend to test rather than a
    /// setting: they emit different IR from the same bytes, all of them are in
    /// the cache key, and all of them must agree with the one interpreter.
    pub flags: Flags,
    /// Whether this case runs with `CR0.PG` set.
    ///
    /// A **fourth** world rather than a fourth policy: the guest's linear
    /// addresses are unchanged and every one of them now names a different
    /// physical page, the memory path is the interpreter's own rather than
    /// this module's, and the block is keyed on the page its entry resolved
    /// to. See [`Case::paged`].
    pub paged: bool,
    /// Whether this case runs in **64-bit mode**.
    ///
    /// The fifth world, and the one that is not optional about the fourth: a
    /// processor in long mode is a processor with paging on, always, because
    /// `EFER.LMA` is set only when `CR0.PG` goes on with `EFER.LME` and
    /// IA-32e paging requires `CR4.PAE` (*Intel SDM* volume 3 §9.8.5). See
    /// [`Case::long`].
    pub long: bool,
    /// Whether this case runs in **compatibility mode**.
    ///
    /// The fourth world, and the one that is neither of the two it is made
    /// of: a processor with `EFER.LMA` set — so the walk is IA-32e's four
    /// levels of eight-byte entries and the interrupt structures are long
    /// mode's — executing a **32-bit** code segment, where segmentation is
    /// back in force for the data registers and `Bits::B32` is what the
    /// decoder is driven with. `World::of` asks `Sys::sixty_four`, which is
    /// `LMA` *and* `CS.L`, so this is a world it accepts and nothing
    /// generated had ever put it in. See [`Case::compat`].
    pub compat: bool,
}

impl Case {
    /// A case that runs `program` on a 386 with a zeroed register file.
    #[must_use]
    pub fn new(program: Vec<u8>) -> Case {
        Case {
            variant: Variant::I80386,
            program,
            regs: [0; 16],
            pointers: 0,
            keep_esp: false,
            eflags: flags::ALWAYS_SET,
            shape: Shape::default(),
            smc: Smc::default(),
            flags: Flags::default(),
            paged: false,
            long: false,
            compat: false,
        }
    }

    /// How much physical RAM this case's machine has, and how much of it the
    /// two engines are compared over.
    #[must_use]
    pub const fn ram_size(&self) -> u64 {
        if self.long || self.compat {
            LONG_RAM_SIZE
        } else if self.paged {
            PAGED_RAM_SIZE
        } else {
            RAM_SIZE
        }
    }

    /// Where the program is loaded, as an offset into that RAM.
    ///
    /// Zero unless the case pages, where linear [`BASE`] is
    /// [`PAGED_PROGRAM`].
    #[must_use]
    pub const fn program_offset(&self) -> u64 {
        if self.paged { PAGED_PROGRAM - BASE } else { 0 }
    }

    /// A case whose `EAX`..`EBX` point into the data window, spread so that a
    /// small signed displacement off any of them stays inside RAM.
    ///
    /// The companion to [`synthesize`], which takes a memory operand's base
    /// from exactly those four registers. `EAX` is deliberately *not* aligned
    /// to four, so a misaligned access is reachable without the displacement
    /// having to supply the misalignment.
    #[must_use]
    pub fn seeded(program: Vec<u8>) -> Case {
        let mut case = Case::new(program);
        case.regs[0] = DATA + 0x101;
        case.regs[1] = DATA + 0x300;
        case.regs[2] = DATA + 0x1000;
        case.regs[3] = DATA + 0x1800;
        // The same four again in the half only a `REX` prefix can name, so a
        // 64-bit generated program has somewhere to point too. In a 32-bit
        // case nothing can decode a register number above seven, so these are
        // eight numbers both engines start with and neither ever touches.
        case.regs[8] = DATA + 0x201;
        case.regs[9] = DATA + 0x480;
        case.regs[10] = DATA + 0xc00;
        case.regs[11] = DATA + 0x1400;
        case.pointers = 0b0000_1111_0000_1111;
        // Something in every other register, so a generated program reuses a
        // value rather than reading a fresh zero every time.
        case.regs[5] = 0x8000_0001;
        case.regs[6] = 0x0000_ffff;
        case.regs[7] = 0x7fff_ffff;
        case.regs[12] = 0x8000_0000_0000_0001;
        case.regs[13] = 0x0000_0000_ffff_ffff;
        case.regs[14] = 0x7fff_ffff_ffff_ffff;
        case.regs[15] = 0x1234_5678_9abc_def0;
        case
    }

    /// The same case lifted under `shape`.
    #[must_use]
    pub const fn with_shape(mut self, shape: Shape) -> Case {
        self.shape = shape;
        self
    }

    /// The same case under a different self-modifying-code policy.
    #[must_use]
    pub const fn with_smc(mut self, smc: Smc) -> Case {
        self.smc = smc;
        self
    }

    /// The same case under a different flag policy.
    #[must_use]
    pub const fn with_flags(mut self, policy: Flags) -> Case {
        self.flags = policy;
        self
    }

    /// The same case with a register preset.
    ///
    /// The register stops being a data-window pointer, because a caller that
    /// writes a number down means that number in both worlds.
    #[must_use]
    pub const fn with_reg(mut self, n: usize, value: u64) -> Case {
        if n < 16 {
            self.regs[n] = value;
            self.pointers &= !(1u16 << n);
        }
        self
    }

    /// The same case starting with `EFLAGS` set to `value`.
    #[must_use]
    pub const fn with_eflags(mut self, value: u32) -> Case {
        self.eflags = value;
        self
    }

    /// The same case with paging on, which forces two other choices.
    ///
    /// * **The part becomes [`Variant::X86_64`]**, because paged code is only
    ///   in the lifted subset on a part whose instruction and data
    ///   translations are separate arrays — see
    ///   [`World::of`](lift::World::of). A 386 and a 486 are refused there and
    ///   would be refused here.
    /// * **The store policy becomes [`Smc::EndBlock`]**, because
    ///   [`Smc::Guard`] compares linear pages and [`lift`] refuses it under
    ///   paging.
    ///
    /// Both are silent rather than assertions because the point of a
    /// constructor is that the case it builds is one the frontend accepts.
    #[must_use]
    pub const fn paged(mut self) -> Case {
        self.paged = true;
        self.variant = Variant::X86_64;
        self.smc = Smc::EndBlock;
        self
    }

    /// The same case in **64-bit mode**, which forces paging with it.
    ///
    /// Not a sixth policy but a fifth world, and the one whose prerequisites
    /// are not a matter of taste: long mode is [`Case::paged`] plus
    /// `CR4.PAE`, `EFER.LME` and a code segment with its `L` bit set, and a
    /// processor cannot be in it with paging off. So this calls
    /// [`Case::paged`] rather than asking a caller to remember to.
    ///
    /// What changes for a program, beyond the sixteen registers and the `REX`
    /// prefix that names them: `CS`, `DS`, `ES` and `SS` have a base of zero
    /// however their descriptors read, so a pointer register holds a **linear**
    /// address rather than an offset — which is what [`Case::pointers`] exists
    /// to say.
    #[must_use]
    pub const fn long(mut self) -> Case {
        self.long = true;
        self = self.paged();
        self
    }

    /// The same case in **compatibility mode**, which is long mode's tables
    /// under a 32-bit code segment.
    ///
    /// A world rather than a policy, and the one a 64-bit kernel is in
    /// whenever it runs a 32-bit program — so it is reachable on `pc64` and
    /// `q35-linux` rather than merely legal. What makes it a *different*
    /// world from either of the two it sits between:
    ///
    /// * the walk is **IA-32e's four levels of eight-byte entries**, as long
    ///   mode's is, and not the two-level legacy walk [`Case::paged`] uses;
    /// * the decoder is driven at [`Bits::B32`], so `40`-`4f` are `INC` and
    ///   `DEC` again and no encoding can name a register above seven — which
    ///   is why this world runs [`synthesize`]'s corpus and not
    ///   [`synthesize64`]'s;
    /// * **segmentation is back**: `DS`, `ES` and `SS` have a base and a
    ///   limit again, where 64-bit mode defines all three away, so an access
    ///   past the window is a `#GP` from a segment limit rather than a `#PF`
    ///   from an unmapped page;
    /// * and `cpu::x86::engine`'s `narrow_state_is_clean` is a real check
    ///   here rather than a formality — this is precisely the state its
    ///   documentation names, "a 64-bit kernel's leftovers still in the file
    ///   when a 32-bit code segment starts executing".
    ///
    /// `lift::key` separates it from long mode on bit seven, which is the one
    /// bit that says the two are the same part in two worlds.
    #[must_use]
    pub const fn compat(mut self) -> Case {
        self.compat = true;
        self = self.paged();
        self
    }

    /// What a data-window offset has to be added to before a guest register
    /// can hold it.
    ///
    /// The data segments' base below long mode, and [`HIGH`] in it — where
    /// there is no segment base and a linear address is the whole answer.
    #[must_use]
    pub const fn data_base(&self) -> u64 {
        if self.long { HIGH } else { 0 }
    }

    /// The register file this case actually starts with.
    ///
    /// **Narrowed to the world's own width**, which is the same kind of
    /// normalisation `Case::start_eflags` performs and is not cosmetic: a
    /// 32-bit guest cannot have anything above 2^32 in a register — every
    /// doubleword write zero-extends and every narrower one preserves a top
    /// half that started at zero — and [`lift`]'s slot invariant is that a
    /// slot *holds the architectural register*, so a 32-bit read of one is the
    /// slot itself with no field taken out of it. Seeding a 32-bit case with a
    /// wider number would put the lifter and the interpreter on two different
    /// values and report it as a frontend divergence. The fuzz target builds
    /// register values out of its input, so this is reachable rather than
    /// theoretical.
    #[must_use]
    pub fn start_regs(&self) -> [u64; 16] {
        let mut regs = self.regs;
        let base = self.data_base();
        let width = if self.long { u64::MAX } else { 0xffff_ffff };
        for (n, value) in regs.iter_mut().enumerate() {
            if self.pointers & (1 << n) != 0 {
                *value = value.wrapping_add(base);
            }
            *value &= width;
        }
        if !self.keep_esp {
            regs[4] = STACK.wrapping_add(base) & width;
        }
        regs
    }

    fn start_eflags(&self) -> u32 {
        Regs::normalise_flags(self.variant, self.eflags)
    }
}

/// What comparing one case established.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Verdict {
    /// The first instruction was outside the subset, so there was nothing to
    /// compare. Not a failure: the block is still well-formed, and the
    /// interpreter picks the instruction up itself.
    Nothing,
    /// Both engines stopped on a fault at the same guest instruction, in the
    /// same architectural state.
    Trapped {
        /// How many guest instructions **retired** before the fault. The
        /// faulting one is not among them: it opened its boundary and did not
        /// complete, which is exactly what makes the state comparable.
        insns: usize,
    },
    /// They agreed on every column.
    Agreed {
        /// How many guest instructions the block retired.
        insns: usize,
        /// How many ticks both charged.
        ticks: u64,
    },
}

/// The oracle and the subject disagreed.
///
/// Carries the program, because a fuzzer's finding is useless without the
/// bytes that produced it, and the disassembly, because the bytes are not
/// readable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Divergence {
    /// Which column disagreed, and how.
    pub what: String,
    /// The program, disassembled, and the registers it started with.
    pub program: String,
}

impl core::fmt::Display for Divergence {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}\n{}", self.what, self.program)
    }
}

// ---------------------------------------------------------------------------
// The single-block harness
// ---------------------------------------------------------------------------

/// Compare the lifted path against the interpreter for one case.
///
/// # Errors
///
/// [`Divergence`] when the two disagree on any column, and — because a block
/// the verifier rejects is a frontend bug of exactly the same kind — when the
/// block this frontend produced does not verify.
///
/// # Panics
///
/// If the case's program does not fit in the first page, which is harness
/// misuse rather than a finding.
#[allow(clippy::missing_panics_doc)]
pub fn compare(case: &Case) -> Result<Verdict, Divergence> {
    assert!(
        (case.program.len() as u64) < DATA,
        "a case's program lives in the first page"
    );

    let world = world(case);
    // Two identical machines, so a store in one cannot be seen by the other.
    let (oracle_space, oracle_ram) = machine(case);
    let (subject_space, subject_ram) = machine(case);

    // ---- the subject: lift, verify, run on the portable backend ----------
    let mut src = Bytes {
        program: &case.program,
    };
    let lifted = lift::lift(
        &world,
        BASE,
        &mut src,
        lift::MAX_INSNS,
        case.shape,
        case.smc,
        case.flags,
    )
    .map_err(|e| diverged(case, format!("the frontend refused the case: {e}")))?;
    if lifted.insns == 0 {
        return Ok(Verdict::Nothing);
    }
    if let Err(e) = verify(&lifted.block) {
        return Err(diverged(
            case,
            format!(
                "the frontend produced a block the verifier rejects: {e}\n{}",
                lifted.block
            ),
        ));
    }

    let mut host = Host::new(case, subject_space);
    // The entry fetch translation, before anything runs. See `Host::enter`:
    // the block makes no fetches and still owes the one the interpreter's
    // first fetch makes.
    if host.enter(world.linear(BASE)).is_err() {
        return Err(diverged(
            case,
            String::from("the entry page could not be translated through the fetch path"),
        ));
    }
    let mut interp = Interp::new();
    let outcome = interp
        .run(&lifted.block, &mut host)
        .map_err(|e| diverged(case, format!("the backend refused the block: {e}")))?;

    // How many guest instructions actually retired, which is not
    // `Lifted::insns` once a block has side exits.
    let retired = interp.boundaries().saturating_sub(1) as usize;
    let subject_faulted = matches!(outcome, Outcome::Fault(_));

    // ---- the oracle: the interpreter, the same instructions --------------
    let cpu = oracle(case, oracle_space);
    let want = retired + usize::from(subject_faulted);
    let mut stepped = 0usize;
    while stepped < want && !cpu.is_halted() {
        cpu.step();
        stepped += 1;
    }

    let oracle_trapped = cpu.is_halted();
    if oracle_trapped != subject_faulted {
        return Err(diverged(
            case,
            format!(
                "the interpreter {} and the lifted block {} (outcome {outcome:?}, after {stepped} \
                 of {want} steps)",
                if oracle_trapped {
                    "trapped"
                } else {
                    "did not trap"
                },
                if subject_faulted {
                    "faulted"
                } else {
                    "did not fault"
                },
            ),
        ));
    }

    if let Outcome::Fault(fault) = &outcome {
        state(case, &cpu, &host, fault.pc, "the block", true)?;
        memory(case, &oracle_ram, &subject_ram)?;
        return Ok(Verdict::Trapped { insns: retired });
    }

    if !matches!(outcome, Outcome::Exit) {
        return Err(diverged(
            case,
            format!("a lifted block must end in exit_tb, but it reported {outcome:?}"),
        ));
    }

    let pc = host.slot(RIP);
    state(case, &cpu, &host, pc, "the lifted block", false)?;

    // The cumulative column the block publishes at its boundaries is *static*
    // — the charges the frontend could know at lift time — and the accesses
    // spend the rest. Those two adding up is the whole of the IR's decision 2,
    // and a fault taken at a boundary hashes on the column rather than on the
    // total.
    let column = interp
        .mark()
        .and_then(|m| lifted.block.marks().get(m as usize))
        .map_or(0, |m| m.ticks);
    if column + host.access_ticks != host.ticks {
        return Err(diverged(
            case,
            format!(
                "the exit boundary's tick column says {column} and the accesses spent {}, \
                 but {} ticks were charged",
                host.access_ticks, host.ticks
            ),
        ));
    }

    memory(case, &oracle_ram, &subject_ram)?;

    Ok(Verdict::Agreed {
        insns: retired,
        ticks: host.ticks,
    })
}

/// Every architectural column: the eight registers, `EIP`, `EFLAGS` whole, and
/// the cycle counter.
///
/// **This is the hard half of `ROADMAP.md` §9** when `at_fault` holds: *"when
/// a load faults halfway through a translated block, the guest must observe
/// exactly the architectural state its ISA specifies at that instruction — the
/// right PC, the right registers, and nothing from instructions that had not
/// yet retired."* A trace faults with eight registers and six flags living in
/// temporaries and a `EIP` that is a constant in a boundary record rather than
/// anything the block computed, so "the right registers" is a claim about the
/// whole lazy-publication scheme rather than about the load.
fn state(
    case: &Case,
    cpu: &X86,
    host: &Host,
    pc: u64,
    what: &str,
    at_fault: bool,
) -> Result<(), Divergence> {
    let regs = cpu.regs();
    let when = if at_fault { " at the fault" } else { "" };
    let names = if case.long { REG_NAMES64 } else { REG_NAMES };
    // All sixteen, at their full width, in both worlds. A 32-bit case cannot
    // decode a register number above seven and cannot leave anything in the
    // top half of one below it — `Regs::set_dword` zero-extends — so comparing
    // sixty-four bits of sixteen registers there is the same assertion said
    // more strongly, and it is one comparison rather than two.
    for n in 0..16u8 {
        let want = regs.qword(n);
        let got = host.slot(r_slot(n));
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "{}{when}: the interpreter says {want:#018x}, {what} says {got:#018x}",
                    names[n as usize]
                ),
            ));
        }
    }

    // At an exit the resume `EIP` is in its slot; at a fault it is not, and
    // deliberately so — `EIP` is bound only at an exit boundary, and the
    // architectural program counter of a faulting instruction is carried by
    // `Fault::pc` instead. A frontend that bound it at every boundary would
    // spend a constant move per guest instruction to say what the boundary
    // record already says.
    let want_pc = regs.rip;
    let got_pc = pc;
    if want_pc != got_pc {
        return Err(diverged(
            case,
            format!(
                "{}{when}: the interpreter says {want_pc:#018x}, {what} says {got_pc:#018x}",
                if case.long { "rip" } else { "eip" }
            ),
        ));
    }

    // The flags, whole. Six of them live in their own slots and the rest in
    // one more; a lifter that elided a flag it should have kept, or that
    // published the wrong boundary's map, shows up here and nowhere else.
    let want_flags = regs.eflags;
    let got_flags = host.eflags();
    if want_flags != got_flags {
        let differing = want_flags ^ got_flags;
        return Err(diverged(
            case,
            format!(
                "eflags{when}: the interpreter says {want_flags:#010x}, {what} says \
                 {got_flags:#010x} — differing in {}",
                name_flags(differing)
            ),
        ));
    }

    let want_ticks = cpu.cycles();
    if want_ticks != host.ticks {
        return Err(diverged(
            case,
            format!(
                "ticks{when}: the interpreter charged {want_ticks}, {what} charged {}. A cache \
                 hit and a cache miss must be indistinguishable to the guest, including in cycle \
                 accounting (ROADMAP.md §0)",
                host.ticks
            ),
        ));
    }
    Ok(())
}

/// Compare guest RAM byte for byte.
fn memory(case: &Case, oracle: &RamStore, subject: &RamStore) -> Result<(), Divergence> {
    for off in 0..case.ram_size() {
        let want = oracle.read_u8(off).unwrap_or(0);
        let got = subject.read_u8(off).unwrap_or(0);
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "memory at {:#x}: the interpreter left {want:#04x}, the lifted block left \
                     {got:#04x}",
                    BASE + off
                ),
            ));
        }
    }
    Ok(())
}

/// The names of the flags in a mask, for a report nobody has to decode by
/// hand.
fn name_flags(mask: u32) -> String {
    let mut out = String::new();
    for (bit, name) in [
        (flags::CF, "CF"),
        (flags::PF, "PF"),
        (flags::AF, "AF"),
        (flags::ZF, "ZF"),
        (flags::SF, "SF"),
        (flags::OF, "OF"),
        (flags::TF, "TF"),
        (flags::IF, "IF"),
        (flags::DF, "DF"),
    ] {
        if mask & bit != 0 {
            if !out.is_empty() {
                out.push(' ');
            }
            out.push_str(name);
        }
    }
    if out.is_empty() {
        out.push_str("no named bit");
    }
    out
}

const REG_NAMES: [&str; 16] = [
    "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi", "r8", "r9", "r10", "r11", "r12", "r13",
    "r14", "r15",
];

/// The same sixteen as long mode names them.
const REG_NAMES64: [&str; 16] = [
    "rax", "rcx", "rdx", "rbx", "rsp", "rbp", "rsi", "rdi", "r8", "r9", "r10", "r11", "r12", "r13",
    "r14", "r15",
];

/// Build the report for a disagreement, disassembling the program into it.
fn diverged(case: &Case, what: String) -> Divergence {
    let mut program = String::new();
    let bytes = &case.program;
    let listing = super::disasm::disassemble_run_as(
        case.variant.map(),
        world(case).bits,
        CODE_SEL,
        BASE,
        32,
        |addr| {
            addr.checked_sub(BASE)
                .and_then(|off| usize::try_from(off).ok())
                .and_then(|i| bytes.get(i))
                .copied()
        },
    );
    for line in listing {
        program.push_str(&format!("  {:#010x}  {line}\n", line.ip));
    }
    let regs = case.start_regs();
    let names = if case.long { REG_NAMES64 } else { REG_NAMES };
    for (n, value) in regs.iter().enumerate() {
        if *value != 0 {
            program.push_str(&format!("  {} = {value:#018x}\n", names[n]));
        }
    }
    program.push_str(&format!(
        "  eflags = {:#010x}  shape {:?}  smc {:?}  flags {:?}  world {:?}{}\n",
        case.start_eflags(),
        case.shape,
        case.smc,
        case.flags,
        world(case).bits,
        if case.paged { " paged" } else { "" },
    ));
    Divergence { what, program }
}

/// The configuration the oracle is built from.
#[must_use]
pub fn config(case: &Case) -> Config {
    Config::I8088.with_variant(case.variant)
}

/// The world this case lifts in.
///
/// Written out rather than derived, because the machine below is built to *be*
/// this world; `a_hand_written_world_is_the_one_world_of_finds` asserts that
/// [`World::of`] agrees, which is the property that matters.
///
/// Public so that `benches/x86_dispatch.rs` measures **this** machine rather
/// than one of its own that has drifted from it.
#[must_use]
pub fn world(case: &Case) -> World {
    // Flat code, based data. `CS` is the odd one out and has to be written
    // down as such: the code lives at linear [`BASE`] because the program
    // counter starts there, not because the segment moves it.
    //
    // In 64-bit mode every one of the six is flat, and that is the
    // architecture's answer rather than this machine's: `CS`, `DS`, `ES` and
    // `SS` are treated as having a base of zero whatever their descriptors
    // hold, and `FS` and `GS` take theirs from an MSR this harness leaves at
    // zero (*Intel SDM* volume 3 §3.4.4).
    let mut seg_base = [if case.long { 0 } else { BASE }; seg::COUNT];
    seg_base[usize::from(seg::CS)] = 0;
    World {
        variant: case.variant,
        bits: if case.long { Bits::B64 } else { Bits::B32 },
        cs_base: 0,
        seg_base,
        // A 386 and a 486 both have `CMOVcc` clear by default, and `Exec`
        // raises `#UD` for one — so this has to be the core's own answer or the
        // lifter lifts an instruction the interpreter refuses.
        cmov: config(case).features.cmov,
        generation: 0,
        // The physical page linear `BASE` resolves to, which is what names a
        // paged block. Written down here and *derived* by the harness itself:
        // `a_paged_entry_resolves_to_the_page_the_world_claims` asserts that
        // the machine's own tables agree, which is the property that matters.
        origin: if case.paged {
            Origin::Paged {
                phys: PAGED_PROGRAM,
            }
        } else {
            Origin::Flat
        },
    }
}

/// The four-byte legacy page-table entry `value` at physical address `phys`.
fn put_entry(ram: &RamStore, phys: u64, value: u32) {
    for i in 0..4u64 {
        ram.write_u8(phys - BASE + i, (value >> (8 * i as u32)) as u8)
            .expect("a table entry is inside the region");
    }
}

/// The eight-byte entry `value` at physical address `phys`, which is what
/// every level of an IA-32e walk holds.
fn put_entry64(ram: &RamStore, phys: u64, value: u64) {
    for i in 0..8u64 {
        ram.write_u8(phys - BASE + i, (value >> (8 * i as u32)) as u8)
            .expect("a table entry is inside the region");
    }
}

/// Map the four guest pages through the **four-level** walk long mode
/// requires.
///
/// One entry per level down to a page table that maps the four pages one at a
/// time — deliberately not a 2 MiB large page, because a large page has no
/// page table and this harness wants the walk to be four reads deep and the
/// accessed and dirty bits to land in an entry that maps exactly one page.
///
/// Supervisor, present and writable, with accessed and dirty clear, for the
/// same reasons [`map_pages`] gives.
fn map_pages64(ram: &RamStore) {
    let table = pte::PRESENT | pte::WRITABLE;
    // The low window, where the code is.
    put_entry64(ram, PML4 + 8 * ((BASE >> 39) & 0x1ff), PDPT | table);
    put_entry64(ram, PDPT + 8 * ((BASE >> 30) & 0x1ff), PDIR64 | table);
    put_entry64(ram, PDIR64 + 8 * ((BASE >> 21) & 0x1ff), PTAB64 | table);
    let first = (BASE >> 12) & 0x1ff;
    for k in 0..RAM_SIZE / 4096 {
        put_entry64(
            ram,
            PTAB64 + 8 * (first + k),
            (PAGED_PROGRAM + k * 4096) | table,
        );
    }
    // The high window, at [`HIGH`], onto the **same four frames**. A separate
    // chain from the top level down, because two and a half tebibytes up is a
    // different entry of every table on the way — which is what makes an
    // address here one that only sixty-four bits can hold.
    put_entry64(ram, PML4 + 8 * ((HIGH >> 39) & 0x1ff), PDPT_HIGH | table);
    put_entry64(
        ram,
        PDPT_HIGH + 8 * ((HIGH >> 30) & 0x1ff),
        PDIR_HIGH | table,
    );
    put_entry64(
        ram,
        PDIR_HIGH + 8 * ((HIGH >> 21) & 0x1ff),
        PTAB_HIGH | table,
    );
    let first = (HIGH >> 12) & 0x1ff;
    for k in 0..RAM_SIZE / 4096 {
        put_entry64(
            ram,
            PTAB_HIGH + 8 * (first + k),
            (PAGED_PROGRAM + k * 4096) | table,
        );
    }
}

/// Map the four guest pages, in the two-level scheme a 32-bit `CR0.PG` with
/// `CR4.PAE` clear puts in force.
///
/// Supervisor pages, present and writable — the harness runs at ring 0 — so
/// nothing here faults for a *permission* reason and every page fault a case
/// can take is one the case caused. The accessed and dirty bits start clear on
/// purpose: the first fetch and the first write to each page are what set
/// them, both engines must set exactly the same ones, and [`memory`] compares
/// the tables along with everything else.
fn map_pages(ram: &RamStore) {
    let pde = BASE >> 22;
    put_entry(
        ram,
        PDIR + 4 * pde,
        (PTAB | pte::PRESENT | pte::WRITABLE) as u32,
    );
    let first = (BASE >> 12) & 0x3ff;
    for k in 0..RAM_SIZE / 4096 {
        let frame = PAGED_PROGRAM + k * 4096;
        put_entry(
            ram,
            PTAB + 4 * (first + k),
            (frame | pte::PRESENT | pte::WRITABLE) as u32,
        );
    }
}

/// One RAM, one space, the program loaded.
///
/// Public for the same reason [`world`] is: a benchmark that built its own
/// machine would eventually measure a different one.
#[must_use]
pub fn machine(case: &Case) -> (Arc<AddressSpace>, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(case.ram_size()));
    let at = case.program_offset();
    for (n, byte) in case.program.iter().enumerate() {
        ram.write_u8(at + n as u64, *byte)
            .expect("the program fits");
    }
    // A byte the lifter refuses, so a run that falls off the end of the
    // program stops cleanly rather than executing whatever the data window
    // happens to hold.
    ram.write_u8(at + case.program.len() as u64, 0xf4)
        .expect("the terminator fits");
    if case.long || case.compat {
        map_pages64(&ram);
    } else if case.paged {
        map_pages(&ram);
    }
    let space = AddressSpace::new("mem", 32);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), BASE)
        .expect("one region maps");
    (Arc::new(space), ram)
}

/// The system registers this case's machine runs with.
///
/// One factory, because two of them must agree exactly: the oracle is an
/// [`X86`] and the subject's memory path is an `Exec` over its own
/// `State`, and a difference in a segment limit or a `CR3` between them
/// would show up as a divergence in the *lifter*.
#[must_use]
pub fn system(case: &Case) -> Sys {
    let mut sys = Sys::reset();
    sys.cr0 |= cr0::PE;
    // Zero limit, on purpose: see the module docs. The first exception cannot
    // read its gate, escalates, and shuts the processor down with the
    // architectural state of the faulting instruction still in place.
    sys.idtr.base = 0;
    sys.idtr.limit = 0;
    sys.gdtr.base = 0;
    sys.gdtr.limit = 0;
    sys.segs[usize::from(seg::CS)] = SegReg {
        selector: CODE_SEL,
        base: 0,
        limit: 0xffff_ffff,
        ar: if case.long { CODE64 } else { CODE32 },
    };
    for index in [seg::DS, seg::ES, seg::SS, seg::FS, seg::GS] {
        sys.segs[usize::from(index)] = SegReg {
            selector: DATA_SEL,
            // 64-bit mode gives the data segments a base of zero and no limit
            // whatever the descriptor says, so writing the limit down here
            // would be writing down something nothing reads. The **fault** a
            // 64-bit case takes on an address outside the window is a `#PF`
            // from an unmapped page rather than a `#GP` from a segment limit,
            // which is a different vector and the same compared column: both
            // engines stop at the same instruction in the same state.
            base: if case.long { 0 } else { BASE },
            limit: if case.long {
                0xffff_ffff
            } else {
                (RAM_SIZE - 1) as u32
            },
            ar: DATA32,
        };
    }
    if case.long || case.compat {
        // The order is the manual's, minus the guest instructions that would
        // have performed it: `CR4.PAE`, `CR3`, `EFER.LME`, then `CR0.PG` —
        // at which point *the processor* sets `EFER.LMA`. This harness sets
        // both bits itself because it builds the state rather than reaching
        // it, and `cpu::x86::tests` is where the transition is executed as
        // real instructions.
        sys.cr4 |= cr4::PAE;
        sys.cr3 = PML4;
        sys.efer |= efer::LME | efer::LMA;
        sys.cr0 |= cr0::PG;
    } else if case.paged {
        // `CR4.PAE` stays clear, so this is the two-level walk a 32-bit guest
        // uses whatever the part is wide enough to do — `Mode::Legacy`, four
        // bytes an entry, one directory and one table.
        sys.cr3 = PDIR;
        sys.cr0 |= cr0::PG;
    }
    sys
}

/// A core already in the world [`world`] describes, with the reset sequence
/// discharged and its interrupt table deliberately unusable.
///
/// See the module docs for why the table is unusable rather than absent.
#[must_use]
pub fn oracle(case: &Case, space: Arc<AddressSpace>) -> X86 {
    let cpu = X86::new(config(case));
    cpu.attach_space(space);
    cpu.set_sys(system(case));

    let mut regs = Regs::new();
    regs.cs = CODE_SEL;
    regs.ss = DATA_SEL;
    regs.ds = DATA_SEL;
    regs.es = DATA_SEL;
    regs.fs = DATA_SEL;
    regs.gs = DATA_SEL;
    let start = case.start_regs();
    for (n, value) in start.iter().enumerate() {
        regs.set_qword(n as u8, *value);
    }
    regs.rip = BASE;
    regs.eflags = case.start_eflags();
    cpu.set_regs(regs);
    cpu.session.lock().state.reset_pending = false;
    cpu
}

/// The lifter's view of the program: bytes out of the case's own vector, with
/// nothing outside it readable.
struct Bytes<'a> {
    program: &'a [u8],
}

impl lift::InsnSource for Bytes<'_> {
    fn byte(&mut self, addr: u64) -> Option<u8> {
        let off = addr.checked_sub(BASE)?;
        self.program.get(usize::try_from(off).ok()?).copied()
    }
}

// ---------------------------------------------------------------------------
// The guest state a lifted block runs against
// ---------------------------------------------------------------------------

/// The segment checks a data access makes, and the bus transaction it costs.
///
/// `Exec::seg_linear` in the shape a host can offer: present, permission,
/// limit, then base — and the checks happen *before* anything is charged,
/// which is why a `#GP` costs nothing and a real access costs one bus cycle.
#[derive(Debug, Clone, Copy)]
struct Segments {
    base: [u64; seg::COUNT],
    limit: [u64; seg::COUNT],
}

impl Segments {
    const fn flat_data() -> Segments {
        Segments {
            base: [BASE; seg::COUNT],
            limit: [RAM_SIZE - 1; seg::COUNT],
        }
    }

    /// The linear address, or the fault a segment check raised.
    fn linear(&self, sr: u8, offset: u64, size: u64) -> MemResult<u64> {
        let sr = usize::from(sr);
        let last = offset.checked_add(size - 1).ok_or(BusError::Protected)?;
        if last > self.limit[sr] {
            // `#GP`, or `#SS` through the stack. The IR carries one bus error
            // and the vector is the interpreter's business; what matters here
            // is that both engines agree there *was* a fault, at the same
            // instruction, in the same state.
            return Err(BusError::Protected);
        }
        Ok(self.base[sr].wrapping_add(offset))
    }
}

/// What an entry translation cost, which is nothing at all when it failed.
///
/// **A failed entry translation is not the block's charge**, and getting that
/// backwards is a double charge rather than a missing one. The runtime
/// translates a block's entry *before* deciding to run it; when the
/// translation faults the block does not run, the guest PC goes back to the
/// interpreter, and the interpreter's own fetch walks those same tables and
/// charges for them. A successful walk is different in exactly the way that
/// matters: it filled the translation buffer, so the interpreter's fetch finds
/// the entry and charges nothing.
///
/// Rolling the counter back is safe because a walk that fails writes nothing —
/// `translate_access` returns before the accessed and dirty bits are written,
/// on a missing entry and on a refused permission alike — so the only trace of
/// it is the cycles, and `CR2`, which the interpreter is about to latch again
/// itself.
///
/// Discovered by `the_paged_corpus_agrees_through_the_cached_and_chained_runtime`,
/// on a case whose conditional jump left the mapped window: the block cache
/// entered at an unmapped PC, charged the walk that failed there, and came out
/// four ticks — one two-level walk — ahead of the interpreter.
fn charge_of(state: &mut State, before: u64, ok: bool) -> u64 {
    if !ok {
        state.cycles = before;
        return 0;
    }
    state.cycles.wrapping_sub(before)
}

/// The guest state a lifted block runs against.
///
/// Slots rather than a register struct, because that is all the backend knows
/// about: the frontend numbered them and nothing below it interprets the
/// numbering.
struct Host {
    slots: [u64; SLOT_COUNT as usize],
    space: Arc<AddressSpace>,
    attrs: MemAttrs,
    segs: Segments,
    bus: u64,
    /// Ticks charged, by `CHARGE` and by the accesses this host performed.
    ticks: u64,
    /// Of those, the ones the accesses spent — the data-dependent half, which
    /// the frontend deliberately leaves out of [`InsnStart::ticks`].
    access_ticks: u64,
    /// The interpreter's own memory path, present exactly when the case pages.
    mmu: Option<Mmu>,
}

/// Enough of a core for [`Exec`] to exist over: the memory path under paging.
///
/// **Not a memory path that agrees with the interpreter's — the
/// interpreter's.** `Exec::read_mem` and `Exec::write_mem` are what
/// `Exec::step` itself calls, so the segment check, the translation with its
/// accessed and dirty bits, the page-crossing split and the bus transaction
/// are one implementation rather than two. That matters more under paging than
/// anywhere else: a second implementation would have to reproduce a walk's
/// tick cost, the order its entries are read in, and the rule that a write to
/// a page whose dirty bit is clear takes the long way round even on a
/// translation-buffer hit — and getting any of those wrong would be reported
/// as a *lifter* divergence.
///
/// The unpaged path deliberately keeps [`Segments`] instead, because that is
/// the harness this frontend's nineteen-of-twenty bug-injection score was
/// measured with and changing it would be changing the instrument.
#[derive(Debug)]
struct Mmu {
    state: State,
    cfg: Config,
    lines: Lines,
}

impl Host {
    fn new(case: &Case, space: Arc<AddressSpace>) -> Host {
        let mut slots = [0u64; SLOT_COUNT as usize];
        let start = case.start_regs();
        for (n, value) in start.iter().enumerate() {
            slots[n] = *value;
        }
        slots[RIP.0 as usize] = BASE;
        let eflags = case.start_eflags();
        for (i, bit) in FLAG_BITS.iter().enumerate() {
            slots[FLAG_SLOTS[i].0 as usize] = u64::from(eflags & bit != 0);
        }
        slots[EFLAGS_REST.0 as usize] = u64::from(eflags & !ARITH_MASK);
        Host {
            slots,
            space,
            attrs: MemAttrs::DEFAULT,
            segs: Segments::flat_data(),
            bus: u64::from(case.variant.bus_clocks()),
            ticks: 0,
            access_ticks: 0,
            mmu: case.paged.then(|| Mmu::new(case)),
        }
    }

    /// The entry fetch translation this block owes, charged.
    ///
    /// **The contract the module docs of [`lift`] call the one that looks like
    /// a working JIT.** A translated block makes no fetches, but the
    /// instruction it replaced translated its first byte through the fetch
    /// path — walking the tables on a buffer miss, charging two bus reads per
    /// level and writing the accessed bit — and a block that skipped that
    /// would run the same instructions for fewer ticks. So it happens here,
    /// on every execution of every block, exactly as `cpu::riscv::engine`'s
    /// `admit` does it, and the physical page it answers is what the block is
    /// keyed on.
    ///
    /// The ticks land in [`Host::access_ticks`] rather than in the block's
    /// static column, because the frontend cannot know at lift time whether
    /// the buffer will hit.
    fn enter(&mut self, linear: u64) -> MemResult<u64> {
        let Host {
            mmu,
            space,
            ticks,
            access_ticks,
            ..
        } = self;
        let Some(mmu) = mmu.as_mut() else {
            // With `CR0.PG` clear a linear address is a physical one and there
            // is nothing to translate, which is a different fact from there
            // being nothing to charge.
            return Ok(linear);
        };
        let before = mmu.state.cycles;
        let user = mmu.state.regs.cs & 3 == 3;
        let answer = {
            let mut exec = Exec::new(&mut mmu.state, space, None, &mmu.cfg, &mmu.lines);
            exec.translate_access(linear, Access::fetch(user))
        };
        let spent = charge_of(&mut mmu.state, before, answer.is_ok());
        *ticks += spent;
        *access_ticks += spent;
        answer.map_err(|_| BusError::Protected)
    }

    /// One data access through the interpreter's own path.
    fn paged_access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let Host {
            mmu,
            space,
            ticks,
            access_ticks,
            ..
        } = self;
        let mmu = mmu.as_mut().expect("only a paged host reaches here");
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let size = mem.size.bytes() as u8;
        let before = mmu.state.cycles;
        let answer = {
            let mut exec = Exec::new(&mut mmu.state, space, None, &mmu.cfg, &mmu.lines);
            match value {
                None => exec.read_mem(sr, addr, size),
                Some(v) => exec.write_mem(sr, addr, size, v).map(|()| 0),
            }
        };
        let spent = mmu.state.cycles.wrapping_sub(before);
        *ticks += spent;
        *access_ticks += spent;
        // The IR carries one bus error and the vector is the interpreter's
        // business: `#GP`, `#SS` and now `#PF` all arrive here as *a fault*,
        // and what is compared is that both engines took one at the same
        // instruction in the same state.
        answer.map_err(|_| BusError::Protected)
    }

    fn slot(&self, slot: RegSlot) -> u64 {
        self.slots[slot.0 as usize]
    }

    /// The packed flags word, reassembled from the seven slots that hold it.
    fn eflags(&self) -> u32 {
        let mut value = self.slots[EFLAGS_REST.0 as usize] as u32;
        for (i, bit) in FLAG_BITS.iter().enumerate() {
            if self.slots[FLAG_SLOTS[i].0 as usize] & 1 != 0 {
                value |= bit;
            }
        }
        value
    }

    fn charge_bus(&mut self) {
        self.ticks += self.bus;
        self.access_ticks += self.bus;
    }

    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        // `lift::TRANSFER` is not a space anything is in: it is a computed
        // near transfer asking whether its target may be transferred to, which
        // is `Exec::jump_near`'s canonical test. No memory, no bus cycle, no
        // clocks. See `lift`'s constant for the whole contract.
        if mem.space == lift::TRANSFER {
            return if crate::cpu::x86::prot::canonical(addr) {
                Ok(0)
            } else {
                Err(BusError::Protected)
            };
        }
        if self.mmu.is_some() {
            return self.paged_access(mem, addr, value);
        }
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let lin = self.segs.linear(sr, addr, mem.size.bytes())?;
        // Paging is out of the lifted subset, so a whole access is one bus
        // transaction whatever its alignment: only a page crossing splits one,
        // and `Exec::linear_read` only splits when paging is on.
        self.charge_bus();
        match value {
            None => self.space.read(lin, mem.size, self.attrs),
            Some(v) => self.space.write(lin, mem.size, v, self.attrs).map(|()| 0),
        }
    }
}

impl Mmu {
    /// The interpreter's state, in the world [`system`] describes.
    ///
    /// Only the parts a memory access reads are meaningful: the system
    /// registers, the translation buffers, the cycle counter, and `CS`'s
    /// selector — whose low two bits are the privilege level every translation
    /// consults. The general registers live in [`Host::slots`], because that
    /// is all a lifted block knows about.
    fn new(case: &Case) -> Mmu {
        let cfg = config(case);
        let mut state = State::new(cfg.variant);
        state.sys = system(case);
        state.regs.cs = CODE_SEL;
        state.cycles = 0;
        state.reset_pending = false;
        Mmu {
            state,
            cfg,
            lines: Lines::default(),
        }
    }

    /// Where a linear address is — the whole physical address, not its page —
    /// without touching anything.
    ///
    /// Wanted by the cached path, which logs a store by the physical page it
    /// reached, and by the test that checks this machine's tables resolve the
    /// entry to the page [`world`] claims. Neither exists in a build with no
    /// `jit` and no tests, hence the gate.
    ///
    /// The self-modifying-code half needs the *physical* page a store reached
    /// and `Exec::write_mem` does not report one. A debug walk answers it with
    /// none of the side effects — no accessed bit, no `CR2`, no buffer fill,
    /// no cycles — which is exactly right here: every one of those has already
    /// happened, on the executing walk the store itself made.
    #[cfg(any(feature = "jit", test))]
    fn phys_of(&self, space: &AddressSpace, linear: u64) -> Option<u64> {
        match debug_translate(&self.state.sys, self.cfg.features, space, linear) {
            DebugTranslation::Identity => Some(linear),
            DebugTranslation::Mapped(phys) => Some(phys),
            DebugTranslation::Unmapped => None,
        }
    }
}

/// No table for a backend to inline: this host reaches the address space
/// directly, and every load takes the call.
#[cfg(feature = "jit")]
impl FastMem for Host {}

impl IrHost for Host {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slot(slot))
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots[slot.0 as usize] = value as u64;
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        self.access(mem, addr, None)
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        self.access(mem, addr, Some(value)).map(|_| ())
    }

    fn charge(&mut self, ticks: u64) {
        self.ticks += ticks;
    }

    fn insn_start(&mut self, _mark: &InsnStart) {}
}

// ---------------------------------------------------------------------------
// The cached and chained path
// ---------------------------------------------------------------------------

/// The same comparison, run through the translation runtime rather than one
/// block at a time.
///
/// [`compare`] lifts one block, runs it, and stops. That is the right shape for
/// testing a *frontend*, and it is blind to every mechanism `jit` adds. So this
/// is the second harness:
///
/// | | [`compare`] | [`compare_cached`] |
/// | --- | --- | --- |
/// | blocks | one | up to `blocks`, chained |
/// | translations | one, always fresh | cached under `(pc, key)`, and re-served |
/// | exits | back to the caller | patched straight to the successor |
/// | memory | the address space directly | through `jit::Tlb`, which must answer identically |
/// | instruction bytes | the case's own `Vec<u8>` | **guest RAM**, so a store into the code page is visible |
/// | invalidation | nothing to invalidate | a guest write into a translated page |
///
/// The last two rows are what make self-modifying code testable, and on x86
/// that is the whole point: the architecture guarantees a coherent instruction
/// cache, so a store into a running block's own page must be honoured before
/// the next instruction executes.
///
/// # Errors
///
/// [`Divergence`], on the same columns [`compare`] compares, plus one of its
/// own: a block cache whose back edges stopped being symmetric.
///
/// # Panics
///
/// As [`compare`].
#[cfg(feature = "jit")]
#[allow(clippy::missing_panics_doc)]
pub fn compare_cached(case: &Case, blocks: usize) -> Result<Verdict, Divergence> {
    cached(case, blocks, false)
}

/// [`compare_cached`], with the blocks executed as **host code**.
///
/// The third harness. Everything [`compare_cached`] compares is compared here
/// against the same oracle — the eight general registers, `EIP`, the flags word
/// assembled from its seven slots, the cycle counter, guest memory, and at a
/// fault the state at the faulting instruction — with the only difference being
/// which engine ran the block.
///
/// x86 is the harder of the two frontends for a backend, and deliberately so:
/// an instruction with live flags lifts to something like fifteen IR
/// instructions where an RV64I one lifts to two or three, so this path
/// exercises `movcond`, `popcount`, `deposit`, `extract`, the rotates through
/// carry and both widening multiplies — every one of which the RISC-V harness
/// never emits.
///
/// # Errors
///
/// As [`compare_cached`].
///
/// # Panics
///
/// As [`compare_cached`], plus a code buffer the kernel would not give.
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
#[cfg_attr(docsrs, doc(cfg(feature = "jit-x86")))]
#[allow(clippy::missing_panics_doc)]
pub fn compare_compiled(case: &Case, blocks: usize) -> Result<Verdict, Divergence> {
    cached(case, blocks, true)
}

/// A dispatcher, with the host code generator attached when asked for and
/// available. `compiled` is ignored on a target with no backend.
#[cfg(feature = "jit")]
fn dispatcher(compiled: bool) -> Dispatcher {
    let disp = Dispatcher::with_cache(BlockCache::with_capacity(256));
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    if compiled {
        return disp.with_backend(
            crate::jit::x86::Engine::new().expect("the kernel gave a W^X code buffer"),
        );
    }
    let _ = compiled;
    disp
}

#[cfg(feature = "jit")]
#[allow(clippy::missing_panics_doc)]
fn cached(case: &Case, blocks: usize, compiled: bool) -> Result<Verdict, Divergence> {
    assert!(
        (case.program.len() as u64) < DATA,
        "a case's program lives in the first page"
    );

    let (oracle_space, oracle_ram) = machine(case);
    let (subject_space, subject_ram) = machine(case);

    let mut front = Lifter::new(case, Arc::clone(&subject_space));
    let mut host = CachedHost::new(case, subject_space);
    let mut disp = dispatcher(compiled);
    let run = disp
        .run(&mut front, &mut host, BASE, blocks)
        .map_err(|e| diverged(case, format!("the dispatcher refused a block: {e}")))?;
    if let Some(e) = front.rejected.take() {
        return Err(diverged(case, e));
    }
    if let Err(e) = disp.cache().check() {
        return Err(diverged(
            case,
            format!("the block cache is inconsistent: {e}"),
        ));
    }
    // Nothing retired **and** nothing faulted: the first instruction was
    // outside the subset. A fault on the first instruction of the first block
    // also retires nothing and is the opposite of nothing happening — it is
    // the precise-state comparison at its sharpest, and reporting it as
    // `Nothing` was how this harness quietly passed a computed near transfer
    // whose target it never judged.
    if run.insns == 0 && !matches!(run.stop, Stop::Fault(_)) {
        return Ok(Verdict::Nothing);
    }

    let cpu = oracle(case, oracle_space);
    let subject_faulted = matches!(run.stop, Stop::Fault(_));
    let want = run.insns + usize::from(subject_faulted);
    let mut stepped = 0usize;
    while stepped < want && !cpu.is_halted() {
        cpu.step();
        stepped += 1;
    }

    let oracle_trapped = cpu.is_halted();
    if oracle_trapped != subject_faulted {
        return Err(diverged(
            case,
            format!(
                "the interpreter {} and the cached path {} (stop {:?}, after {stepped} of {want} \
                 steps)",
                if oracle_trapped {
                    "trapped"
                } else {
                    "did not trap"
                },
                if subject_faulted {
                    "faulted"
                } else {
                    "did not fault"
                },
                run.stop,
            ),
        ));
    }

    // `Run::pc` is where the dispatcher would resume, read out of the `EIP`
    // slot at an ordinary exit; at a fault it is the faulting instruction's own
    // address, which `Fault::pc` carries instead.
    let pc = match &run.stop {
        Stop::Fault(fault) => fault.pc,
        _ => run.pc,
    };
    let view = HostView {
        slots: host.slots,
        ticks: host.ticks,
        access_ticks: 0,
    };
    state(
        case,
        &cpu,
        &view.as_host(),
        pc,
        if subject_faulted {
            "the cached path at the fault"
        } else {
            "the cached path"
        },
        subject_faulted,
    )?;
    memory(case, &oracle_ram, &subject_ram)?;

    if subject_faulted {
        return Ok(Verdict::Trapped { insns: run.insns });
    }
    Ok(Verdict::Agreed {
        insns: run.insns,
        ticks: host.ticks,
    })
}

/// Enough of a [`Host`] for [`state`] to read, without a second address space.
#[cfg(feature = "jit")]
struct HostView {
    slots: [u64; SLOT_COUNT as usize],
    ticks: u64,
    access_ticks: u64,
}

#[cfg(feature = "jit")]
impl HostView {
    fn as_host(&self) -> Host {
        Host {
            slots: self.slots,
            space: Arc::new(AddressSpace::new("view", 32)),
            attrs: MemAttrs::DEFAULT,
            segs: Segments::flat_data(),
            bus: 0,
            ticks: self.ticks,
            access_ticks: self.access_ticks,
            // A view, not a machine: nothing here performs an access.
            mmu: None,
        }
    }
}

/// [`compare_cached`], reporting what the run exercised as well as whether it
/// agreed.
///
/// Separate from [`Verdict`] because "the two engines agreed" and "the cache
/// was actually used" are different assertions, and a harness that conflates
/// them stops noticing the day it quietly stops exercising what it was written
/// for.
#[cfg(feature = "jit")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CachedRun {
    /// The verdict.
    pub verdict: Verdict,
    /// Blocks executed.
    pub blocks: usize,
    /// Guest instructions retired across those blocks.
    pub insns_retired: usize,
    /// Blocks translated — one per distinct `(pc, key)` that survived.
    pub translated: u64,
    /// Blocks reached by following a patched exit, with no lookup at all.
    pub chained: u64,
    /// Blocks invalidated by a guest store into their page.
    pub smc: u64,
    /// Blocks executed as compiled host code rather than interpreted.
    ///
    /// Zero on the cached path. What it is *for* is the compiled path: a
    /// harness that quietly stopped compiling anything would still agree with
    /// the interpreter, perfectly, forever.
    pub compiled: u64,
}

/// [`compare_cached`], with the counters that say what it exercised.
///
/// # Errors
///
/// As [`compare_cached`].
///
/// # Panics
///
/// As [`compare_cached`].
#[cfg(feature = "jit")]
#[allow(clippy::missing_panics_doc)]
pub fn measure_cached(case: &Case, blocks: usize) -> Result<CachedRun, Divergence> {
    measure(case, blocks, false)
}

/// [`measure_cached`] with the host code generator attached.
///
/// # Errors
///
/// As [`compare_compiled`].
///
/// # Panics
///
/// As [`compare_compiled`].
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
#[cfg_attr(docsrs, doc(cfg(feature = "jit-x86")))]
#[allow(clippy::missing_panics_doc)]
pub fn measure_compiled(case: &Case, blocks: usize) -> Result<CachedRun, Divergence> {
    measure(case, blocks, true)
}

#[cfg(feature = "jit")]
#[allow(clippy::missing_panics_doc)]
fn measure(case: &Case, blocks: usize, compiled: bool) -> Result<CachedRun, Divergence> {
    let verdict = cached(case, blocks, compiled)?;
    // A second, independent run on a fresh machine: the counters come from a
    // run that agreed with the interpreter, and running it twice is itself a
    // determinism check on the whole path.
    let (space, _ram) = machine(case);
    let mut front = Lifter::new(case, Arc::clone(&space));
    let mut host = CachedHost::new(case, space);
    let mut disp = dispatcher(compiled);
    let run = disp
        .run(&mut front, &mut host, BASE, blocks)
        .map_err(|e| diverged(case, format!("the dispatcher refused a block: {e}")))?;
    Ok(CachedRun {
        verdict,
        blocks: run.blocks,
        insns_retired: run.insns,
        translated: disp.stats().translated,
        chained: disp.stats().chained,
        smc: disp.stats().smc,
        compiled: disp.stats().compiled,
    })
}

/// The x86 half of the dispatcher's contract: lift on demand, out of guest RAM.
#[cfg(feature = "jit")]
struct Lifter {
    world: World,
    shape: Shape,
    smc: Smc,
    flags: Flags,
    space: Arc<AddressSpace>,
    attrs: MemAttrs,
    /// The first block the verifier rejected, reported as a divergence rather
    /// than swallowed.
    rejected: Option<String>,
}

#[cfg(feature = "jit")]
impl Lifter {
    fn new(case: &Case, space: Arc<AddressSpace>) -> Lifter {
        Lifter {
            world: world(case),
            shape: case.shape,
            smc: case.smc,
            flags: case.flags,
            space,
            attrs: MemAttrs::DEFAULT,
            rejected: None,
        }
    }
}

#[cfg(feature = "jit")]
impl Frontend<CachedHost> for Lifter {
    fn epoch(&mut self) -> Epoch {
        // Nothing in the lifted subset can change the world — no segment load,
        // no `CR0` write, no `LGDT`, no `MOV CR3` — so the world generation
        // never moves and the topology half is the only one that can. A
        // dispatcher wired to a real machine bumps `World::generation`
        // instead, and that lands in `Block::key` rather than here; a paged
        // one is named by the physical page `enter` resolves below, which is
        // in `Block::key` too and for the same reason.
        Epoch {
            topology: self.space.generation(),
            translation: 0,
        }
    }

    /// The entry translation, on **every** execution of the block.
    ///
    /// This is the hook `cpu::riscv::engine` acquired for exactly this job,
    /// and the contract `lift`'s module docs call the one that looks like a
    /// working JIT: a cached block skips its own fetches, and the instruction
    /// it replaced still translated its first byte through the fetch path,
    /// walking the tables on a buffer miss and charging for the walk. Doing it
    /// here rather than at lift time is what makes a served block cost what an
    /// uncached one cost.
    ///
    /// It also *names* the block: the physical page this resolved to goes into
    /// the world, and `Frontend::key` — which the dispatcher asks next — reads
    /// it. There is deliberately no other way to obtain that number.
    fn enter(&mut self, pc: u64, host: &mut CachedHost) -> crate::core::error::Result<Entry> {
        let Origin::Paged { .. } = self.world.origin else {
            return Ok(Entry::Ready);
        };
        let Ok(phys) = host.enter(self.world.linear(pc)) else {
            // The entry page is not there. The interpreter is the oracle for
            // what a `#PF` does next, so the run stops and hands the PC back.
            return Ok(Entry::Leave);
        };
        self.world.origin = Origin::Paged { phys };
        Ok(Entry::Ready)
    }

    fn key(&mut self) -> u64 {
        lift::key(&self.world, self.shape, self.smc, self.flags)
    }

    fn pc_slot(&self) -> RegSlot {
        RIP
    }

    fn translate(&mut self, pc: u64) -> crate::core::error::Result<Translation> {
        // Out of guest RAM, not out of the case's `Vec<u8>`: a store that
        // rewrote an instruction has to be visible here, or the whole
        // self-modifying-code mechanism is untested.
        //
        // The lifter reads **linear** addresses and a block never leaves the
        // page its entry is on, so under paging the one translation `enter`
        // just made covers every byte it may read: the offset within the page
        // is carried and the frame comes from the entry. That is a read-ahead
        // of up to sixty-four instructions the guest has not asked for, which
        // is why it is a plain physical read rather than a second walk — the
        // walk, with its accessed bit and its charge, has already happened.
        let space = Arc::clone(&self.space);
        let attrs = self.attrs;
        let frame = match self.world.origin {
            Origin::Flat => None,
            Origin::Paged { phys } => Some(phys & !lift::PAGE_MASK),
        };
        let mut src = |addr: u64| {
            let at = match frame {
                None => addr,
                Some(frame) => frame | (addr & lift::PAGE_MASK),
            };
            space.read(at, Width::U8, attrs).ok().map(|v| v as u8)
        };
        let lifted = lift::lift(
            &self.world,
            pc,
            &mut src,
            lift::MAX_INSNS,
            self.shape,
            self.smc,
            self.flags,
        )?;
        if self.rejected.is_none()
            && let Err(e) = verify(&lifted.block)
        {
            self.rejected = Some(format!(
                "the frontend produced a block the verifier rejects: {e}\n{}",
                lifted.block
            ));
        }
        Ok(Translation {
            page: lifted.page,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

/// [`Host`], with the memory path routed through a software TLB and every store
/// recorded for the block cache.
#[cfg(feature = "jit")]
struct CachedHost {
    slots: [u64; SLOT_COUNT as usize],
    tlb: Tlb,
    attrs: MemAttrs,
    segs: Segments,
    bus: u64,
    ticks: u64,
    dirty: DirtyPages,
    /// The address space, kept beside the software TLB because the paged path
    /// reaches it through an [`Exec`] rather than through the table.
    space: Arc<AddressSpace>,
    /// The interpreter's own memory path, present exactly when the case pages.
    mmu: Option<Mmu>,
}

/// The world a ring-0 access happens in, with paging off.
#[cfg(feature = "jit")]
const RING0: TlbContext = TlbContext {
    level: 0,
    translating: false,
};

#[cfg(feature = "jit")]
impl CachedHost {
    fn new(case: &Case, space: Arc<AddressSpace>) -> CachedHost {
        let seed = Host::new(case, Arc::clone(&space));
        CachedHost {
            slots: seed.slots,
            tlb: Tlb::new(Arc::clone(&space)),
            attrs: MemAttrs::DEFAULT,
            segs: Segments::flat_data(),
            bus: u64::from(case.variant.bus_clocks()),
            ticks: 0,
            dirty: DirtyPages::new(),
            space,
            mmu: seed.mmu,
        }
    }

    /// The entry fetch translation, charged — [`Host::enter`]'s contract, on
    /// the path where it matters most.
    ///
    /// A cached block is served without being lifted again and a chained one
    /// is reached without a lookup at all, so this is the only thing left that
    /// still costs what the interpreter's first fetch cost. Skipping it is
    /// precisely the bug that makes a second run of the same block cheaper
    /// than its first.
    fn enter(&mut self, linear: u64) -> MemResult<u64> {
        let CachedHost {
            mmu, space, ticks, ..
        } = self;
        let Some(mmu) = mmu.as_mut() else {
            return Ok(linear);
        };
        let before = mmu.state.cycles;
        let user = mmu.state.regs.cs & 3 == 3;
        let answer = {
            let mut exec = Exec::new(&mut mmu.state, space, None, &mmu.cfg, &mmu.lines);
            exec.translate_access(linear, Access::fetch(user))
        };
        *ticks += charge_of(&mut mmu.state, before, answer.is_ok());
        answer.map_err(|_| BusError::Protected)
    }

    /// One data access through the interpreter's path, with the store logged
    /// by the **physical** page it reached.
    ///
    /// Both ends of the self-modifying-code mechanism are physical here, which
    /// is what a linear guard could not be: `jit::cache` invalidates by the
    /// page a translation's bytes came from, and under paging that is not the
    /// page the store's address names.
    fn paged_access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let CachedHost {
            mmu,
            space,
            ticks,
            dirty,
            ..
        } = self;
        let mmu = mmu.as_mut().expect("only a paged host reaches here");
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let size = mem.size.bytes() as u8;
        let before = mmu.state.cycles;
        let (answer, lin) = {
            let mut exec = Exec::new(&mut mmu.state, space, None, &mmu.cfg, &mmu.lines);
            let lin = exec.seg_linear(sr, addr, u64::from(size), value.is_some());
            let answer = match value {
                None => exec.read_mem(sr, addr, size),
                Some(v) => exec.write_mem(sr, addr, size, v).map(|()| 0),
            };
            (answer, lin)
        };
        *ticks += mmu.state.cycles.wrapping_sub(before);
        if answer.is_ok()
            && value.is_some()
            && let Ok(lin) = lin
        {
            // The first and the last byte, because a store that crosses a page
            // boundary lands in two frames that need not be adjacent — and
            // `DirtyPages::note` takes a run of bytes in *one* frame.
            for at in [lin, lin + u64::from(size) - 1] {
                if let Some(phys) = mmu.phys_of(space, at) {
                    dirty.note(phys, 1);
                }
            }
        }
        answer.map_err(|_| BusError::Protected)
    }

    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        // `lift::TRANSFER` is not a space anything is in: it is a computed
        // near transfer asking whether its target may be transferred to, which
        // is `Exec::jump_near`'s canonical test. No memory, no bus cycle, no
        // clocks. See `lift`'s constant for the whole contract.
        if mem.space == lift::TRANSFER {
            return if crate::cpu::x86::prot::canonical(addr) {
                Ok(0)
            } else {
                Err(BusError::Protected)
            };
        }
        if self.mmu.is_some() {
            return self.paged_access(mem, addr, value);
        }
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let lin = self.segs.linear(sr, addr, mem.size.bytes())?;
        self.ticks += self.bus;
        match value {
            None => self
                .tlb
                .read(AccessKind::Load, lin, lin, mem.size, RING0, self.attrs),
            Some(v) => {
                let done = self
                    .tlb
                    .write(lin, lin, mem.size, v, RING0, self.attrs)
                    .map(|()| 0);
                if done.is_ok() {
                    // The self-modifying-code hook. Drained by the dispatcher
                    // at the next block boundary — which the lifter's own
                    // page guard is what *makes* reachable in time, because on
                    // x86 the next instruction may be the one that was
                    // rewritten.
                    self.dirty.note(lin, mem.size.bytes());
                }
                done
            }
        }
    }
}

#[cfg(feature = "jit")]
impl IrHost for CachedHost {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slots[slot.0 as usize])
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots[slot.0 as usize] = value as u64;
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        self.access(mem, addr, None)
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        self.access(mem, addr, Some(value)).map(|_| ())
    }

    fn charge(&mut self, ticks: u64) {
        self.ticks += ticks;
    }

    fn insn_start(&mut self, _mark: &InsnStart) {}
}

#[cfg(feature = "jit")]
impl StoreLog for CachedHost {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        self.dirty.drain_dirty(sink);
    }
}

/// **x86 publishes no fast path, and that is a property of the guest.**
///
/// A load's address here is an *effective* address: the segment base is added
/// and the limit checked before anything reaches the TLB, by `Segments::linear`
/// on this host and by the descriptor cache on a real core. The backend's
/// inlined probe tags on the address it is handed, so it would be tagging on a
/// number one translation short — and the frontend says so in the block, since
/// `cpu::x86::lift` gives every [`MemOp`] a [`SegId`](crate::ir::SegId).
///
/// So the backend refuses to inline a segmented access (`Compiler::inlinable`)
/// and this host offers nothing, which is the same answer said twice.
/// Inlining x86's loads means lowering the segment fold into generated code —
/// a base add and a limit compare against state a `MOV DS, ax` can change
/// between two instructions — and that is a frontend change, not a backend
/// one.
#[cfg(feature = "jit")]
impl FastMem for CachedHost {}

// ---------------------------------------------------------------------------
// The generator
// ---------------------------------------------------------------------------

/// How many general registers [`synthesize`] writes to.
///
/// Seven, not eight: `ESP` is left alone, because a random stack pointer makes
/// every push a segment fault and the corpus would measure the trap path
/// instead of the lifter.
pub const SYNTH_REGS: [u8; 7] = [0, 1, 2, 3, 5, 6, 7];

/// Encode one instruction from inside the lifter's subset.
///
/// `form` picks the encoding and `fields` supplies the register numbers, the
/// immediate and the displacement, so a generator — a fuzzer's byte stream, a
/// seeded pseudo-random sequence — produces programs that *lift* rather than
/// programs that stop at their first instruction. Both numbers are reduced, so
/// every pair of values encodes something.
///
/// The choices that are not arbitrary:
///
/// * A memory operand takes its base from `EAX`..`EBX`, which [`Case::seeded`]
///   points into the data window, with an 8-bit signed displacement. A
///   generator that picked base registers uniformly would fault nearly every
///   time.
/// * Nothing writes `ESP`. See [`SYNTH_REGS`].
/// * A branch displacement is small and signed, so a target stays inside the
///   entry page — a target outside it is a block the lifter refuses, which is
///   correct and uninteresting to generate a thousand of.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn synthesize(form: u32, fields: u32) -> Vec<u8> {
    let reg = SYNTH_REGS[(fields % 7) as usize];
    let rm = SYNTH_REGS[((fields >> 4) % 7) as usize];
    let base = ((fields >> 8) & 3) as u8;
    let disp = (((fields >> 12) & 0x7f) as i32 - 64) as i8;
    let imm8 = (fields >> 16) as u8;
    let imm32 = fields.rotate_left(11);
    let cc = ((fields >> 20) & 15) as u8;
    // Halfword-granular and small, so a taken branch lands on a real
    // instruction boundary more often than not and never leaves the page.
    let rel = ((((fields >> 24) & 0x1f) as i32) - 16) as i8;
    // A byte register number over the **whole** eight, so numbers four to
    // seven — `AH`, `CH`, `DH` and `BH`, which are the *top* halves of
    // `EAX`..`EBX` and are what `Opcode::EXTRACT`'s documentation names x86
    // for. Everything else in this generator writes `reg & 3`.
    let breg = ((fields >> 4) & 7) as u8;
    let breg2 = ((fields >> 20) & 7) as u8;
    // A high byte register only: `mov ah, imm` and `setcc bh` are the two
    // forms here that *write* one, and a write of at most `0x3f` into the
    // second byte of a data-window pointer leaves it inside the segment.
    let high = 4 | (breg & 3);
    // An absolute offset inside the data window, for the addressing modes
    // whose whole address is the displacement.
    let window = (DATA as u32) + ((fields >> 3) & 0xff8);

    // `mod=11` — both operands are registers.
    let rr = |op: u8, r: u8, m: u8| vec![op, 0xc0 | (r << 3) | m];
    // `mod=01` — a base register and an 8-bit displacement. Base 4 would need
    // a SIB byte and base 5 is `EBP`, so the two are simply not generated.
    let rm8 = |op: u8, r: u8, b: u8, d: i8| vec![op, 0x40 | (r << 3) | b, d as u8];
    // The same two behind a `66` operand-size prefix, and a group encoding
    // with the extension in the `reg` field.
    let rr16 = |op: u8, r: u8, m: u8| vec![0x66, op, 0xc0 | (r << 3) | m];
    let rm16 = |op: u8, r: u8, b: u8, d: i8| vec![0x66, op, 0x40 | (r << 3) | b, d as u8];
    let grp16 = |op: u8, n: u8, m: u8| vec![0x66, op, 0xc0 | (n << 3) | m];
    // A SIB byte. `index == 4` means *no index* and `base == 5` with `mod == 0`
    // means *no base*, which are two separate arms of `Lifter::ea`.
    let sib = |scale: u8, index: u8, b: u8| (scale << 6) | (index << 3) | b;

    match form % 98 {
        // -- the ALU, register to register, at three widths -----------------
        0 => rr(0x01, reg, rm),         // add r/m32, r32
        1 => rr(0x03, reg, rm),         // add r32, r/m32
        2 => rr(0x00, reg & 3, rm & 3), // add r/m8, r8
        3 => rr(0x09, reg, rm),         // or
        4 => rr(0x11, reg, rm),         // adc
        5 => rr(0x19, reg, rm),         // sbb
        6 => rr(0x21, reg, rm),         // and
        7 => rr(0x29, reg, rm),         // sub
        8 => rr(0x31, reg, rm),         // xor
        9 => rr(0x39, reg, rm),         // cmp
        10 => rr(0x85, reg, rm),        // test
        // -- the ALU against memory ----------------------------------------
        11 => rm8(0x01, reg, base, disp),
        12 => rm8(0x03, reg, base, disp),
        13 => rm8(0x29, reg, base, disp),
        14 => rm8(0x33, reg, base, disp),
        15 => rm8(0x89, reg, base, disp),     // mov [base+d], r32
        16 => rm8(0x8b, reg, base, disp),     // mov r32, [base+d]
        17 => rm8(0x88, reg & 3, base, disp), // mov [base+d], r8
        18 => rm8(0x8a, reg & 3, base, disp), // mov r8, [base+d]
        19 => rm8(0x8d, reg, base, disp),     // lea
        // -- immediates -----------------------------------------------------
        20 => {
            // group 81 /n imm32
            let ext = (fields >> 24) as u8 & 7;
            let mut out = vec![0x81, 0xc0 | (ext << 3) | rm];
            out.extend_from_slice(&imm32.to_le_bytes());
            out
        }
        21 => {
            // group 83 /n imm8, sign-extended
            let ext = (fields >> 24) as u8 & 7;
            vec![0x83, 0xc0 | (ext << 3) | rm, imm8]
        }
        22 => {
            // mov r32, imm32
            let mut out = vec![0xb8 | reg];
            out.extend_from_slice(&imm32.to_le_bytes());
            out
        }
        23 => vec![0xb0 | (reg & 3), imm8], // mov r8, imm8
        24 => vec![0x40 | reg],             // inc r32
        25 => vec![0x48 | reg],             // dec r32
        // -- shifts and rotates ---------------------------------------------
        26 => {
            let ext = (fields >> 24) as u8 & 7;
            vec![0xc1, 0xc0 | (ext << 3) | rm, imm8 & 0x1f]
        }
        27 => {
            let ext = (fields >> 24) as u8 & 7;
            vec![0xd1, 0xc0 | (ext << 3) | rm]
        }
        28 => {
            let ext = (fields >> 24) as u8 & 7;
            vec![0xd3, 0xc0 | (ext << 3) | rm]
        }
        29 => {
            // the byte forms, where the flag widths are narrowest
            let ext = (fields >> 24) as u8 & 7;
            vec![0xc0, 0xc0 | (ext << 3) | (rm & 3), imm8 & 0x1f]
        }
        // -- multiplies -----------------------------------------------------
        30 => vec![0xf7, 0xe0 | rm],                    // mul r/m32
        31 => vec![0xf7, 0xe8 | rm],                    // imul r/m32
        32 => vec![0xf6, 0xe0 | (rm & 3)],              // mul r/m8
        33 => vec![0x0f, 0xaf, 0xc0 | (reg << 3) | rm], // imul r32, r/m32
        34 => vec![0x6b, 0xc0 | (reg << 3) | rm, imm8], // imul r32, r/m32, imm8
        // -- the unary group ------------------------------------------------
        35 => vec![0xf7, 0xd0 | rm], // not
        36 => vec![0xf7, 0xd8 | rm], // neg
        // -- extensions and bit scans ---------------------------------------
        37 => vec![0x0f, 0xb6, 0xc0 | (reg << 3) | (rm & 3)], // movzx r32, r8
        38 => vec![0x0f, 0xbe, 0xc0 | (reg << 3) | (rm & 3)], // movsx r32, r8
        39 => vec![0x0f, 0xbc, 0xc0 | (reg << 3) | rm],       // bsf
        40 => vec![0x0f, 0xbd, 0xc0 | (reg << 3) | rm],       // bsr
        // -- the condition codes, read three different ways -----------------
        41 => vec![0x70 | cc, rel as u8],             // jcc rel8
        42 => vec![0x0f, 0x90 | cc, 0xc0 | (rm & 3)], // setcc r/m8
        43 => vec![0x0f, 0x40 | cc, 0xc0 | (reg << 3) | rm], // cmovcc
        // -- the stack, and the flag instructions ---------------------------
        44 => vec![0x50 | reg, 0x58 | rm], // push then pop, so the stack stays put
        // -- a load whose only consumer is the flags -----------------------
        //
        // The shape that makes `MemOp::volatile` load-bearing: nothing keeps
        // the value, so a lifter that marked the load eliminable would let
        // dead-code elimination take the bus cycle and its tick with it — and
        // only once the flags it fed are themselves elided, which is why this
        // needs the generator rather than a hand-written case.
        45 => rm8(0x3b, reg, base, disp), // cmp r32, [base+d]
        46 => rm8(0x39, reg, base, disp), // cmp [base+d], r32
        47 => rm8(0x85, reg, base, disp), // test [base+d], r32
        // -- the computed near transfers -----------------------------------
        //
        // In the flat 4 GiB code segment `World::of` insists on there is no
        // target to reject, so all three are lifted here with no check at
        // all — which is the half of `lift::TRANSFER`'s rule that says
        // *nothing*, and the half a corpus that never generated a computed
        // transfer had never run either.
        48 => vec![0xc3],            // ret
        49 => vec![0xff, 0xe0 | rm], // jmp r/m32
        50 => vec![0xff, 0xd0 | rm], // call r/m32
        // -- the reserved-NOP space, and a repeat prefix in front of one ----
        //
        // `0F 1F /0` is the multi-byte NOP a compiler pads with and `F3 90`
        // is `PAUSE`; both decode a prefix or an operand that is never used
        // and do nothing at all. On a real kernel they were 59 M of the
        // instructions this frontend handed back.
        51 => vec![0x0f, 0x1f, 0x40 | base, disp as u8], // nop dword [base+d]
        52 => vec![0x66, 0x0f, 0x1f, 0xc0 | rm],         // nop word r/m
        53 => vec![0xf3, 0x90],                          // pause
        // A **byte** shift by `CL`, which is the one combination neither
        // corpus had: `D3` is the word and doubleword form and `C0` is the
        // byte form with an immediate, so a byte destination and a count that
        // may be zero never met. That is the pair `Lifter::shift_slot` is
        // about — `AH` is register number four and lives in slot zero — and a
        // mutation that wrote back the wrong slot survived a sweep because of
        // it.
        54 => {
            let ext = (fields >> 24) as u8 & 7;
            vec![0xd2, 0xc0 | (ext << 3) | (rm & 7)]
        }
        // -- a 16-bit operand size, which neither corpus had at all ---------
        //
        // `66` is a whole operand **width** rather than a policy, and every
        // one of these was already lifted and never compared: `set_szp` reads
        // bit fifteen, `add`'s carry is at sixteen, `sub`'s borrow with it,
        // a rotate's overflow comes off bits fifteen and fourteen, and a word
        // write is a `deposit` that must **preserve** the doubleword above it
        // where a doubleword write zero-extends.
        55 => rr16(0x01, reg, rm),  // add r/m16, r16
        56 => rr16(0x03, reg, rm),  // add r16, r/m16
        57 => rr16(0x19, reg, rm),  // sbb — the borrow at a width neither
        58 => rr16(0x29, reg, rm),  // sub    corpus reached
        59 => rr16(0x31, reg, rm),  // xor
        60 => rr16(0x39, reg, rm),  // cmp
        61 => rr16(0x85, reg, rm),  // test
        62 => rr16(0x89, reg, rm),  // mov r/m16, r16
        63 => rm16(0x8b, reg, base, disp), // mov r16, [base+d]
        64 => rm16(0x89, reg, base, disp), // mov [base+d], r16
        65 => {
            let ext = (fields >> 24) as u8 & 7;
            let mut out = grp16(0xc1, ext, rm); // shift r/m16, imm8
            out.push(imm8 & 0x1f);
            out
        }
        // A shift by `CL` at sixteen bits: the count masks to five bits at
        // every width but sixty-four, so a count of 16..31 shifts a word
        // *out* of existence and the carry it leaves is not the one a
        // sixteen-bit reading would give.
        66 => grp16(0xd3, (fields >> 24) as u8 & 7, rm),
        // `MUL`, `IMUL`, `NOT` and `NEG` at sixteen bits, where the product
        // is `DX:AX` rather than `EDX:EAX`. `/6` and `/7` are `DIV` and
        // `IDIV`, which are outside the subset.
        67 => grp16(0xf7, [2u8, 3, 4, 5][((fields >> 24) & 3) as usize], rm),
        68 => vec![0x66, 0x0f, 0xaf, 0xc0 | (reg << 3) | rm], // imul r16, r/m16
        69 => vec![0x66, if fields & 1 == 0 { 0x40 | reg } else { 0x48 | reg }],
        // `CBW` and `CWD` at this size are `AL`→`AX` and `AX`→`DX:AX`, which
        // is the *half*-width case `Plan::Cbw`'s table has and the corpus
        // only ever drove at thirty-two.
        70 => vec![0x66, if fields & 1 == 0 { 0x98 } else { 0x99 }],
        71 => {
            let mut out = vec![0x66, 0xb8 | reg]; // mov r16, imm16
            out.extend_from_slice(&(imm32 as u16).to_le_bytes());
            out
        }
        72 => {
            let mut out = grp16(0x83, (fields >> 24) as u8 & 7, rm);
            out.push(imm8);
            out
        }
        73 => vec![0x66, 0x0f, 0x40 | cc, 0xc0 | (reg << 3) | rm], // cmovcc r16
        // `BSWAP` at a 16-bit operand is undefined in the manual and is a
        // **doubleword** swap on the silicon, which `Exec` reproduces — so
        // the frontend has to reproduce it too, and nothing checked that.
        74 => vec![0x66, 0x0f, 0xc8 | reg],
        // -- `MOVZX`/`MOVSX` from a **word** --------------------------------
        //
        // `Plan::MovX` takes `src_size` from whether the source is `Arg::Eb`,
        // so the two-byte arm of it is a different number and the corpus only
        // ever wrote the one-byte one.
        75 => vec![
            0x0f,
            if fields & 1 == 0 { 0xb7 } else { 0xbf },
            0xc0 | (reg << 3) | rm,
        ],
        76 => vec![
            0x0f,
            if fields & 1 == 0 { 0xb7 } else { 0xbf },
            0x40 | (reg << 3) | base,
            disp as u8,
        ],
        // -- `AH`, `CH`, `DH` and `BH` --------------------------------------
        //
        // A byte register number of four to seven is the *top* half of one of
        // the first four registers, which is why `Lifter::read_reg` has a
        // `pos` of eight and `Lifter::shift_slot` exists — and the corpus
        // wrote `reg & 3` everywhere, so the whole of that path was reached
        // by one form. These four read one without writing anything, write
        // one with a value small enough to keep a pointer inside its segment,
        // and move one down into a low byte.
        77 => rr(0x84, breg, breg2),   // test r/m8, r8
        78 => rr(0x38, breg, breg2),   // cmp r/m8, r8
        79 => rr(0x8a, reg & 3, breg), // mov r8, r/m8 — a high byte read
        80 => vec![0xb0 | high, imm8 & 0x3f], // mov ah/ch/dh/bh, imm8
        81 => vec![0x0f, 0x90 | cc, 0xc0 | high], // setcc ah/ch/dh/bh
        // Group `80`: the byte-width immediate group, which is a third
        // encoding of the eight ALU operations and was not generated at all.
        82 => vec![0x80, 0xc0 | (((fields >> 24) as u8 & 7) << 3) | (rm & 3), imm8],
        // -- the accumulator-immediate forms --------------------------------
        //
        // A different *encoding* of operations the corpus already covers, and
        // the only one that reaches `Arg::Al` and `Arg::Ax` — the two operand
        // kinds `Lifter::read_arg` answers with a fixed register number and
        // no `REX` bit, whatever prefix the instruction carries.
        83 => match (fields >> 28) & 7 {
            0 => {
                let mut out = vec![0x05]; // add eax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            1 => {
                let mut out = vec![0x2d]; // sub eax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            2 => {
                let mut out = vec![0x25]; // and eax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            3 => {
                let mut out = vec![0x3d]; // cmp eax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            4 => {
                let mut out = vec![0xa9]; // test eax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            5 => vec![0x04, imm8], // add al, imm8
            6 => vec![0x0c, imm8], // or  al, imm8
            _ => vec![0xa8, imm8], // test al, imm8
        },
        // -- the addressing modes the corpus never wrote --------------------
        //
        // `Lifter::ea`'s SIB arm — the base, the scaled index, "no index" and
        // "no base" — was reached by nothing at all: every memory form above
        // is `mod=01` with a base of zero to three. The base and the index
        // here are two of the four data-window pointers and the scale is at
        // most one, so `base + index*2 + disp` stays inside the window
        // whatever the draw.
        84 => vec![
            0x8b,
            0x44 | (reg << 3),
            sib(
                ((fields >> 10) & 1) as u8,
                ((fields >> 9) & 1) as u8,
                ((fields >> 8) & 1) as u8,
            ),
            disp as u8,
        ],
        // The same with **no index**, which is a different branch of
        // `Fields::has_index` and the one a compiler emits for `[esp+n]`.
        85 => vec![
            0x8b,
            0x44 | (reg << 3),
            sib(((fields >> 10) & 3) as u8, 4, base),
            disp as u8,
        ],
        // `mod=00`, `r/m=100`, `base=101`: no base at all, so the
        // displacement stands alone — the one arm of `Lifter::ea` that reads
        // no register and the one `Exec::ea_offset` answers the same way.
        86 => {
            let mut out = vec![0x8b, 0x04 | (reg << 3), sib(0, 4, 5)];
            out.extend_from_slice(&window.to_le_bytes());
            out
        }
        // `mod=00`, `r/m=101`: a bare 32-bit displacement, which is absolute
        // here and `RIP`-relative in long mode.
        87 => {
            let mut out = vec![0x8b, (reg << 3) | 5];
            out.extend_from_slice(&window.to_le_bytes());
            out
        }
        // `mod=10`: the same base with a 32-bit displacement rather than an
        // 8-bit one.
        88 => {
            let mut out = vec![0x8b, 0x80 | (reg << 3) | base];
            out.extend_from_slice(&i32::from(disp).to_le_bytes());
            out
        }
        // `MOV` to and from a direct offset: no ModRM byte at all, and the
        // one operand kind whose immediate is an **address** — so its width
        // follows the address size rather than the operand size.
        89 => {
            let mut out = vec![if fields & 1 == 0 { 0xa1 } else { 0xa3 }];
            out.extend_from_slice(&window.to_le_bytes());
            out
        }
        90 => {
            let mut out = vec![if fields & 1 == 0 { 0xa0 } else { 0xa2 }];
            out.extend_from_slice(&window.to_le_bytes());
            out
        }
        // -- the stack, and the direct transfers ----------------------------
        //
        // `PUSH imm` is the only push whose operand is not a register, and
        // `LEAVE`, `CALL rel`, `JMP rel` and the near `Jcc` were reached by
        // hand-written cases and by nothing generated — including the merged
        // `CALL` whose self-modifying-code exit has to resume at the call's
        // *target* rather than after it.
        91 => {
            let mut out = vec![0x68]; // push imm32
            out.extend_from_slice(&imm32.to_le_bytes());
            out
        }
        92 => vec![0x6a, imm8], // push imm8, sign-extended to the stack width
        93 => vec![0x89, 0xe5, 0xc9], // mov ebp, esp ; leave
        94 => {
            let mut out = vec![0xe8]; // call rel32
            out.extend_from_slice(&i32::from(rel).to_le_bytes());
            out
        }
        95 => {
            let mut out = vec![0xe9]; // jmp rel32
            out.extend_from_slice(&i32::from(rel).to_le_bytes());
            out
        }
        96 => {
            let mut out = vec![0x0f, 0x80 | cc]; // jcc rel32
            out.extend_from_slice(&i32::from(rel).to_le_bytes());
            out
        }
        _ => match (fields >> 28) & 7 {
            0 => vec![0xf8],             // clc
            1 => vec![0xf9],             // stc
            2 => vec![0xf5],             // cmc
            3 => vec![0x9f],             // lahf
            4 => vec![0x9e],             // sahf
            5 => vec![0x98],             // cwde
            6 => vec![0x99],             // cdq
            _ => vec![0x0f, 0xc8 | reg], // bswap
        },
    }
}

/// The register numbers [`synthesize64`] writes to.
///
/// Fifteen, not sixteen: `RSP` is left alone for the reason [`SYNTH_REGS`]
/// gives, and `R8`-`R15` are in because a corpus that never named them would
/// leave half the register file — and the whole of `REX.R` and `REX.B` —
/// untested.
pub const SYNTH_REGS64: [u8; 15] = [0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];

/// The registers [`Case::seeded`] points into the data window.
///
/// Eight of them in long mode rather than four, so `REX.B` reaches a base
/// register that holds something usable rather than a value that faults.
pub const SYNTH_BASES64: [u8; 8] = [0, 1, 2, 3, 8, 9, 10, 11];

/// Where a `RIP`-relative operand is aimed, as a displacement from the
/// instruction after it.
///
/// The second page of the window, which is data: a program is at most a couple
/// of hundred bytes and starts at [`BASE`], so `RIP` plus anything in this
/// range lands inside the four mapped pages and outside the code page. Aiming
/// it at the code page would be a self-modifying-code case pretending to be an
/// addressing-mode one.
const RIP_WINDOW: u32 = 0x1000;

/// Encode one instruction from inside the lifter's **long-mode** subset.
///
/// The 64-bit counterpart of [`synthesize`], written out rather than derived
/// from it by prefixing a `REX`: three of that function's forms mean something
/// else in long mode — `40`-`4f` *are* the prefix, so `INC r32` and `DEC r32`
/// have to be encoded through group `FF`, and `B8+r` with `REX.W` takes an
/// eight-byte immediate that would swallow the next instruction — and three
/// addressing modes exist here that have no 32-bit spelling at all.
///
/// What it covers that [`synthesize`] cannot:
///
/// * **`REX` itself**, on nearly every instruction, including the `40` that
///   sets no bit and still renames `AH` to `SPL`;
/// * **`R8`-`R15`**, as operands, as memory bases and as byte registers;
/// * **a 64-bit operand size**, where `ADD`'s carry has no bit above it,
///   a shift count is masked to six bits rather than five, and `MUL` needs a
///   double-width product;
/// * **a 32-bit operand size in long mode**, which zero-extends into the whole
///   register where a byte or word write preserves what is above it — half the
///   arithmetic forms drop `REX.W` for exactly that;
/// * **`RIP`-relative addressing**, whose effective address depends on the
///   instruction's own length.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn synthesize64(form: u32, fields: u32) -> Vec<u8> {
    let reg = SYNTH_REGS64[(fields % 15) as usize];
    let rm = SYNTH_REGS64[((fields >> 4) % 15) as usize];
    let base = SYNTH_BASES64[((fields >> 8) & 7) as usize];
    let disp = (((fields >> 12) & 0x7f) as i32 - 64) as i8;
    let imm8 = (fields >> 16) as u8;
    let imm32 = fields.rotate_left(11);
    let cc = ((fields >> 20) & 15) as u8;
    let ext = (fields >> 24) as u8 & 7;
    // Halfword-granular and small, so a taken branch lands on a real
    // instruction boundary more often than not and never leaves the page.
    let rel = ((((fields >> 24) & 0x1f) as i32) - 16) as i8;
    // Whether this form widens its operand. Left to the generator rather than
    // fixed, because *both* answers are interesting in long mode: with
    // `REX.W` the operand is the one the IR has no carry bit above, and
    // without it the write zeroes the upper half.
    let wide = (fields >> 23) & 1 == 1;
    let rip = (RIP_WINDOW + ((fields >> 12) & 0x7f8)) as i32;

    // `REX` is emitted on nearly everything, `40` included: it is a prefix
    // with no operand of its own and it still changes what a byte-sized
    // register number four means.
    let rex = |w: bool, r: u8, b: u8| 0x40 | (u8::from(w) << 3) | ((r >> 3) << 2) | (b >> 3);
    // `mod=11` — both operands are registers.
    let rr =
        |w: bool, op: u8, r: u8, m: u8| vec![rex(w, r, m), op, 0xc0 | ((r & 7) << 3) | (m & 7)];
    // `mod=01` — a base register and an 8-bit displacement. Every base in
    // `SYNTH_BASES64` has a low field of 0-3, so none of them needs a SIB byte
    // and none of them is the `RBP` encoding.
    let rmd = |w: bool, op: u8, r: u8, b: u8, d: i8| {
        vec![rex(w, r, b), op, 0x40 | ((r & 7) << 3) | (b & 7), d as u8]
    };
    // `mod=00`, `r/m=101` — `RIP` plus a 32-bit displacement.
    let riprel = |op: u8, r: u8, d: i32| {
        let mut out = vec![rex(true, r, 0), op, ((r & 7) << 3) | 5];
        out.extend_from_slice(&d.to_le_bytes());
        out
    };
    // A group encoding with the extension in the `reg` field.
    let group = |w: bool, op: u8, n: u8, m: u8| vec![rex(w, 0, m), op, 0xc0 | (n << 3) | (m & 7)];
    // The same three behind a `66` operand-size prefix, which is a 16-bit
    // operand here and **not** a 64-bit one: `REX.W` beats `66`, so every one
    // of these drops it. Legacy prefixes come before `REX` and `REX` is the
    // last prefix before the opcode (*Intel SDM* volume 2 §2.2.1).
    let rr16 = |op: u8, r: u8, m: u8| {
        vec![0x66, rex(false, r, m), op, 0xc0 | ((r & 7) << 3) | (m & 7)]
    };
    let rmd16 = |op: u8, r: u8, b: u8, d: i8| {
        vec![
            0x66,
            rex(false, r, b),
            op,
            0x40 | ((r & 7) << 3) | (b & 7),
            d as u8,
        ]
    };
    let grp16 = |op: u8, n: u8, m: u8| {
        vec![0x66, rex(false, 0, m), op, 0xc0 | (n << 3) | (m & 7)]
    };
    // A byte register number over the whole eight **without** a `REX` prefix,
    // where four to seven are `AH`, `CH`, `DH` and `BH` — the top halves of
    // the first four registers rather than `SPL`..`DIL`.
    let breg = ((fields >> 4) & 7) as u8;
    let breg2 = ((fields >> 20) & 7) as u8;
    let high = 4 | (breg & 3);
    // A SIB byte, and an absolute linear address inside the high window.
    let sib = |scale: u8, index: u8, b: u8| (scale << 6) | ((index & 7) << 3) | (b & 7);
    let win64 = HIGH + u64::from(DATA as u32 + ((fields >> 3) & 0xff8));

    match form % 97 {
        // -- the ALU, register to register, at both operand sizes -----------
        0 => rr(true, 0x01, reg, rm),  // add r/m64, r64
        1 => rr(wide, 0x03, reg, rm),  // add r, r/m
        2 => rr(false, 0x00, reg, rm), // add r/m8, r8 — `REX` byte registers
        3 => rr(wide, 0x09, reg, rm),  // or
        4 => rr(true, 0x11, reg, rm),  // adc
        5 => rr(true, 0x19, reg, rm),  // sbb
        6 => rr(wide, 0x21, reg, rm),  // and
        7 => rr(true, 0x29, reg, rm),  // sub
        8 => rr(wide, 0x31, reg, rm),  // xor
        9 => rr(true, 0x39, reg, rm),  // cmp
        10 => rr(true, 0x85, reg, rm), // test
        // -- the ALU against memory ----------------------------------------
        11 => rmd(true, 0x01, reg, base, disp),
        12 => rmd(wide, 0x03, reg, base, disp),
        13 => rmd(true, 0x29, reg, base, disp),
        14 => rmd(wide, 0x33, reg, base, disp),
        15 => rmd(true, 0x89, reg, base, disp), // mov [base+d], r64
        16 => rmd(true, 0x8b, reg, base, disp), // mov r64, [base+d]
        17 => rmd(false, 0x88, reg, base, disp), // mov [base+d], r8
        18 => rmd(false, 0x8a, reg, base, disp), // mov r8, [base+d]
        19 => rmd(true, 0x8d, reg, base, disp), // lea
        // -- immediates -----------------------------------------------------
        20 => {
            // group 81 /n imm32, sign-extended to sixty-four bits
            let mut out = group(true, 0x81, ext, rm);
            out.extend_from_slice(&imm32.to_le_bytes());
            out
        }
        21 => {
            // group 83 /n imm8, sign-extended
            let mut out = group(wide, 0x83, ext, rm);
            out.push(imm8);
            out
        }
        22 => {
            // mov r64, imm32 — sign-extended. `B8+r` with `REX.W` takes an
            // eight-byte immediate instead, which would swallow the next
            // instruction out of a generated stream.
            let mut out = group(true, 0xc7, 0, rm);
            out.extend_from_slice(&imm32.to_le_bytes());
            out
        }
        23 => vec![rex(false, 0, reg), 0xb0 | (reg & 7), imm8], // mov r8, imm8
        // `40+r` is the `REX` prefix in long mode, so the increments moved
        // into group `FF`.
        24 => group(true, 0xff, 0, rm), // inc r/m64
        25 => group(wide, 0xff, 1, rm), // dec r/m
        // -- shifts and rotates ---------------------------------------------
        26 => {
            // A count of up to sixty-three, which is the mask a 64-bit operand
            // uses where every narrower one masks to five bits.
            let mut out = group(true, 0xc1, ext, rm);
            out.push(imm8 & 0x3f);
            out
        }
        27 => group(true, 0xd1, ext, rm),
        28 => group(true, 0xd3, ext, rm),
        29 => {
            // The byte forms, **with their immediate**. `C0 /n` takes an
            // `ib` and this arm did not push one, so every draw of it
            // swallowed the first byte of whatever instruction followed and
            // shifted the rest of the program along by one. Both engines
            // still agreed — they decode the same bytes — so nothing failed;
            // what was lost is that the form after it was not the form the
            // generator drew. `every_generated_form_is_one_the_frontend_lifts`
            // is what noticed, because a truncated encoding lifts nothing.
            let mut out = group(false, 0xc0, ext, rm);
            out.push(imm8 & 0x1f);
            out
        }
        // -- multiplies -----------------------------------------------------
        30 => group(true, 0xf7, 4, rm),  // mul r/m64
        31 => group(true, 0xf7, 5, rm),  // imul r/m64
        32 => group(false, 0xf6, 4, rm), // mul r/m8
        33 => {
            let mut out = vec![rex(true, reg, rm), 0x0f, 0xaf];
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7));
            out
        }
        34 => {
            let mut out = rr(true, 0x6b, reg, rm); // imul r64, r/m64, imm8
            out.push(imm8);
            out
        }
        // -- the unary group ------------------------------------------------
        35 => group(true, 0xf7, 2, rm), // not
        36 => group(wide, 0xf7, 3, rm), // neg
        // -- extensions and bit scans ---------------------------------------
        37 => {
            let mut out = vec![rex(true, reg, rm), 0x0f, 0xb6]; // movzx r64, r/m8
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7));
            out
        }
        38 => {
            let mut out = vec![rex(true, reg, rm), 0x0f, 0xbe]; // movsx r64, r/m8
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7));
            out
        }
        39 => {
            let mut out = vec![rex(true, reg, rm), 0x0f, 0xbc]; // bsf
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7));
            out
        }
        40 => {
            let mut out = vec![rex(true, reg, rm), 0x0f, 0xbd]; // bsr
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7));
            out
        }
        // -- the condition codes, read three different ways -----------------
        41 => vec![0x70 | cc, rel as u8], // jcc rel8
        42 => {
            let mut out = vec![rex(false, 0, rm), 0x0f, 0x90 | cc]; // setcc r/m8
            out.push(0xc0 | (rm & 7));
            out
        }
        43 => {
            let mut out = vec![rex(true, reg, rm), 0x0f, 0x40 | cc]; // cmovcc
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7));
            out
        }
        // -- the stack, which is eight bytes wide here whatever the prefix --
        44 => vec![
            rex(false, 0, reg),
            0x50 | (reg & 7),
            rex(false, 0, rm),
            0x58 | (rm & 7),
        ],
        // -- a load whose only consumer is the flags ------------------------
        45 => rmd(true, 0x3b, reg, base, disp), // cmp r64, [base+d]
        46 => rmd(true, 0x39, reg, base, disp), // cmp [base+d], r64
        47 => rmd(wide, 0x85, reg, base, disp), // test [base+d], r
        // -- `RIP`-relative addressing, which has no 32-bit spelling --------
        48 => riprel(0x8b, reg, rip), // mov r64, [rip+d]
        49 => riprel(0x89, reg, rip), // mov [rip+d], r64
        50 => riprel(0x8d, reg, rip), // lea r64, [rip+d]
        51 => {
            let mut out = vec![rex(true, reg, rm), 0x63]; // movsxd r64, r/m32
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7));
            out
        }
        // -- the computed near transfers, which long mode can reject --------
        //
        // The whole point of `lift::TRANSFER`, and the reason these are here
        // rather than only in the 32-bit corpus: `Case::seeded` leaves
        // `R12`, `R14` and `R15` holding values that are **not canonical**,
        // so a generated `jmp r/m64` over `SYNTH_REGS64` draws targets the
        // transfer must reject as well as targets it must take, and
        // `compare` asserts the state at the fault against the interpreter's.
        // `RET` pops whatever the data window holds.
        52 => vec![0xc3],                                     // ret
        53 => vec![rex(false, 0, rm), 0xff, 0xe0 | (rm & 7)], // jmp r/m64
        54 => vec![rex(false, 0, rm), 0xff, 0xd0 | (rm & 7)], // call r/m64
        // -- the reserved-NOP space -----------------------------------------
        //
        // `0F 1F /0` is the multi-byte NOP a compiler pads with; `F3 0F 1E FA`
        // is `ENDBR64`, which begins every function of a kernel built with
        // indirect-branch tracking; `F3 90` is `PAUSE`. All three are one
        // static charge and nothing else, on both sides.
        55 => vec![0x0f, 0x1f, 0x40 | (base & 7), disp as u8], // nop dword [base+d]
        56 => vec![0x66, 0x0f, 0x1f, 0xc0 | (rm & 7)],         // nop word r/m
        57 => vec![0xf3, 0x0f, 0x1e, 0xfa],                    // endbr64
        58 => vec![0xf3, 0x90],                                // pause
        // The byte shift by `CL`, in both of its long-mode spellings: without
        // a `REX` prefix register number four is `AH`, with one it is `SPL`.
        59 => vec![0xd2, 0xc0 | (ext << 3) | (rm & 7)],
        60 => group(false, 0xd2, ext, rm),
        // -- a 16-bit operand size, which neither corpus had at all ---------
        //
        // A third operand width beside the two this generator already writes,
        // and the one where a write is a `deposit` that must **preserve** the
        // sixty-four bits above it — the exact opposite of what the 32-bit
        // form beside it does. `REX` is present and `REX.W` is not, because
        // `REX.W` beats `66`.
        62 => rr16(0x01, reg, rm), // add r/m16, r16
        63 => rr16(0x03, reg, rm), // add r16, r/m16
        64 => rr16(0x19, reg, rm), // sbb
        65 => rr16(0x29, reg, rm), // sub
        66 => rr16(0x31, reg, rm), // xor
        67 => rr16(0x39, reg, rm), // cmp
        68 => rr16(0x89, reg, rm), // mov r/m16, r16
        69 => rmd16(0x8b, reg, base, disp), // mov r16, [base+d]
        70 => rmd16(0x89, reg, base, disp), // mov [base+d], r16
        71 => {
            let mut out = grp16(0xc1, ext, rm); // shift r/m16, imm8
            out.push(imm8 & 0x1f);
            out
        }
        72 => grp16(0xd3, ext, rm), // shift r/m16, cl
        // `MUL`, `IMUL`, `NOT` and `NEG` at sixteen bits, where the product is
        // `DX:AX`. `/6` and `/7` are `DIV` and `IDIV` and are out of the subset.
        73 => grp16(0xf7, [2u8, 3, 4, 5][((fields >> 24) & 3) as usize], rm),
        74 => {
            let mut out = vec![0x66, rex(false, reg, rm), 0x0f, 0xaf];
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7)); // imul r16, r/m16
            out
        }
        // `40+r` is the `REX` prefix, so the 16-bit increments go through
        // group `FF` exactly as the 64-bit ones do.
        75 => grp16(0xff, u8::from(fields & 1 == 1), rm),
        // `CBW` and `CWD` at sixteen bits: `AL`→`AX` and `AX`→`DX:AX`, the
        // half-width arm of `Plan::Cbw`'s table.
        76 => vec![0x66, if fields & 1 == 0 { 0x98 } else { 0x99 }],
        // `BSWAP` at a 16-bit operand is undefined in the manual and is a
        // doubleword swap on the silicon, which `Exec` reproduces.
        77 => vec![0x66, rex(false, 0, reg), 0x0f, 0xc8 | (reg & 7)],
        78 => {
            let mut out = grp16(0x83, ext, rm); // group 83 at sixteen bits
            out.push(imm8);
            out
        }
        // -- `MOVZX`/`MOVSX` from a **word** --------------------------------
        //
        // `Plan::MovX` takes `src_size` from whether the source is `Arg::Eb`,
        // and the two-byte arm of it had never been drawn.
        79 => {
            let mut out = vec![rex(true, reg, rm), 0x0f];
            out.push(if fields & 1 == 0 { 0xb7 } else { 0xbf });
            out.push(0xc0 | ((reg & 7) << 3) | (rm & 7));
            out
        }
        80 => {
            let mut out = vec![rex(true, reg, base), 0x0f];
            out.push(if fields & 1 == 0 { 0xb7 } else { 0xbf });
            out.push(0x40 | ((reg & 7) << 3) | (base & 7));
            out.push(disp as u8);
            out
        }
        // -- `AH`, `CH`, `DH` and `BH`, which need **no** `REX` -------------
        //
        // With any `REX` prefix byte register four is `SPL`; without one it is
        // `AH`, the *top* half of `RAX`. This generator prefixes nearly
        // everything, so the second of those had one form — a shift — and
        // nothing else. These read one without writing anything, move one down
        // into a low byte, and write one with a value small enough to leave a
        // data-window pointer inside its own window.
        81 => vec![0x84, 0xc0 | (breg2 << 3) | breg], // test r/m8, r8
        82 => vec![0x38, 0xc0 | (breg2 << 3) | breg], // cmp r/m8, r8
        83 => vec![0x8a, 0xc0 | ((reg & 3) << 3) | breg], // mov r8, r/m8
        84 => vec![0xb0 | high, imm8 & 0x3f],         // mov ah/ch/dh/bh, imm8
        85 => vec![0x0f, 0x90 | cc, 0xc0 | high],     // setcc ah/ch/dh/bh
        // Group `80`: the byte-width immediate group, a third encoding of the
        // eight ALU operations that was not generated at all.
        86 => {
            let mut out = group(false, 0x80, ext, rm);
            out.push(imm8);
            out
        }
        // -- the accumulator-immediate forms --------------------------------
        //
        // The only encodings that reach `Arg::Al` and `Arg::Ax`, which
        // `Lifter::read_arg` answers with a fixed register number and **no**
        // `REX` bit whatever prefix the instruction carries.
        87 => match (fields >> 28) & 7 {
            0 => {
                let mut out = vec![0x48, 0x05]; // add rax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            1 => {
                let mut out = vec![0x48, 0x2d]; // sub rax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            2 => {
                let mut out = vec![0x48, 0x25]; // and rax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            3 => {
                let mut out = vec![0x48, 0x3d]; // cmp rax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            4 => {
                let mut out = vec![0x48, 0xa9]; // test rax, imm32
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            5 => vec![0x04, imm8], // add al, imm8
            6 => vec![0x0c, imm8], // or  al, imm8
            _ => vec![0xa8, imm8], // test al, imm8
        },
        // -- the addressing modes the corpus never wrote --------------------
        //
        // `Lifter::ea`'s SIB arm was reached by nothing: every memory form
        // above is `mod=01` with a base and no SIB byte. A scaled index needs
        // a register holding something small — both windows are two and a
        // half tebibytes up, so two pointers added together are nowhere — so
        // this form makes one: `mov r13d, n` zero-extends a value under
        // sixteen into `R13`, and the access then indexes by it through
        // `REX.X`, which nothing else in this generator sets.
        88 => {
            let n = ((fields >> 2) & 0xf) as u8;
            let mut out = vec![0x41, 0xc7, 0xc5, n, 0, 0, 0]; // mov r13d, n
            out.push(0x48 | (((reg >> 3) & 1) << 2) | 2 | ((base >> 3) & 1));
            out.push(0x8b);
            out.push(0x40 | ((reg & 7) << 3) | 4);
            out.push(sib(((fields >> 10) & 3) as u8, 5, base));
            out.push(disp as u8);
            out
        }
        // The same with **no index**, which is a different branch of
        // `Fields::has_index` and the one a compiler emits for `[rsp+n]`.
        89 => vec![
            rex(true, reg, base),
            0x8b,
            0x40 | ((reg & 7) << 3) | 4,
            sib(((fields >> 10) & 3) as u8, 4, base),
            disp as u8,
        ],
        // `mod=10`: the same base with a 32-bit displacement rather than an
        // 8-bit one.
        90 => {
            let mut out = vec![rex(true, reg, base), 0x8b];
            out.push(0x80 | ((reg & 7) << 3) | (base & 7));
            out.extend_from_slice(&i32::from(disp).to_le_bytes());
            out
        }
        // `MOV` to and from a direct offset. In long mode the immediate is a
        // **`moffs64`** — eight bytes, because the width of this one operand
        // kind follows the address size and not the operand size — which is
        // the only encoding in the instruction set that carries a full 64-bit
        // address, and the only reason `Lifter::ea`'s `None` arm exists.
        91 => {
            let mut out = vec![0x48, if fields & 1 == 0 { 0xa1 } else { 0xa3 }];
            out.extend_from_slice(&win64.to_le_bytes());
            out
        }
        // -- the stack, and the direct transfers ----------------------------
        //
        // `PUSH imm` is the only push whose operand is not a register;
        // `LEAVE`, `CALL rel`, `JMP rel` and the near `Jcc` were reached by
        // hand-written cases and by nothing generated — including the merged
        // `CALL` whose self-modifying-code exit resumes at the call's target.
        92 => match (fields >> 28) & 3 {
            0 => {
                let mut out = vec![0x68]; // push imm32, sign-extended to 64
                out.extend_from_slice(&imm32.to_le_bytes());
                out
            }
            1 => vec![0x6a, imm8],                // push imm8
            2 => vec![0x48, 0x89, 0xe5, 0xc9],    // mov rbp, rsp ; leave
            _ => vec![0xeb, rel as u8],           // jmp rel8
        },
        93 => {
            let mut out = vec![0xe8]; // call rel32
            out.extend_from_slice(&i32::from(rel).to_le_bytes());
            out
        }
        94 => {
            let mut out = vec![if fields & 1 == 0 { 0xe9 } else { 0x0f }];
            if fields & 1 != 0 {
                out.push(0x80 | cc); // jcc rel32
            }
            out.extend_from_slice(&i32::from(rel).to_le_bytes());
            out
        }
        95 => {
            let mut out = rr(true, 0x69, reg, rm); // imul r64, r/m64, imm32
            out.extend_from_slice(&imm32.to_le_bytes());
            out
        }
        _ => match (fields >> 28) & 7 {
            0 => vec![0xf8],       // clc
            1 => vec![0xf9],       // stc
            2 => vec![0xf5],       // cmc
            3 => vec![0x9f],       // lahf
            4 => vec![0x9e],       // sahf
            5 => vec![0x48, 0x98], // cdqe
            6 => vec![0x48, 0x99], // cqo
            // `90` with `REX.B` is `XCHG R8, RAX` and not a no-operation, so
            // the frontend refuses it and the block ends there. Generated
            // deliberately: the encoding used to lift as a `NOP` while the
            // interpreter performed the exchange, which is a divergence
            // nothing in the corpus could reach because nothing in the corpus
            // wrote it.
            _ => vec![0x49, 0x90],
        },
    }
}

/// A whole long-mode program of `len` generated instructions, from the same
/// seeded generator [`program`] uses.
#[must_use]
pub fn program64(seed: u64, len: usize) -> Vec<u8> {
    let mut state = seed;
    let mut out = Vec::new();
    for _ in 0..len {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        out.extend_from_slice(&synthesize64((state >> 40) as u32, state as u32));
    }
    out
}

/// A whole program of `len` generated instructions, from a seeded generator.
///
/// The generator is a 64-bit linear congruential sequence — Knuth's MMIX
/// multiplier and increment — so the corpus is identical on every machine and
/// in every run (`ROADMAP.md` §0): a failure is reproducible from the seed
/// printed beside it, and a new failure is a real regression rather than a
/// different draw.
#[must_use]
pub fn program(seed: u64, len: usize) -> Vec<u8> {
    let mut state = seed;
    let mut out = Vec::new();
    for _ in 0..len {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        out.extend_from_slice(&synthesize((state >> 40) as u32, state as u32));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The machine this harness builds must be the world the lifter is told it
    /// is in, and [`World::of`] is the only thing that can say so.
    ///
    /// Without this the two could drift and every case would still pass, for
    /// the worst possible reason: the subject and the oracle would agree
    /// because the subject was told what the oracle was doing.
    #[test]
    fn a_hand_written_world_is_the_one_world_of_finds() {
        for case in [
            Case::new(vec![0xf4]),
            Case::new(vec![0xf4]).paged(),
            Case::new(vec![0xf4]).long(),
            Case::new(vec![0xf4]).compat(),
        ] {
            let (space, _ram) = machine(&case);
            let cpu = oracle(&case, space);
            let want = world(&case);
            let found = World::of(
                &cpu.regs(),
                &cpu.sys(),
                &config(&case),
                cpu.a20_open(),
                0,
                want.origin,
            )
            .expect("the harness builds a world the frontend lifts");
            assert_eq!(found, want);
        }
    }

    /// The physical page [`world`] writes down must be the page the machine's
    /// own tables resolve the entry to.
    ///
    /// The one claim in a paged case that is written by hand rather than
    /// derived, and the one that would make every paged case pass for the
    /// wrong reason: a block keyed on a page its bytes did not come from is
    /// exactly the stale translation the key exists to prevent, and a harness
    /// that agreed with itself would never notice.
    #[test]
    fn a_paged_entry_resolves_to_the_page_the_world_claims() {
        let case = Case::new(vec![0xf4]).paged();
        let (space, _ram) = machine(&case);
        let mmu = Mmu::new(&case);
        let phys = mmu
            .phys_of(&space, world(&case).linear(BASE))
            .expect("the entry page is mapped");
        assert_eq!(phys & !0xfff, PAGED_PROGRAM);
        assert_ne!(phys & !0xfff, BASE, "an identity map would test nothing");
        assert_eq!(world(&case).origin, Origin::Paged { phys });
    }

    /// The paged world is a world the frontend lifts, and the interpreter
    /// agrees with it instruction for instruction, flag for flag and tick for
    /// tick — including the ticks the page-table walks cost and the accessed
    /// and dirty bits they wrote, which [`memory`] compares along with the
    /// rest of RAM.
    #[test]
    fn a_paged_case_agrees_with_the_interpreter() {
        // mov eax, [ebx] ; add eax, ecx ; mov [ebx+4], eax ; hlt
        let program = vec![0x8b, 0x03, 0x01, 0xc8, 0x89, 0x43, 0x04, 0xf4];
        let case = Case::seeded(program).paged();
        match compare(&case) {
            Ok(v) => assert!(matches!(v, Verdict::Agreed { insns: 3, .. }), "{v:?}"),
            Err(e) => panic!("{e}"),
        }
    }

    /// A paged case really does write the accessed and dirty bits, so the
    /// byte-for-byte memory comparison above is comparing something.
    ///
    /// Asserted separately because "both engines left RAM identical" is true
    /// of a machine where neither engine touched a page table at all, and that
    /// would be a harness measuring nothing.
    /// Compatibility mode is a **fourth world** rather than a re-run of the
    /// paged one, and the walk is what says so.
    ///
    /// The same bytes, the same segment bases, the same physical frames — and
    /// four levels of eight-byte entries rather than two of four, which is two
    /// more descriptor reads for every translation that misses the buffer.
    /// That is a tick column, and both engines have to agree about it, so a
    /// harness that had quietly built the legacy walk here would be running
    /// `the_same_corpus_agrees_with_paging_on` a second time under a different
    /// name.
    #[test]
    fn compatibility_mode_walks_four_levels_where_the_paged_world_walks_two() {
        // A load and a store through a data segment, so the walk happens and
        // the dirty bit is written.
        let program = vec![0x8b, 0x03, 0x89, 0x43, 0x04, 0xf4];
        let legacy = Case::seeded(program.clone()).paged();
        let compat = Case::seeded(program).compat();
        let (a, b) = match (compare(&legacy), compare(&compat)) {
            (Ok(a), Ok(b)) => (a, b),
            (a, b) => panic!("{a:?}\n{b:?}"),
        };
        let (
            Verdict::Agreed {
                insns: ia,
                ticks: ta,
            },
            Verdict::Agreed {
                insns: ib,
                ticks: tb,
            },
        ) = (a.clone(), b.clone())
        else {
            panic!("both worlds run this program to completion: {a:?} {b:?}");
        };
        assert_eq!(ia, ib, "the same bytes retire the same instructions");
        assert!(
            tb > ta,
            "compatibility mode cost no more than the two-level walk ({ta} against {tb}), so \
             it is not walking four levels"
        );
        // And it is the *32-bit* world, not long mode's: `World::of` derives
        // the code segment's width from `Sys::sixty_four`, which is `LMA` and
        // `CS.L` together.
        assert!(!world(&Case::new(vec![0xf4]).compat()).long());
    }

    #[test]
    fn a_paged_case_writes_the_accessed_and_dirty_bits() {
        let program = vec![0x8b, 0x03, 0x89, 0x43, 0x04, 0xf4];
        let case = Case::seeded(program).paged();
        let (space, ram) = machine(&case);
        let before = ram
            .read_u8(PTAB - BASE + 4 * ((BASE >> 12) & 0x3ff))
            .unwrap();
        assert_eq!(u64::from(before) & (pte::ACCESSED | pte::DIRTY), 0);
        let cpu = oracle(&case, space);
        for _ in 0..3 {
            cpu.step();
        }
        // The code page: fetched, so accessed and not dirty.
        let code = ram
            .read_u8(PTAB - BASE + 4 * ((BASE >> 12) & 0x3ff))
            .unwrap();
        assert_eq!(u64::from(code) & pte::ACCESSED, pte::ACCESSED);
        assert_eq!(u64::from(code) & pte::DIRTY, 0);
        // The data page — the one `EBX` points into — written, so both.
        let touched = BASE + case.start_regs()[3];
        let data = ram
            .read_u8(PTAB - BASE + 4 * ((touched >> 12) & 0x3ff))
            .unwrap();
        assert_eq!(
            u64::from(data) & (pte::ACCESSED | pte::DIRTY),
            pte::ACCESSED | pte::DIRTY
        );
    }

    /// The fifth world: long mode, which is the fourth plus `CR4.PAE`,
    /// `EFER.LME` and a code segment with its `L` bit set.
    ///
    /// `48` in front of each instruction is `REX.W`, so every operand here is
    /// sixty-four bits wide — the width at which `ADD`'s carry has no bit
    /// above it to be read from and at which a `MUL` needs a double-width
    /// product the IR has one opcode for.
    #[test]
    fn a_long_mode_case_agrees_with_the_interpreter() {
        // mov rax, [rbx] ; add rax, rcx ; mov [rbx+8], rax ; hlt
        let program = vec![
            0x48, 0x8b, 0x03, 0x48, 0x01, 0xc8, 0x48, 0x89, 0x43, 0x08, 0xf4,
        ];
        let case = Case::seeded(program).long();
        match compare(&case) {
            Ok(v) => assert!(matches!(v, Verdict::Agreed { insns: 3, .. }), "{v:?}"),
            Err(e) => panic!("{e}"),
        }
    }

    /// The registers `REX` invented, which a 32-bit encoding cannot name at
    /// all — and the four-level walk that reaches them.
    #[test]
    fn the_upper_eight_registers_are_reachable_and_compared() {
        // mov r12, [r11] ; add r12, r13 ; mov [r11+8], r12 ; hlt
        let program = vec![
            0x4d, 0x8b, 0x23, 0x4d, 0x01, 0xec, 0x4d, 0x89, 0x63, 0x08, 0xf4,
        ];
        let case = Case::seeded(program).long();
        match compare(&case) {
            Ok(v) => assert!(matches!(v, Verdict::Agreed { insns: 3, .. }), "{v:?}"),
            Err(e) => panic!("{e}"),
        }
    }

    /// A 64-bit `MUL`, whose product does not fit in one temporary.
    ///
    /// The instruction [`lift`]'s "What the IR could not say" named as the one
    /// that would reach [`Opcode::MULU2`](crate::ir::Opcode::MULU2), and the
    /// reason it stayed unexercised for three rounds.
    #[test]
    fn a_sixty_four_bit_multiply_agrees_in_both_halves() {
        for (a, b) in [
            (0x1234_5678_9abc_def0u64, 0xfedc_ba98_7654_3210u64),
            (u64::MAX, u64::MAX),
            (1 << 63, 3),
            (0, 0x55),
        ] {
            for op in [0xe3u8, 0xeb] {
                // mul rbx / imul rbx, then hlt.
                let program = vec![0x48, 0xf7, op, 0xf4];
                let case = Case::new(program)
                    .with_reg(0, a)
                    .with_reg(3, b)
                    .with_reg(2, 0x0bad_0bad_0bad_0bad)
                    .long();
                match compare(&case) {
                    Ok(v) => assert!(matches!(v, Verdict::Agreed { insns: 1, .. }), "{v:?}"),
                    Err(e) => panic!("{a:#x} * {b:#x} ({op:#x}): {e}"),
                }
            }
        }
    }

    /// `RIP`-relative addressing: the one mode whose effective address depends
    /// on the instruction's own length.
    #[test]
    fn a_rip_relative_operand_agrees() {
        // mov rax, [rip+0x1000] ; mov [rip+0x1008], rax ; hlt
        let mut program = vec![0x48, 0x8b, 0x05];
        program.extend_from_slice(&0x1000u32.to_le_bytes());
        program.extend_from_slice(&[0x48, 0x89, 0x05]);
        program.extend_from_slice(&0x1008u32.to_le_bytes());
        program.push(0xf4);
        let case = Case::seeded(program).long();
        match compare(&case) {
            Ok(v) => assert!(matches!(v, Verdict::Agreed { insns: 2, .. }), "{v:?}"),
            Err(e) => panic!("{e}"),
        }
    }

    /// A 32-bit case cannot start with a register wider than the world it runs
    /// in, however wide the field holding it is.
    ///
    /// [`Case::regs`] is sixty-four bits because the host is, and the fuzz
    /// target fills registers from its input — so a number above 2^32 in a
    /// 32-bit case is reachable. It would put the two engines on different
    /// values, because [`lift`]'s slot invariant is that a slot holds the
    /// architectural register and a 32-bit read of one is therefore the whole
    /// slot, while `Regs::dword` truncates. That is a harness bug that would
    /// be reported as a frontend divergence, which is the worst shape a
    /// harness bug can have.
    #[test]
    fn a_narrow_world_narrows_the_registers_it_starts_with() {
        let wide = 0x1234_5678_9abc_def0u64;
        let flat = Case::new(vec![0x01, 0xc8, 0xf4])
            .with_reg(0, wide)
            .with_reg(1, wide);
        assert_eq!(flat.start_regs()[0], 0x9abc_def0);
        // And it does not narrow the world that can hold it.
        let long = Case::new(vec![0xf4]).with_reg(0, wide).long();
        assert_eq!(long.start_regs()[0], wide);
        // The two engines then agree, which is the property the narrowing is
        // for rather than the narrowing itself.
        match compare(&flat) {
            Ok(v) => assert!(matches!(v, Verdict::Agreed { insns: 1, .. }), "{v:?}"),
            Err(e) => panic!("{e}"),
        }
    }

    #[test]
    fn a_handful_of_instructions_agree() {
        // mov eax, 0x12345678 ; add eax, ecx ; sub eax, 1 ; inc ebx ; hlt
        let program = vec![
            0xb8, 0x78, 0x56, 0x34, 0x12, 0x01, 0xc8, 0x83, 0xe8, 0x01, 0x43, 0xf4,
        ];
        let case = Case::seeded(program);
        match compare(&case) {
            Ok(v) => assert!(matches!(v, Verdict::Agreed { insns: 4, .. }), "{v:?}"),
            Err(e) => panic!("{e}"),
        }
    }

    #[test]
    fn the_generator_produces_programs_the_frontend_actually_lifts() {
        // A generator that had stopped producing encodings in the subset would
        // leave every sweep passing and measuring nothing.
        let mut lifted = 0usize;
        for n in 0..200u64 {
            let case = Case::seeded(program(0x1234_0000 + n, 6));
            if let Ok(Verdict::Agreed { insns, .. } | Verdict::Trapped { insns }) = compare(&case) {
                lifted += insns;
            }
        }
        assert!(lifted > 400, "only {lifted} guest instructions were lifted");
    }

    /// The same corpus, executed as host code.
    ///
    /// x86 is where the backend earns its keep: `cpu::x86::lift` turns one
    /// guest instruction with live flags into a dozen or more IR instructions —
    /// a `popcount` for `PF`, an `extract` for `AF`, two comparisons and a
    /// `movcond` — which is exactly the shape an interpreter is worst at and a
    /// code generator is best at. It is also the only frontend in the tree that
    /// emits `rotlc`, `mulu2`, `bswap`, `clz` and `ctz` at all, so this is the
    /// only harness that covers those lowerings against a real guest.
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    mod compiled {
        use super::*;

        fn agreed(case: &Case, blocks: usize) -> CachedRun {
            match measure_compiled(case, blocks) {
                Ok(run) => run,
                Err(e) => panic!("diverged on the compiled path:\n{e}"),
            }
        }

        #[test]
        fn a_generated_corpus_agrees_when_it_is_compiled() {
            let mut compiled = 0u64;
            let mut steps = 0usize;
            for n in 0..400u64 {
                let case = Case::seeded(program(0x9e37_0000 + n, 6));
                let run = agreed(&case, 8);
                compiled += run.compiled;
                steps += run.insns_retired;
            }
            assert!(
                compiled > 400,
                "only {compiled} blocks were compiled across 400 cases"
            );
            assert!(steps > 400, "only {steps} guest instructions retired");
        }

        #[test]
        fn every_policy_pair_agrees_when_it_is_compiled() {
            // The flag policy and the store policy are both in the cache key
            // and both change the IR a guest instruction lifts to — elision
            // removes the dead flag arithmetic, and the store guard adds a
            // load, a compare and a branch *inside* the block. A backend has to
            // be right about all four combinations.
            for flags in [lift::Flags::Eager, lift::Flags::Elide] {
                for smc in [lift::Smc::EndBlock, lift::Smc::Guard] {
                    let mut case = Case::seeded(program(0x5150_0001, 8));
                    case.flags = flags;
                    case.smc = smc;
                    let run = agreed(&case, 12);
                    assert!(run.compiled > 0, "{flags:?}/{smc:?} compiled nothing");
                }
            }
        }
    }

    /// Every form the generator can draw must be one the frontend actually
    /// lifts — or one of the two it is deliberately asked to refuse.
    ///
    /// A form that quietly left the subset would keep every sweep passing: the
    /// case would report [`Verdict::Nothing`], the thresholds are counted over
    /// whole programs rather than over forms, and the coverage the form was
    /// added for would simply stop happening. That is the failure this test
    /// exists for, and it is checked form by form rather than in aggregate.
    #[test]
    fn every_generated_form_is_one_the_frontend_lifts() {
        for (long, forms) in [(false, 98u32), (true, 97u32)] {
            let mut refused = Vec::new();
            for form in 0..forms {
                let mut lifted = false;
                'draws: for draw in 0..64u32 {
                    let fields = draw.wrapping_mul(0x9e37_79b9).rotate_left(draw % 32)
                        ^ (draw << 7);
                    let bytes = if long {
                        synthesize64(form, fields)
                    } else {
                        synthesize(form, fields)
                    };
                    // Both parts the sweeps run, because one of them decides
                    // whether an encoding is in the subset at all: `CMOVcc`
                    // raises `#UD` on a 386 and the frontend refuses it there
                    // (`World::cmov`), so a form carrying one lifts on the
                    // wider part and nowhere else.
                    for wide in [false, true] {
                        let case = Case::seeded(bytes.clone());
                        let case = if long {
                            case.long()
                        } else if wide {
                            Case {
                                variant: Variant::X86_64,
                                ..case
                            }
                        } else {
                            case
                        };
                        if !matches!(compare(&case), Ok(Verdict::Nothing)) {
                            lifted = true;
                            break 'draws;
                        }
                    }
                }
                if !lifted {
                    refused.push(form);
                }
            }
            assert!(
                refused.is_empty(),
                "{} form(s) the generator draws lift nothing at all: {refused:?}",
                refused.len(),
            );
        }
    }

    #[test]
    fn the_synthesizer_is_total() {
        // Every pair of numbers has to encode *something*, or a fuzzer's byte
        // stream turns into an empty program rather than a case.
        for form in 0..64u32 {
            for fields in [0u32, 0x1234_5678, u32::MAX, 0x8000_0001] {
                assert!(!synthesize(form, fields).is_empty(), "{form}/{fields:#x}");
                assert!(
                    !synthesize64(form, fields).is_empty(),
                    "64 {form}/{fields:#x}"
                );
            }
        }
    }

    /// The long-mode generator has to produce programs the frontend actually
    /// lifts, or a sweep over it passes while measuring nothing.
    #[test]
    fn the_long_mode_generator_produces_programs_the_frontend_lifts() {
        let mut lifted = 0usize;
        for n in 0..200u64 {
            let case = Case::seeded(program64(0x6400_0000 + n, 6)).long();
            if let Ok(Verdict::Agreed { insns, .. } | Verdict::Trapped { insns }) = compare(&case) {
                lifted += insns;
            }
        }
        assert!(lifted > 400, "only {lifted} guest instructions were lifted");
    }
}
