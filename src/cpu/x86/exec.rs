//! The interpreter: decode, execute, and the flag rules.
//!
//! # The bus is the clock
//!
//! Every guest access goes through [`AddressSpace`] one transfer at a time and
//! charges four clocks, which is what an 8088 bus cycle costs with no wait
//! states. There is no table of instruction lengths: `add [bp+di-64h], cl`
//! reads its operand, writes it back, and pays for both because the accesses
//! happened, and a device watching the bus sees them in the order hardware
//! would.
//!
//! What this does *not* model is the overlap between the bus interface unit
//! and the execution unit. A real 8088 prefetches while the previous
//! instruction is still executing, so its instruction time is roughly the
//! larger of the two rather than their sum. Here the prefetch queue is filled
//! at instruction boundaries and refilled on demand, and every fetch is
//! charged, so a cycle count taken over a long run is an upper bound. That is
//! a deliberate trade: getting the T-state-exact overlap right means modelling
//! the microcode, and the accuracy that actually matters for correctness —
//! *which* accesses happen, in what order, with what values — is checked
//! against hardware in [`conformance`](super).
//!
//! # The undefined flags
//!
//! Intel documents a dozen instructions as leaving some flags undefined. The
//! silicon is not undefined, merely unspecified, and software has been found
//! to depend on it. Each of the following was measured against
//! `SingleStepTests/8088` — hardware output, not anyone's emulator — rather
//! than guessed, and each is implemented where it is described:
//!
//! | Instruction | Officially undefined | What the 8088 actually does | Vectors |
//! | --- | --- | --- | --- |
//! | `AND`, `OR`, `XOR`, `TEST` | `AF` | cleared, always ([`Exec::logic_flags8`]) | exact |
//! | `SHL`/`SAL` | `AF` | bit 4 of the result — the microcode is an `ADD dst,dst` ([`Exec::shift8`]) | exact |
//! | `SHR`, `SAR` | `AF` | cleared | exact |
//! | `MUL` | `SF`, `ZF`, `AF`, `PF` | set from the **high half** of the product: `ZF` if it is zero, `SF` from its top bit, `PF` from its low byte, `AF` cleared ([`Exec::mul_flags`]) | exact |
//! | `DAA`, `DAS` | `OF` | from the *single* correcting add or subtract of `0x00`/`0x06`/`0x60`/`0x66`; and the high correction's threshold moves with `AF` ([`Exec::decimal_adjust`]) | exact |
//! | `AAA`, `AAS` | `OF`, `SF`, `ZF`, `PF` | from an 8-bit `AL ± 6` that happens even when no adjustment is needed, with an operand of zero ([`Exec::ascii_adjust`]) | exact |
//! | `AAM 0` | all | the flags of a zero result, then a divide error ([`Exec::aam`]) | exact |
//! | `AAD` | `OF`, `AF`, `CF` | the adjustment really is an addition and sets them ([`Exec::aad`]) | exact |
//! | `SETMO` (`D0`-`D3` `/6`) | all | the operand becomes all ones and the flags follow it as a logical result | exact |
//! | `IMUL` | `SF`, `ZF`, `AF`, `PF` | the sign-corrected magnitude loop's residue; modelled as `MUL`'s rule | ~66% |
//! | `DIV`, `IDIV` | all | `CF` is the complement of the quotient's top bit (exact); the rest are the last trial subtraction's ([`Exec::cord`]) | ~35% |
//!
//! The last two rows are the whole of `conformance::KNOWN_FAILURES`, and the
//! percentages there are measured rather than estimated.
//!
//! One result that is not a flag belongs in the same list, because it is the
//! same kind of surprise: a shift or rotate **by zero** still writes its
//! operand back. Nothing changes and no flag moves, but the write happens on
//! the bus, where a memory-mapped device would see it ([`Exec::shift`]).
//!
//! # Sources
//!
//! Intel's *iAPX 86/88 User's Manual* for the instruction semantics, the flag
//! definitions and the timing tables; `docs/cpu/x86.md` for the rest. The
//! undefined-flag column above is measurement, and the measurement is
//! reproducible by anyone with the corpus. No copyleft emulator was consulted.

use alloc::vec::Vec;

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::isa::{self, Arg, Fields, Op, Rep, seg};
use super::{Config, Lines, Model, Regs, flags, linear};

/// Clocks in one bus cycle, with no wait states. T1 through T4.
const BUS_CLOCKS: u32 = 4;

/// Clocks the reset sequence spends before the first fetch.
const RESET_CLOCKS: u32 = 7;

/// Type 0: divide error.
const VEC_DIVIDE: u8 = 0;
/// Type 1: single step, taken after an instruction when `TF` is set.
const VEC_SINGLE_STEP: u8 = 1;
/// Type 2: NMI.
const VEC_NMI: u8 = 2;
/// Type 3: the one-byte breakpoint.
const VEC_BREAKPOINT: u8 = 3;
/// Type 4: `INTO`.
const VEC_OVERFLOW: u8 = 4;

// ---------------------------------------------------------------------------
// The prefetch queue
// ---------------------------------------------------------------------------

/// The bus interface unit's instruction queue.
///
/// Four bytes on an 8088, six on an 8086. It is not an implementation detail:
/// the queue status lines are pins, an interrupted string instruction restarts
/// from it, and self-modifying code that writes within a few bytes of `IP`
/// behaves differently because of it — which is why `docs/cpu/x86.md` calls
/// self-modifying code mandatory rather than optional.
///
/// The queue holds the bytes at `CS:IP` through `CS:IP+len-1`, so the offset
/// the bus interface unit fetches next is always `IP + len`. Keeping that
/// invariant rather than a separate fetch pointer means a snapshot of `IP` and
/// the queue contents is a complete description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Queue {
    bytes: [u8; 6],
    len: u8,
    depth: u8,
}

impl Queue {
    /// An empty queue of the depth this part has.
    pub(super) const fn new(model: Model) -> Queue {
        Queue {
            bytes: [0; 6],
            len: 0,
            depth: model.queue_bytes(),
        }
    }

    /// Discard everything, as a control transfer does.
    pub(super) const fn flush(&mut self) {
        self.len = 0;
    }

    /// How many bytes are queued.
    pub(super) const fn len(&self) -> u8 {
        self.len
    }

    /// How many bytes the queue holds when full.
    pub(super) const fn depth(&self) -> u8 {
        self.depth
    }

    fn push(&mut self, byte: u8) {
        if self.len < self.depth {
            self.bytes[self.len as usize] = byte;
            self.len += 1;
        }
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.bytes[0];
        // Shifting four bytes down beats the modular arithmetic a ring buffer
        // would need on every single instruction byte.
        let mut i = 1usize;
        while i < self.len as usize {
            self.bytes[i - 1] = self.bytes[i];
            i += 1;
        }
        self.len -= 1;
        Some(byte)
    }

    /// The queued bytes, oldest first.
    pub(super) fn contents(&self) -> Vec<u8> {
        self.bytes[..self.len as usize].to_vec()
    }

    /// Replace the contents, as if the bus interface unit had fetched them.
    ///
    /// # Errors
    ///
    /// If more bytes are given than the queue holds.
    pub(super) fn install(&mut self, bytes: &[u8]) -> Result<(), ()> {
        if bytes.len() > self.depth as usize {
            return Err(());
        }
        self.len = bytes.len() as u8;
        self.bytes[..bytes.len()].copy_from_slice(bytes);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Execution state
// ---------------------------------------------------------------------------

/// The architectural state one core owns.
///
/// Split from [`super::I8086`] because the interrupt *lines* live outside the
/// lock: a device asserting `INTR` from inside a CPU-initiated MMIO write
/// would otherwise re-enter the CPU's own critical section and deadlock (the
/// re-entrancy contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct State {
    /// The register file.
    pub regs: Regs,
    /// Clock cycles executed since power-on.
    pub cycles: u64,
    /// Set by `HLT`; cleared by any interrupt or reset.
    pub halted: bool,
    /// A reset was requested and its sequence has not run yet.
    pub reset_pending: bool,
    /// Interrupts are inhibited for the next instruction.
    ///
    /// Set by `MOV SS,x` and `POP SS` so that the `SS:SP` pair can be reloaded
    /// atomically. On an 8086 the shadow covers NMI too.
    pub int_shadow: bool,
    /// The bus interface unit's queue.
    pub queue: Queue,
    /// The last value driven on the data bus.
    ///
    /// An 8086 has no bus-error input: an access nothing answers leaves the
    /// previous value on the bus, and that is what the CPU reads.
    pub open_bus: u8,
    /// How many accesses an address space refused.
    pub faults: u64,
    /// Physical address of the most recent refused access.
    pub last_fault: u32,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) const fn new(model: Model) -> State {
        State {
            regs: Regs::new(),
            cycles: 0,
            halted: false,
            reset_pending: true,
            int_shadow: false,
            queue: Queue::new(model),
            open_bus: 0,
            faults: 0,
            last_fault: 0,
        }
    }
}

/// One step's worth of execution, borrowing everything it needs.
///
/// Created per step rather than stored: it holds the per-instruction
/// bookkeeping — the effective address, the restart point — that is
/// meaningless between instructions, and dropping it makes that explicit.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    mem: &'a AddressSpace,
    io: Option<&'a AddressSpace>,
    cfg: &'a Config,
    lines: &'a Lines,
    attrs: MemAttrs,
    /// The memory operand's `(segment register, offset)`, computed once.
    ea: Option<(u8, u16)>,
    /// Where the current instruction started, prefixes included, so a string
    /// operation interrupted between iterations can be restarted.
    start_ip: u16,
    /// Clocks this step has charged.
    used: u64,
}

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(
        state: &'a mut State,
        mem: &'a AddressSpace,
        io: Option<&'a AddressSpace>,
        cfg: &'a Config,
        lines: &'a Lines,
    ) -> Exec<'a> {
        let attrs = MemAttrs::DEFAULT.with_requester(cfg.requester);
        Exec {
            state,
            mem,
            io,
            cfg,
            lines,
            attrs,
            ea: None,
            start_ip: 0,
            used: 0,
        }
    }

    /// Run one reset sequence, interrupt sequence, or instruction.
    ///
    /// Returns the clocks charged — zero only when the CPU is halted with
    /// nothing pending, which the caller must notice rather than spin on.
    pub(super) fn step(&mut self) -> u64 {
        if self.state.reset_pending {
            self.reset_sequence();
            return self.used;
        }

        // The shadow suppresses the interrupt check for exactly one
        // instruction. It is read here and cleared below, so an instruction
        // that sets it again extends it by one more.
        let shadow = self.state.int_shadow;
        if !shadow {
            if self.lines.take_nmi_pending() {
                self.state.halted = false;
                self.charge(Op::INT.clocks());
                self.service(VEC_NMI);
                return self.used;
            }
            if self.flag(flags::IF) && self.lines.intr_pending() {
                self.state.halted = false;
                // Two INTA bus cycles; a PC's 8259A drives the vector onto the
                // data bus during the second.
                self.charge(2 * BUS_CLOCKS + Op::INT.clocks());
                let vector = self.lines.intr_vector();
                self.service(vector);
                return self.used;
            }
        }

        if self.state.halted {
            return 0;
        }

        // `TF` is sampled before the instruction runs, which is why a `POPF`
        // that sets it does not trap on itself.
        let trap = self.flag(flags::TF) && !shadow;
        self.state.int_shadow = false;
        self.instruction();
        if trap && !self.state.int_shadow {
            self.charge(Op::INT.clocks());
            self.service(VEC_SINGLE_STEP);
        }
        self.used
    }

    // -----------------------------------------------------------------
    // The clock and the bus
    // -----------------------------------------------------------------

    fn charge(&mut self, clocks: u32) {
        let clocks = u64::from(clocks);
        self.used += clocks;
        self.state.cycles = self.state.cycles.wrapping_add(clocks);
    }

    /// One byte-wide bus read.
    fn bus_read8(&mut self, addr: u32) -> u8 {
        self.charge(BUS_CLOCKS);
        match self.mem.read(u64::from(addr), Width::U8, self.attrs) {
            Ok(value) => {
                let byte = value as u8;
                self.state.open_bus = byte;
                byte
            }
            Err(_) => {
                // No bus-error input exists on this chip, so the honest model
                // is open bus — and the fault counter is how anyone finds out.
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = addr;
                self.state.open_bus
            }
        }
    }

    /// One byte-wide bus write.
    fn bus_write8(&mut self, addr: u32, value: u8) {
        self.charge(BUS_CLOCKS);
        self.state.open_bus = value;
        if self
            .mem
            .write(u64::from(addr), Width::U8, u64::from(value), self.attrs)
            .is_err()
        {
            self.state.faults = self.state.faults.wrapping_add(1);
            self.state.last_fault = addr;
        }
    }

    /// Whether a word at this segment and offset is one bus cycle on this part.
    ///
    /// Only on an 8086, and only when the transfer is aligned and does not
    /// straddle the end of the segment — an 8088 has eight data pins and
    /// always takes two.
    fn word_is_one_cycle(&self, base: u32, offset: u16) -> bool {
        self.cfg.model.bus_bytes() == 2 && base.is_multiple_of(2) && offset != 0xffff
    }

    /// Read a word at an explicit segment value.
    ///
    /// The *offset* wraps, not the physical address: a word at offset `0xffff`
    /// takes its high byte from offset `0x0000` of the same segment. Both
    /// halves then go through the same 20-bit adder, so a segment near the top
    /// of memory wraps at 1 MiB exactly as a byte access would.
    fn read16_seg(&mut self, segment: u16, offset: u16) -> u16 {
        let base = linear(segment, offset);
        if self.word_is_one_cycle(base, offset) {
            self.charge(BUS_CLOCKS);
            return match self.mem.read(u64::from(base), Width::U16, self.attrs) {
                Ok(value) => {
                    self.state.open_bus = (value >> 8) as u8;
                    value as u16
                }
                Err(_) => {
                    self.state.faults = self.state.faults.wrapping_add(1);
                    self.state.last_fault = base;
                    let byte = u16::from(self.state.open_bus);
                    byte | (byte << 8)
                }
            };
        }
        let lo = self.bus_read8(base);
        let hi = self.bus_read8(linear(segment, offset.wrapping_add(1)));
        u16::from(lo) | (u16::from(hi) << 8)
    }

    /// Write a word at an explicit segment value, low byte first.
    fn write16_seg(&mut self, segment: u16, offset: u16, value: u16) {
        let base = linear(segment, offset);
        if self.word_is_one_cycle(base, offset) {
            self.charge(BUS_CLOCKS);
            self.state.open_bus = (value >> 8) as u8;
            if self
                .mem
                .write(u64::from(base), Width::U16, u64::from(value), self.attrs)
                .is_err()
            {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = base;
            }
            return;
        }
        self.bus_write8(base, value as u8);
        self.bus_write8(linear(segment, offset.wrapping_add(1)), (value >> 8) as u8);
    }

    fn read8(&mut self, sr: u8, offset: u16) -> u8 {
        let segment = self.state.regs.segment(sr);
        self.bus_read8(linear(segment, offset))
    }

    fn write8(&mut self, sr: u8, offset: u16, value: u8) {
        let segment = self.state.regs.segment(sr);
        self.bus_write8(linear(segment, offset), value);
    }

    fn read16(&mut self, sr: u8, offset: u16) -> u16 {
        let segment = self.state.regs.segment(sr);
        self.read16_seg(segment, offset)
    }

    fn write16(&mut self, sr: u8, offset: u16, value: u16) {
        let segment = self.state.regs.segment(sr);
        self.write16_seg(segment, offset, value);
    }

    /// One I/O read. A core with no I/O space sees an unterminated bus, which
    /// reads as ones — the same answer the corpus expects from a bare 8088.
    fn io_read8(&mut self, port: u16) -> u8 {
        self.charge(BUS_CLOCKS);
        let Some(io) = self.io else {
            return 0xff;
        };
        match io.read(u64::from(port), Width::U8, self.attrs) {
            Ok(value) => value as u8,
            Err(_) => {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = u32::from(port);
                0xff
            }
        }
    }

    fn io_write8(&mut self, port: u16, value: u8) {
        self.charge(BUS_CLOCKS);
        let Some(io) = self.io else {
            return;
        };
        if io
            .write(u64::from(port), Width::U8, u64::from(value), self.attrs)
            .is_err()
        {
            self.state.faults = self.state.faults.wrapping_add(1);
            self.state.last_fault = u32::from(port);
        }
    }

    // -----------------------------------------------------------------
    // Instruction fetch
    // -----------------------------------------------------------------

    /// Top the prefetch queue up, as the bus interface unit does whenever the
    /// execution unit leaves the bus free.
    fn fill_queue(&mut self) {
        while self.state.queue.len() < self.state.queue.depth() {
            let offset = self
                .state
                .regs
                .ip
                .wrapping_add(u16::from(self.state.queue.len()));
            let byte = self.read8(seg::CS, offset);
            self.state.queue.push(byte);
        }
    }

    /// Take the next instruction byte, refilling the queue if it is empty.
    fn fetch_byte(&mut self) -> u8 {
        if self.state.queue.len() == 0 {
            let offset = self.state.regs.ip;
            let byte = self.read8(seg::CS, offset);
            self.state.queue.push(byte);
        }
        let byte = self.state.queue.pop().unwrap_or(self.state.open_bus);
        // Guest arithmetic wraps: IP is 16 bits and `ffff` is followed by
        // `0000` in the same code segment.
        self.state.regs.ip = self.state.regs.ip.wrapping_add(1);
        byte
    }

    // -----------------------------------------------------------------
    // Flags
    // -----------------------------------------------------------------

    fn flag(&self, mask: u16) -> bool {
        self.state.regs.flags & mask != 0
    }

    fn set_flag(&mut self, mask: u16, on: bool) {
        if on {
            self.state.regs.flags |= mask;
        } else {
            self.state.regs.flags &= !mask;
        }
    }

    /// Even parity of the low eight bits — the only parity an 8086 computes.
    const fn parity(value: u8) -> bool {
        (value.count_ones() & 1) == 0
    }

    fn set_szp8(&mut self, value: u8) {
        self.set_flag(flags::ZF, value == 0);
        self.set_flag(flags::SF, value & 0x80 != 0);
        self.set_flag(flags::PF, Self::parity(value));
    }

    fn set_szp16(&mut self, value: u16) {
        self.set_flag(flags::ZF, value == 0);
        self.set_flag(flags::SF, value & 0x8000 != 0);
        self.set_flag(flags::PF, Self::parity(value as u8));
    }

    // -----------------------------------------------------------------
    // The ALU
    // -----------------------------------------------------------------

    fn add8(&mut self, a: u8, b: u8, carry: bool) -> u8 {
        let sum = u16::from(a) + u16::from(b) + u16::from(carry);
        let r = sum as u8;
        self.set_flag(flags::CF, sum > 0xff);
        self.set_flag(flags::AF, (a ^ b ^ r) & 0x10 != 0);
        self.set_flag(flags::OF, (!(a ^ b)) & (a ^ r) & 0x80 != 0);
        self.set_szp8(r);
        r
    }

    fn add16(&mut self, a: u16, b: u16, carry: bool) -> u16 {
        let sum = u32::from(a) + u32::from(b) + u32::from(carry);
        let r = sum as u16;
        self.set_flag(flags::CF, sum > 0xffff);
        self.set_flag(flags::AF, (a ^ b ^ r) & 0x10 != 0);
        self.set_flag(flags::OF, (!(a ^ b)) & (a ^ r) & 0x8000 != 0);
        self.set_szp16(r);
        r
    }

    fn sub8(&mut self, a: u8, b: u8, borrow: bool) -> u8 {
        let rhs = u16::from(b) + u16::from(borrow);
        let diff = u16::from(a).wrapping_sub(rhs);
        let r = diff as u8;
        self.set_flag(flags::CF, u16::from(a) < rhs);
        self.set_flag(flags::AF, (a ^ b ^ r) & 0x10 != 0);
        self.set_flag(flags::OF, (a ^ b) & (a ^ r) & 0x80 != 0);
        self.set_szp8(r);
        r
    }

    fn sub16(&mut self, a: u16, b: u16, borrow: bool) -> u16 {
        let rhs = u32::from(b) + u32::from(borrow);
        let diff = u32::from(a).wrapping_sub(rhs);
        let r = diff as u16;
        self.set_flag(flags::CF, u32::from(a) < rhs);
        self.set_flag(flags::AF, (a ^ b ^ r) & 0x10 != 0);
        self.set_flag(flags::OF, (a ^ b) & (a ^ r) & 0x8000 != 0);
        self.set_szp16(r);
        r
    }

    /// Flags after `AND`, `OR`, `XOR` and `TEST`.
    ///
    /// Carry and overflow are documented as cleared. `AF` is documented as
    /// undefined; on this part it is cleared too, on every one of the tens of
    /// thousands of corpus vectors that exercise it, so it is modelled as
    /// cleared rather than left alone.
    fn logic_flags8(&mut self, r: u8) {
        self.set_flag(flags::CF | flags::OF | flags::AF, false);
        self.set_szp8(r);
    }

    fn logic_flags16(&mut self, r: u16) {
        self.set_flag(flags::CF | flags::OF | flags::AF, false);
        self.set_szp16(r);
    }

    // -----------------------------------------------------------------
    // Sequences
    // -----------------------------------------------------------------

    /// The RESET sequence.
    ///
    /// `CS:IP` becomes `ffff:0000`, every other segment register zero, and the
    /// flags only their hard-wired bits. The general registers are not
    /// specified by Intel and are left alone, which is what hardware does — a
    /// cold [`Device::reset`](crate::core::device::Device::reset) zeroes them
    /// separately, because determinism is a first-class mode.
    fn reset_sequence(&mut self) {
        self.state.reset_pending = false;
        self.state.halted = false;
        self.state.int_shadow = false;
        let regs = &mut self.state.regs;
        regs.cs = 0xffff;
        regs.ip = 0;
        regs.ds = 0;
        regs.es = 0;
        regs.ss = 0;
        regs.flags = flags::RESERVED_SET;
        self.state.queue.flush();
        self.charge(RESET_CLOCKS);
    }

    /// Take an interrupt of the given type.
    ///
    /// The order is the one the hardware traces show, and it is not the order
    /// the manual's prose suggests: **the vector is read first**, before
    /// anything is pushed. Then flags, then `CS`, then the return `IP` — and
    /// `IF` and `TF` are cleared between the flags push and the `CS` push, so
    /// the saved flags still have them.
    fn service(&mut self, vector: u8) {
        let base = u16::from(vector) << 2;
        let target_ip = self.read16_seg(0, base);
        let target_cs = self.read16_seg(0, base.wrapping_add(2));

        let saved = self.state.regs.flags;
        self.push_word(saved);
        self.set_flag(flags::IF | flags::TF, false);
        self.push_word(self.state.regs.cs);
        self.push_word(self.state.regs.ip);

        self.state.regs.cs = target_cs;
        self.state.regs.ip = target_ip;
        self.state.queue.flush();
        self.state.halted = false;
    }

    /// Push a word: the stack pointer moves first, then the write happens.
    ///
    /// That order is why `PUSH SP` stores `SP - 2` on an 8086 and the value
    /// before the decrement on a 286.
    fn push_word(&mut self, value: u16) {
        let sp = self.state.regs.sp.wrapping_sub(2);
        self.state.regs.sp = sp;
        self.write16(seg::SS, sp, value);
    }

    fn pop_word(&mut self) -> u16 {
        let sp = self.state.regs.sp;
        let value = self.read16(seg::SS, sp);
        self.state.regs.sp = sp.wrapping_add(2);
        value
    }

    // -----------------------------------------------------------------
    // Decode
    // -----------------------------------------------------------------

    fn instruction(&mut self) {
        self.start_ip = self.state.regs.ip;
        self.fill_queue();
        let fields = isa::decode_stream(&mut || Some(self.fetch_byte()));
        self.prepare_ea(&fields);
        self.charge(fields.insn.op.clocks());
        self.execute(&fields);
    }

    /// Compute the memory operand's address once, before execution.
    ///
    /// The 8086 computes it in microcode and the cost depends on how many
    /// terms are summed, which is why the charge happens here rather than at
    /// the access.
    fn prepare_ea(&mut self, f: &Fields) {
        self.ea = None;
        let insn = f.insn;
        if let Some(m) = f.modrm
            && !m.is_register()
            && [insn.dst, insn.src]
                .iter()
                .any(|a| matches!(a, Arg::Eb | Arg::Ev | Arg::M | Arg::Mp))
        {
            let regs = &self.state.regs;
            let terms = match m.rm {
                0 => regs.bx.wrapping_add(regs.si),
                1 => regs.bx.wrapping_add(regs.di),
                2 => regs.bp.wrapping_add(regs.si),
                3 => regs.bp.wrapping_add(regs.di),
                4 => regs.si,
                5 => regs.di,
                6 if m.md == 0 => 0,
                6 => regs.bp,
                _ => regs.bx,
            };
            let offset = if m.md == 0 && m.rm == 6 {
                f.disp
            } else {
                terms.wrapping_add(f.disp)
            };
            self.ea = Some((f.segment(m.default_segment()), offset));
            self.charge(isa::ea_clocks(m.md, m.rm, f.seg_override.is_some()));
        } else if [insn.dst, insn.src]
            .iter()
            .any(|a| matches!(a, Arg::Ob | Arg::Ov))
        {
            // The direct-offset moves carry their address in the immediate
            // field and have no ModRM byte at all.
            self.ea = Some((f.segment(seg::DS), f.imm16()));
        }
    }

    fn ea(&self) -> (u8, u16) {
        self.ea.unwrap_or((seg::DS, 0))
    }

    // -----------------------------------------------------------------
    // Operands
    // -----------------------------------------------------------------

    fn read_arg8(&mut self, f: &Fields, arg: Arg) -> u8 {
        match arg {
            Arg::Eb => match f.modrm {
                Some(m) if m.is_register() => self.state.regs.byte(m.rm),
                _ => {
                    let (sr, off) = self.ea();
                    self.read8(sr, off)
                }
            },
            Arg::Gb => self.state.regs.byte(f.modrm.map_or(0, |m| m.reg)),
            Arg::Ib => f.imm as u8,
            Arg::Rb => self.state.regs.byte(f.opcode & 7),
            Arg::Al => self.state.regs.ax as u8,
            Arg::Cl => self.state.regs.cx as u8,
            Arg::One => 1,
            Arg::Ob => {
                let (sr, off) = self.ea();
                self.read8(sr, off)
            }
            // String operands are driven by `string_step`, which knows which
            // pointer moves; nothing else should ask for one.
            _ => 0,
        }
    }

    fn write_arg8(&mut self, f: &Fields, arg: Arg, value: u8) {
        match arg {
            Arg::Eb => match f.modrm {
                Some(m) if m.is_register() => self.state.regs.set_byte(m.rm, value),
                _ => {
                    let (sr, off) = self.ea();
                    self.write8(sr, off, value);
                }
            },
            Arg::Gb => self
                .state
                .regs
                .set_byte(f.modrm.map_or(0, |m| m.reg), value),
            Arg::Rb => self.state.regs.set_byte(f.opcode & 7, value),
            Arg::Al => self.state.regs.set_byte(0, value),
            Arg::Cl => self.state.regs.set_byte(1, value),
            Arg::Ob => {
                let (sr, off) = self.ea();
                self.write8(sr, off, value);
            }
            _ => {}
        }
    }

    fn read_arg16(&mut self, f: &Fields, arg: Arg) -> u16 {
        match arg {
            Arg::Ev => match f.modrm {
                Some(m) if m.is_register() => self.state.regs.word(m.rm),
                _ => {
                    let (sr, off) = self.ea();
                    self.read16(sr, off)
                }
            },
            Arg::Gv => self.state.regs.word(f.modrm.map_or(0, |m| m.reg)),
            // Only the low two bits of `reg` are decoded, which is why `8C`
            // and `8E` accept a `reg` of 4-7 and alias down onto ES-DS.
            Arg::Sw => self.state.regs.segment(f.modrm.map_or(0, |m| m.reg) & 3),
            Arg::Sr => self.state.regs.segment((f.opcode >> 3) & 3),
            Arg::Iv | Arg::Ibs => f.imm16(),
            Arg::Rv => self.state.regs.word(f.opcode & 7),
            Arg::Ax => self.state.regs.ax,
            Arg::Dx => self.state.regs.dx,
            Arg::M => self.ea().1,
            Arg::Ov => {
                let (sr, off) = self.ea();
                self.read16(sr, off)
            }
            _ => 0,
        }
    }

    fn write_arg16(&mut self, f: &Fields, arg: Arg, value: u16) {
        match arg {
            Arg::Ev => match f.modrm {
                Some(m) if m.is_register() => self.state.regs.set_word(m.rm, value),
                _ => {
                    let (sr, off) = self.ea();
                    self.write16(sr, off, value);
                }
            },
            Arg::Gv => self
                .state
                .regs
                .set_word(f.modrm.map_or(0, |m| m.reg), value),
            Arg::Sw => {
                let sr = f.modrm.map_or(0, |m| m.reg) & 3;
                self.load_segment(sr, value);
            }
            Arg::Sr => {
                let sr = (f.opcode >> 3) & 3;
                self.load_segment(sr, value);
            }
            Arg::Rv => self.state.regs.set_word(f.opcode & 7, value),
            Arg::Ax => self.state.regs.ax = value,
            Arg::Dx => self.state.regs.dx = value,
            Arg::Ov => {
                let (sr, off) = self.ea();
                self.write16(sr, off, value);
            }
            _ => {}
        }
    }

    /// Load a segment register, opening the interrupt shadow for `SS`.
    ///
    /// Every write to `SS` inhibits interrupts for one instruction, whichever
    /// encoding did it, so that `MOV SS,ax` / `MOV SP,bx` cannot be split by
    /// an interrupt landing on a half-changed stack. Nothing else on an 8086
    /// makes a two-instruction sequence atomic.
    fn load_segment(&mut self, sr: u8, value: u16) {
        self.state.regs.set_segment(sr, value);
        if sr == seg::SS {
            self.state.int_shadow = true;
        }
    }

    /// The operand width this encoding fixes, in bytes.
    fn width(f: &Fields) -> u8 {
        f.insn.width_bytes().unwrap_or(2)
    }

    // -----------------------------------------------------------------
    // Execution
    // -----------------------------------------------------------------

    #[allow(clippy::too_many_lines)]
    fn execute(&mut self, f: &Fields) {
        let insn = f.insn;
        match insn.op {
            Op::ADD | Op::ADC | Op::SUB | Op::SBB | Op::CMP | Op::AND | Op::OR | Op::XOR => {
                self.arith(f);
            }
            Op::TEST => self.test(f),
            Op::INC | Op::DEC => self.inc_dec(f),
            Op::NOT => {
                if Self::width(f) == 1 {
                    let a = self.read_arg8(f, insn.dst);
                    self.write_arg8(f, insn.dst, !a);
                } else {
                    let a = self.read_arg16(f, insn.dst);
                    self.write_arg16(f, insn.dst, !a);
                }
            }
            Op::NEG => {
                if Self::width(f) == 1 {
                    let a = self.read_arg8(f, insn.dst);
                    let r = self.sub8(0, a, false);
                    self.write_arg8(f, insn.dst, r);
                } else {
                    let a = self.read_arg16(f, insn.dst);
                    let r = self.sub16(0, a, false);
                    self.write_arg16(f, insn.dst, r);
                }
            }
            Op::MOV => {
                if Self::width(f) == 1 {
                    let v = self.read_arg8(f, insn.src);
                    self.write_arg8(f, insn.dst, v);
                } else {
                    let v = self.read_arg16(f, insn.src);
                    self.write_arg16(f, insn.dst, v);
                }
            }
            Op::XCHG => self.xchg(f),
            Op::LEA => {
                let offset = self.ea().1;
                self.write_arg16(f, insn.dst, offset);
            }
            Op::LES | Op::LDS => {
                let (sr, off) = self.ea();
                let value = self.read16(sr, off);
                let segment = self.read16(sr, off.wrapping_add(2));
                self.write_arg16(f, insn.dst, value);
                let target = if insn.op == Op::LES { seg::ES } else { seg::DS };
                self.load_segment(target, segment);
            }
            Op::PUSH => {
                // The pointer moves before the operand is read, which is what
                // makes `PUSH SP` push the decremented value.
                let sp = self.state.regs.sp.wrapping_sub(2);
                self.state.regs.sp = sp;
                let value = self.read_arg16(f, insn.dst);
                self.write16(seg::SS, sp, value);
            }
            Op::POP => {
                let value = self.pop_word();
                self.write_arg16(f, insn.dst, value);
            }
            Op::PUSHF => {
                let value = self.state.regs.flags;
                self.push_word(value);
            }
            Op::POPF => {
                let value = self.pop_word();
                self.state.regs.flags = Regs::normalise_flags(value);
            }
            Op::SAHF => {
                let ah = (self.state.regs.ax >> 8) & 0xff;
                let kept = self.state.regs.flags & !flags::LOW_BYTE;
                self.state.regs.flags = Regs::normalise_flags(kept | (ah & flags::LOW_BYTE));
            }
            Op::LAHF => {
                let low = (self.state.regs.flags & 0xff) as u8;
                self.state.regs.set_byte(4, low);
            }
            Op::CBW => {
                let al = self.state.regs.ax as u8;
                self.state.regs.ax = i16::from(al as i8) as u16;
            }
            Op::CWD => {
                self.state.regs.dx = if self.state.regs.ax & 0x8000 != 0 {
                    0xffff
                } else {
                    0
                };
            }
            Op::ROL | Op::ROR | Op::RCL | Op::RCR | Op::SHL | Op::SHR | Op::SAR | Op::SETMO => {
                self.shift(f);
            }
            Op::MUL | Op::IMUL => self.multiply(f),
            Op::DIV | Op::IDIV => self.divide(f),
            Op::AAM => self.aam(f),
            Op::AAD => self.aad(f),
            Op::DAA => self.daa(),
            Op::DAS => self.das(),
            Op::AAA => self.aaa(),
            Op::AAS => self.aas(),
            Op::CLC => self.set_flag(flags::CF, false),
            Op::STC => self.set_flag(flags::CF, true),
            Op::CMC => {
                let cf = self.flag(flags::CF);
                self.set_flag(flags::CF, !cf);
            }
            Op::CLD => self.set_flag(flags::DF, false),
            Op::STD => self.set_flag(flags::DF, true),
            Op::CLI => self.set_flag(flags::IF, false),
            Op::STI => {
                self.set_flag(flags::IF, true);
                // `STI` shares the one-instruction shadow with `MOV SS`: the
                // instruction after it runs before any interrupt can, which is
                // what makes `sti` / `hlt` race-free.
                self.state.int_shadow = true;
            }
            Op::NOP | Op::WAIT | Op::LOCK | Op::REP | Op::REPNE | Op::SEG => {}
            Op::HLT => self.state.halted = true,
            Op::ESC => {
                // The 8088 computes the address and performs the read so the
                // coprocessor can latch the operand off the bus; the value is
                // of no use to the CPU itself.
                if matches!(f.modrm, Some(m) if !m.is_register()) {
                    let (sr, off) = self.ea();
                    let _ = self.read16(sr, off);
                }
            }
            Op::SALC => {
                let value = if self.flag(flags::CF) { 0xff } else { 0x00 };
                self.state.regs.set_byte(0, value);
            }
            Op::XLAT => {
                let sr = f.segment(seg::DS);
                let al = self.state.regs.ax as u8;
                let offset = self.state.regs.bx.wrapping_add(u16::from(al));
                let value = self.read8(sr, offset);
                self.state.regs.set_byte(0, value);
            }
            Op::IN => {
                let port = self.port(f, insn.src);
                if insn.dst == Arg::Al {
                    let value = self.io_read8(port);
                    self.state.regs.set_byte(0, value);
                } else {
                    let lo = self.io_read8(port);
                    let hi = self.io_read8(port.wrapping_add(1));
                    self.state.regs.ax = u16::from(lo) | (u16::from(hi) << 8);
                }
            }
            Op::OUT => {
                let port = self.port(f, insn.dst);
                if insn.src == Arg::Al {
                    let value = self.state.regs.ax as u8;
                    self.io_write8(port, value);
                } else {
                    let value = self.state.regs.ax;
                    self.io_write8(port, value as u8);
                    self.io_write8(port.wrapping_add(1), (value >> 8) as u8);
                }
            }
            Op::CALL => {
                let target = match insn.dst {
                    Arg::Jv | Arg::Jb => self.state.regs.ip.wrapping_add(f.imm16()),
                    _ => self.read_arg16(f, insn.dst),
                };
                let ret = self.state.regs.ip;
                self.push_word(ret);
                self.state.regs.ip = target;
                self.state.queue.flush();
            }
            Op::CALLF => {
                let (offset, segment) = self.far_target(f);
                let cs = self.state.regs.cs;
                let ip = self.state.regs.ip;
                self.push_word(cs);
                self.push_word(ip);
                self.state.regs.cs = segment;
                self.state.regs.ip = offset;
                self.state.queue.flush();
            }
            Op::JMP => {
                let target = match insn.dst {
                    Arg::Jv | Arg::Jb => self.state.regs.ip.wrapping_add(f.imm16()),
                    _ => self.read_arg16(f, insn.dst),
                };
                self.state.regs.ip = target;
                self.state.queue.flush();
            }
            Op::JMPF => {
                let (offset, segment) = self.far_target(f);
                self.state.regs.cs = segment;
                self.state.regs.ip = offset;
                self.state.queue.flush();
            }
            Op::RET => {
                let ip = self.pop_word();
                self.state.regs.ip = ip;
                if insn.dst == Arg::Iv {
                    self.state.regs.sp = self.state.regs.sp.wrapping_add(f.imm16());
                }
                self.state.queue.flush();
            }
            Op::RETF => {
                let ip = self.pop_word();
                let cs = self.pop_word();
                self.state.regs.ip = ip;
                self.state.regs.cs = cs;
                if insn.dst == Arg::Iv {
                    self.state.regs.sp = self.state.regs.sp.wrapping_add(f.imm16());
                }
                self.state.queue.flush();
            }
            Op::IRET => {
                let ip = self.pop_word();
                let cs = self.pop_word();
                let fl = self.pop_word();
                self.state.regs.ip = ip;
                self.state.regs.cs = cs;
                self.state.regs.flags = Regs::normalise_flags(fl);
                self.state.queue.flush();
            }
            Op::INT => {
                let vector = f.imm as u8;
                self.service(vector);
            }
            Op::INT3 => self.service(VEC_BREAKPOINT),
            Op::INTO => {
                if self.flag(flags::OF) {
                    self.service(VEC_OVERFLOW);
                }
            }
            Op::LOOP | Op::LOOPE | Op::LOOPNE => {
                let cx = self.state.regs.cx.wrapping_sub(1);
                self.state.regs.cx = cx;
                let zf = self.flag(flags::ZF);
                let take = cx != 0
                    && match insn.op {
                        Op::LOOPE => zf,
                        Op::LOOPNE => !zf,
                        _ => true,
                    };
                if take {
                    self.state.regs.ip = self.state.regs.ip.wrapping_add(f.imm16());
                    self.state.queue.flush();
                }
            }
            Op::JCXZ => {
                if self.state.regs.cx == 0 {
                    self.state.regs.ip = self.state.regs.ip.wrapping_add(f.imm16());
                    self.state.queue.flush();
                }
            }
            op if op.is_conditional_jump() => {
                if self.condition(op) {
                    self.state.regs.ip = self.state.regs.ip.wrapping_add(f.imm16());
                    self.state.queue.flush();
                }
            }
            op if op.is_string() => self.string(f),
            // Every operation the table can produce is handled above; this arm
            // exists because `Op` is `#[non_exhaustive]` to the compiler.
            _ => {}
        }
    }

    /// The I/O port an `IN` or `OUT` names.
    fn port(&self, f: &Fields, arg: Arg) -> u16 {
        match arg {
            Arg::Dx => self.state.regs.dx,
            _ => u16::from(f.imm as u8),
        }
    }

    /// The `offset:segment` pair a far transfer jumps to.
    fn far_target(&mut self, f: &Fields) -> (u16, u16) {
        if f.insn.dst == Arg::Ap {
            (f.imm16(), f.imm_seg())
        } else {
            let (sr, off) = self.ea();
            let offset = self.read16(sr, off);
            let segment = self.read16(sr, off.wrapping_add(2));
            (offset, segment)
        }
    }

    fn condition(&self, op: Op) -> bool {
        let cf = self.flag(flags::CF);
        let zf = self.flag(flags::ZF);
        let sf = self.flag(flags::SF);
        let of = self.flag(flags::OF);
        let pf = self.flag(flags::PF);
        match op {
            Op::JO => of,
            Op::JNO => !of,
            Op::JB => cf,
            Op::JNB => !cf,
            Op::JZ => zf,
            Op::JNZ => !zf,
            Op::JBE => cf || zf,
            Op::JA => !cf && !zf,
            Op::JS => sf,
            Op::JNS => !sf,
            Op::JP => pf,
            Op::JNP => !pf,
            Op::JL => sf != of,
            Op::JGE => sf == of,
            Op::JLE => zf || (sf != of),
            _ => !zf && (sf == of),
        }
    }

    fn arith(&mut self, f: &Fields) {
        let insn = f.insn;
        let carry = self.flag(flags::CF);
        if Self::width(f) == 1 {
            let a = self.read_arg8(f, insn.dst);
            let b = self.read_arg8(f, insn.src);
            let r = match insn.op {
                Op::ADD => self.add8(a, b, false),
                Op::ADC => self.add8(a, b, carry),
                Op::SUB | Op::CMP => self.sub8(a, b, false),
                Op::SBB => self.sub8(a, b, carry),
                Op::AND => {
                    let r = a & b;
                    self.logic_flags8(r);
                    r
                }
                Op::OR => {
                    let r = a | b;
                    self.logic_flags8(r);
                    r
                }
                _ => {
                    let r = a ^ b;
                    self.logic_flags8(r);
                    r
                }
            };
            if insn.op != Op::CMP {
                self.write_arg8(f, insn.dst, r);
            }
        } else {
            let a = self.read_arg16(f, insn.dst);
            let b = self.read_arg16(f, insn.src);
            let r = match insn.op {
                Op::ADD => self.add16(a, b, false),
                Op::ADC => self.add16(a, b, carry),
                Op::SUB | Op::CMP => self.sub16(a, b, false),
                Op::SBB => self.sub16(a, b, carry),
                Op::AND => {
                    let r = a & b;
                    self.logic_flags16(r);
                    r
                }
                Op::OR => {
                    let r = a | b;
                    self.logic_flags16(r);
                    r
                }
                _ => {
                    let r = a ^ b;
                    self.logic_flags16(r);
                    r
                }
            };
            if insn.op != Op::CMP {
                self.write_arg16(f, insn.dst, r);
            }
        }
    }

    fn test(&mut self, f: &Fields) {
        let insn = f.insn;
        if Self::width(f) == 1 {
            let a = self.read_arg8(f, insn.dst);
            let b = self.read_arg8(f, insn.src);
            self.logic_flags8(a & b);
        } else {
            let a = self.read_arg16(f, insn.dst);
            let b = self.read_arg16(f, insn.src);
            self.logic_flags16(a & b);
        }
    }

    /// `INC` and `DEC` are an add and a subtract that leave carry alone.
    fn inc_dec(&mut self, f: &Fields) {
        let insn = f.insn;
        let carry = self.flag(flags::CF);
        if Self::width(f) == 1 {
            let a = self.read_arg8(f, insn.dst);
            let r = if insn.op == Op::INC {
                self.add8(a, 1, false)
            } else {
                self.sub8(a, 1, false)
            };
            self.write_arg8(f, insn.dst, r);
        } else {
            let a = self.read_arg16(f, insn.dst);
            let r = if insn.op == Op::INC {
                self.add16(a, 1, false)
            } else {
                self.sub16(a, 1, false)
            };
            self.write_arg16(f, insn.dst, r);
        }
        self.set_flag(flags::CF, carry);
    }

    fn xchg(&mut self, f: &Fields) {
        let insn = f.insn;
        if Self::width(f) == 1 {
            let a = self.read_arg8(f, insn.dst);
            let b = self.read_arg8(f, insn.src);
            self.write_arg8(f, insn.dst, b);
            self.write_arg8(f, insn.src, a);
        } else {
            let a = self.read_arg16(f, insn.dst);
            let b = self.read_arg16(f, insn.src);
            self.write_arg16(f, insn.dst, b);
            self.write_arg16(f, insn.src, a);
        }
    }

    // -----------------------------------------------------------------
    // Shifts and rotates
    // -----------------------------------------------------------------

    fn shift(&mut self, f: &Fields) {
        let insn = f.insn;
        // The 8086 uses the whole of `CL`: masking the count to five bits is
        // 80186 behaviour, and the corpus masks `CL` to six bits precisely to
        // catch a core that made that mistake.
        let count = if insn.src == Arg::One {
            1
        } else {
            self.state.regs.cx as u8
        };
        // The write-back happens even when the count is zero and nothing
        // changed: the 8088 still drives the operand back onto the bus, which
        // a memory-mapped device would see. The flags, by contrast, really are
        // left alone.
        if Self::width(f) == 1 {
            let a = self.read_arg8(f, insn.dst);
            let r = self.shift8(insn.op, a, count);
            self.write_arg8(f, insn.dst, r);
        } else {
            let a = self.read_arg16(f, insn.dst);
            let r = self.shift16(insn.op, a, count);
            self.write_arg16(f, insn.dst, r);
        }
    }

    fn shift8(&mut self, op: Op, value: u8, count: u8) -> u8 {
        if count == 0 {
            return value;
        }
        if op == Op::SETMO {
            // Undocumented, and the corpus says every flag is affected: the
            // result is all ones and the flags follow it as a logical result
            // would, carry and overflow cleared.
            self.logic_flags8(0xff);
            return 0xff;
        }
        let mut v = value;
        let mut cf = self.flag(flags::CF);
        let mut of = self.flag(flags::OF);
        for _ in 0..count {
            match op {
                Op::ROL => {
                    cf = v & 0x80 != 0;
                    v = (v << 1) | u8::from(cf);
                    of = (v & 0x80 != 0) != cf;
                }
                Op::ROR => {
                    cf = v & 1 != 0;
                    v = (v >> 1) | (u8::from(cf) << 7);
                    of = ((v >> 7) ^ (v >> 6)) & 1 != 0;
                }
                Op::RCL => {
                    let carry_in = cf;
                    cf = v & 0x80 != 0;
                    v = (v << 1) | u8::from(carry_in);
                    of = (v & 0x80 != 0) != cf;
                }
                Op::RCR => {
                    let carry_in = cf;
                    of = (v & 0x80 != 0) != carry_in;
                    cf = v & 1 != 0;
                    v = (v >> 1) | (u8::from(carry_in) << 7);
                }
                Op::SHL => {
                    cf = v & 0x80 != 0;
                    v <<= 1;
                    of = (v & 0x80 != 0) != cf;
                }
                Op::SHR => {
                    of = v & 0x80 != 0;
                    cf = v & 1 != 0;
                    v >>= 1;
                }
                _ => {
                    of = false;
                    cf = v & 1 != 0;
                    v = ((v as i8) >> 1) as u8;
                }
            }
        }
        self.set_flag(flags::CF, cf);
        self.set_flag(flags::OF, of);
        if matches!(op, Op::SHL | Op::SHR | Op::SAR) {
            self.set_szp8(v);
            // `AF` is documented as undefined for the shifts. On this part a
            // left shift leaves bit 4 of the result there — the microcode is
            // an `ADD dst,dst`, so the auxiliary carry is the real one — and
            // the right shifts clear it.
            self.set_flag(flags::AF, op == Op::SHL && v & 0x10 != 0);
        }
        v
    }

    fn shift16(&mut self, op: Op, value: u16, count: u8) -> u16 {
        if count == 0 {
            return value;
        }
        if op == Op::SETMO {
            self.logic_flags16(0xffff);
            return 0xffff;
        }
        let mut v = value;
        let mut cf = self.flag(flags::CF);
        let mut of = self.flag(flags::OF);
        for _ in 0..count {
            match op {
                Op::ROL => {
                    cf = v & 0x8000 != 0;
                    v = (v << 1) | u16::from(cf);
                    of = (v & 0x8000 != 0) != cf;
                }
                Op::ROR => {
                    cf = v & 1 != 0;
                    v = (v >> 1) | (u16::from(cf) << 15);
                    of = ((v >> 15) ^ (v >> 14)) & 1 != 0;
                }
                Op::RCL => {
                    let carry_in = cf;
                    cf = v & 0x8000 != 0;
                    v = (v << 1) | u16::from(carry_in);
                    of = (v & 0x8000 != 0) != cf;
                }
                Op::RCR => {
                    let carry_in = cf;
                    of = (v & 0x8000 != 0) != carry_in;
                    cf = v & 1 != 0;
                    v = (v >> 1) | (u16::from(carry_in) << 15);
                }
                Op::SHL => {
                    cf = v & 0x8000 != 0;
                    v <<= 1;
                    of = (v & 0x8000 != 0) != cf;
                }
                Op::SHR => {
                    of = v & 0x8000 != 0;
                    cf = v & 1 != 0;
                    v >>= 1;
                }
                _ => {
                    of = false;
                    cf = v & 1 != 0;
                    v = ((v as i16) >> 1) as u16;
                }
            }
        }
        self.set_flag(flags::CF, cf);
        self.set_flag(flags::OF, of);
        if matches!(op, Op::SHL | Op::SHR | Op::SAR) {
            self.set_szp16(v);
            self.set_flag(flags::AF, op == Op::SHL && v & 0x10 != 0);
        }
        v
    }

    // -----------------------------------------------------------------
    // Multiply and divide
    // -----------------------------------------------------------------

    /// The flags a multiply leaves.
    ///
    /// Intel documents `SF`, `ZF`, `AF` and `PF` as undefined here. The
    /// hardware sets them from the **high half** of the product, which is what
    /// the microcode's last step operates on: `ZF` if that half is zero, `SF`
    /// from its top bit, `PF` from its low byte, and `AF` cleared. `CF` and
    /// `OF` are the documented ones, and the caller supplies them because
    /// `MUL` and `IMUL` disagree about what "the result does not fit" means.
    fn mul_flags(&mut self, high: u16, overflow: bool) {
        self.set_flag(flags::ZF, high == 0);
        self.set_flag(flags::SF, high & 0x8000 != 0);
        self.set_flag(flags::PF, Self::parity(high as u8));
        self.set_flag(flags::AF, false);
        self.set_flag(flags::CF | flags::OF, overflow);
    }

    fn multiply(&mut self, f: &Fields) {
        let insn = f.insn;
        let signed = insn.op == Op::IMUL;
        if Self::width(f) == 1 {
            let src = self.read_arg8(f, insn.dst);
            let al = self.state.regs.ax as u8;
            let product = if signed {
                (i16::from(al as i8) * i16::from(src as i8)) as u16
            } else {
                u16::from(al) * u16::from(src)
            };
            self.state.regs.ax = product;
            let high = (product >> 8) as u8;
            let overflow = if signed {
                // Signed overflow means the product does not fit back in AL
                // with its sign intact.
                high != if product & 0x80 != 0 { 0xff } else { 0x00 }
            } else {
                high != 0
            };
            // The high half of a byte multiply is `AH`, so the same rule
            // reads its sign out of bit 7 rather than bit 15.
            self.mul_flags(u16::from(high) << 8, overflow);
            self.set_flag(flags::PF, Self::parity(high));
        } else {
            let src = self.read_arg16(f, insn.dst);
            let ax = self.state.regs.ax;
            let product = if signed {
                (i32::from(ax as i16) * i32::from(src as i16)) as u32
            } else {
                u32::from(ax) * u32::from(src)
            };
            self.state.regs.ax = product as u16;
            self.state.regs.dx = (product >> 16) as u16;
            let high = (product >> 16) as u16;
            let overflow = if signed {
                high != if product & 0x8000 != 0 {
                    0xffff
                } else {
                    0x0000
                }
            } else {
                high != 0
            };
            self.mul_flags(high, overflow);
        }
    }

    fn divide(&mut self, f: &Fields) {
        let insn = f.insn;
        let signed = insn.op == Op::IDIV;
        // A `REP` prefix in front of `IDIV` inverts the sign of the quotient
        // on this part. It is not a documented feature and not useful, but it
        // is deterministic, the corpus exercises it deliberately, and software
        // that stumbles on it deserves to be emulated correctly.
        let negate = signed && f.rep.is_some();
        let bits = if Self::width(f) == 1 { 8 } else { 16 };

        let (source, dividend) = if bits == 8 {
            (
                u32::from(self.read_arg8(f, insn.dst)),
                u32::from(self.state.regs.ax),
            )
        } else {
            let src = u32::from(self.read_arg16(f, insn.dst));
            let dividend = (u32::from(self.state.regs.dx) << 16) | u32::from(self.state.regs.ax);
            (src, dividend)
        };

        // The hardware divides magnitudes and applies the signs afterwards;
        // for an unsigned divide the magnitudes are the operands.
        let (magnitude, divisor_magnitude) = if signed {
            (
                Self::sign_extend(dividend, bits * 2).unsigned_abs(),
                Self::sign_extend(source, bits).unsigned_abs(),
            )
        } else {
            (u64::from(dividend), u64::from(source))
        };

        // Run the loop even when the result will not fit: the flags it leaves
        // are visible either way, because a divide error pushes them.
        let (quotient_magnitude, remainder_magnitude) =
            self.cord(magnitude, divisor_magnitude, bits);

        // Whether the result is representable is decided separately: the loop
        // produces exactly `bits` bits of quotient whatever the true one would
        // have been, so it cannot report an overflow itself.
        let mask = (1u64 << bits) - 1;
        let (quotient, remainder, fault) = if divisor_magnitude == 0 {
            (0, 0, true)
        } else if signed {
            let n = Self::sign_extend(dividend, bits * 2);
            let d = Self::sign_extend(source, bits);
            let mut q = n / d;
            if negate {
                q = q.wrapping_neg();
            }
            let limit = 1i64 << (bits - 1);
            (
                (q as u64) & mask,
                ((n % d) as u64) & mask,
                !(-limit..limit).contains(&q),
            )
        } else {
            let q = u64::from(dividend) / divisor_magnitude;
            // The loop and the host's divide have to agree whenever the result
            // fits; if they ever do not, the loop is the bug.
            debug_assert!(q > mask || q == quotient_magnitude);
            (
                quotient_magnitude & mask,
                remainder_magnitude & mask,
                q > mask,
            )
        };
        if fault {
            self.divide_error();
            return;
        }

        // Measured, and exact on every corpus vector that completes: the carry
        // comes out as the complement of the quotient's top bit.
        self.set_flag(flags::CF, quotient & (1 << (bits - 1)) == 0);

        if bits == 8 {
            self.state.regs.ax = ((quotient as u16) & 0xff) | (((remainder as u16) & 0xff) << 8);
        } else {
            self.state.regs.ax = quotient as u16;
            self.state.regs.dx = remainder as u16;
        }
    }

    /// Sign-extend the low `bits` of `value`.
    const fn sign_extend(value: u32, bits: u32) -> i64 {
        let shift = 64 - bits;
        ((value as i64) << shift) >> shift
    }

    /// The restoring-division loop an 8086 runs in microcode, and the flag
    /// residue it leaves.
    ///
    /// Intel describes `DIV` as a sequence of shifts and conditional
    /// subtractions, and that is what this is: the running remainder is
    /// shifted up one bit at a time, the divisor is subtracted on trial, and
    /// the quotient bit is the complement of the borrow. Running the loop
    /// rather than calling the host's divide is what brings the officially
    /// undefined flag results near the hardware's — what an 8088 leaves behind
    /// is the last trial subtraction's flags. "Near", not "equal":
    /// `conformance::KNOWN_FAILURES` records what is still missing.
    fn cord(&mut self, dividend: u64, divisor: u64, bits: u32) -> (u64, u64) {
        let mask = (1u64 << bits) - 1;
        let top = 1u64 << (bits - 1);
        let mut remainder = (dividend >> bits) & mask;
        let mut quotient = dividend & mask;
        for _ in 0..bits {
            let carried = (quotient >> (bits - 1)) & 1;
            quotient = (quotient << 1) & mask;
            let overflowed = remainder & top != 0;
            let shifted = ((remainder << 1) | carried) & mask;
            let difference = shifted.wrapping_sub(divisor) & mask;
            let borrow = shifted < divisor;
            self.set_flag(flags::CF, borrow);
            self.set_flag(flags::AF, (shifted ^ divisor ^ difference) & 0x10 != 0);
            self.set_flag(
                flags::OF,
                (shifted ^ divisor) & (shifted ^ difference) & top != 0,
            );
            self.set_flag(flags::SF, difference & top != 0);
            self.set_flag(flags::ZF, difference == 0);
            self.set_flag(flags::PF, Self::parity(difference as u8));
            if overflowed || !borrow {
                remainder = difference;
                quotient |= 1;
            } else {
                remainder = shifted;
            }
        }
        (quotient, remainder)
    }

    /// Take a type-0 interrupt.
    ///
    /// On an 8088 the return address pushed is the address of the *next*
    /// instruction, not of the faulting one — a detail later parts changed and
    /// generic x86 emulators habitually get wrong. `IP` has already been
    /// advanced past the instruction by the time this runs, so pushing it is
    /// exactly right.
    fn divide_error(&mut self) {
        self.service(VEC_DIVIDE);
    }

    // -----------------------------------------------------------------
    // The decimal adjustments
    // -----------------------------------------------------------------

    fn aam(&mut self, f: &Fields) {
        let base = f.imm as u8;
        if base == 0 {
            // The microcode enters the divide, produces nothing, and faults.
            // What it leaves behind is the flag set of a zero result, on all
            // 47 corpus vectors that reach it.
            self.set_flag(flags::CF | flags::OF | flags::AF, false);
            self.set_szp8(0);
            self.divide_error();
            return;
        }
        let al = self.state.regs.ax as u8;
        let quotient = al / base;
        let remainder = al % base;
        self.state.regs.ax = u16::from(remainder) | (u16::from(quotient) << 8);
        self.set_szp8(remainder);
        self.set_flag(flags::CF | flags::OF | flags::AF, false);
    }

    fn aad(&mut self, f: &Fields) {
        let base = f.imm as u8;
        let al = self.state.regs.ax as u8;
        let ah = (self.state.regs.ax >> 8) as u8;
        let product = ah.wrapping_mul(base);
        // The adjustment really is an addition, so it sets carry and the
        // auxiliary carry even though Intel calls them undefined.
        let r = self.add8(product, al, false);
        self.state.regs.ax = u16::from(r);
    }

    fn daa(&mut self) {
        self.decimal_adjust(false);
    }

    fn das(&mut self) {
        self.decimal_adjust(true);
    }

    /// `DAA` and `DAS`, which differ only in the sign of the correction.
    ///
    /// Two things here are not what Intel's later pseudocode says, and both
    /// are measured on all 20 000 corpus vectors:
    ///
    /// - The two corrections are **one** arithmetic operation, not two. The
    ///   value added or subtracted is `0x00`, `0x06`, `0x60` or `0x66`, and
    ///   the officially undefined overflow flag is that single operation's.
    /// - The high correction's threshold **moves with the auxiliary carry**:
    ///   `AL > 0x9f` when `AF` is set, `AL > 0x99` when it is not. So `daa` on
    ///   `AL = 0x9a` with `AF` set corrects only the low digit, where the
    ///   published algorithm would correct both. Sixty-odd vectors per opcode
    ///   turn on it.
    fn decimal_adjust(&mut self, subtract: bool) {
        let al = self.state.regs.ax as u8;
        let auxiliary = self.flag(flags::AF);
        let low = (al & 0x0f) > 9 || auxiliary;
        let threshold = if auxiliary { 0x9f } else { 0x99 };
        let high = self.flag(flags::CF) || al > threshold;
        let correction = if low { 0x06 } else { 0x00 } + if high { 0x60 } else { 0x00 };
        let adjusted = if subtract {
            self.sub8(al, correction, false)
        } else {
            self.add8(al, correction, false)
        };
        self.state.regs.set_byte(0, adjusted);
        // The carry and the auxiliary carry are the *conditions*, not the
        // arithmetic's — a correction that happens not to carry still sets
        // them.
        self.set_flag(flags::CF, high);
        self.set_flag(flags::AF, low);
    }

    fn aaa(&mut self) {
        self.ascii_adjust(false);
    }

    fn aas(&mut self) {
        self.ascii_adjust(true);
    }

    /// `AAA` and `AAS`, which likewise differ only in sign.
    ///
    /// Intel calls the sign, zero, parity and overflow results undefined here.
    /// They are not: they are the flags of the 8-bit `AL ± 6`, and that
    /// operation happens **whether or not the adjustment is needed** — with an
    /// operand of zero when it is not, which is why an unadjusted `AAA` leaves
    /// the sign, zero and parity of the original `AL`. Measured on all 20 000
    /// corpus vectors.
    fn ascii_adjust(&mut self, subtract: bool) {
        let al = self.state.regs.ax as u8;
        let adjust = (al & 0x0f) > 9 || self.flag(flags::AF);
        let operand = if adjust { 6 } else { 0 };
        let adjusted = if subtract {
            self.sub8(al, operand, false)
        } else {
            self.add8(al, operand, false)
        };
        let ah = (self.state.regs.ax >> 8) as u8;
        let ah = match (adjust, subtract) {
            (true, false) => ah.wrapping_add(1),
            (true, true) => ah.wrapping_sub(1),
            (false, _) => ah,
        };
        // Only the low digit survives; the carry and auxiliary carry report
        // whether a digit was carried out of it.
        self.state.regs.ax = (u16::from(ah) << 8) | u16::from(adjusted & 0x0f);
        self.set_flag(flags::CF | flags::AF, adjust);
    }

    // -----------------------------------------------------------------
    // String operations
    // -----------------------------------------------------------------

    fn string(&mut self, f: &Fields) {
        let op = f.insn.op;
        let width = u16::from(Self::width(f));
        let delta = if self.flag(flags::DF) {
            width.wrapping_neg()
        } else {
            width
        };
        let Some(rep) = f.rep else {
            self.string_step(f, delta);
            return;
        };
        // `REP` with `CX == 0` does nothing at all — not even one iteration.
        while self.state.regs.cx != 0 {
            self.string_step(f, delta);
            self.state.regs.cx = self.state.regs.cx.wrapping_sub(1);
            if matches!(op, Op::CMPSB | Op::CMPSW | Op::SCASB | Op::SCASW) {
                let zf = self.flag(flags::ZF);
                let stop = match rep {
                    Rep::While => !zf,
                    Rep::WhileNot => zf,
                };
                if stop {
                    break;
                }
            }
            if self.state.regs.cx == 0 {
                break;
            }
            self.charge(op.clocks());
            // A repeat is interruptible between iterations: the 8086 backs the
            // instruction pointer up to the prefix and re-enters. Modelled as
            // a clean restart; the erratum where an 8086 forgets all but the
            // last prefix on resume is not.
            if self.lines.nmi_pending() || (self.flag(flags::IF) && self.lines.intr_pending()) {
                self.state.regs.ip = self.start_ip;
                self.state.queue.flush();
                return;
            }
        }
    }

    fn string_step(&mut self, f: &Fields, delta: u16) {
        let op = f.insn.op;
        // The source of a string move is overridable; its destination is
        // always `ES:DI`, and no prefix can change that.
        let src_seg = f.segment(seg::DS);
        let regs = self.state.regs;
        match op {
            Op::MOVSB => {
                let value = self.read8(src_seg, regs.si);
                self.write8(seg::ES, regs.di, value);
                self.advance_si_di(delta, true, true);
            }
            Op::MOVSW => {
                let value = self.read16(src_seg, regs.si);
                self.write16(seg::ES, regs.di, value);
                self.advance_si_di(delta, true, true);
            }
            Op::CMPSB => {
                let a = self.read8(src_seg, regs.si);
                let b = self.read8(seg::ES, regs.di);
                self.sub8(a, b, false);
                self.advance_si_di(delta, true, true);
            }
            Op::CMPSW => {
                let a = self.read16(src_seg, regs.si);
                let b = self.read16(seg::ES, regs.di);
                self.sub16(a, b, false);
                self.advance_si_di(delta, true, true);
            }
            Op::STOSB => {
                let value = regs.ax as u8;
                self.write8(seg::ES, regs.di, value);
                self.advance_si_di(delta, false, true);
            }
            Op::STOSW => {
                let value = regs.ax;
                self.write16(seg::ES, regs.di, value);
                self.advance_si_di(delta, false, true);
            }
            Op::LODSB => {
                let value = self.read8(src_seg, regs.si);
                self.state.regs.set_byte(0, value);
                self.advance_si_di(delta, true, false);
            }
            Op::LODSW => {
                let value = self.read16(src_seg, regs.si);
                self.state.regs.ax = value;
                self.advance_si_di(delta, true, false);
            }
            Op::SCASB => {
                let b = self.read8(seg::ES, regs.di);
                let a = self.state.regs.ax as u8;
                self.sub8(a, b, false);
                self.advance_si_di(delta, false, true);
            }
            _ => {
                let b = self.read16(seg::ES, regs.di);
                let a = self.state.regs.ax;
                self.sub16(a, b, false);
                self.advance_si_di(delta, false, true);
            }
        }
    }

    fn advance_si_di(&mut self, delta: u16, si: bool, di: bool) {
        if si {
            self.state.regs.si = self.state.regs.si.wrapping_add(delta);
        }
        if di {
            self.state.regs.di = self.state.regs.di.wrapping_add(delta);
        }
    }
}
