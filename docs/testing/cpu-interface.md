# What the 6502 core must expose

The conformance harness was written before the core existed, against the
smallest interface that can drive all three suites. This page is the contract in
prose; [`tests/conformance/cpu.rs`](../../tests/conformance/cpu.rs) is the same
thing in Rust, and its `adapter` module is where `rsemu::cpu::mos6502` is bound
to it.

Four methods, one of them optional. Nothing a runner can work out for itself —
elapsed cycles, instruction bytes, disassembly text — is in the trait.

## The interface

```rust
/// One bus access. Exactly one per CPU cycle, no exceptions.
pub trait Bus6502 {
    fn read(&mut self, addr: u16) -> u8;
    fn write(&mut self, addr: u16, value: u8);
}

/// The architectural register file.
pub struct Regs {
    pub pc: u16,
    pub s:  u8,   // the low byte of $01xx
    pub a:  u8,
    pub x:  u8,
    pub y:  u8,
    pub p:  u8,   // NV1BDIZC
}

pub trait Cpu6502: Send {
    /// Overwrite the architectural state and discard all microarchitectural state.
    fn set_regs(&mut self, regs: Regs);

    /// Read the architectural state back.
    fn regs(&self) -> Regs;

    /// Execute exactly one instruction, driving `bus` once per cycle in cycle
    /// order. Returns the number of cycles consumed.
    fn step(&mut self, bus: &mut dyn Bus6502) -> u32;

    /// Optional; used only by `nestest` in strict mode.
    fn disassemble(&self, pc: u16, bytes: &[u8]) -> Option<String> { None }
}

/// The seam. Returns `None` today; return a core and every suite starts running.
pub fn new_cpu(variant: Variant) -> Option<Box<dyn Cpu6502>>;

pub enum Variant { Nmos6502, Ricoh2A03 }
```

## What each one has to mean

### `Bus6502` — one call per cycle, always

A 6502 has no idle cycles. Every cycle is a read or a write, including the dummy
reads that page crossings and read-modify-write instructions perform, and the
dummy write that RMW instructions perform before the real one
([`../cpu/6502.md`](../cpu/6502.md)). SingleStepTests checks that access by
access, so a core that computes a result and then does its memory traffic in a
batch cannot pass no matter how correct the result is. This is the single
constraint most likely to force a rewrite if it is discovered late — the cycle
structure has to be in the design from the first opcode.

Both methods take `&mut self` on the bus, so a core must not hold a borrow of it
across a cycle.

### `set_regs` — clear the microarchitecture too

Not just the six registers: any half-executed instruction, any latched or
pending interrupt, any prefetched operand, any "the previous instruction was a
branch" flag. After the call the core must behave exactly as if it had been in
this state at an instruction boundary with nothing pending.

The failure this causes is nasty: a handful of vectors per opcode fail with no
pattern, because the residue depends on whichever vector ran before. If a run
shows a small, scattered, non-reproducible failure set, suspect this first.

### `step` — one instruction, and tell the truth about it

Execute exactly one instruction and return the cycle count. The runner asserts
that the returned count equals the number of bus calls made, so a post-hoc
lookup table that says 5 while the core makes 6 accesses is caught immediately
rather than passing on a technicality. `CLAUDE.md` (CPU cores) requires
per-access cycle accounting anyway; this just makes it checkable.

Long-running or looping instructions are bounded: after 64 bus accesses in one
`step` the harness aborts that vector and reports a cycle overrun. A panic is
also caught and reported as a failed vector with the panic message, so one
unimplemented opcode does not take down the other 255.

### `regs` — the `P` byte, exactly as hardware presents it

Bit 5 reads as 1 on real silicon and the corpus encodes it that way. A core that
stores it as 0 fails every single vector for a reason that has nothing to do
with the instruction under test. Bit 4 (`B`) is not a real flip-flop — it exists
only in the byte pushed to the stack, set by `PHP` and `BRK`, clear for `IRQ`
and `NMI` — and AccuracyCoin's "The B Flag" test has nine separate error codes
for getting that wrong.

### `disassemble` — optional, and only for strict nestest

`nestest.log`'s disassembly column is one emulator's formatting convention
(`STX $00 = 00`, `LDA ($33),Y @ 0400 = 5B`), not something the hardware
specifies. It is compared only under `RSEMU_NESTEST_DISASM=1`. `ROADMAP.md` §6
requires the disassembler to be generated from the same declarative table as the
decoder, so it should exist; returning `None` just means this runner will not
check its text.

### `Variant` — pick the right corpus

`Ricoh2A03` is the NES part: decimal mode disabled in `ADC`/`SBC`, while the `D`
flag itself still sets, clears and pushes normally. `Nmos6502` is the original
with working BCD. The corpus is split the same way (`nes6502/` and `6502/`), and
choosing the wrong one produces a confident, uniform, entirely wrong failure
across the arithmetic opcodes. `docs/cpu/6502.md` says decimal mode must be a
construction property rather than a `#[cfg]`, which is exactly what makes both
runnable from one core.

### `Send`

The vector runner shards the corpus across threads, one core per thread. Nothing
is shared between them, so `Send` is enough — `Sync` is not required.

## Wiring it up

`new_cpu` is bound to `rsemu::cpu::mos6502` under `#[cfg(feature =
"cpu-mos6502")]`; the adapter is the `adapter` module in the same file.

The adapter stays in the test tree on purpose. The shape the corpus wants — a
`&mut dyn Bus6502` and a whole instruction per call — is a *testing* shape, not
the shape the scheduler drives the core with. Putting it in `src/` would be
letting the tests design the core, which is how a core ends up with an API that
only its tests use.

Bridging the two shapes is not four forwarding calls, and the reason is a
lifetime. The core reaches memory through an `AddressSpace` built from
`Arc<dyn MemOps>` — shared, `Send + Sync`, `'static` — while the runner hands
over a `&mut dyn Bus6502` that lives for one `step`. There is no safe way to
put that borrow inside the `'static` `Arc`, and `unsafe` is not available: the
six sanctioned sites are listed in `CLAUDE.md` and a test harness is not one of
them. So the borrow stays where it is and the *core* moves — it runs on its own
thread, and every access it makes becomes a message the calling thread services
against the borrowed bus. One channel round trip per read; writes ride the
channel's ordering and need no reply.

Two consequences worth knowing:

* **`set_regs` really does have to discard microarchitectural state.** One core
  serves all 10 000 vectors of an opcode file, so the `JAM` in the first vector
  of `02.json` freezes it for the other 9 999 unless the adapter clears the
  halt. A jammed 6502 is cleared by exactly one thing, and it is not a method
  call, so the adapter runs a real reset sequence with the proxy detached.
* **The power-on reset is consumed at start-up**, the same way, so the core is
  at a genuine instruction boundary with nothing pending before the first
  vector — which is what `set_regs` promises and what the corpus assumes.

## Skips that are allowed to be skips

A suite may skip because the gate is closed, because the corpus was not
fetched, or because this build does not include the component. It may **not**
skip because the harness was never wired up: `cpu::require_cpu` and
`machine::require_nes` assert in that case, and
`every_seam_this_build_can_bind_is_bound` fails on a plain `cargo test
--all-features` without any corpus at all. `nestest` spent months reporting
`SKIP … no 6502 core is bound in tests/conformance/cpu.rs` — green, and
measuring nothing — which is the failure mode that rule exists to make
impossible.

## Checking your work before the real core exists

[`tests/conformance/mock.rs`](../../tests/conformance/mock.rs) has a three-opcode
stand-in (`LDA #`, `NOP`, `STA abs`) that implements this trait, plus five
deliberately broken variants — one per check the runner makes. It passes all
30 000 real upstream vectors for its three opcodes, which is how the harness
demonstrates it works without a 6502 in the tree. If you want to see what a
failure report looks like before writing any code, point `new_cpu` at
`MockCpu::default()` and run a fourth opcode.
