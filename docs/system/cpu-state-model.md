# The engine-independent architectural CPU-state model

Consumed by: `accel/state.rs`, `accel/cpu.rs`, `cpu/x86/mod.rs`'s `save`/`load`.
This is the layer that lets a snapshot taken while a guest ran on host silicon
be restored into an interpreter, and the reverse.

`ROADMAP.md` phase 7 names it as a deliverable rather than as something that
emerges:

> Snapshot compatibility across an engine switch also requires an
> **engine-independent architectural CPU-state model**: for x86-64 that is the
> full MSR set, the XSAVE area, LAPIC/x2APIC state, and the TSC offset.

and gates it on *"snapshots taken under KVM restore under the JIT and vice
versa"*. `tests/x86_arch_state.rs` is that gate.

## Sources

| Source | Covers | Access |
| --- | --- | --- |
| *Intel 64 and IA-32 Architectures Software Developer's Manual*, volume 3A | The system-register set: control registers, descriptor caches, the local APIC, `RESET` state (Table 9-1), the memory-type range registers (§12.11) | **[browser]** — free download |
| *Intel SDM* volume 4 | Table 2-2, the architectural model-specific registers and their addresses | Free |
| *AMD64 Architecture Programmer's Manual*, volume 2 | The `SYSCALL` register set (`STAR`, `LSTAR`, `CSTAR`, `SFMASK`), long mode | Free |
| `Documentation/virt/kvm/api.rst`, `include/uapi/linux/kvm.h` | The stable KVM ABI: which `ioctl` reports which register, and in what shape | Published ABI; **transcribed**, never included — see `src/accel/kvm.rs`'s own source note |

No emulator source was consulted for any of it. The rule in `CLAUDE.md` applies
here with unusual force, because a state-transfer table is exactly the kind of
thing it is tempting to copy: the *set* of registers a hypervisor reports is
hardware fact and is in the manuals, and somebody else's list of them is
expression.

## The idea

Two engines can execute a guest: an emulated core (`cpu::x86`, in either its
interpreted or its JIT-compiled form — they share one `X86` and one chunk) and a
vCPU (`accel::kvm`). Each has its own idea of what a processor's state is:

| | emulated | accelerated |
| --- | --- | --- |
| register file | `cpu::x86::Regs` | `struct kvm_regs` |
| system state | `cpu::x86::prot::Sys` | `struct kvm_sregs`, plus `KVM_GET_MSRS` |
| debug registers | `Sys::dr` | `struct kvm_debugregs` |
| floating point | `fpu::X87`, `fpu::Sse` | `struct kvm_fpu` |
| the counter | `X86::cycles` | `IA32_TSC` |
| the interrupt shadow | `State::int_shadow` | `struct kvm_vcpu_events` |
| `CR8` | *the local APIC device* | `kvm_run.cr8` |

The model is the **translation between them**, plus the claim that the
translation is total over the state a guest can observe. `accel::state` is that
translation for x86, and it is field-for-field rather than a re-derivation,
because both sides were written from the same SDM description of what a segment
register holds.

## What is architectural and what is not

The rule is `CLAUDE.md`'s: derived state is never serialized. Applied here it
draws a sharper line than it first looks like.

**Architectural, and therefore carried:** the register file; the six segment
registers *with their cached bases, limits and access rights*, because that
cache is what the silicon holds and what `LAR` reports; `GDTR`, `IDTR`, `LDTR`,
`TR`; `CR0`, `CR2`, `CR3`, `CR4`, `EFER`; the debug registers; the x87 and SSE
files; thirty-four model-specific registers; and the time-stamp counter's value.

**Not architectural, and therefore never in a chunk and never translated:** the
translation-lookaside buffer, the JIT's block cache, decoded tables, host
pointers into guest RAM, a hypervisor's memory slots, and a **TSC offset**.

That last one is the interesting case and it is why the roadmap's phrase "the
TSC offset" is, on inspection, the wrong thing to store.

## The time-stamp counter

A hypervisor does not keep a counter. It keeps an *offset*, and the guest's
`RDTSC` reads the host's own counter plus that offset. The offset is derived
from two things: the value the guest should see, and the host counter at the
instant of the write. It is therefore meaningless on any other host, and a
snapshot carrying one could only be restored on the machine that took it.

What is architectural is the **value** — what a `RDTSC` in the guest returns —
and both engines have exactly that. `KVM_GET_MSRS` of `IA32_TSC` on one side;
`X86::cycles` on the other, because that is where this core's `RDMSR` of
`msr::TSC` reads from. So the model stores the value, and the destination engine
recomputes whatever offset it needs. On KVM that recomputation is one
`KVM_SET_MSRS` of `IA32_TSC` and nothing else.

**When it is written matters as much as what.** `accel::state` has three calls
and the counter appears in exactly two:

| call | when | writes `IA32_TSC`? |
| --- | --- | --- |
| `store_from_vcpu` | after every hardware slice | reads it, always |
| `load_into_vcpu` | at the top of a slice whose shell is ahead | **no** |
| `restore_into_vcpu` | a snapshot load, and a reset | yes |

`load_into_vcpu` runs routinely — once per reset vector on `q35-linux`, and at
every restart sequence — and writing the counter there would rewind the guest's
clock by the userspace time between the store and the load, on *every* slice. A
guest that timed anything would see its counter stutter, which is a worse lie
than the one below. `accel::cpu` keeps a second flag, `tsc_dirty`, precisely so
that the two cases are distinguishable.

### What is not continuous, and cannot be made so

The **rate**. An accelerated guest's counter advances at `KVM_GET_TSC_KHZ`,
roughly the host's; the emulated core's advances at four ticks per bus cycle
plus the manual's execution figures. A guest that has calibrated its counter
against a periodic timer — Linux does, against the PIT or the HPET — and is then
restored under the other engine will find its calibration wrong, and will
recalibrate or mark the clocksource unstable.

No snapshot field fixes that, because a rate is a property of the machine rather
than of the guest. A board that wants an engine switch to be invisible has to
pin the two rates together, which is a machine-configuration deliverable and is
not built. What *is* guaranteed is monotonicity, which is what a guest's
timekeeping actually depends on, and `tests/x86_arch_state.rs` asserts it in
both directions.

## Interruptibility, and the interrupt shadow

Not every piece of architectural state is a register, and the interrupt shadow
is the one that caught this model out.

It is the single instruction after a `STI`, a `MOV SS` or a `POP SS` during
which an external interrupt is not recognised (*Intel SDM* volume 2B, `STI`;
volume 3A §6.8.3). It is why `STI; HLT` cannot lose a wakeup and why the second
half of an `SS:SP` reload cannot be interrupted. And it is **not in `RFLAGS`,
not in a control register and not in any model-specific register**: on hardware
it lives in the VMCS's interruptibility state or the VMCB's `INT_STATE`, and
`KVM_GET_VCPU_EVENTS` is the only request that reports it.

So a bridge assembled out of `KVM_GET_REGS`, `KVM_GET_SREGS`, `KVM_GET_MSRS`,
`KVM_GET_DEBUGREGS` and `KVM_GET_FPU` could not see it however carefully it was
written — which is exactly what happened. `cpu::x86` has had `int_shadow` in
its chunk since long before there was an accelerator; the field existed at both
ends and was never filled from hardware, so a snapshot taken between a `STI` and
the instruction after it restored with the shadow gone and the guest took an
interrupt one instruction early.

`cpu::x86` keeps one boolean where a hypervisor keeps two bits — blocking by
`STI` and blocking by `MOV SS`. Out of hardware, either bit means *shadowed*.
Into hardware, `accel::state` writes the `MOV SS` bit, for two reasons:

* It is the one that means what the interpreter means. Blocking by `STI`
  suppresses `INTR` and leaves `NMI` alone; blocking by `MOV SS` suppresses
  both, because the point of that shadow is that an `SS:SP` pair is half-loaded
  and nothing may be pushed onto it (SDM volume 3A §6.8.3). `cpu::x86` checks
  `int_shadow` ahead of *both* the `NMI` latch and the `INTR` pin, so its
  shadow is the `MOV SS` one whatever raised it.
* It is the one a VM entry will always accept. VMX checks the guest's
  interruptibility state on entry (SDM volume 3C, the checks on guest
  non-register state), and blocking by `STI` is not a state a processor can be
  in with `RFLAGS.IF` clear — which an interpreter shadow raised by a `MOV SS`
  with interrupts already off legitimately is.

The other three members of `kvm_vcpu_events` — a pending or injected exception,
a pending `NMI`, and the `NMI`-blocked flag — have no field in `cpu::x86` and do
not cross. A *load* therefore overlays rather than replaces: the flags word is
what tells KVM which members to take, so the bits for `nmi.pending`,
`sipi_vector` and the `smi` group are cleared and the kernel keeps its own. A
vCPU that owed the guest a `#PF` still owes it after a slice-boundary push.

## The local APIC

Phase 7's list says "LAPIC/x2APIC state", and there is no `KVM_GET_LAPIC`
anywhere in the backend. That is the design rather than a gap.

**rsemu never issues `KVM_CREATE_IRQCHIP`.** A vCPU here has no in-kernel local
APIC to read out; the board's local APIC is `dev::pc::apic`, an ordinary device
with an ordinary chunk, and both engines reach it the same way — a store to its
register page in the address space the machine file mapped it into. There is
consequently nothing to translate: the same object, saved by the same `save`,
restored by the same `load`, whichever engine was running the processor beside
it.

`CR8` is the same story told about one register. It is the top nibble of the
local APIC's task-priority register (SDM volume 3A §11.8.6.1), so `cpu::x86` has
no `cr8` field at all and `prot::Exec::write_task_priority` stores to the device
whichever route the guest took. There is one home for the byte and the snapshot
already carries it — in the APIC's chunk.

**On hardware there are two homes**, and neither sees the other: `MOV CR8`
lands in the vCPU without an exit, while a store to the APIC's register page
leaves hardware as MMIO and lands in the device. What that needs is not a field
in the transfer but for the two copies to agree, so it is not `accel::state`'s
job — it belongs to the party holding both the memory space and the
`LocalController` link, which is `accel::cpu`.

`AccelCpu::sync_task_priority` is that reconciliation. It keeps the class the
two sides last agreed on and compares each against it, so it can tell which of
them moved: a blind push of the vCPU's `CR8` would undo an MMIO write the device
had already acted on, and a blind pull the other way would discard a `MOV CR8`.
The device wins a tie, because its write is the one already observed — a
`pc.lapic` re-evaluates delivery on a TPR write and may already have dropped
`INTR`. The shadow itself is derived and is **not** in any chunk; a reset or a
snapshot load lands in `adopt_task_priority`, which takes the device's value
outright.

The vCPU's side of it is `kvm_run.cr8`, which `api.rst` documents as *"in
(pre_kvm_run), out (post_kvm_run)"* and valid *"only if in-kernel local APIC is
not used"* — permanently true here. So the whole sync costs no `ioctl`: the
kernel writes the field on every exit and reads it on every entry.

**The re-entrancy question**, which is why this was deferred once. `CLAUDE.md`
asks that a device mutate its own state in a short critical section, release it,
and only then make an outward call. The store into the APIC's register page is
an outward call from inside a run loop, and it is made holding **no lock of
`AccelCpu`'s**: the controller link is cloned out from under its mutex and
dropped, the agreed value is an atomic, and the vCPU write takes only the vCPU's
own lock. It is the same position — and the same lock depth — from which
`poll_controller` already asks the controller for a Start-Up. The APIC is free
to drive this processor's `INTR` pin back while it is answering, which is the
point rather than a hazard: lowering the priority class is how a blocked
interrupt becomes deliverable. The granularity is once per slice, which is
already what an accelerated processor has for everything else its board can see.

An in-kernel irqchip would make the APIC's state a third representation to keep
in step, would need its own `KVM_GET_LAPIC`/`KVM_SET_LAPIC` translation, and
would move interrupt delivery out of the board where `machines/pc-apic.machine`
puts it.

## The model-specific registers

Thirty-four, generated from the SDM volume 4 Table 2-2 addresses rather than
typed out — thirty-four hand-written hexadecimal constants is thirty-four
chances to transpose one, and a transposed memory-type range is invisible in any
test that uses a uniform state.

| group | count | why it is here |
| --- | --- | --- |
| `STAR`, `LSTAR`, `CSTAR`, `SFMASK`, `KERNEL_GS_BASE` | 5 | a 64-bit guest that loses these has a `SYSCALL` that jumps to zero |
| `IA32_MISC_ENABLE` | 1 | Linux clears the execute-disable lock before it looks for `NX`, and firmware sets it |
| `IA32_MTRR_DEF_TYPE` | 1 | `E = 0` is not "the default"; SDM volume 3A §12.11.2.1 makes it *every physical address uncacheable* |
| the fixed memory-type ranges | 11 | firmware programs them, and a guest reads them back |
| the variable ranges, base and mask | 16 | as above |

`EFER` is not in the list because it *is* in `kvm_sregs`, and `FS_BASE`/`GS_BASE`
are not because KVM reports them as the `fs` and `gs` segment bases — which is
where the hardware keeps them, and carrying them twice would let the two copies
disagree.

`IA32_MISC_ENABLE` is the one register **merged rather than replaced** on the
way out of hardware, and the reason is that not every bit of it is guest state:
bits 11 and 12 say *this part has no branch-trace store and no precise
event-based sampling*, a host kernel reports `0x1800` there and rsemu's core
reports `0x1`, and they are describing two different processors. What a guest
can have changed is exactly `misc_enable::WRITABLE` — a `WRMSR` to any other bit
raises `#GP(0)` on this core — so those bits transfer and the rest stay the
part's.

## Chunk versioning

The `cpu.x86` chunk did **not** change for any of the above, and that is a
finding rather than an omission.

The defect the model was built to fix was not a missing field. The interpreter's
chunk already wrote `mtrr_def_type`, `mtrr_fix`, `mtrr_var`, `misc_enable` and
`cycles`, and had since before there was an accelerator. What was missing was
the *bridge*: `accel::state::CARRIED_MSRS` listed five registers, so a vCPU with
its memory-type ranges programmed handed the shell zeros, and those zeros were
written to the chunk faithfully. Fixing a bridge changes no bytes, so nothing
bumps and no snapshot an earlier build wrote is orphaned.

`tests/x86_arch_state.rs` pins that claim with an assertion on
`cpu::x86::CLASS.version`, so the next person to widen the model finds out there
whether they have crossed the line into a version bump. `src/machine/migrate.rs`
is what they have to do when they have: bump the class version **and** register
the `vN -> vN+1` step in the same commit, and add it to `default_migrations`.

The `XSAVE` work below is the case that will cross it: an AVX register file is
new chunk bytes.

## What is not carried yet

`src/accel/state.rs`'s module documentation is the maintained list and says what
each one would take. In summary:

| gap | what closing it takes |
| --- | --- |
| `MXCSR` into hardware | `KVM_SET_XSAVE` in place of `KVM_SET_FPU`; the kernel's `KVM_SET_FPU` path does not write that field |
| the x87 last-instruction selectors | the same — `kvm_fpu` has no room for `CS`/`DS`, the `XSAVE` legacy area does |
| `XSAVE` beyond x87 and SSE | an AVX register file in `cpu::x86::fpu` first (there is nothing for a `YMM` half to land in), then `XCR0` via `KVM_GET_XCRS`, then the standard format's component walk at `CPUID.(EAX=0Dh)` offsets, then a chunk change and a version bump |
| the rest of `kvm_vcpu_events` | a pending or injected exception, a pending `NMI`, and the `NMI`-blocked flag. Not an ioctl: `cpu::x86` has no field for any of them — it delivers an exception within the step that raised it, keeps its `NMI` latch on the pin, and does not model `IRET`-scoped `NMI` blocking at all. A pending-event field in the execution state and a chunk version to hold it, which is a core deliverable |
| `IA32_APIC_BASE` | a holder: `LocalController::set_base_register` is the route and `store_from_vcpu` has no third party to hand the value to |
| `SYSENTER` MSRs | `cpu::x86` does not model them at all — no field in `Sys`, `#GP(0)` from `RDMSR`. An ISA deliverable before it is a snapshot one |

## Testing

`tests/x86_arch_state.rs` is the gate, and its shape is worth copying for any
future engine: **the guest is the witness.** Comparing rsemu's internal fields
across a restore only proves that two copies of this crate agree with each
other. So the guest program programs its registers once and then loops reading
them back with `RDMSR` and writing what it sees into RAM; the test erases that
witness area *after* the restore and before running on, so every value it then
reads was produced by an instruction the destination engine executed.

That distinction is not academic. `tests/kvm_smp.rs` already asserted
byte-identical chunks across an engine switch and passed throughout — both
boards wrote the same wrong bytes.

Its board no longer has that excuse. `machines/pc-apic.machine` used to declare
two plain 80486s, on the strength of a comment saying the validator's schema did
not list `cpu.x86`'s feature toggles — which had stopped being true when that
schema started being generated from `CLASS.properties` instead of written out a
second time. The only accelerated board in the tree therefore had no
model-specific registers to lose, which is why a passing byte-for-byte test said
nothing. It now declares `msr = true` and `mtrr = true`.

### Witnessing something that is not a register

The interrupt shadow needed a second witness, because there is no `RDMSR` for
it. `tests/x86_arch_state.rs`'s `shadow_guest` is that program, and the shape is
worth reading before writing another:

```text
loop:
    inc dword [rbp+TICK]
    mov al, 0x5a
    cli
    sti
    out 0x80, al          ; <- the instruction the STI shadows
    jmp loop

handler:                  ; IDT vector 0x20
    mov eax, [rsp]        ; the RIP the interrupt interrupted
    mov [rbp+SEEN_RIP], eax
```

The handler reads the interrupted `RIP` off its own stack frame, so the guest
itself reports whether the shadowed instruction ran before the interrupt or not.
`SEEN_RIP` pointing at the `OUT` means the shadow was dropped in the transfer;
pointing past it means it survived. The `INTR` pin is driven high only *after*
the restore, so the delivery under test is the destination engine's.

The `OUT` is not decoration. A hypervisor's guest is only observable at an exit,
and the snapshot has to be taken *inside* the shadow — measured on an AMD host:
with `CLI; STI; OUT` in a loop, every `KVM_RUN` comes back with `RIP` on the
`OUT` and `interrupt.shadow` still set, because a port write leaves hardware
before it retires. Put any other instruction between the `STI` and the exit and
the shadow is already spent by the time userspace can look.
