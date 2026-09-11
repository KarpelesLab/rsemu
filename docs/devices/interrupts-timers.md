# Interrupt controllers, timers, and clocks

Consumed by: `dev/intc/*`, `dev/timer/*`. In rsemu these are
ordinary devices with wire sinks and sources — the core knows nothing about
"interrupts" (`ROADMAP.md` §4.3).

## PC devices

| Device | Source |
| --- | --- |
| 8259A PIC | Intel 8259A datasheet ([bitsavers](https://bitsavers.org/)); [OSDev: 8259 PIC](https://wiki.osdev.org/8259_PIC) |
| Local APIC / IOAPIC | Intel SDM Volume 3 (the authoritative source), plus the 82093AA IOAPIC datasheet; [OSDev: APIC](https://wiki.osdev.org/APIC) |
| 8254 PIT | Intel 8253/8254 datasheet; [OSDev: PIT](https://wiki.osdev.org/Programmable_Interval_Timer) |
| MC146818 RTC / CMOS | Motorola MC146818 datasheet; [OSDev: CMOS](https://wiki.osdev.org/CMOS) |
| HPET | Intel *IA-PC HPET Specification*; [OSDev: HPET](https://wiki.osdev.org/HPET) |

## Non-PC

| Device | Source |
| --- | --- |
| RISC-V CLINT | Privileged spec — `mtime` / `mtimecmp` |
| RISC-V PLIC | [RISC-V PLIC specification](https://github.com/riscv/riscv-plic-spec) |
| ARM GIC v2/v3/v4 | Arm IHI 0069 **[browser]** |
| STM32 TIM (`st.tim`) | ST RM0090 §16 (advanced-control TIM1/TIM8), §17 (general-purpose TIM2–TIM5), §18 (general-purpose TIM9–TIM14), §19 (basic TIM6/TIM7); RM0351 §31–§34 is the same block on an L4. `CK_INT` comes from §7.2, and on an F4 it is **twice** the APB clock whenever that bus has a prescaler — a board's `clock =` expression says which, never the device |
| STM32 EXTI + SYSCFG | ST RM0090 §9 and §12 (F4), RM0351 §9 and §13 (L4) |
| STM32 IWDG / WWDG | ST RM0090 §20 and §21, RM0351 §35 and §36 |
| NES / Game Boy interrupt lines | The platform documentation — these machines have wires, not controllers |

## Implementation notes

- **Level versus edge is a device property, not a flag on the wire.** Model the
  edge detector as a device so that it snapshots correctly. `st.exti` is the
  worked example: it keeps the last level it saw for each line *in its
  snapshot*, unlike `st.gpio`, which deliberately leaves pad levels out. The
  difference is that a pad level is only ever reported, while an edge detector
  *acts* on a remembered one — a restored EXTI that believed every line low
  would post an interrupt the first time a line that had been high was
  re-driven, for an edge that never happened.
- **Where the vector number lives is a board question.** A peripheral drives an
  output pin and the machine file says where it lands. `st.exti` has one `irq`
  pin per line for exactly that reason: on an STM32 lines 5–9 share IRQ 23 and
  10–15 share IRQ 40, which is a fact about the part, so five pins are wired to
  one core input and the wire's fan-in resolves them.
- **A watchdog is a scheduler problem, not a bus one.** An IWDG counts its own
  low-speed oscillator: it keeps counting when the PLL is reprogrammed, when the
  core halts, and when the APB clock stops, so it cannot be a counter something
  decrements from a register access. `st.iwdg` and `st.wwdg` are lazily-advanced
  devices that publish the tick their counter next does something at.
- **A device that resets the machine must let go of its own lock first.** A
  reset reaches every device on the machine, including the one that asked for
  it, so both watchdogs decide inside a short critical section, release the
  state lock, and only then pulse their `reset` pin. Driving it from inside
  would deadlock against the device's own `Device::reset`.
- **Which timer feeds which is a board question too.** A TIM's slave-mode
  controller selects one of four internal triggers with `SMCR.TS`, and what
  `ITR0`–`ITR3` are wired to is fixed per part — RM0090's "TIMx internal trigger
  connection" tables. `st.tim` therefore publishes a `trgo` output and four
  `itrN` inputs and knows nothing about the matrix; `machines/stm32f407.machine`
  carries the twenty-four lines that make an F4's six chainable timers chainable.
  The same argument as the vector number above, and the same one `st.dma`'s
  request matrix uses.
- **A trigger output is a pulse, and a pulse has to be counted, not flagged.**
  `CR2.MMS` has three pulse modes and four level modes. A lazily-advanced timer
  can cross several update events inside one `advance_to`, so the pending pulses
  are a *count* in its state: a slave in external clock mode 1 counts edges, and
  a boolean "there is a pulse waiting" silently loses periods whenever something
  syncs the device across more than one. The DMA request lines are the opposite
  case and are a set rather than a count, because `st.dma` latches a request in
  one boolean per unit — extra pulses could not buy extra beats anyway.
- Timers are the most common source of guest-visible timing bugs. Every timer
  registers events on its clock domain; none of them reads the host clock. A
  timer that "catches up" by sampling wall time will break record/replay
  silently.
- The 8259's cascade wiring, the IOAPIC's redirection table, and the APIC's
  priority/EOI handling are each small state machines worth unit-testing
  independently of any guest.
- These devices are also where `MemAttrs::debug` earns its keep — a monitor
  reading an interrupt controller's status register must not acknowledge an
  interrupt.
- **A shadowed register is the classic timer bug.** An STM32 TIM buffers `PSC`
  always, `ARR` under `CR1.ARPE` and `CCRx` under `CCMRx.OCxPE`, so a write is
  visible in the preload register immediately and in the counter's behaviour
  only at the next update event. A model that writes straight through looks
  correct until firmware changes a PWM duty cycle mid-period. Test it directly:
  write the register mid-period, assert the *old* value still governs, then
  assert the new one takes over at the wrap.
- A lazily advanced timer (`Device::is_lazy`) must publish `next_event_tick` as
  the exact tick of its next observable change — the next overflow, underflow or
  compare match — and compute the span between two of them in closed form. A
  per-tick loop is fine when the events are hundreds of ticks apart, as on a
  Game Boy; it is not when a prescaler of 65535 puts them four billion apart.
