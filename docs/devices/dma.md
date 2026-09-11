# DMA engines, and what a requesting peripheral has to do

Consumed by: `dev/pc/dma.rs` (8237A), `dev/stm32/dma.rs` (`st.dma`), and every
peripheral that wants to be fed by one.

A DMA controller is the one kind of device that is also a **bus master**: it
issues its own reads and writes into an address space, and those reach other
devices. Two rules in `CLAUDE.md` bear on that directly — the re-entrancy
contract and the ranked lock order — and a controller that gets either wrong
deadlocks in release and panics in debug. This page is the shared part.

| Device | Source |
| --- | --- |
| Intel 8237A | *Intel 8237A High Performance Programmable DMA Controller* data sheet; *IBM PC AT Technical Reference* for the page latches |
| STM32 stream controller (F2/F4/F7) | ST **RM0090** §10 |
| STM32 channel controller (F0/F1/F3/L0/L1/L4/G0/G4/WB) | ST **RM0351** §11, Table 41 for the data-width conversion |
| STM32 DMAMUX (L4+/G4/H7/WB) | ST **RM0432** §14 — *not implemented*; `st.dmamux` does not exist |

## The three seams a controller sits on

1. **The address space it masters.** Its object declares `space = …` and the
   machine layer hands it the space plus a [`RequesterId`] during `bind`. This
   is also how a board expresses a bus matrix: a master that must not reach a
   region is simply given a space in which that region is not mapped. An F4's
   CCM is the canonical case — the core reaches it and DMA does not.
2. **The request line.** A wire from the peripheral into the controller.
3. **The completion interrupt.** A wire out, one per channel or stream.

## Two shapes of data path, and which one to copy

The 8237 and the STM32 controllers differ in a way that is not a modelling
choice but a fact about the silicon, and picking the wrong one makes a
peripheral unwritable:

- **The 8237 has a data path of its own.** `DACK` and `IOR`/`IOW` move a byte
  between the chip and the peripheral without an address. That is what
  [`core::wire::DmaPeripheral`] is: `dma_read`/`dma_write`, one byte, with a
  `terminal` flag for the `TC` pulse. A peripheral offers it from
  `Device::dma_peripheral` on its `DRQ` pin and the realizer hands it to
  whatever is the sink on that net.
- **An STM32 controller has none.** `CPAR` is an ordinary address — the
  peripheral's own data register — and the controller reaches it through the
  bus matrix like any other master. The request line is the *whole* seam; the
  access that services it is the same access firmware would have made, and it
  is that access which clears `TXE`/`RXNE` and satisfies the peripheral.

So a new controller should ask which of those its manual describes, and a new
peripheral should ask which controller its part is wired to.

## Writing a peripheral that `st.dma` feeds

The contract is small on purpose, because peripherals outnumber controllers:

- Drive **one output wire per request**. Name it for what it is — `dma_tx`,
  `dma_rx` — and let the board file wire it to the stream or channel that the
  part's request matrix puts it on (RM0090 Table 43, RM0351 Table 40). The
  matrix is a fact about the *part*, so it belongs in the board file, exactly
  like the NVIC position in `wire usart2.irq -> cpu.irq38`.
- **`Level::High` means "I want service".** Two styles both work and neither
  needs a flag:
  - Hold it high while you have data (or room) and drop it when the
    controller's bus access to your data register has satisfied you. You get
    continuous service — this is what a FIFO peripheral does.
  - Pulse it high then low once per item. Each rising edge is latched and buys
    exactly one beat.
- **Raising it is safe from inside your own register handler.** The controller
  records the request in an atomic and returns; the beat happens later, in its
  `run`. Nothing calls back into you from `set_level`.
- **Expect the beat to arrive as an ordinary MMIO access to your data
  register**, on the controller's `RequesterId` rather than the CPU's. Your
  normal `MemOps::write` runs. Mutate your own state in a short critical
  section, release it, and only then drop the request line — the same
  re-entrancy contract as everywhere else.
- Optionally publish a [`DmaPeripheral`] from `Device::dma_peripheral` on the
  same pin. `st.dma` polls only `dma_ready()`, as the level; it never calls
  `dma_read`/`dma_write`, because the data goes over the bus.

## Implementation notes for a controller

- **A transfer is not instantaneous.** The scheduler owns time: be
  `is_runnable`, move a bounded number of beats per `run`, and let the board
  pick the rate by picking the clock domain. Transferring a whole buffer inside
  the MMIO write that set the enable bit is observably wrong to firmware that
  polls a peripheral flag or watches the count, and it is a beat landing
  mid-instruction, which costs determinism as well.
- **Plan, release, move, retake, commit, then drive.** The register state lock
  is never held across a bus access, a wire change or a call into a peripheral.
  Both controllers in the tree are written this way, and `st.dma`'s
  `a_peripheral_may_raise_its_request_from_inside_its_own_register_write` is
  the test that proves it: a mock peripheral takes a `DEVICE`-ranked lock of
  its own from inside the DMA's write to it, which `core::sync`'s rank ladder
  turns into a panic if the controller still held one.
- **The request latch must not be a lock.** An atomic per channel means the
  inbound wire path can never deadlock against the outbound one, whatever a
  peripheral is holding when it asks.
- **A bus fault is reportable state**, not a silent stop: the STM32 faces raise
  `TEIF` and disable the unit; the 8237 has no status bit and simply stops.
- **Snapshot the live pointers, not just the programmed registers.** A
  transfer caught half done has a current address and a current count that are
  not the values the guest wrote, and a snapshot that dropped them would
  restart the transfer from the top.

[`RequesterId`]: https://docs.rs/rsemu/latest/rsemu/core/space/struct.RequesterId.html
[`core::wire::DmaPeripheral`]: https://docs.rs/rsemu/latest/rsemu/core/wire/trait.DmaPeripheral.html
[`DmaPeripheral`]: https://docs.rs/rsemu/latest/rsemu/core/wire/trait.DmaPeripheral.html
