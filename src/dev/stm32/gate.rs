//! The peripheral clock gate: what `RCC_xxENR` and `RCC_xxRSTR` do to a
//! peripheral.
//!
//! `st.rcc` has exported every `xxENR` and `xxRSTR` bit as an output pin since
//! it was written. This is the other end of that wire — the two input pins a
//! peripheral grows so that the bits mean something, and the one piece of
//! shared state that decides what an access to a gated block does.
//!
//! # What a gated peripheral does
//!
//! ST says the same thing under every peripheral clock enable register: while
//! a peripheral's clock is not active its registers are **not readable** — the
//! value that comes back is zero — and accesses to them are **not effective**,
//! so a write is lost (RM0090 §7.3.10–§7.3.15 for the F4's `AHB1ENR` …
//! `APB2ENR`, RM0351 §6.4.16 ff. for the L4's). It is not a bus fault: an
//! STM32's AHB/APB bridge answers, it simply answers with nothing, which is
//! why a firmware that forgets one `RCC_AHB1ENR` write finds a GPIO port whose
//! `MODER` reads back as zero however often it is written rather than a
//! HardFault that would have told it what it did.
//!
//! So, while the gate is shut:
//!
//! | | |
//! | --- | --- |
//! | a read | returns zero, for every register of the block |
//! | a write | is dropped |
//! | the state behind the registers | **keeps its values** — gating removes the clock, it does not reset anything; `xxRSTR` is what resets |
//! | anything the peripheral counts | stops where it is, and goes on from there when the clock comes back |
//!
//! That last row is the half a register model alone would get wrong. A timer
//! whose `TIMxEN` is clear does not count: there is no clock edge to count.
//! [`ClockGate::clocked`] is therefore asked in two places — at the register
//! block, and wherever the device's own advance decides whether to run —
//! rather than only at the first.
//!
//! **A byte in flight is not modelled.** Removing the clock from a USART
//! mid-character would corrupt that character on the die; here a character
//! crosses the [`chardev`](crate::host::chardev) seam in one indivisible step
//! at the `TDR`/`DR` write, so there is never a frame half-shifted for the
//! gate to interrupt. Nothing is lost by the simplification and a guest cannot
//! tell: the byte either left before the gate shut or was never written.
//!
//! # The pins
//!
//! | Pin | Driven by | High means |
//! | --- | --- | --- |
//! | `enable` | `RCC_xxENR` bit *n* | the peripheral clock is running |
//! | `reset` | `RCC_xxRSTR` bit *n* | the peripheral reset line is pulled |
//!
//! ```text
//! # RM0090 §7.3.13: APB1ENR bit 17 is USART2EN, APB1RSTR bit 17 is USART2RST.
//! wire rcc.apb1en17  -> usart2.enable
//! wire rcc.apb1rst17 -> usart2.reset
//! ```
//!
//! `reset` is a level, not a pulse: its rising edge puts the block back to its
//! reset values through [`Gated::gate_reset`], and the block stays deaf while
//! it is high, which is what a peripheral held in reset does. RCC drives
//! either pin *after* its own critical section, so a sink may take its device
//! lock here.
//!
//! # An unwired pin is a clocked peripheral
//!
//! The rule comes from [`st.firewall`](super::firewall), which grew a `clken`
//! sink before this module existed: a board that drew the wire has said the
//! clock is the RCC's to give, and a board that drew none is not a board whose
//! peripherals are all switched off — it is a board that does not model the
//! gate. So [`ClockGate::clocked`] answers *true* until something drives the
//! pin, and every machine file written before this module keeps working
//! unchanged.
//!
//! `st.firewall` keeps its own `clken` pin rather than using this module:
//! `RCC_APB2ENR.FWEN` is write-once ("Set by software, reset by hardware",
//! RM0351 §6.4.16) and its level is inside that device's snapshot for a
//! reason of its own. The design is the same one; only the spelling differs.
//!
//! # Why the level is not serialized
//!
//! A [`ClockGate`] holds a level a *sibling* drives, and the usual rule for
//! such a level is to save it — chunk load order is nobody's to depend on. It
//! is not saved here, because the RCC closes the hole from its own end:
//! `Rcc::load` restores `xxENR`/`xxRSTR` and then republishes every connected
//! gate pin, and no peripheral's `load` touches this state, so the two cannot
//! disagree whichever order they load in. Keeping it out of the chunks also
//! means adopting the gate moves no device's snapshot encoding.

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::device::SinkPin;
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

/// The name of the clock-enable input: `RCC_xxENR`'s bit for this peripheral.
pub const ENABLE_PIN: &str = "enable";

/// The name of the peripheral-reset input: `RCC_xxRSTR`'s bit.
pub const RESET_PIN: &str = "reset";

/// A peripheral's clock gate: the two levels RCC drives and what they mean.
///
/// Lives in whatever struct the device's `MemOps` is implemented on, beside
/// the register state rather than inside it — it is wiring, not chip state.
#[derive(Debug)]
pub struct ClockGate {
    /// Whether a board drew an `enable` wire at all.
    ///
    /// Topology, not state: it answers the question a bare pin level cannot,
    /// which is whether the low it sits at means "RCC has not enabled this" or
    /// "nobody models an RCC here".
    wired: AtomicBool,
    /// `RCC_xxENR`'s bit, as the wire last delivered it.
    clocked: AtomicBool,
    /// `RCC_xxRSTR`'s bit, as the wire last delivered it.
    in_reset: AtomicBool,
    /// The pins built for this gate. A net holds a sink weakly, so the strong
    /// reference has to live somewhere; here is the one place every device
    /// that adopts the gate already has.
    pins: Mutex<Vec<Arc<GatePin>>>,
}

impl Default for ClockGate {
    fn default() -> ClockGate {
        ClockGate::new()
    }
}

impl ClockGate {
    /// A gate nothing drives: clocked, not in reset.
    #[must_use]
    pub fn new() -> ClockGate {
        ClockGate {
            wired: AtomicBool::new(false),
            clocked: AtomicBool::new(false),
            in_reset: AtomicBool::new(false),
            pins: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        }
    }

    /// Whether the peripheral has a clock.
    ///
    /// True for an unwired gate, which is what keeps a board that models no
    /// RCC working (see the module documentation).
    #[must_use]
    #[inline]
    pub fn clocked(&self) -> bool {
        !self.wired.load(Ordering::Relaxed) || self.clocked.load(Ordering::Relaxed)
    }

    /// Whether `RCC_xxRSTR`'s bit is standing.
    #[must_use]
    #[inline]
    pub fn in_reset(&self) -> bool {
        self.in_reset.load(Ordering::Relaxed)
    }

    /// Whether the register block answers at all: clocked and not held in
    /// reset.
    ///
    /// The one question a `MemOps` implementation asks.
    #[must_use]
    #[inline]
    pub fn live(&self) -> bool {
        self.clocked() && !self.in_reset()
    }

    /// Drive `enable` directly — the route a test with no RCC takes.
    ///
    /// Driving the pin at all is what makes it load-bearing, exactly as a
    /// board's `wire` is.
    pub fn set_enable(&self, high: bool) {
        self.wired.store(true, Ordering::Relaxed);
        self.clocked.store(high, Ordering::Relaxed);
    }

    /// Record `reset`'s level, and say whether this was its rising edge.
    ///
    /// The caller does the resetting: only the device knows what its reset
    /// values are.
    fn take_reset(&self, high: bool) -> bool {
        !self.in_reset.swap(high, Ordering::Relaxed) && high
    }
}

/// A device whose register block is gated by [`ClockGate`].
///
/// Implemented on the shared inner struct — the one behind the `Arc` the
/// device's regions and pins hold — because that is what outlives both.
pub trait Gated: Send + Sync {
    /// The gate this device keeps.
    fn clock_gate(&self) -> &ClockGate;

    /// Put the block back to its reset values: `RCC_xxRSTR`'s bit went high.
    ///
    /// Called with no lock held. The default does nothing, for a peripheral
    /// that takes the clock gate but whose reset line no board drives —
    /// adopting the two pins one at a time is the point of the seam.
    fn gate_reset(&self) {}
}

/// Which of the two gate inputs a pin is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Input {
    /// `RCC_xxENR`'s bit.
    Enable,
    /// `RCC_xxRSTR`'s bit.
    Reset,
}

/// One gate input, as something a wire can drive.
///
/// Holds its device **weakly**: the device owns the pin through its own
/// [`ClockGate`], so a strong reference back would be a cycle nothing could
/// drop. The upgrade cannot fail while the device is alive, and a level
/// arriving at a device that is gone has nothing to do anyway.
#[derive(Debug)]
pub struct GatePin {
    owner: Weak<dyn Gated>,
    which: Input,
    inputs: FanIn,
}

impl GatePin {
    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for GatePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        let Some(owner) = self.owner.upgrade() else {
            return;
        };
        match self.which {
            Input::Enable => owner.clock_gate().set_enable(high),
            Input::Reset => {
                if owner.clock_gate().take_reset(high) {
                    owner.gate_reset();
                }
            }
        }
    }
}

/// The sink for `enable` or `reset`, or `None` for any other port name.
///
/// A device's [`Device::sink`](crate::core::device::Device::sink) offers this
/// first and falls through to its own pins:
///
/// ```ignore
/// fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
///     if let Some(pin) = gate::sink(&self.regs, port, sources) {
///         return Some(pin);
///     }
///     // … the device's own inputs …
/// }
/// ```
#[must_use]
pub fn sink<T>(owner: &Arc<T>, port: &str, sources: &[WireId]) -> Option<SinkPin>
where
    T: Gated + 'static,
{
    let which = match port {
        ENABLE_PIN => Input::Enable,
        RESET_PIN => Input::Reset,
        _ => return None,
    };
    if which == Input::Enable {
        // Drawing the wire is the assertion that RCC owns this clock, and an
        // `xxENR` bit resets low.
        owner.clock_gate().wired.store(true, Ordering::Relaxed);
    }
    let pin = Arc::new(GatePin {
        owner: Arc::downgrade(owner) as Weak<dyn Gated>,
        which,
        inputs: FanIn::new(sources),
    });
    owner.clock_gate().pins.lock().push(Arc::clone(&pin));
    Some(SinkPin { sink: pin, line: 0 })
}

/// Declare both gate pins on a class schema.
///
/// One call, so that a class cannot declare the enable and forget the reset —
/// and so that the two names are spelled in exactly one place.
#[must_use]
pub fn ports(
    schema: crate::machine::validate::ClassSchema,
) -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::PortDir;
    schema
        .port(ENABLE_PIN, PortDir::In)
        .port(RESET_PIN, PortDir::In)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A device with nothing but a gate, which is all these tests need.
    #[derive(Debug)]
    struct Toy {
        gate: ClockGate,
        resets: AtomicBool,
    }

    impl Gated for Toy {
        fn clock_gate(&self) -> &ClockGate {
            &self.gate
        }

        fn gate_reset(&self) {
            self.resets.store(true, Ordering::Relaxed);
        }
    }

    fn toy() -> Arc<Toy> {
        Arc::new(Toy {
            gate: ClockGate::new(),
            resets: AtomicBool::new(false),
        })
    }

    #[test]
    fn an_unwired_gate_is_clocked() {
        // The whole reason every board written before the gate keeps working.
        let toy = toy();
        assert!(toy.gate.clocked());
        assert!(toy.gate.live());
    }

    #[test]
    fn a_wired_gate_starts_shut() {
        // An `xxENR` bit resets low, so a board that drew the wire gets a
        // peripheral whose clock its firmware has to switch on.
        let toy = toy();
        let pin = sink(&toy, ENABLE_PIN, &[WireId::new(1)]).expect("the gate offers `enable`");
        assert!(!toy.gate.clocked(), "drawing the wire is the assertion");
        pin.sink.set_level(WireId::new(1), 0, Level::High);
        assert!(toy.gate.clocked());
        pin.sink.set_level(WireId::new(1), 0, Level::Low);
        assert!(!toy.gate.clocked());
    }

    #[test]
    fn the_reset_pin_resets_on_its_rising_edge_only() {
        let toy = toy();
        let pin = sink(&toy, RESET_PIN, &[WireId::new(2)]).expect("the gate offers `reset`");
        assert!(!toy.gate.in_reset());
        // A reset wire that nobody drives leaves the block alone, and an
        // `enable` nobody drew stays clocked, so the block is live.
        assert!(toy.gate.live());
        pin.sink.set_level(WireId::new(2), 0, Level::High);
        assert!(toy.resets.load(Ordering::Relaxed), "the edge did the work");
        assert!(toy.gate.in_reset());
        assert!(!toy.gate.live(), "a block held in reset answers nothing");
        toy.resets.store(false, Ordering::Relaxed);
        pin.sink.set_level(WireId::new(2), 0, Level::High);
        assert!(
            !toy.resets.load(Ordering::Relaxed),
            "a level that did not move is not an edge"
        );
        pin.sink.set_level(WireId::new(2), 0, Level::Low);
        assert!(
            toy.gate.live(),
            "and the block comes back when it is let go"
        );
    }

    #[test]
    fn a_port_that_is_not_a_gate_pin_is_not_claimed() {
        // The device's own `sink` has to see its own pins.
        let toy = toy();
        assert!(sink(&toy, "irq", &[WireId::new(3)]).is_none());
    }

    #[test]
    fn a_pin_does_not_keep_its_device_alive() {
        // The pin holds the device weakly and the device holds the pin
        // strongly; dropping the device has to drop both.
        let toy = toy();
        let weak = Arc::downgrade(&toy);
        let _ = sink(&toy, ENABLE_PIN, &[WireId::new(1)]);
        drop(toy);
        assert!(weak.upgrade().is_none(), "the pair is not a cycle");
    }
}
