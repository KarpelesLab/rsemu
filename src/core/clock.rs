//! The oscillator forest and virtual time (`ROADMAP.md` §4.2).
//!
//! # Why a forest and not a tree
//!
//! A machine has as many clock roots as the real board has crystals: one for a
//! Game Boy, two for a SNES, a dozen for a PC. Each root is an **oscillator**
//! with a *rational* frequency — the NES master is `236250000/11` Hz and the
//! PC's PIT is `105000000/88` Hz, neither of which is an integer — and each
//! oscillator carries a tree of [`ClockDomain`]s below it, every domain rated
//! `parent × mul / div`.
//!
//! Where a domain sits decides what kind of time it gets, and that is the whole
//! point of the shape:
//!
//! * **Within one oscillator's tree the ratios are exact**, by construction and
//!   forever. The NES CPU is master ÷ 12 and the PPU is master ÷ 4, so the PPU
//!   advances exactly three dots per CPU cycle on every console ever made, and
//!   games depend on that absolutely. Intra-tree arithmetic is small-integer
//!   multiply and divide over the divisors and **never routes through absolute
//!   time** (`ROADMAP.md` §15, invariant 2).
//! * **Across oscillators exactness is meaningless**, because two crystals have
//!   independent tolerances, temperature coefficients and power-on phase. Real
//!   hardware has no fixed relationship there, so no correct guest software can
//!   depend on one. Cross-tree ordering goes through the [`GlobalTime`]
//!   fixed-point timeline with a per-root residual accumulator: the error is
//!   bounded below one 2⁻⁶⁴-second unit and — because the accumulator keeps the
//!   result a pure function of the tick count — it is *non-accumulating*, no
//!   matter how many ticks pass.
//!
//! No floating point appears anywhere in this module, nor anywhere its results
//! are used. That is not a stylistic preference: an `f64` in the time path is a
//! determinism bug (`ROADMAP.md` §0, §15).
//!
//! # The per-tree unit tick
//!
//! Each tree derives one **unit tick** — the fastest tick in the tree — such
//! that every domain's tick is a whole number of unit ticks. If domain *i* runs
//! at `root × aᵢ/bᵢ` (reduced), the unit rate is `root × A` with
//! `A = lcm(aᵢ)`, and domain *i* advances one tick per `kᵢ = (A / aᵢ) × bᵢ`
//! unit ticks. For the NES every `aᵢ` is 1, so `A = 1`: the unit tick *is* the
//! master tick, `k_cpu = 12`, `k_ppu = 4`, and the PPU tick count is
//! `units / 4 = 3 × (units / 12)` — exactly 3:1, with no absolute time in
//! sight.
//!
//! The lcm is taken **per tree, over the derived rates inside that tree** —
//! never across trees. That is what keeps it a number like 12 instead of the
//! 10⁵-fold blow-up a global lcm suffers the moment somebody adds an ordinary
//! 32.768 kHz RTC crystal. When a tree's lcm genuinely cannot be computed — a
//! guest programs a PLL to an arbitrary ratio — the operation **fails with
//! [`ClockError::LcmUnavailable`], naming the domains**, and leaves the forest
//! untouched. A timing model that quietly degrades is worse than one that
//! refuses.
//!
//! A tree's unit rate only ever gets *finer* (`A` is monotonically
//! non-decreasing). Adding or re-rating a domain rescales the tree's positions
//! by the exact integer factor `A' / A`; tick counts are unchanged by the
//! rescale, because `kᵢ` scales by the same factor.
//!
//! # State
//!
//! Per-domain `u64` tick counters are the authoritative architectural state and
//! are what a snapshot stores. They are held as an exact `(base_ticks,
//! base_unit, k)` triple so that re-rating and gating rebase without rewriting
//! history and so the whole tree stays exactly consistent. The global timeline
//! is *derived* and can always be recomputed from the counters.

use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

// ---------------------------------------------------------------------------
// small integer helpers
// ---------------------------------------------------------------------------

/// Greatest common divisor, by Euclid. `gcd(0, n) == n`.
const fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a
}

/// Least common multiple, or `None` when it does not fit in a `u64`.
///
/// This is the operation that fails when a guest programs a PLL to an arbitrary
/// ratio, and the failure is the point: see [`ClockError::LcmUnavailable`].
fn checked_lcm(a: u64, b: u64) -> Option<u64> {
    if a == 0 || b == 0 {
        return Some(0);
    }
    (a / gcd(a, b)).checked_mul(b)
}

/// A 192-bit unsigned integer, just wide enough for the cross-tree conversions.
///
/// Turning a `u128` fixed-point instant into ticks needs a 128×64 product and
/// then a division, and `core` has no `u256`. Three `u64` limbs,
/// most-significant first, are all it takes, and every step stays exact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct U192([u64; 3]);

impl U192 {
    /// `a * b`, exactly.
    fn mul_u128_u64(a: u128, b: u64) -> U192 {
        let (ah, al) = ((a >> 64) as u64, a as u64);
        let lo = (al as u128) * (b as u128);
        let hi = (ah as u128) * (b as u128) + (lo >> 64);
        U192([(hi >> 64) as u64, hi as u64, lo as u64])
    }

    /// Divide by a `u64`, returning the quotient and the remainder.
    fn div_u64(self, d: u64) -> (U192, u64) {
        debug_assert!(d != 0);
        let mut rem: u128 = 0;
        let mut q = [0u64; 3];
        for (out, limb) in q.iter_mut().zip(self.0) {
            // `rem < d <= u64::MAX`, so the shift cannot overflow a `u128`.
            let cur = (rem << 64) | (limb as u128);
            *out = (cur / (d as u128)) as u64;
            rem = cur % (d as u128);
        }
        (U192(q), rem as u64)
    }

    /// The value as a `u128`, or `None` if it does not fit.
    fn to_u128(self) -> Option<u128> {
        if self.0[0] != 0 {
            return None;
        }
        Some(((self.0[1] as u128) << 64) | (self.0[2] as u128))
    }

    /// The value divided by 2⁶⁴, exactly.
    fn shr64(self) -> U192 {
        U192([0, self.0[0], self.0[1]])
    }

    /// The value multiplied by 2⁶⁴, or `None` if the top limb would be lost.
    ///
    /// Only the independent reference computation in the tests needs this
    /// direction; the forest itself never multiplies up.
    #[cfg(test)]
    fn shl64(self) -> Option<U192> {
        if self.0[0] != 0 {
            return None;
        }
        Some(U192([self.0[1], self.0[2], 0]))
    }

    /// The value minus one. The value must not be zero.
    fn dec(self) -> U192 {
        let mut l = self.0;
        for i in (0..3).rev() {
            match l[i].checked_sub(1) {
                Some(v) => {
                    l[i] = v;
                    return U192(l);
                }
                None => l[i] = u64::MAX,
            }
        }
        debug_assert!(false, "decrement of zero");
        U192(l)
    }
}

// ---------------------------------------------------------------------------
// rational frequencies
// ---------------------------------------------------------------------------

/// An exact non-negative rational number, always kept in lowest terms.
///
/// Frequencies are rational, not integral: the NES master crystal is
/// `236250000/11` Hz = 21477272.72… Hz and the PC's PIT is `105000000/88` Hz.
/// Rounding either at declaration time would bake a permanent error into every
/// cross-tree conversion, so the declared value is kept as written and all
/// arithmetic on it is exact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rational {
    num: u64,
    den: u64,
}

impl Rational {
    /// The multiplicative identity, `1/1`.
    pub const ONE: Rational = Rational { num: 1, den: 1 };

    /// Builds `num / den`, reduced.
    ///
    /// # Errors
    ///
    /// [`ClockError::ZeroDenominator`] if `den` is zero.
    pub fn new(num: u64, den: u64) -> ClockResult<Rational> {
        if den == 0 {
            return Err(ClockError::ZeroDenominator);
        }
        if num == 0 {
            return Ok(Rational { num: 0, den: 1 });
        }
        let g = gcd(num, den);
        Ok(Rational {
            num: num / g,
            den: den / g,
        })
    }

    /// Builds the whole number `n / 1`.
    pub const fn integer(n: u64) -> Rational {
        Rational { num: n, den: 1 }
    }

    /// The numerator, in lowest terms.
    #[inline]
    pub const fn num(self) -> u64 {
        self.num
    }

    /// The denominator, in lowest terms. Never zero.
    #[inline]
    pub const fn den(self) -> u64 {
        self.den
    }

    /// True for exactly zero.
    #[inline]
    pub const fn is_zero(self) -> bool {
        self.num == 0
    }

    /// `self × other`, or `None` if the reduced result does not fit `u64/u64`.
    ///
    /// Cross-reduction happens before multiplication, so a chain of divider
    /// ratios never overflows on the way to a small answer.
    pub fn checked_mul(self, other: Rational) -> Option<Rational> {
        if self.is_zero() || other.is_zero() {
            return Some(Rational { num: 0, den: 1 });
        }
        let g1 = gcd(self.num, other.den);
        let g2 = gcd(other.num, self.den);
        let num = (self.num / g1).checked_mul(other.num / g2)?;
        let den = (self.den / g2).checked_mul(other.den / g1)?;
        Some(Rational { num, den })
    }

    /// `self × mul / div`, or `None` on overflow or a zero `div`.
    pub fn checked_scale(self, mul: u64, div: u64) -> Option<Rational> {
        self.checked_mul(Rational::new(mul, div).ok()?)
    }
}

impl PartialOrd for Rational {
    fn partial_cmp(&self, other: &Rational) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Rational {
    fn cmp(&self, other: &Rational) -> core::cmp::Ordering {
        // u64 × u64 always fits in a u128, so the comparison is exact.
        let a = (self.num as u128) * (other.den as u128);
        let b = (other.num as u128) * (self.den as u128);
        a.cmp(&b)
    }
}

impl fmt::Display for Rational {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.den == 1 {
            write!(f, "{}", self.num)
        } else {
            write!(f, "{}/{}", self.num, self.den)
        }
    }
}

// ---------------------------------------------------------------------------
// the global timeline
// ---------------------------------------------------------------------------

/// The number of fractional bits in a [`GlobalTime`]: it counts 2⁻⁶⁴ seconds.
pub const GLOBAL_TIME_FRAC_BITS: u32 = 64;

/// A point on the machine-wide fixed-point timeline, in units of 2⁻⁶⁴ seconds.
///
/// This exists **only to order events across oscillator trees** and to drive
/// rate control. Two domains that share a crystal must never be related through
/// it — that would throw away the exactness the forest exists to preserve
/// (`ROADMAP.md` §15, invariant 2); use [`ClockForest::convert_ticks`].
///
/// A `u128` of 2⁻⁶⁴-second units spans about 10¹² years, so the timeline never
/// needs to wrap, and the resolution is fine enough that one whole unit of
/// error is below any physically meaningful quantity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Hash)]
pub struct GlobalTime(u128);

impl GlobalTime {
    /// The origin of the timeline.
    pub const ZERO: GlobalTime = GlobalTime(0);
    /// The largest representable instant, used as "never".
    pub const MAX: GlobalTime = GlobalTime(u128::MAX);

    /// Wraps a raw count of 2⁻⁶⁴-second units.
    #[inline]
    pub const fn from_raw(raw: u128) -> GlobalTime {
        GlobalTime(raw)
    }

    /// The raw count of 2⁻⁶⁴-second units.
    #[inline]
    pub const fn raw(self) -> u128 {
        self.0
    }

    /// The instant `nanos` nanoseconds after the origin, rounded down.
    pub const fn from_nanos(nanos: u64) -> GlobalTime {
        // `nanos << 64` always fits a u128 because `nanos < 2^64`.
        GlobalTime(((nanos as u128) << GLOBAL_TIME_FRAC_BITS) / 1_000_000_000)
    }

    /// This instant in whole nanoseconds since the origin, rounded down and
    /// saturating.
    ///
    /// Computed in two halves so the intermediate product cannot overflow.
    pub const fn as_nanos(self) -> u64 {
        let whole = self.0 >> GLOBAL_TIME_FRAC_BITS;
        let frac = self.0 & ((1u128 << GLOBAL_TIME_FRAC_BITS) - 1);
        let secs_ns = match whole.checked_mul(1_000_000_000) {
            Some(v) => v,
            None => return u64::MAX,
        };
        let frac_ns = (frac * 1_000_000_000) >> GLOBAL_TIME_FRAC_BITS;
        let total = secs_ns + frac_ns;
        if total > u64::MAX as u128 {
            u64::MAX
        } else {
            total as u64
        }
    }

    /// Addition that saturates at [`GlobalTime::MAX`].
    #[inline]
    pub const fn saturating_add(self, other: GlobalTime) -> GlobalTime {
        GlobalTime(self.0.saturating_add(other.0))
    }

    /// Subtraction that saturates at [`GlobalTime::ZERO`].
    #[inline]
    pub const fn saturating_sub(self, other: GlobalTime) -> GlobalTime {
        GlobalTime(self.0.saturating_sub(other.0))
    }
}

// ---------------------------------------------------------------------------
// identifiers and errors
// ---------------------------------------------------------------------------

/// A handle to a [`ClockDomain`] within one [`ClockForest`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DomainId(u32);

impl DomainId {
    /// The handle's index, for callers keeping parallel arrays.
    #[inline]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A handle to one oscillator — that is, to one tree of the forest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OscillatorId(u32);

impl OscillatorId {
    /// The handle's index, for callers keeping parallel arrays.
    #[inline]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// Everything the clock forest refuses to do.
///
/// Every variant a machine description or a guest divider write can provoke
/// **names the domains involved**, because an error the author cannot locate is
/// an error they cannot fix.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum ClockError {
    /// A rational was built with a zero denominator.
    ZeroDenominator,
    /// A frequency, multiplier or divider was zero where a positive value is
    /// required.
    ZeroRate(String),
    /// The handle does not belong to this forest.
    UnknownDomain(DomainId),
    /// The handle does not belong to this forest.
    UnknownOscillator(OscillatorId),
    /// The tree's internal lcm does not fit in a `u64`, so no exact common unit
    /// tick exists — typically a guest programming a PLL to an arbitrary ratio.
    ///
    /// Deliberately fatal to the operation that caused it: the forest is left
    /// exactly as it was, and the machine must say what to do instead.
    LcmUnavailable {
        /// The domains of the tree that could not be reconciled.
        domains: Vec<String>,
    },
    /// A tick, unit or timeline counter would overflow.
    Overflow {
        /// What was being computed.
        what: &'static str,
        /// The domains involved.
        domains: Vec<String>,
    },
    /// Reparenting would make a domain its own ancestor.
    Cycle(String),
    /// The operation is not valid on this domain — a root where a derived
    /// domain is wanted, or the reverse.
    NotApplicable(String),
    /// The two domains are on different oscillators, so no exact ratio exists.
    ///
    /// Returned rather than silently converting through absolute time: crossing
    /// trees is a physical statement about the machine and must be explicit.
    CrossTree(DomainId, DomainId),
    /// A gated domain cannot be advanced by its own ticks.
    Gated(String),
}

impl fmt::Display for ClockError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ClockError::ZeroDenominator => f.write_str("zero denominator"),
            ClockError::ZeroRate(w) => write!(f, "`{w}`: rate must be greater than zero"),
            ClockError::UnknownDomain(id) => write!(f, "no clock domain #{}", id.0),
            ClockError::UnknownOscillator(id) => write!(f, "no oscillator #{}", id.0),
            ClockError::LcmUnavailable { domains } => write!(
                f,
                "no exact common clock unit for domains [{}]: the tree's internal lcm does \
                 not fit in 64 bits",
                domains.join(", ")
            ),
            ClockError::Overflow { what, domains } => {
                write!(f, "{what} overflowed for domains [{}]", domains.join(", "))
            }
            ClockError::Cycle(name) => write!(f, "reparenting `{name}` would create a cycle"),
            ClockError::NotApplicable(msg) => f.write_str(msg),
            ClockError::CrossTree(a, b) => write!(
                f,
                "domains #{} and #{} are driven by different oscillators; no exact ratio exists",
                a.0, b.0
            ),
            ClockError::Gated(name) => write!(f, "clock domain `{name}` is gated"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ClockError {}

impl From<ClockError> for crate::core::Error {
    /// Clock failures surface as configuration errors.
    ///
    /// `core::Error` has no `Clock` variant yet — it is `#[non_exhaustive]` and
    /// one belongs there; when it lands this conversion should target it. The
    /// message survives either way, and it is the message that names the
    /// domains.
    fn from(e: ClockError) -> Self {
        crate::core::Error::Config {
            at: String::from("clock"),
            message: e.to_string(),
        }
    }
}

/// Shorthand for a fallible clock operation.
pub type ClockResult<T> = core::result::Result<T, ClockError>;

// ---------------------------------------------------------------------------
// domains
// ---------------------------------------------------------------------------

/// One clock domain: `parent × mul / div`, with its own tick counter.
///
/// A domain with no parent is an oscillator root. The fields are private
/// because several are derived and have to move together — use
/// [`ClockForest::set_rating`], [`ClockForest::reparent`] and
/// [`ClockForest::set_gated`] to change one at runtime, which is what a PLL, a
/// guest reprogramming a divider, or a halting CPU amounts to.
#[derive(Debug, Clone)]
pub struct ClockDomain {
    name: String,
    parent: Option<DomainId>,
    mul: u64,
    div: u64,
    children: Vec<DomainId>,

    // Derived; recomputed whenever the tree changes.
    root: OscillatorId,
    /// Rate relative to the root oscillator's frequency.
    ratio: Rational,
    /// Tree unit ticks per one tick of this domain. Never zero.
    units_per_tick: u64,

    // The authoritative tick counter, held exactly: the count is
    // `base_ticks + (tree units − base_unit) / units_per_tick`.
    base_unit: u64,
    base_ticks: u64,
    gated: bool,
}

impl ClockDomain {
    /// The domain's name, as used in diagnostics.
    #[inline]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The parent domain, or `None` for an oscillator root.
    #[inline]
    pub const fn parent(&self) -> Option<DomainId> {
        self.parent
    }

    /// The multiplier applied to the parent's rate.
    #[inline]
    pub const fn mul(&self) -> u64 {
        self.mul
    }

    /// The divisor applied to the parent's rate.
    #[inline]
    pub const fn div(&self) -> u64 {
        self.div
    }

    /// The oscillator whose tree this domain belongs to.
    #[inline]
    pub const fn root(&self) -> OscillatorId {
        self.root
    }

    /// This domain's rate as an exact fraction of its oscillator's frequency.
    #[inline]
    pub const fn ratio_to_root(&self) -> Rational {
        self.ratio
    }

    /// How many of the tree's unit ticks make up one tick of this domain.
    ///
    /// This is the number every exact intra-tree conversion is built from: the
    /// NES CPU's is 12 and the PPU's is 4.
    #[inline]
    pub const fn units_per_tick(&self) -> u64 {
        self.units_per_tick
    }

    /// Whether the domain is currently gated (stopped).
    #[inline]
    pub const fn is_gated(&self) -> bool {
        self.gated
    }

    /// The tick count this domain has reached at a given tree position.
    #[inline]
    fn ticks_at(&self, units: u64) -> u64 {
        if self.gated {
            self.base_ticks
        } else {
            self.base_ticks + (units - self.base_unit) / self.units_per_tick
        }
    }
}

/// One oscillator: a declared crystal and the root of one tree.
#[derive(Debug, Clone)]
struct Oscillator {
    name: String,
    freq: Rational,
    root: DomainId,
    /// `lcm` of the tree's ratio numerators; the unit rate is `freq × unit_mul`.
    /// Monotonically non-decreasing, so the unit tick only ever gets finer and a
    /// rescale is always an exact integer multiplication.
    unit_mul: u64,
    /// The unit rate, in Hz.
    unit_rate: Rational,
    /// The tree's position in unit ticks: exact, and the only intra-tree state.
    units: u64,

    // Cross-tree conversion. `time` is the exact floor of the elapsed seconds
    // in 2^-64 units and `residual` is the remainder that makes it exact;
    // together they give an error below one unit that never accumulates.
    /// Unit-tick position at which the current frequency segment started.
    base_units: u64,
    /// Timeline position at which the current frequency segment started.
    base_time: u128,
    /// Whole 2⁻⁶⁴-second units per unit tick.
    step: u128,
    /// The remainder of that division, fed into `residual`.
    rem: u128,
    /// The tree's timeline position, in 2⁻⁶⁴-second units.
    time: u128,
    /// Accumulated remainder, always `< unit_rate.num()`.
    residual: u128,
    /// False once this oscillator has been locked into another one's tree.
    active: bool,
}

impl Oscillator {
    /// Recomputes the fixed-point step and its remainder from the unit rate.
    fn recompute_conversion(&mut self) {
        let num = self.unit_rate.num() as u128;
        let den = self.unit_rate.den() as u128;
        debug_assert!(num != 0);
        // Seconds per unit tick is den/num; in 2⁻⁶⁴ units that is
        // den × 2⁶⁴ / num, split into quotient and remainder.
        let scaled = den << GLOBAL_TIME_FRAC_BITS;
        self.step = scaled / num;
        self.rem = scaled % num;
    }

    fn overflow(&self, what: &'static str) -> ClockError {
        ClockError::Overflow {
            what,
            domains: alloc::vec![self.name.clone()],
        }
    }

    /// Advances the timeline by `units` unit ticks, keeping the residual exact.
    ///
    /// This is the per-root residual accumulator: `time` gains the whole part
    /// and `residual` carries the fraction, so `time` is always the exact floor
    /// of the true elapsed time and the error can never grow past one unit.
    fn accumulate(&mut self, units: u64) -> ClockResult<()> {
        let n = units as u128;
        let add = n
            .checked_mul(self.step)
            .ok_or_else(|| self.overflow("global timeline"))?;
        let carry_in = n
            .checked_mul(self.rem)
            .ok_or_else(|| self.overflow("global timeline"))?;
        let mut time = self
            .time
            .checked_add(add)
            .ok_or_else(|| self.overflow("global timeline"))?;
        let mut residual = self
            .residual
            .checked_add(carry_in)
            .ok_or_else(|| self.overflow("global timeline"))?;
        let num = self.unit_rate.num() as u128;
        if residual >= num {
            time = time
                .checked_add(residual / num)
                .ok_or_else(|| self.overflow("global timeline"))?;
            residual %= num;
        }
        self.time = time;
        self.residual = residual;
        Ok(())
    }

    /// The closed form of the same computation: the exact floor, from scratch.
    fn time_for_units(&self, units: u64) -> ClockResult<u128> {
        if units <= self.base_units {
            return Ok(self.base_time);
        }
        let n = (units - self.base_units) as u128;
        let num = self.unit_rate.num() as u128;
        let whole = n
            .checked_mul(self.step)
            .ok_or_else(|| self.overflow("global timeline"))?;
        // n × rem < 2^64 × 2^64, so this product always fits.
        let frac = n
            .checked_mul(self.rem)
            .ok_or_else(|| self.overflow("global timeline"))?
            / num;
        self.base_time
            .checked_add(whole)
            .and_then(|t| t.checked_add(frac))
            .ok_or_else(|| self.overflow("global timeline"))
    }

    /// The exact inverse of [`Oscillator::time_for_units`]: the largest unit
    /// position whose timeline instant is at or before `t`.
    ///
    /// Written as a true inverse rather than as the obvious
    /// `t × rate` because `t` is itself a floored quantity: multiplying it back
    /// would lose one tick on every position that does not divide exactly, and
    /// the loss would show up as a tree that can never quite reach its own
    /// reported time. With `S = den × 2⁶⁴ / num`,
    /// `max{u : floor(u × S) ≤ dt}` is `floor(((dt + 1) × num − 1) / (den × 2⁶⁴))`.
    fn units_at(&self, t: u128) -> ClockResult<u64> {
        if t <= self.base_time {
            return Ok(self.base_units);
        }
        let dt = t - self.base_time;
        // `dt + 1` cannot overflow: `dt <= t < u128::MAX` in every reachable
        // case, and saturating here would only ever under-report by one tick.
        let numerator = U192::mul_u128_u64(dt.saturating_add(1), self.unit_rate.num()).dec();
        // Dividing by `den × 2⁶⁴` in two exact steps, since the divisor itself
        // does not fit in a u64.
        let (q, _) = numerator.div_u64(self.unit_rate.den());
        let units = q
            .shr64()
            .to_u128()
            .and_then(|v| u64::try_from(v).ok())
            .ok_or_else(|| self.overflow("unit position"))?;
        self.base_units
            .checked_add(units)
            .ok_or_else(|| self.overflow("unit position"))
    }
}

/// A forest of clock domains: one tree per physical oscillator.
///
/// See the module documentation for the design. In short: intra-tree
/// relationships are exact integer arithmetic over the domains'
/// `units_per_tick`, cross-tree relationships go through [`GlobalTime`], and
/// the two are never confused for one another.
#[derive(Debug, Clone, Default)]
pub struct ClockForest {
    domains: Vec<ClockDomain>,
    oscillators: Vec<Oscillator>,
}

impl ClockForest {
    /// An empty forest.
    pub fn new() -> ClockForest {
        ClockForest::default()
    }

    // -- construction -------------------------------------------------------

    /// Declares an oscillator — a crystal — and returns its root domain.
    ///
    /// The frequency is exact and rational: `Rational::new(236250000, 11)` is
    /// the NES master clock, and it is *not* rounded.
    ///
    /// # Errors
    ///
    /// [`ClockError::ZeroRate`] if the frequency is zero.
    pub fn add_oscillator(&mut self, name: &str, freq: Rational) -> ClockResult<DomainId> {
        if freq.is_zero() {
            return Err(ClockError::ZeroRate(String::from(name)));
        }
        let osc = OscillatorId(self.oscillators.len() as u32);
        let root = DomainId(self.domains.len() as u32);
        self.domains.push(ClockDomain {
            name: String::from(name),
            parent: None,
            mul: 1,
            div: 1,
            children: Vec::new(),
            root: osc,
            ratio: Rational::ONE,
            units_per_tick: 1,
            base_unit: 0,
            base_ticks: 0,
            gated: false,
        });
        let mut o = Oscillator {
            name: String::from(name),
            freq,
            root,
            unit_mul: 1,
            unit_rate: freq,
            units: 0,
            base_units: 0,
            base_time: 0,
            step: 0,
            rem: 0,
            time: 0,
            residual: 0,
            active: true,
        };
        o.recompute_conversion();
        self.oscillators.push(o);
        Ok(root)
    }

    /// Adds a derived domain, rated `parent × mul / div`.
    ///
    /// The new domain starts at tick zero at the tree's current position. The
    /// tree's unit tick is refined if the new rate needs it, which leaves every
    /// existing tick counter unchanged.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`], [`ClockError::ZeroRate`], or
    /// [`ClockError::LcmUnavailable`] naming the tree's domains when no exact
    /// common unit tick exists. The forest is unchanged on failure.
    pub fn add_domain(
        &mut self,
        name: &str,
        parent: DomainId,
        mul: u64,
        div: u64,
    ) -> ClockResult<DomainId> {
        self.check_domain(parent)?;
        if mul == 0 || div == 0 {
            return Err(ClockError::ZeroRate(String::from(name)));
        }
        let backup = self.clone();
        let osc = self.domains[parent.index()].root;
        let id = DomainId(self.domains.len() as u32);
        let units_now = self.oscillators[osc.index()].units;
        self.domains.push(ClockDomain {
            name: String::from(name),
            parent: Some(parent),
            mul,
            div,
            children: Vec::new(),
            root: osc,
            ratio: Rational::ONE,
            units_per_tick: 1,
            base_unit: units_now,
            base_ticks: 0,
            gated: false,
        });
        self.domains[parent.index()].children.push(id);
        if let Err(e) = self.recompute_tree(osc) {
            *self = backup;
            return Err(e);
        }
        Ok(id)
    }

    // -- inspection ---------------------------------------------------------

    /// Borrows a domain.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`] if the handle is not from this forest.
    pub fn domain(&self, id: DomainId) -> ClockResult<&ClockDomain> {
        self.domains
            .get(id.index())
            .ok_or(ClockError::UnknownDomain(id))
    }

    /// The number of domains in the forest.
    #[inline]
    pub fn domain_count(&self) -> usize {
        self.domains.len()
    }

    /// Every oscillator in the forest, in declaration order.
    pub fn oscillators(&self) -> impl Iterator<Item = OscillatorId> + '_ {
        (0..self.oscillators.len() as u32).map(OscillatorId)
    }

    /// Every domain in the forest, in creation order.
    ///
    /// Handles are opaque and a forest is built by whoever holds it, so without
    /// this the only way to reach a domain's tick counter — the authoritative
    /// architectural state of `ROADMAP.md` §4.2, and part of every snapshot —
    /// was to have remembered each [`DomainId`] as it was created. A snapshot
    /// writer should be able to ask the forest what it contains, and the order
    /// is creation order so the answer is the same on both sides of a
    /// save/load.
    ///
    /// Roots come first within each tree only in the sense that an oscillator's
    /// root domain is created by [`ClockForest::add_oscillator`]; the sequence
    /// as a whole is simply the order the forest was built in.
    pub fn domains(&self) -> impl Iterator<Item = DomainId> + '_ {
        (0..self.domains.len() as u32).map(DomainId)
    }

    /// Whether an oscillator still drives its own tree.
    ///
    /// An oscillator that has been locked into another one's tree
    /// ([`ClockForest::lock_oscillator`]) is inactive: its crystal no longer
    /// decides anything, so nothing should advance it.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] if the handle is not from this forest.
    pub fn is_active(&self, osc: OscillatorId) -> ClockResult<bool> {
        Ok(self.osc(osc)?.active)
    }

    /// The declared frequency of an oscillator, in Hz.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] if the handle is not from this forest.
    pub fn frequency(&self, osc: OscillatorId) -> ClockResult<Rational> {
        Ok(self.osc(osc)?.freq)
    }

    /// The rate of a domain, in Hz, as an exact rational.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`], or [`ClockError::Overflow`] if the
    /// product does not reduce into `u64/u64`.
    pub fn domain_frequency(&self, id: DomainId) -> ClockResult<Rational> {
        let d = self.domain(id)?;
        let f = self.oscillators[d.root.index()].freq;
        f.checked_mul(d.ratio).ok_or_else(|| ClockError::Overflow {
            what: "domain frequency",
            domains: alloc::vec![d.name.clone()],
        })
    }

    /// The oscillator driving a domain.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`] if the handle is not from this forest.
    pub fn root_of(&self, id: DomainId) -> ClockResult<OscillatorId> {
        Ok(self.domain(id)?.root)
    }

    /// Whether a domain is currently gated.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`] if the handle is not from this forest.
    pub fn is_gated(&self, id: DomainId) -> ClockResult<bool> {
        Ok(self.domain(id)?.gated)
    }

    /// The domain's tick counter — the authoritative architectural state.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`] if the handle is not from this forest.
    pub fn ticks(&self, id: DomainId) -> ClockResult<u64> {
        let d = self.domain(id)?;
        Ok(d.ticks_at(self.oscillators[d.root.index()].units))
    }

    /// The tree's position, in that tree's unit ticks.
    ///
    /// Exposed for snapshots and diagnostics: it is the exact common
    /// denominator every domain in the tree is derived from.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] if the handle is not from this forest.
    pub fn unit_position(&self, osc: OscillatorId) -> ClockResult<u64> {
        Ok(self.osc(osc)?.units)
    }

    /// The tree's unit-tick rate, in Hz.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] if the handle is not from this forest.
    pub fn unit_rate(&self, osc: OscillatorId) -> ClockResult<Rational> {
        Ok(self.osc(osc)?.unit_rate)
    }

    // -- exact intra-tree conversion ---------------------------------------

    /// Converts a tick count of `from` into ticks of `to`, **exactly**.
    ///
    /// Both domains must share an oscillator; the result is
    /// `ticks × k_from / k_to` — small-integer arithmetic over the divisors,
    /// rounded down. For the NES this is what makes
    /// `convert_ticks(cpu, ppu, n) == 3n` true for every `n`, forever, with no
    /// absolute time involved.
    ///
    /// # Errors
    ///
    /// [`ClockError::CrossTree`] if the domains are on different oscillators:
    /// there is no exact answer, and pretending otherwise would emulate a
    /// precision the hardware does not have. [`ClockError::Overflow`] if the
    /// intermediate does not fit in a `u64`.
    pub fn convert_ticks(&self, from: DomainId, to: DomainId, ticks: u64) -> ClockResult<u64> {
        let a = self.domain(from)?;
        let b = self.domain(to)?;
        if a.root != b.root {
            return Err(ClockError::CrossTree(from, to));
        }
        let units = ticks
            .checked_mul(a.units_per_tick)
            .ok_or_else(|| ClockError::Overflow {
                what: "tick conversion",
                domains: alloc::vec![a.name.clone(), b.name.clone()],
            })?;
        Ok(units / b.units_per_tick)
    }

    // -- advancing ----------------------------------------------------------

    /// Advances the tree by `ticks` ticks of `id`.
    ///
    /// This is how guest execution moves time forward: a CPU consumes its
    /// budget, reports the ticks, and every other domain on the same crystal
    /// follows exactly, with no rounding anywhere.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`], [`ClockError::Gated`] if the domain is
    /// stopped, or [`ClockError::Overflow`] if the tree's unit counter or the
    /// timeline would overflow.
    pub fn advance_domain(&mut self, id: DomainId, ticks: u64) -> ClockResult<()> {
        let d = self.domain(id)?;
        if d.gated {
            return Err(ClockError::Gated(d.name.clone()));
        }
        let delta = ticks
            .checked_mul(d.units_per_tick)
            .ok_or_else(|| ClockError::Overflow {
                what: "unit position",
                domains: alloc::vec![d.name.clone()],
            })?;
        let osc = d.root;
        self.advance_units(osc, delta)
    }

    /// Advances a tree by a number of its own unit ticks.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] or [`ClockError::Overflow`].
    pub fn advance_units(&mut self, osc: OscillatorId, units: u64) -> ClockResult<()> {
        let idx = osc.index();
        if idx >= self.oscillators.len() {
            return Err(ClockError::UnknownOscillator(osc));
        }
        let o = &mut self.oscillators[idx];
        let new_units = o
            .units
            .checked_add(units)
            .ok_or_else(|| o.overflow("unit position"))?;
        o.accumulate(units)?;
        o.units = new_units;
        Ok(())
    }

    /// The timeline position of a tree, derived from its unit counter.
    ///
    /// Maintained incrementally by the residual accumulator, and equal — always,
    /// not approximately — to [`ClockForest::global_time_of_units`] at the
    /// tree's current position.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] if the handle is not from this forest.
    pub fn global_time(&self, osc: OscillatorId) -> ClockResult<GlobalTime> {
        Ok(GlobalTime(self.osc(osc)?.time))
    }

    /// The timeline position a tree reaches after `units` unit ticks, in closed
    /// form.
    ///
    /// Exactly `floor(base_time + (units − base_units) × den × 2⁶⁴ / num)`, so
    /// the error against the true value is strictly below one 2⁻⁶⁴-second unit
    /// and is a pure function of `units` — it cannot accumulate.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] or [`ClockError::Overflow`].
    pub fn global_time_of_units(&self, osc: OscillatorId, units: u64) -> ClockResult<GlobalTime> {
        Ok(GlobalTime(self.osc(osc)?.time_for_units(units)?))
    }

    /// The timeline position of a given tick of a domain.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`] or [`ClockError::Overflow`].
    pub fn global_time_of_tick(&self, id: DomainId, tick: u64) -> ClockResult<GlobalTime> {
        let d = self.domain(id)?;
        let o = &self.oscillators[d.root.index()];
        if tick <= d.base_ticks {
            return Ok(GlobalTime(o.time_for_units(d.base_unit)?));
        }
        let units = (tick - d.base_ticks)
            .checked_mul(d.units_per_tick)
            .and_then(|u| u.checked_add(d.base_unit))
            .ok_or_else(|| ClockError::Overflow {
                what: "unit position",
                domains: alloc::vec![d.name.clone()],
            })?;
        Ok(GlobalTime(o.time_for_units(units)?))
    }

    /// The tree's unit position at a timeline instant, rounded down.
    ///
    /// The cross-tree direction, and the only place a tree's position is derived
    /// from absolute time. Never use it to relate two domains that share an
    /// oscillator.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] or [`ClockError::Overflow`].
    pub fn units_at_global(&self, osc: OscillatorId, at: GlobalTime) -> ClockResult<u64> {
        self.osc(osc)?.units_at(at.0)
    }

    /// Advances a tree so that its position is at or after `at`, monotonically.
    ///
    /// Used for trees no runnable drives — a bare RTC crystal, say. Returns the
    /// number of unit ticks added, which is zero if the tree is already there.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] or [`ClockError::Overflow`].
    pub fn advance_to_global(&mut self, osc: OscillatorId, at: GlobalTime) -> ClockResult<u64> {
        let target = self.units_at_global(osc, at)?;
        let cur = self.osc(osc)?.units;
        if target <= cur {
            return Ok(0);
        }
        let delta = target - cur;
        self.advance_units(osc, delta)?;
        Ok(delta)
    }

    // -- runtime topology changes ------------------------------------------

    /// Re-rates a domain to `parent × mul / div` at runtime.
    ///
    /// The domain's tick counter is rebased first, so history is preserved
    /// exactly: ticks already counted stay counted at the rate they were counted
    /// at. This is a guest writing a divider register, or a PLL relocking.
    ///
    /// # Errors
    ///
    /// [`ClockError::NotApplicable`] on an oscillator root (use
    /// [`ClockForest::set_frequency`]), [`ClockError::ZeroRate`], or
    /// [`ClockError::LcmUnavailable`] naming the tree's domains when the new
    /// rating admits no exact common unit tick. **The forest is unchanged when
    /// this fails**; it never degrades to an approximation.
    pub fn set_rating(&mut self, id: DomainId, mul: u64, div: u64) -> ClockResult<()> {
        self.check_domain(id)?;
        if self.domains[id.index()].parent.is_none() {
            return Err(ClockError::NotApplicable(alloc::format!(
                "`{}` is an oscillator root; set its frequency instead",
                self.domains[id.index()].name
            )));
        }
        if mul == 0 || div == 0 {
            return Err(ClockError::ZeroRate(self.domains[id.index()].name.clone()));
        }
        let backup = self.clone();
        let osc = self.domains[id.index()].root;
        self.rebase_subtree(id);
        self.domains[id.index()].mul = mul;
        self.domains[id.index()].div = div;
        if let Err(e) = self.recompute_tree(osc) {
            *self = backup;
            return Err(e);
        }
        Ok(())
    }

    /// Re-rates an oscillator at runtime.
    ///
    /// The timeline position is kept where it is and a new conversion segment
    /// starts, so no time travels backwards and the residual stays bounded.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] or [`ClockError::ZeroRate`].
    pub fn set_frequency(&mut self, osc: OscillatorId, freq: Rational) -> ClockResult<()> {
        let idx = osc.index();
        if idx >= self.oscillators.len() {
            return Err(ClockError::UnknownOscillator(osc));
        }
        if freq.is_zero() {
            return Err(ClockError::ZeroRate(self.oscillators[idx].name.clone()));
        }
        let backup = self.clone();
        self.oscillators[idx].freq = freq;
        if let Err(e) = self.recompute_tree(osc) {
            *self = backup;
            return Err(e);
        }
        Ok(())
    }

    /// Moves a domain, and everything below it, under a new parent.
    ///
    /// Reparenting within a tree keeps every tick counter exact. Reparenting
    /// *across* trees rebases the moved domains onto the destination tree's
    /// position: their counters are preserved, but they now advance with a
    /// different crystal, which is exactly what the operation means.
    ///
    /// # Errors
    ///
    /// [`ClockError::Cycle`] if the new parent is inside the moved subtree,
    /// [`ClockError::ZeroRate`], or [`ClockError::LcmUnavailable`] /
    /// [`ClockError::Overflow`] — in which case the forest is left unchanged.
    pub fn reparent(
        &mut self,
        id: DomainId,
        new_parent: DomainId,
        mul: u64,
        div: u64,
    ) -> ClockResult<()> {
        self.check_domain(id)?;
        self.check_domain(new_parent)?;
        if mul == 0 || div == 0 {
            return Err(ClockError::ZeroRate(self.domains[id.index()].name.clone()));
        }
        if id == new_parent || self.is_ancestor(id, new_parent) {
            return Err(ClockError::Cycle(self.domains[id.index()].name.clone()));
        }

        let backup = self.clone();
        let old_osc = self.domains[id.index()].root;
        let new_osc = self.domains[new_parent.index()].root;

        // Materialize every counter in the moving subtree before it changes
        // frame of reference.
        self.rebase_subtree(id);

        match self.domains[id.index()].parent {
            Some(p) => self.domains[p.index()].children.retain(|c| *c != id),
            // An oscillator root being locked into another tree: its own crystal
            // stops deciding anything.
            None => self.oscillators[old_osc.index()].active = false,
        }
        self.domains[id.index()].parent = Some(new_parent);
        self.domains[id.index()].mul = mul;
        self.domains[id.index()].div = div;
        self.domains[new_parent.index()].children.push(id);

        if old_osc != new_osc {
            let dest_units = self.oscillators[new_osc.index()].units;
            for m in self.subtree(id) {
                self.domains[m.index()].root = new_osc;
                self.domains[m.index()].base_unit = dest_units;
            }
        }

        // Locking an oscillator moves its root domain away, which leaves the old
        // oscillator owning nothing; there is then no tree left to recompute.
        let old_root = self.oscillators[old_osc.index()].root;
        let old_tree_survives =
            old_osc != new_osc && self.domains[old_root.index()].root == old_osc;
        let outcome = self.recompute_tree(new_osc).and_then(|()| {
            if old_tree_survives {
                self.recompute_tree(old_osc)
            } else {
                Ok(())
            }
        });
        if let Err(e) = outcome {
            *self = backup;
            return Err(e);
        }
        Ok(())
    }

    /// Locks one oscillator into another's tree at an exact declared ratio.
    ///
    /// This is the machine file's `lock spc700 = master * a / b`. It is
    /// deliberately implemented as reparenting rather than as a special case:
    /// once locked, the two are one tree and the relationship is exact by the
    /// same construction as every other intra-tree ratio. Nothing about it is
    /// silent — a machine has to ask for it, and say why.
    ///
    /// # Errors
    ///
    /// As [`ClockForest::reparent`].
    pub fn lock_oscillator(
        &mut self,
        osc: OscillatorId,
        parent: DomainId,
        mul: u64,
        div: u64,
    ) -> ClockResult<()> {
        let root = self.osc(osc)?.root;
        self.reparent(root, parent, mul, div)
    }

    /// Gates (stops) or ungates a domain at runtime.
    ///
    /// A gated domain's tick counter holds still while the rest of its tree
    /// keeps moving — a halted CPU, a clock-gated peripheral. Ungating resumes
    /// from the counter's current value, so no ticks are invented.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`] if the handle is not from this forest.
    pub fn set_gated(&mut self, id: DomainId, gated: bool) -> ClockResult<()> {
        self.check_domain(id)?;
        let units = self.oscillators[self.domains[id.index()].root.index()].units;
        let d = &mut self.domains[id.index()];
        if d.gated == gated {
            return Ok(());
        }
        d.base_ticks = d.ticks_at(units);
        d.base_unit = units;
        d.gated = gated;
        Ok(())
    }

    /// Restores a tree's unit position, recomputing its timeline from it.
    ///
    /// For snapshot restore. The timeline is *derived*: a snapshot stores the
    /// counters, and this rebuilds the global position and the residual exactly
    /// from them, so a restored machine is bit-identical to the one that was
    /// saved rather than merely close to it.
    ///
    /// Restore the unit position **before** any [`ClockForest::restore_ticks`]
    /// on that tree, since the tick counters are anchored to it.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownOscillator`] or [`ClockError::Overflow`].
    pub fn restore_unit_position(&mut self, osc: OscillatorId, units: u64) -> ClockResult<()> {
        let idx = osc.index();
        if idx >= self.oscillators.len() {
            return Err(ClockError::UnknownOscillator(osc));
        }
        let o = &mut self.oscillators[idx];
        o.base_units = 0;
        o.base_time = 0;
        o.units = units;
        o.time = o.time_for_units(units)?;
        // The residual is the exact remainder that the floor above discarded,
        // which is what keeps the error from growing after the restore.
        let num = o.unit_rate.num() as u128;
        o.residual = (units as u128)
            .checked_mul(o.rem)
            .ok_or_else(|| o.overflow("global timeline"))?
            % num;
        Ok(())
    }

    /// Overwrites a domain's tick counter. For snapshot restore only.
    ///
    /// # Errors
    ///
    /// [`ClockError::UnknownDomain`] if the handle is not from this forest.
    pub fn restore_ticks(&mut self, id: DomainId, ticks: u64) -> ClockResult<()> {
        self.check_domain(id)?;
        let units = self.oscillators[self.domains[id.index()].root.index()].units;
        let d = &mut self.domains[id.index()];
        d.base_ticks = ticks;
        d.base_unit = units;
        Ok(())
    }

    // -- internals ----------------------------------------------------------

    fn osc(&self, osc: OscillatorId) -> ClockResult<&Oscillator> {
        self.oscillators
            .get(osc.index())
            .ok_or(ClockError::UnknownOscillator(osc))
    }

    fn check_domain(&self, id: DomainId) -> ClockResult<()> {
        if id.index() >= self.domains.len() {
            return Err(ClockError::UnknownDomain(id));
        }
        Ok(())
    }

    /// Every domain in the subtree rooted at `id`, in deterministic BFS order —
    /// which also guarantees a parent appears before any of its children.
    fn subtree(&self, id: DomainId) -> Vec<DomainId> {
        let mut out = alloc::vec![id];
        let mut i = 0;
        while i < out.len() {
            let cur = out[i];
            out.extend_from_slice(&self.domains[cur.index()].children);
            i += 1;
        }
        out
    }

    fn is_ancestor(&self, maybe_ancestor: DomainId, of: DomainId) -> bool {
        let mut cur = Some(of);
        while let Some(c) = cur {
            if c == maybe_ancestor {
                return true;
            }
            cur = self.domains[c.index()].parent;
        }
        false
    }

    /// Materializes the tick counters of a subtree at the tree's current
    /// position, so a rating change cannot retroactively rewrite them.
    fn rebase_subtree(&mut self, id: DomainId) {
        let units = self.oscillators[self.domains[id.index()].root.index()].units;
        for m in self.subtree(id) {
            let d = &mut self.domains[m.index()];
            d.base_ticks = d.ticks_at(units);
            d.base_unit = units;
        }
    }

    /// Recomputes a tree's ratios, unit tick and per-domain `units_per_tick`.
    ///
    /// Everything is computed into locals and committed only once all of it has
    /// succeeded, so a failure leaves the tree exactly as it was.
    fn recompute_tree(&mut self, osc: OscillatorId) -> ClockResult<()> {
        let root_domain = self.oscillators[osc.index()].root;
        let order = self.subtree(root_domain);
        let names = |forest: &ClockForest| -> Vec<String> {
            order
                .iter()
                .map(|d| forest.domains[d.index()].name.clone())
                .collect()
        };

        // Each domain's ratio to the root. BFS order puts a parent before its
        // children, so one pass is enough.
        let mut ratios: Vec<Rational> = Vec::with_capacity(order.len());
        for id in &order {
            let d = &self.domains[id.index()];
            let r = match d.parent {
                None => Rational::ONE,
                Some(p) => {
                    let pi = order
                        .iter()
                        .position(|x| *x == p)
                        .expect("BFS order lists a parent before its children");
                    ratios[pi].checked_scale(d.mul, d.div).ok_or_else(|| {
                        ClockError::LcmUnavailable {
                            domains: names(self),
                        }
                    })?
                }
            };
            ratios.push(r);
        }

        // The tree's unit rate is `freq × A` with `A = lcm(numerators)`. A only
        // ever grows, which is what makes the rescale below an exact integer.
        let old_a = self.oscillators[osc.index()].unit_mul;
        let mut a = old_a;
        for r in &ratios {
            a = checked_lcm(a, r.num()).ok_or_else(|| ClockError::LcmUnavailable {
                domains: names(self),
            })?;
        }

        // One tick of domain i is `(A / aᵢ) × bᵢ` unit ticks.
        let mut ks: Vec<u64> = Vec::with_capacity(order.len());
        for r in &ratios {
            ks.push((a / r.num()).checked_mul(r.den()).ok_or_else(|| {
                ClockError::LcmUnavailable {
                    domains: names(self),
                }
            })?);
        }

        let f = a / old_a;
        let new_units = self.oscillators[osc.index()]
            .units
            .checked_mul(f)
            .ok_or_else(|| ClockError::Overflow {
                what: "unit position rescale",
                domains: names(self),
            })?;
        let mut new_bases: Vec<u64> = Vec::with_capacity(order.len());
        for id in &order {
            new_bases.push(
                self.domains[id.index()]
                    .base_unit
                    .checked_mul(f)
                    .ok_or_else(|| ClockError::Overflow {
                        what: "unit position rescale",
                        domains: names(self),
                    })?,
            );
        }

        let unit_rate = self.oscillators[osc.index()]
            .freq
            .checked_mul(Rational::integer(a))
            .ok_or_else(|| ClockError::LcmUnavailable {
                domains: names(self),
            })?;

        // Commit.
        for (i, id) in order.iter().enumerate() {
            let d = &mut self.domains[id.index()];
            d.ratio = ratios[i];
            d.units_per_tick = ks[i];
            d.base_unit = new_bases[i];
        }
        let o = &mut self.oscillators[osc.index()];
        o.units = new_units;
        o.unit_mul = a;
        o.unit_rate = unit_rate;
        // A changed unit rate starts a new conversion segment anchored at the
        // timeline position already reached: time never moves backwards, and the
        // residual restarts from exact zero.
        o.base_units = new_units;
        o.base_time = o.time;
        o.residual = 0;
        o.recompute_conversion();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    /// The NES master crystal: 236250000/11 Hz, which is not an integer, which
    /// is the whole reason frequencies are rational.
    fn nes() -> (ClockForest, DomainId, DomainId, DomainId) {
        let mut f = ClockForest::new();
        let master = f
            .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
            .unwrap();
        let cpu = f.add_domain("cpu", master, 1, 12).unwrap();
        let ppu = f.add_domain("ppu", master, 1, 4).unwrap();
        (f, master, cpu, ppu)
    }

    /// A deterministic pseudo-random generator, so "irregular" chunk sizes are
    /// reproducible on every host. Numerical Recipes' 64-bit LCG.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            self.0
        }
    }

    /// `ticks × den × 2⁶⁴ / num`, computed independently of the implementation
    /// through a 192-bit intermediate. This is the reference the accumulator is
    /// checked against.
    fn exact_time(units: u64, num: u64, den: u64) -> u128 {
        let shifted = U192::mul_u128_u64(units as u128, den)
            .shl64()
            .expect("test inputs must fit");
        let (q, _) = shifted.div_u64(num);
        q.to_u128().expect("test inputs must fit")
    }

    #[test]
    fn rationals_reduce_and_compare_exactly() {
        let r = Rational::new(236_250_000, 11).unwrap();
        assert_eq!(r.num(), 236_250_000);
        assert_eq!(r.den(), 11);
        assert_eq!(Rational::new(4, 8).unwrap(), Rational::new(1, 2).unwrap());
        assert!(Rational::new(1, 3).unwrap() < Rational::new(1, 2).unwrap());
        assert_eq!(
            Rational::new(1, 0).unwrap_err(),
            ClockError::ZeroDenominator
        );
        assert_eq!(Rational::new(3, 6).unwrap().to_string(), "1/2");
    }

    #[test]
    fn u192_arithmetic_is_exact() {
        let p = U192::mul_u128_u64(u128::MAX, u64::MAX);
        let (q, r) = p.div_u64(u64::MAX);
        assert_eq!(r, 0);
        assert_eq!(q, U192([0, (u128::MAX >> 64) as u64, u128::MAX as u64]));
        assert_eq!(U192([0, 0, 1]).dec(), U192([0, 0, 0]));
        assert_eq!(U192([1, 0, 0]).dec(), U192([0, u64::MAX, u64::MAX]));
        assert_eq!(U192([0, 1, 0]).shr64(), U192([0, 0, 1]));
        assert_eq!(U192([0, 0, 1]).shl64(), Some(U192([0, 1, 0])));
        assert_eq!(U192([1, 0, 0]).shl64(), None);
    }

    #[test]
    fn nes_domains_derive_the_master_tick() {
        let (f, master, cpu, ppu) = nes();
        let osc = f.root_of(cpu).unwrap();
        // Every numerator is 1, so the tree's unit tick is the master tick and
        // the per-domain factors are the divisors themselves.
        assert_eq!(f.unit_rate(osc).unwrap(), f.frequency(osc).unwrap());
        assert_eq!(f.domain(master).unwrap().units_per_tick(), 1);
        assert_eq!(f.domain(cpu).unwrap().units_per_tick(), 12);
        assert_eq!(f.domain(ppu).unwrap().units_per_tick(), 4);
        assert_eq!(
            f.domain_frequency(cpu).unwrap(),
            Rational::new(236_250_000, 132).unwrap()
        );
    }

    /// The invariant every NES game depends on, asserted at every step rather
    /// than sampled: three dots per CPU cycle, for over 10⁹ CPU ticks.
    #[test]
    fn nes_cpu_ppu_ratio_is_exactly_three_to_one_forever() {
        let (mut f, _master, cpu, ppu) = nes();

        // Tick by tick for the first stretch: the ratio holds at every single
        // CPU cycle, not merely at chunk boundaries.
        for i in 1..=2_000_000u64 {
            f.advance_domain(cpu, 1).unwrap();
            assert_eq!(f.ticks(cpu).unwrap(), i);
            assert_eq!(f.ticks(ppu).unwrap(), 3 * i);
        }

        // Then in irregular chunks out past 10⁹ CPU ticks, asserting after every
        // advance. Nothing here samples: every state the forest passes through
        // is a state this checks.
        let mut rng = Lcg(0x5eed_1234_dead_beef);
        let mut cpu_ticks = 2_000_000u64;
        while cpu_ticks < 2_000_000_000 {
            let n = (rng.next() % 5_000) + 1;
            f.advance_domain(cpu, n).unwrap();
            cpu_ticks += n;
            assert_eq!(f.ticks(cpu).unwrap(), cpu_ticks);
            assert_eq!(f.ticks(ppu).unwrap(), 3 * cpu_ticks);
            // And the exact conversion agrees, without touching absolute time.
            assert_eq!(f.convert_ticks(cpu, ppu, cpu_ticks).unwrap(), 3 * cpu_ticks);
            assert_eq!(f.convert_ticks(ppu, cpu, 3 * cpu_ticks).unwrap(), cpu_ticks);
        }
        assert!(cpu_ticks > 1_000_000_000);
    }

    #[test]
    fn intra_tree_conversion_never_touches_absolute_time() {
        // A tree whose absolute frequency is deliberately absurd: the ratios are
        // still exact, because they are a property of the divisors, not the Hz.
        let mut f = ClockForest::new();
        let root = f
            .add_oscillator("odd", Rational::new(3, 7_919).unwrap())
            .unwrap();
        let a = f.add_domain("a", root, 1, 12).unwrap();
        let b = f.add_domain("b", root, 1, 4).unwrap();
        assert_eq!(f.convert_ticks(a, b, 1_000_003).unwrap(), 3_000_009);
    }

    #[test]
    fn cross_tree_conversion_is_refused_not_approximated() {
        let (mut f, _m, cpu, _ppu) = nes();
        let spc = f
            .add_oscillator("spc700", Rational::integer(24_576_000))
            .unwrap();
        let dsp = f.add_domain("dsp", spc, 1, 768).unwrap();
        assert_eq!(
            f.convert_ticks(cpu, dsp, 1).unwrap_err(),
            ClockError::CrossTree(cpu, dsp)
        );
    }

    /// The cross-tree claim: bounded below one unit, and non-accumulating over
    /// 10¹² ticks. Proven three ways at once — the incremental accumulator, the
    /// closed form, and an independent 192-bit reference all agree exactly.
    #[test]
    fn cross_tree_drift_is_bounded_and_non_accumulating() {
        let mut f = ClockForest::new();
        let master = f
            .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
            .unwrap();
        let rtc = f.add_oscillator("rtc", Rational::integer(32_768)).unwrap();
        let cpu = f.add_domain("cpu", master, 1, 12).unwrap();
        let _ = f.add_domain("sec", rtc, 1, 32_768).unwrap();
        let m_osc = f.root_of(master).unwrap();
        let r_osc = f.root_of(rtc).unwrap();
        let (m_num, m_den) = {
            let r = f.unit_rate(m_osc).unwrap();
            (r.num(), r.den())
        };

        let mut rng = Lcg(0x1234_5678_9abc_def0);
        let mut cpu_ticks = 0u64;
        // 12 unit ticks per CPU tick, so this is >10¹² master ticks.
        while cpu_ticks < 100_000_000_000 {
            let n = (rng.next() % 1_000_000) + 1;
            f.advance_domain(cpu, n).unwrap();
            cpu_ticks += n;

            let units = f.unit_position(m_osc).unwrap();
            assert_eq!(units, cpu_ticks * 12);

            // The accumulator matches the closed form, which matches the
            // independent reference. Error against true time is therefore the
            // truncation of a single floor: < 1 unit, and it never grows.
            let acc = f.global_time(m_osc).unwrap().raw();
            let closed = f.global_time_of_units(m_osc, units).unwrap().raw();
            assert_eq!(acc, closed);
            assert_eq!(acc, exact_time(units, m_num, m_den));
        }
        assert!(f.unit_position(m_osc).unwrap() > 1_000_000_000_000);

        // The independent crystal is untouched by any of that; advancing it to
        // the same instant is the only cross-tree operation, and it round-trips
        // to within the same single unit.
        let now = f.global_time(m_osc).unwrap();
        f.advance_to_global(r_osc, now).unwrap();
        let back = f
            .global_time_of_units(r_osc, f.unit_position(r_osc).unwrap())
            .unwrap();
        assert!(back <= now);
        // One RTC unit tick is 1/32768 s; landing within one tick of the target
        // is the whole claim, and it is a floor, not a drift.
        let one_tick = GlobalTime::from_raw((1u128 << 64) / 32_768);
        assert!(now.saturating_sub(back) < one_tick);
    }

    #[test]
    fn timeline_error_stays_below_one_unit_per_step() {
        // A frequency chosen so the per-tick step has a large remainder: 3 Hz
        // divides 2⁶⁴ with remainder 1, so a naive multiply-by-a-truncated-step
        // would lose a full unit every three ticks.
        let mut f = ClockForest::new();
        let osc_root = f.add_oscillator("awkward", Rational::integer(3)).unwrap();
        let osc = f.root_of(osc_root).unwrap();
        for n in 1..=3_000u64 {
            f.advance_units(osc, 1).unwrap();
            let t = f.global_time(osc).unwrap().raw();
            let exact = ((n as u128) << 64) / 3;
            assert_eq!(t, exact, "at tick {n}");
        }
    }

    #[test]
    fn lcm_failure_is_reported_and_names_the_domains() {
        // A guest programs a PLL to an arbitrary ratio: lcm(3, p) for a large
        // prime p does not fit in 64 bits, so no exact common unit tick exists.
        const P: u64 = 18_446_744_073_709_551_557; // largest prime < 2^64
        let mut f = ClockForest::new();
        let root = f.add_oscillator("xtal", Rational::integer(1_000)).unwrap();
        let a = f.add_domain("pll_a", root, 3, 1).unwrap();
        let err = f.add_domain("pll_b", root, P, 1).unwrap_err();
        match &err {
            ClockError::LcmUnavailable { domains } => {
                assert!(domains.iter().any(|d| d == "xtal"));
                assert!(domains.iter().any(|d| d == "pll_a"));
                assert!(domains.iter().any(|d| d == "pll_b"));
            }
            other => panic!("expected LcmUnavailable, got {other:?}"),
        }
        let text = err.to_string();
        assert!(text.contains("pll_a") && text.contains("pll_b"));

        // Nothing degraded silently: the forest is exactly as it was.
        assert_eq!(f.domain_count(), 2);
        assert_eq!(f.domain(a).unwrap().units_per_tick(), 1);

        // The same failure through a runtime re-rating, which is the case that
        // actually matters: a write handler must be able to refuse.
        let b = f.add_domain("pll_b", root, 5, 1).unwrap();
        let before = f.domain(b).unwrap().units_per_tick();
        assert!(matches!(
            f.set_rating(b, P, 1),
            Err(ClockError::LcmUnavailable { .. })
        ));
        assert_eq!(f.domain(b).unwrap().units_per_tick(), before);
        assert_eq!(f.domain(b).unwrap().mul(), 5);
    }

    #[test]
    fn re_rating_preserves_history_and_keeps_the_tree_exact() {
        let (mut f, master, cpu, ppu) = nes();
        f.advance_domain(cpu, 1_000).unwrap();
        assert_eq!(f.ticks(cpu).unwrap(), 1_000);
        assert_eq!(f.ticks(ppu).unwrap(), 3_000);

        // A guest halves the CPU divider: past ticks stay counted at the old
        // rate, and the new ratio is exact from here on.
        f.set_rating(cpu, 1, 6).unwrap();
        assert_eq!(f.ticks(cpu).unwrap(), 1_000);
        f.advance_domain(master, 60).unwrap();
        assert_eq!(f.ticks(cpu).unwrap(), 1_010);
        assert_eq!(f.ticks(ppu).unwrap(), 3_015);
    }

    #[test]
    fn a_finer_domain_refines_the_unit_without_disturbing_counters() {
        let (mut f, master, cpu, ppu) = nes();
        f.advance_domain(cpu, 500).unwrap();
        let osc = f.root_of(cpu).unwrap();
        let unit_rate_before = f.unit_rate(osc).unwrap();

        // A PLL at 5/2 of the master forces A = 5: the unit tick gets five times
        // finer and every existing counter is untouched.
        let pll = f.add_domain("pll", master, 5, 2).unwrap();
        assert_eq!(
            f.unit_rate(osc).unwrap(),
            unit_rate_before.checked_mul(Rational::integer(5)).unwrap()
        );
        assert_eq!(f.ticks(cpu).unwrap(), 500);
        assert_eq!(f.ticks(ppu).unwrap(), 1_500);
        assert_eq!(f.domain(cpu).unwrap().units_per_tick(), 60);
        assert_eq!(f.domain(ppu).unwrap().units_per_tick(), 20);
        assert_eq!(f.domain(pll).unwrap().units_per_tick(), 2);

        f.advance_domain(cpu, 10).unwrap();
        assert_eq!(f.ticks(ppu).unwrap(), 1_530);
        assert_eq!(f.ticks(pll).unwrap(), 300);
    }

    #[test]
    fn gating_stops_a_domain_without_stopping_its_tree() {
        let (mut f, _m, cpu, ppu) = nes();
        f.advance_domain(cpu, 100).unwrap();
        f.set_gated(cpu, true).unwrap();
        assert!(f.is_gated(cpu).unwrap());
        assert!(matches!(
            f.advance_domain(cpu, 1),
            Err(ClockError::Gated(_))
        ));

        // The PPU keeps running on the same crystal while the CPU is halted.
        f.advance_domain(ppu, 300).unwrap();
        assert_eq!(f.ticks(cpu).unwrap(), 100);
        assert_eq!(f.ticks(ppu).unwrap(), 600);

        f.set_gated(cpu, false).unwrap();
        f.advance_domain(cpu, 10).unwrap();
        assert_eq!(f.ticks(cpu).unwrap(), 110);
    }

    #[test]
    fn locking_an_oscillator_makes_the_relationship_exact() {
        let mut f = ClockForest::new();
        let master = f
            .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
            .unwrap();
        let cpu = f.add_domain("cpu", master, 1, 12).unwrap();
        let spc_root = f
            .add_oscillator("spc700", Rational::integer(24_576_000))
            .unwrap();
        let spc = f.root_of(spc_root).unwrap();
        let dsp = f.add_domain("dsp", spc_root, 1, 2).unwrap();

        // Loose by default: the two crystals are unrelated.
        assert!(f.convert_ticks(cpu, dsp, 1).is_err());

        // The machine declares a lock, and from then on the ratio is exact by
        // the same construction as any other intra-tree ratio.
        f.lock_oscillator(spc, master, 32, 21).unwrap();
        assert!(!f.is_active(spc).unwrap());
        assert_eq!(f.root_of(dsp).unwrap(), f.root_of(cpu).unwrap());
        let n = f.convert_ticks(cpu, dsp, 21).unwrap();
        assert_eq!(n, 21 * 12 * 16 / 21);
    }

    #[test]
    fn reparenting_rejects_cycles() {
        let (mut f, master, cpu, _ppu) = nes();
        assert!(matches!(
            f.reparent(master, cpu, 1, 1),
            Err(ClockError::Cycle(_))
        ));
        assert!(matches!(
            f.reparent(cpu, cpu, 1, 1),
            Err(ClockError::Cycle(_))
        ));
    }

    #[test]
    fn unknown_handles_are_errors_not_panics() {
        let (f, _m, _c, _p) = nes();
        let bogus = DomainId(99);
        assert_eq!(
            f.ticks(bogus).unwrap_err(),
            ClockError::UnknownDomain(bogus)
        );
        let bogus_osc = OscillatorId(99);
        assert_eq!(
            f.frequency(bogus_osc).unwrap_err(),
            ClockError::UnknownOscillator(bogus_osc)
        );
    }

    #[test]
    fn clock_errors_convert_into_the_crate_error() {
        let e: crate::core::Error = ClockError::LcmUnavailable {
            domains: alloc::vec!["cpu".to_string(), "ppu".to_string()],
        }
        .into();
        let text = alloc::format!("{e}");
        assert!(text.contains("cpu") && text.contains("ppu"));
    }

    #[test]
    fn a_restored_forest_is_identical_not_merely_close() {
        let (mut f, _m, cpu, ppu) = nes();
        f.advance_domain(cpu, 1_234_567).unwrap();
        let osc = f.root_of(cpu).unwrap();
        let saved_units = f.unit_position(osc).unwrap();
        let saved_cpu = f.ticks(cpu).unwrap();
        let saved_ppu = f.ticks(ppu).unwrap();
        let saved_time = f.global_time(osc).unwrap();

        // A fresh forest of the same topology, restored from the counters alone.
        let (mut g, _m2, cpu2, ppu2) = nes();
        let osc2 = g.root_of(cpu2).unwrap();
        g.restore_unit_position(osc2, saved_units).unwrap();
        g.restore_ticks(cpu2, saved_cpu).unwrap();
        g.restore_ticks(ppu2, saved_ppu).unwrap();

        assert_eq!(g.ticks(cpu2).unwrap(), saved_cpu);
        assert_eq!(g.ticks(ppu2).unwrap(), saved_ppu);
        // The timeline is derived, and it comes back exactly — not within a
        // unit, exactly.
        assert_eq!(g.global_time(osc2).unwrap(), saved_time);

        // And it stays exact afterwards: the residual was restored too.
        f.advance_domain(cpu, 99_991).unwrap();
        g.advance_domain(cpu2, 99_991).unwrap();
        assert_eq!(g.global_time(osc2).unwrap(), f.global_time(osc).unwrap());
        assert_eq!(g.ticks(ppu2).unwrap(), f.ticks(ppu).unwrap());
    }

    #[test]
    fn global_time_round_trips_through_ticks() {
        let (mut f, _m, cpu, _p) = nes();
        f.advance_domain(cpu, 1_789_773).unwrap();
        let osc = f.root_of(cpu).unwrap();
        let t = f.global_time(osc).unwrap();
        // About one second of NES CPU time.
        let ns = t.as_nanos();
        assert!((999_000_000..=1_001_000_000).contains(&ns), "{ns}");
        assert_eq!(
            f.units_at_global(osc, t).unwrap(),
            f.unit_position(osc).unwrap()
        );
        assert_eq!(f.global_time_of_tick(cpu, 1_789_773).unwrap(), t);
    }

    #[test]
    fn nanosecond_conversions_are_integer_and_never_overshoot() {
        assert_eq!(GlobalTime::from_nanos(0), GlobalTime::ZERO);
        assert_eq!(
            GlobalTime::from_nanos(1_000_000_000),
            GlobalTime::from_raw(1u128 << 64)
        );
        // Both directions floor, so a round trip can land one nanosecond low and
        // never one high. Rate control only ever needs the "not yet due" side of
        // that, which is why flooring is the right choice in both directions.
        for ns in [1u64, 999, 123_456_789, 4_000_000_000] {
            let back = GlobalTime::from_nanos(ns).as_nanos();
            assert!(back == ns || back == ns - 1, "{ns} -> {back}");
        }
        assert_eq!(GlobalTime::MAX.as_nanos(), u64::MAX);
    }
}
