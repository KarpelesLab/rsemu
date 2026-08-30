//! Crate-wide error and result types.

use alloc::string::String;
use core::fmt;

/// The result type used throughout rsemu.
pub type Result<T> = core::result::Result<T, Error>;

/// Everything that can go wrong, from config parsing to a guest bus fault.
///
/// Deliberately one enum rather than a per-module hierarchy: an emulator error
/// almost always crosses layers on its way to the user, and a single type keeps
/// the `?` operator usable without a web of `From` impls.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// A machine description could not be parsed or resolved.
    ///
    /// Carries a human-readable location because a config error the user cannot
    /// locate is a config error they cannot fix (`ROADMAP.md` §5).
    Config {
        /// Where the problem is, as `file:line:col` where available.
        at: String,
        /// What is wrong.
        message: String,
    },
    /// A device class was requested that this build does not contain.
    ///
    /// Usually a missing Cargo feature rather than a typo, so the message says
    /// so.
    UnknownClass(String),
    /// A property was missing, of the wrong type, or out of range.
    Property(String),
    /// A guest access could not be completed.
    Bus(BusError),
    /// A snapshot could not be written or restored.
    State(String),
    /// The operation is not implemented in this build yet.
    ///
    /// Distinct from an error: it means "rsemu has not got here", not "you did
    /// something wrong". Scaffolding returns this a lot.
    Unimplemented(&'static str),
}

/// Why a guest memory or I/O access failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BusError {
    /// Nothing is mapped at the address, and the space's policy is to fault.
    Unassigned,
    /// The access width or alignment is not permitted here.
    ///
    /// A 32-bit-only register must reject a byte write rather than silently
    /// accept it (`ROADMAP.md` §4.1).
    BadAccess,
    /// The target is busy; the access may be retried.
    ///
    /// Only legal *before* any side effect or partial transfer — a retry that
    /// re-runs a half-completed multi-byte access is a correctness bug, so the
    /// dispatcher rejects this after first commit.
    Retry,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Config { at, message } => write!(f, "{at}: {message}"),
            Error::UnknownClass(name) => {
                write!(f, "unknown device class `{name}` (is its feature enabled?)")
            }
            Error::Property(message) => write!(f, "invalid property: {message}"),
            Error::Bus(e) => write!(f, "bus error: {e}"),
            Error::State(message) => write!(f, "snapshot error: {message}"),
            Error::Unimplemented(what) => write!(f, "not implemented yet: {what}"),
        }
    }
}

impl fmt::Display for BusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            BusError::Unassigned => "nothing mapped at this address",
            BusError::BadAccess => "access width or alignment not permitted",
            BusError::Retry => "target busy, retry",
        };
        f.write_str(s)
    }
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Error::Bus(e)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn unknown_class_hints_at_the_likely_cause() {
        // The common case is a disabled Cargo feature, not a typo, so the
        // message has to say so or every report becomes a support round-trip.
        let e = Error::UnknownClass("pci.nvme".to_string());
        let text = e.to_string();
        assert!(text.contains("pci.nvme"));
        assert!(text.contains("feature"));
    }

    #[test]
    fn config_errors_lead_with_their_location() {
        let e = Error::Config {
            at: "nes.machine:12:5".to_string(),
            message: "unknown property `clok`".to_string(),
        };
        assert!(e.to_string().starts_with("nes.machine:12:5: "));
    }

    #[test]
    fn bus_errors_convert() {
        let e: Error = BusError::Unassigned.into();
        assert_eq!(e, Error::Bus(BusError::Unassigned));
    }
}
