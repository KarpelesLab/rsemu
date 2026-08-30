//! The emulation core: the generic machinery every machine is built from.
//!
//! This module is **never feature-gated** and is `no_std + alloc`. Nothing here
//! may name `std::thread`, `std::sync`, or the host clock directly
//! (`ROADMAP.md` §15, invariant 4).
//!
//! # What belongs here
//!
//! Per `ROADMAP.md` §4, in roughly the order it will be built:
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`error`] | crate-wide error and result types |
//! | [`value`] | access widths, endianness, typed conversions |
//! | `space` | address spaces, regions, flat views, dispatch (§4.1) |
//! | `clock` | the oscillator forest and virtual time (§4.2) |
//! | `sched` | event queue, execution budgets, threading modes (§4.2) |
//! | `wire` | interrupt and GPIO lines (§4.3) |
//! | `device` | the device trait, lifecycle, composition (§4.4) |
//! | `props` | dynamic property values and typed extraction (§4.4) |
//! | `registry` | by-name device construction (§4.4) |
//! | `state` | versioned snapshots (§4.5) |
//! | `sync` | the concurrency portability seam (§4.7) |
//!
//! The module tree is declared up front so that the shape of the core is a
//! settled decision rather than one rediscovered per module; several are still
//! stubs.

pub mod clock;
pub mod device;
pub mod error;
pub mod props;
pub mod registry;
pub mod sched;
pub mod space;
pub mod state;
pub mod sync;
pub mod value;
pub mod wire;

pub use error::{BusError, Error, Result};
pub use value::{Endian, Width};
