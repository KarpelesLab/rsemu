//! Networking: the seam a NIC model talks to, and the NICs.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`link`] | `dev-net` | the seam: frames out, frames in against a virtual tick, link state, MAC address — plus [`NetPort`], the deterministic in-memory backend |
//! | [`ne2000`] | `dev-ne2000` | a Novell NE2000 card: a DP8390 NIC, 16 KiB of buffer RAM, the data window and the reset strap |
//! | [`pktkit`] | `net-pktkit` | the bridge to `pktkit`: any `L2Device` — a hub, a TAP, slirp, a WireGuard tunnel — as a [`NetLink`] |
//!
//! # The seam is not `pktkit::L2Device`, and that is on purpose
//!
//! `ROADMAP.md` §7.2 promises that every emulated NIC *is* a
//! `pktkit::L2Device`. It cannot be: `L2Device` delivers a received frame by
//! calling a handler on whichever host thread produced it, and an emulated
//! machine has no defined position in virtual time at that instant. A NIC that
//! accepted frames there would put them in the guest's receive ring at a
//! different guest cycle on every run — the exact non-determinism `CLAUDE.md`
//! forbids, and the one that makes a state hash worthless.
//!
//! So [`link::NetLink`] is a **pull**: an arriving frame is queued against a
//! virtual tick and the NIC takes it out at a tick the scheduler chose. A
//! `pktkit::L2Device` is then one implementation of that seam ([`pktkit`]) and
//! everything `pktkit` offers — hubs, slirp, TAP, WireGuard, pcap — is reachable
//! through it unchanged. What is *not* reinvented here is packet parsing:
//! nothing in this module knows what ARP or IP is, because that is `pktkit`'s
//! job.
//!
//! # `std`
//!
//! `ROADMAP.md` §0 and `CLAUDE.md` both list `dev/net/*` as a documented `std`
//! exception "because `pktkit` is a `std` crate". Only [`pktkit`] is: the seam
//! and the NE2000 are `no_std + alloc` and dependency-free, so an NE2000 on a
//! Z80 board builds in a `no_std` configuration and is tested there by CI's
//! feature sweep. The exception is real but it is one file wide.
//!
//! [`NetPort`]: link::NetPort
//! [`NetLink`]: link::NetLink

pub mod link;

#[cfg(feature = "dev-ne2000")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-ne2000")))]
pub mod ne2000;

#[cfg(feature = "net-pktkit")]
#[cfg_attr(docsrs, doc(cfg(feature = "net-pktkit")))]
pub mod pktkit;
