# rsemu

**A generic, multiplatform emulator framework in pure Rust.**

rsemu is not an emulator — it is the machinery emulators are built from:
address spaces, clock domains, wires, devices, buses, and a translation IR,
with concrete components (CPU cores, PCI, USB, storage, NICs) layered on top,
and a **machine description language** that wires arbitrary topologies together
at runtime.

If you want a NES, you write a `.machine` file. If you want four heterogeneous
CPUs sharing one RAM region across three bus fabrics, you write a stranger
`.machine` file. The framework does not care which.

## Design principles

- **Pure Rust, no foreign code.** No C, no FFI, no vendored assembly, no build
  scripts. The default build has an empty `cargo tree`; dependencies are
  first-party Karpelès Lab crates only, and every one is feature-gated.
- **`unsafe` is quarantined.** `unsafe_code = "deny"` crate-wide. Only the RAM
  host-pointer fast path, the JIT code buffer, the raw-syscall accel backends,
  and the C ABI opt back in — each scoped, each with a safety comment.
- **Determinism is a mode, not an accident.** Deterministic runs are
  bit-reproducible across hosts and across execution engines, which is what
  makes save states, record/replay, rewind, and the regression suite possible.
- **Accuracy is measured.** Every CPU core ships with a published conformance
  suite and a known-failures ledger that only ever shrinks.
- **Generic first.** A device that needs a new mechanism gets it added to the
  core generically. No device type ever appears in a `core::` signature.
- **`no_std` + `alloc` core.** Host I/O, JIT, acceleration and frontends live
  above the `std` line.
- **One crate, one feature per component.** A NES build links a 6502 and
  nothing else.

## Status

Planning. Nothing is implemented yet.

**Read [`ROADMAP.md`](ROADMAP.md)** — it contains the architecture (memory,
time, devices, state, IR), the machine description language, the phase plan
with acceptance gates, and the design invariants.

## Built on

[`pktkit`](https://github.com/KarpelesLab/pktkit-rs) (all networking),
[`compcol`](https://github.com/KarpelesLab/compcol) (image + snapshot
compression), [`purecrypto`](https://github.com/KarpelesLab/purecrypto)
(disk/snapshot encryption, emulated crypto devices),
[`noroi`](https://github.com/KarpelesLab/noroi) (monitor TUI).

## License

MIT — see [LICENSE](LICENSE).
