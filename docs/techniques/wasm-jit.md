# The JIT without `mmap`: a WebAssembly backend

Consumed by: `src/jit/wasm/`, `ROADMAP.md` §11.4 and §11.5, phase 11.
Companion to [`webassembly.md`](webassembly.md), which is about running *rsemu*
as wasm; this one is about rsemu *emitting* it.

Sources are the **WebAssembly Core Specification** (W3C Recommendation 2.0) —
§2 for the structure, §4 for execution semantics, §5 for the binary format —
and the **WebAssembly JavaScript Interface** and **Web API** recommendations for
the embedder half. All open standards, which is why this subsystem could be
written at all: the alternative references for "how does one emit wasm" are
copyleft toolchains, and CLAUDE.md's provenance rule outranks convenience.

## The problem, in one paragraph

Both native backends do the same thing: `mmap` a buffer, write host
instructions into it, `mprotect` it executable, and jump to it through a
function pointer. wasm has no step in that sequence. There is no
writable-then-executable memory, there is no instruction encoding the engine
will execute from linear memory, and there is no way to take the address of
generated code. The only mechanism the platform offers is to hand a *module*
to the embedder and get back something callable — which means the unit of
generated code is a module, the call into it goes through the embedder, and
nothing inside it can ever be patched afterwards.

Everything below follows from that.

## What was kept from §11.4, and what changed

§11.4 is short and specific. Taking it clause by clause.

| §11.4 says | What was built |
| --- | --- |
| "IR → wasm bytecode module → `WebAssembly.Module`" | kept: `jit::wasm::compile` emits a complete module per block |
| "a translation block is a wasm function" | kept: one exported function, `(param i32 i32) (result i64)` |
| "guest RAM is the shared linear memory" | **changed**, see below |
| "helper calls are imports" | kept, and widened: *every* host interaction is an import |
| "dispatched through a function table" | **changed**: the dispatcher re-enters in Rust, not through a table |
| "block chaining is impossible here" | kept, and it is |
| "only tiers up superblocks" | **not yet**: there is no tiering heuristic, because there is nothing to measure one against |
| "module count is bounded with an LRU eviction" | kept: `jit::wasm::rt::Engine` is a bounded slot table with LRU eviction |
| "the portable IR interpreter is always the fallback" | kept: a refused block is interpreted, and the refusal names why |

Two changes need their reasons stated.

### "Guest RAM is the shared linear memory" — nearly, but not through the module

The sentence is right about where guest RAM *lives* and wrong about how a
generated block reaches it. Guest RAM in a wasm build is a `RamStore`, which
sits in rsemu's own linear memory and is addressed by byte offset precisely so
it can live in a `SharedArrayBuffer` (CLAUDE.md, "Targets"). A generated module
importing that memory could load from it directly — but only after resolving a
*guest-physical* address to an offset, which is the software TLB's job, and only
for the pages the TLB is willing to hand out at all. Rebasable leaves, MMIO,
big-endian regions and misaligned splits are all cases where the answer is "call
the host".

So this round emits a call for every guest access, and the module imports linear
memory for one thing only: the **temporary frame**, which is where a load's
result and a published temporary cross back. That keeps the byte-offset rule
intact — nothing hands out a `&mut [u8]` and nothing bakes a host pointer into a
module — and it leaves the inlined probe as a measurable increment rather than a
speculative one. `ROADMAP.md` §9.1's first mechanism is the biggest single win
on a native host; whether it is one here depends on the cost of the import call
it avoids, which is the embedder's business and is not knowable from inside the
crate. Measuring it needs an embedder. Inventing the number does not.

### "Dispatched through a function table"

A table dispatch is how an *embedder* re-enters generated code: JS puts every
compiled function into one `WebAssembly.Table` and a shared trampoline module
does `call_indirect` on it, so a block's successor can be reached without a JS
call per block. That is the right design for the browser and it is in the ABI
below.

It is not what a Rust caller does. `jit::dispatch::Dispatcher` already holds the
block cache, the epoch counters and the SMC log, and it already enters a block
and looks at an `Outcome`. The wasm engine plugs in exactly there —
`Engine::run(block, code, host) -> Option<Result<Outcome>>`, the same signature
`jit::x86::rt::Engine::run` has — and the dispatcher's own loop chooses the
successor. On a native host that is all there is. In a browser the same
`Outcome` would come back through the embedder instead, and the table is what
makes that cheap.

## The module

One block, one module. The alternatives were considered and rejected for now:

* **A module per trace or superblock.** Better, and it is what §11.4's "only
  tiers up superblocks" is pointing at — instantiation is amortised over more
  guest work. It is a *frontend* question first: superblock formation is
  `cpu::*::lift`'s `Shape`, and this backend compiles whatever block it is
  given, so it gets this for free the day the frontend produces one.
* **A growing module re-instantiated as blocks are added.** Re-instantiating
  invalidates every handle into the old instance, so the cost is paid on every
  block rather than amortised, and the module grows without bound. Worse on both
  axes.
* **One module with an internal dispatch loop over a table.** This is the
  design that would actually beat the others in a browser: guest work stays
  inside wasm across block boundaries, and the `call_indirect` in §11.4 becomes
  a real mechanism rather than a re-entry. It needs the embedder, a table, and
  a growth strategy, and it needs the numbers from the simple version first.

Per-block also makes invalidation trivial, which is the next section.

### Shape

```wat
(module
  (type $blk  (func (param i32 i32) (result i64)))          ;; ctx, frame -> status
  (type $slot (func (param i32 i32) (result i64)))
  (type $ld   (func (param i32 i32 i64 i32) (result i32)))
  (type $st   (func (param i32 i32 i64 i64 i32) (result i32)))
  (type $note (func (param i32 i32 i64) (result i32)))
  (import "e" "g" (func $slot (type $slot)))                ;; get_slot
  (import "e" "l" (func $ld   (type $ld)))                  ;; load
  (import "e" "s" (func $st   (type $st)))                  ;; store
  (import "e" "n" (func $note (type $note)))                ;; charge / insn_start
  (import "e" "m" (memory 1))
  (func $b (type $blk) …)
  (export "b" (func $b)))
```

`$ctx` is opaque to generated code and handed back unexamined to every import.
`$frame` is a byte offset into the imported memory:

```text
frame + 0            the out word: a successor PC, or a load's result
frame + 8 + 8*n      temporary n
```

The result is a status, and the numbers are `jit::x86::rt::status`'s so a reader
of one backend knows the other: 0 exit, 1 goto, 2 lookup, 3 fault, 4 spent, plus
5 for "report this as an `Err`", which is new because a wasm function cannot
return a `Result` and `Interp` has two conditions that are one.

### Why locals, not a register allocator

`ir::linear_scan` is shared between the two native backends because everything
it decides — live ranges, spill choices, which intervals cross a call — is a
property of the block rather than of a host. This backend does not call it at
all. wasm functions declare as many locals as they like and the *engine's* own
backend allocates registers for them, so every IR temporary is an `i64` local
and the allocator would be deciding something twice. That is the single largest
simplification the target buys, and it is most of why `jit::wasm` is about a
third the size of either native backend.

The consequence is the **write-through set**. A temporary in a local is
invisible to Rust once the function returns, so any temporary an `InsnStart`
names is also stored to the frame at its definition — three extra instructions,
at definitions only. That is the same write-through both native backends use to
make a fault precise; here it is also the only channel through which
architectural state gets published at all.

### One value representation

Every temporary is an `i64` holding the value canonically masked to its type,
exactly as `Interp` holds it. An `i32` operation is an `i64` operation plus a
mask. The two native backends each had to write down what "canonical" means for
their sub-register aliasing; here there is one width and the rule is uniform,
and the cost is one instruction an operation.

## Chaining and invalidation without a patchable jump

**Chaining.** There is none, and §11.4 is right that this removes the
second-largest win in §9's list. Concretely: `jit::dispatch::Chain` — the direct
link where a predecessor's compiled code jumps into a successor's without
returning to Rust — does not exist for this backend and cannot. What is *kept*
is everything the block cache does above the backend, because none of it knows
how a block executes: the `(pc, key)` lookup, the patched cache-level exits, the
FIFO eviction, the dirty-page filter. A `goto_tb` costs a return to
`Dispatcher::run` and a cache lookup, which is what `engine = "jit"` — the
portable IR backend — already costs today, and that engine is measured at 1.18×
the interpreter on a RISC-V Linux boot. So the ceiling this design gives up is
real but the floor is not zero.

**Invalidation.** The native backends invalidate by *unpatching* a
predecessor's jump. There is nothing to unpatch here and nothing needs to be,
because a module is only ever reached through the block cache. Three things drop
a block, and all three already existed:

1. a **guest store** into the page the block was lifted from, via the dirty log
   drained at every block boundary;
2. a **topology bump**, which flushes the whole cache (`jit::mod`'s staleness
   table);
3. **eviction** from this backend's own module table, which bumps that slot's
   generation so the `CodeRef` naming it stops answering `is_live` and the
   dispatcher compiles again.

There is no window in which a stale module is reachable, because reachability is
decided one level up. This is strictly simpler than the native answer, and it is
the one place where having no patchable code is an advantage.

## Instantiation cost, and when compiling is worth it

§11.4: *"per-module instantiation overhead makes tiny blocks a loss, so the
wasm backend only tiers up superblocks"*. The mechanism for bounding the damage
is here — a fixed slot table with LRU eviction, so a program with a large cold
working set cannot make the embedder hold thousands of modules — but the
**tiering heuristic is not**, and it should not be until there is a number.

The shape of the decision is worth writing down even so. Compiling a block is
worth it when

```text
instantiate + n × compiled_cost  <  n × interpreted_cost
```

for the number of times `n` that block runs before it is invalidated or evicted.
Everything on the left except `instantiate` is measurable in-crate today;
`instantiate` is a property of the embedder, and published figures for small
modules in mainstream engines are in the tens of microseconds — which against an
IR block of a few dozen ops means `n` in the thousands before compiling pays.
That is a strong argument that per-block modules will not win in a browser and
that §11.4's own pessimism is correct, and it is exactly why the superblock and
the whole-trace module are written down above rather than dismissed.

What that argues for, concretely, once an embedder exists: a run counter on the
cache entry, a threshold, and compilation deferred until a block crosses it.
`BlockCache` has nowhere to put the counter today, which is the one structural
change the tiering work would need.

## The embedder seam

Instantiation is the host's job and cannot be done from `no_std` core code.
`core/`, `ir/` and `cpu/` are untouched by all of this; `jit/wasm/` is
`no_std + alloc` and contains no host call of any kind. The seam is therefore
**data**: `jit::wasm::abi` is the whole contract — the import names, the
signatures, the frame layout and the status codes — and an implementation is
whatever can turn those bytes into something callable.

There are two, and only the first is built.

### The reference executor, which every build has

`jit::wasm::exec` is a WebAssembly interpreter over exactly the subset the code
generator emits. It is **not a speed path**: interpreting wasm generated from IR
is slower than interpreting the IR, so `engine = "jit-wasm"` on a native host is
the slowest of the four engines. What it buys is that the translation is
*executed* on every target rather than merely encoded, which is what lets
`tests/riscv_virt_engines.rs` assert the state hash across this backend on the
x86-64 runner that gates every commit. `jit::arm64` cannot make that claim: its
`mod executed` compiles only on an aarch64 runner, and until that job runs it,
the strongest claim available there is "it emits the instructions the manual
says it does".

### The browser embedder, which is specified here and not built

Nothing in this round ships JavaScript. What it would take is small, and writing
it down is most of the work, so here it is.

**rsemu exports** (added to `src/wasm.rs`, the existing `ffi` boundary):

| export | signature | what it does |
| --- | --- | --- |
| `rsemu_jit_slot` | `(i32, i32) -> i64` | `Thunks::call(func::SLOT, …)` |
| `rsemu_jit_load` | `(i32, i32, i64, i32) -> i32` | `Thunks::call(func::LOAD, …)` |
| `rsemu_jit_store` | `(i32, i32, i64, i64, i32) -> i32` | `Thunks::call(func::STORE, …)` |
| `rsemu_jit_note` | `(i32, i32, i64) -> i32` | `Thunks::call(func::NOTE, …)` |

**Embedder imports** rsemu would declare, following §11.5's convention
(`rsemu.now`, `rsemu.random_get`, `rsemu.log`):

| import | signature | what it does |
| --- | --- | --- |
| `rsemu.jit_compile` | `(ptr: i32, len: i32) -> i32` | `new WebAssembly.Module(bytes)`, instantiated against the import object below; returns a handle, or 0 if the engine refused |
| `rsemu.jit_enter` | `(handle: i32, ctx: i32, frame: i32) -> i64` | calls that instance's `b` export |
| `rsemu.jit_release` | `(handle: i32)` | drops the instance, on eviction |

**The JS glue is wiring and no logic** — which is the point of putting the
semantics in `Thunks` rather than in the embedder:

```js
const imports = { e: {
  m: rsemu.exports.memory,
  g: rsemu.exports.rsemu_jit_slot,
  l: rsemu.exports.rsemu_jit_load,
  s: rsemu.exports.rsemu_jit_store,
  n: rsemu.exports.rsemu_jit_note,
}};
```

Two things stand between that and a commit, and both are stated rather than
skipped:

* **`ctx` becomes a real pointer.** A thunk export receives an `i32` and has to
  turn it back into `&mut Ctx`. That is `unsafe`, and it belongs to the `ffi`
  site CLAUDE.md already sanctions (`src/wasm.rs` carries
  `#![allow(unsafe_code)]` for exactly this) — so it is not an eighth site, but
  it is a review, and shipping it untested would be the wrong order.
* **It cannot be tested in CI as things stand.** `check.sh wasm` builds the
  three wasm targets and `web/check.mjs` drives the built site headlessly under
  node; an end-to-end JIT test would extend the latter. That is the right place
  for it and it is a piece of work with its own shape.

### What WASI would need, and why it does not have it

`ROADMAP.md` §11.5 says *"Under WASI the same functions bind to preview-1
imports instead"*. That is true of `rsemu.now` and `rsemu.random_get`, which are
`clock_time_get` and `random_get`. It is **not true of `rsemu.compile`**, and
this is worth being exact about: WASI preview 1 has no interface for compiling
or instantiating a module, and neither does preview 2 — module instantiation is
a *component model* concern and is not something a preview-1 command module can
ask its host for. There is no standard import to bind to.

So: **a plain `wasm32-wasip1` or `wasm32-wasip1-threads` build has no wasm JIT
and cannot have one**, and would fall back to the reference executor, which
means running a wasm interpreter inside a wasm interpreter — correct, and
pointless. A WASI *runtime* could of course offer a non-standard import
(wasmtime and wasmer both let an embedder add one), and rsemu's side of that is
the same three imports above; but that is a runtime-specific extension, not
WASI, and calling it WASI support would be a lie. The supported embedder is the
browser, and `wasm32-unknown-unknown` is its target.

## Validating the encoding

The reference executor is written from the specification, and it is the only
thing that executes these modules in-crate — so on its own it validates the code
generator against *this project's reading* of §4 and §5 and nothing more. Two
things narrow that gap:

* The executor **rejects** every byte outside the emitted subset rather than
  guessing, so a code generator that emitted an opcode nobody implemented fails
  loudly instead of quietly.
* `tests/wasm_jit_v8.rs` hands the emitted modules to a **real engine**: when
  `node` is on `PATH` it runs `WebAssembly.validate` over every module the
  backend produces for a corpus of blocks, and instantiates and runs one against
  JS stub imports, comparing the status and the frame against the reference
  executor's. It skips when node is absent, so it is not a build dependency —
  and when it runs, a disagreement is a finding about the encoder or about the
  executor, either of which is worth knowing.

That is the honest state: the encoding is checked against V8 where V8 is
available, and the *semantics* of a compiled block are checked against
`ir::Interp` everywhere.

## Determinism

`ROADMAP.md` §0 requires a bit-identical state hash across the interpreter and
the JIT for the same guest. Two places in this backend are where that would be
lost, and both are handled rather than hoped for.

**Shifts.** wasm reduces a shift count modulo the operand width (§4.4.1);
`Interp` takes the mathematical answer. The IR calls an out-of-range shift
undefined, and both readings are legal — but *undefined* is not *whatever the
host does* when the same block may run compiled on one pass and interpreted on
the next, and a `jit-wasm` board and an `interp` board have to hash the same.
So `compile` selects the interpreter's answer explicitly, at three instructions
per shift: the in-range result, the out-of-range result, and a `select`.

This is a place where this backend is *stricter* than the two native ones, which
inherit their host's masking. That is not an inconsistency to fix by relaxing
here — the frontends guard their shifts, so nothing reaches it — but it is worth
noticing that the native backends are relying on the frontend where this one is
not.

**Floating point.** There is no float instruction in the emitted subset at all,
and both float types are refused outright. Guest FP is a helper call into
`float::soft` (§9.1) precisely so a guest's NaN payloads and rounding never
become the host's, and a wasm engine is a host like any other — arguably a worse
one, since its `f64` is the platform's.

The evidence is three layers deep:

1. `src/jit/wasm/tests.rs` — the IR differential: blocks over the compiled
   opcode set, run against `Interp` and against a generated module over two
   identical hosts, compared on every observable temporary, every guest slot,
   the tick count, guest memory, the boundary count and the outcome.
2. `src/cpu/riscv/engine.rs`'s `agree` — every fixture in that file, which is a
   dozen including faults, misaligned accesses, page-table walks, interrupts and
   self-modifying code, now runs under `Engine::JitWasm` as well as the other
   two.
3. `tests/riscv_virt_engines.rs` — the machine-level gate: one board, four
   engines, a state hash at ten checkpoints, plus a snapshot taken under one
   engine and carried on under another.

## `unsafe`

**None.** Not a block, not an `#[allow]`. Both native backends opt into
CLAUDE.md's second sanctioned site — the JIT code buffer — because machine code
has to be reached through a raw pointer and has to reconstitute `&mut`
references to call back. A module is a byte vector, entering it is a safe
function call, and an import's arguments are integers. The seven sanctioned
sites are untouched and there is no eighth.

The browser embedder above would use the *existing* `ffi` site, not a new one.

## What is not measured

The number §11.4 actually asks for — is a wasm module faster than the IR
interpreter in a browser — is not in this document, because taking it needs the
embedder that is specified above and not built. Saying otherwise would be
inventing it, which is the one thing a document like this must not do.

What *is* known is the native-host number, and it is worth having because it
bounds the reference executor rather than the backend. Four engines, the same
`riscv-virt` board with the same 12-instruction RV64I firmware, 400 quanta
after a 20-quantum warm-up, release build, one x86-64 Linux machine:

| Engine | 400 quanta | versus `interp` |
| --- | --- | --- |
| `interp` | 11.81 s | 1.00× |
| `jit` (portable IR backend) | 6.53 s | 1.81× |
| `jit-host` (`jit::x86`) | 1.47 s | **8.06×** |
| `jit-wasm` | 21.89 s | **0.54×** |

So `jit-wasm` on a native host is about **1.85× slower than interpreting the
guest directly**, which is what "a block interpreted twice over" costs and is
exactly the shape predicted above. It is not a defect and it is not a target to
optimise: every one of those 21.89 seconds is `jit::wasm::exec` decoding
bytecode that a real engine would have compiled once. The only number that
would say anything about the backend itself is the same board in a browser with
the embedder wired, and that is the measurement this work does not have.

One practical consequence, since this engine is in a per-commit gate:
`tests/riscv_virt_engines.rs` costs about twelve seconds more with `jit-wasm`
in it than without, over a sixty-second test. That is the price of the
determinism evidence and it is worth paying.
