# The JIT without `mmap`: a WebAssembly backend

Consumed by: `src/jit/wasm/`, `src/wasm.rs`, `web/src/jit.js`,
`web/check.mjs`, `benches/wasm_jit_embedder.rs`, `ROADMAP.md` §11.4 and §11.5,
phase 11. Companion to [`webassembly.md`](webassembly.md), which is about
running *rsemu* as wasm; this one is about rsemu *emitting* it.

**Where this stands.** The backend, the reference executor and the browser
embedder are all built. `engine = "jit-wasm"` runs a guest inside modules the
page's own engine compiled, hashes identically to the interpreter doing it, and
is **1.50× the IR interpreter** in V8 — see "The measurement", which is the
number `ROADMAP.md` §11.4 asked for and did not have.

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
| "only tiers up superblocks" | **not needed for the win, still wanted**: there is no tiering heuristic, and the backend beats the interpreter in a browser without one — see "The measurement". The heuristic is what a large cold working set needs, and it is now a measurable question rather than a guess |
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

There are two, and both are built. The seam between them is
`jit::wasm::embed::Embedder`:

```rust
pub trait Embedder: Sync + Debug {
    fn compile(&self, module: &[u8]) -> u32;                                   // handle, or REFUSED
    fn enter(&self, handle: u32, mem: &mut [u8], env: &mut dyn Env) -> Option<i64>;
    fn release(&self, handle: u32);
}
```

A host installs one (`jit::wasm::install`) before it builds a machine;
`Engine::with_capacity` reads it once and caches it, so no lock is taken on the
path a block runs down. With no embedder — every native host — `Engine::run`
calls `exec` exactly as before, and a host engine that *refuses* a module
(`REFUSED`) or has forgotten a handle (`None`) falls back to the same place.
Refusal is not a guest-visible event: the block runs, more slowly, and the only
trace is `EngineStats::instantiated` sitting below `compiled`.

It is a trait rather than a `#[cfg(target_arch = "wasm32")]` on purpose. The
routing — which executor, eviction telling the host, a drop releasing what is
still resident — is then exercised by a test double on x86-64
(`src/jit/wasm/tests.rs`, "The embedder seam"), and only the host call itself
is target-specific. A `cfg` would have put the one code path that matters
somewhere a single CI job compiles.

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

### The browser embedder, which is now built

`src/wasm.rs` (the existing `ffi` boundary) and `web/src/jit.js`. What follows
is what shipped; where it differs from what this section used to specify, the
difference is called out and argued.

**rsemu exports**:

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

Two things stood between that and a commit. Both are settled, and how is worth
more than that they are.

#### `ctx` is a token, not a pointer

This section used to say *"`ctx` becomes a real pointer: a thunk export
receives an `i32` and has to turn it back into `&mut Ctx`"*. It does not, and
that is the one place the built thing differs from the written one.

A pointer handed to an embedder is a pointer the embedder can hand back wrong.
Generated code passes `ctx` through unexamined, so in the ordinary case the
value is right — but "the ordinary case" is not the standard for a
dereference, and a page that is buggy, or hostile, or simply reloaded at the
wrong moment gets to choose that `i32`. Dereferencing a number JavaScript chose
is unsound however carefully the JavaScript is written, and no `// SAFETY:`
comment can say otherwise, because the invariant would have to be upheld by the
embedder rather than by us.

A **token** is checkable. `src/wasm.rs` keeps a per-thread activation stack; a
token names an entry in it and carries a sequence number, so a value that is
stale, forged or simply wrong finds no activation and the import answers
`status::ERROR` — which the engine turns into an `Err` and the dispatcher into
an interpreted block. There is nothing a caller can put in that argument that
is unsound. It costs a linear scan of a stack whose depth is one.

The invariant that makes the round trip sound is then structural rather than
hoped for:

> An activation is reachable **exactly for the dynamic extent of the
> `jit_enter` import call that pushed it**, during which the `&mut dyn Env` and
> the `&mut [u8]` it describes are alive, unaliased and untouched by the frame
> that owns them.

Every clause is upheld on rsemu's side. `with_activation` pushes before the
call and a `Drop` guard pops after it, on every path out; both references are
locals of that same frame, which is blocked in the import call throughout; that
frame derives a raw pointer from each and then never names the reference again,
so the reborrow inside the export is the only live `&mut` to either; and the
stack is a `thread_local!`, so an activation is never visible to a hart running
on another worker. Re-entrancy is bounded by construction — a generated module
never calls `jit_enter`, so an activation is never nested inside its own.

The `thread_local!` is deliberate and is not a `core::sync::Global`. A `Global`
would take a lock on **every guest memory access made from compiled code**, and
two harts on two workers would contend for a table neither can see the other's
half of. Per-call-stack state is what a thread-local is for, and `src/wasm.rs`
is the host boundary rather than `core/`, `cpu/`, `dev/`, `machine/` or `ir/`,
which is where CLAUDE.md's `core::sync` rule bites.

**The `unsafe` is the `ffi` site and nothing else.** Two blocks: the reborrow
above, and the calls to the three imports. `src/wasm.rs` already carries
`#![allow(unsafe_code)]` for exactly this, so the seven sanctioned sites are
unchanged and there is no eighth. Nothing under `jit/` gained an `unsafe` of
any kind — `jit::wasm::embed` is a trait whose arguments are a byte slice, a
`&mut [u8]` and a `&mut dyn Env`.

#### It is tested in CI

`web/check.mjs` grew a section (§1c) and `scripts/check.sh wasm` and the CI
`wasm` job both run it. What it does:

* asserts the `jit-wasm` module declares **exactly** the three `rsemu.jit_*`
  imports and no others, and the four thunk exports plus the harness;
* instantiates it against `web/src/jit.js`;
* runs one guest for one span under `engine = "interp"` and under
  `engine = "jit-wasm"`, and asserts **one hash**;
* asserts, beside that hash, that blocks really ran inside modules this engine
  compiled — because a hash that matched by falling back to the interpreter
  would prove nothing, and that is precisely the way this could pass while
  doing nothing;
* times all three engines and prints the multiplier.

**Node, not a headless browser**, and the difference is worth stating.
`WebAssembly.Module`, `Instance` and `Memory` are the *JavaScript Interface*
recommendation, which node implements through the same V8 that Chrome does, and
nothing on this path touches the *Web API* half — no streaming compilation, no
`fetch`, no worker, no `SharedArrayBuffer`. So for the seam and the determinism
claim the two are the same engine. What node does not cover is a browser's own
tiering policy, its module-count limits and its memory pressure, and those are
exactly the things a synthetic loop would not have measured honestly anyway.

The JIT build is a **second module**, not a feature of the demo: the demo has
no RISC-V board and no reason to carry a code generator, and the harness has no
reason to carry six consoles. Both are cdylibs from one crate, so `check.sh`
and CI copy the first aside before building the second.

### What WASI would need, and why it does not have it

`ROADMAP.md` §11.5 says *"Under WASI the same functions bind to preview-1
imports instead"*. That is true of `rsemu.now` and `rsemu.random_get`, which are
`clock_time_get` and `random_get`. It is **not true of `rsemu.compile`**, and
this is worth being exact about: WASI preview 1 has no interface for compiling
or instantiating a module, and neither does preview 2 — module instantiation is
a *component model* concern and is not something a preview-1 command module can
ask its host for. There is no standard import to bind to.

So: **a plain `wasm32-wasip1` or `wasm32-wasip1-threads` build has no wasm JIT
and cannot have one**, and falls back to the reference executor, which means
running a wasm interpreter inside a wasm interpreter — correct, and pointless.
That is why the imports in `src/wasm.rs` are declared under
`all(target_arch = "wasm32", target_os = "unknown")` and not merely under
`target_arch = "wasm32"`: a WASI module declaring an import no WASI runtime can
satisfy would fail to *instantiate*, which is a worse answer than being slow. A WASI *runtime* could of course offer a non-standard import
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

* `web/check.mjs` §1c closes it the rest of the way, because there the modules
  are not merely validated and run against stubs — they are the modules a whole
  guest is executed out of, against rsemu's own thunks, and the state hash is
  compared with the interpreter's. `tests/wasm_jit_v8.rs` checks the
  *encoding* against a real engine over a corpus; that checks the **backend**
  against one over a program.

So: the encoding is checked against V8 wherever V8 is available, the semantics
of a compiled block are checked against `ir::Interp` everywhere, and a whole
guest run out of V8-compiled modules is checked against the interpreter on
every commit.

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
4. `web/check.mjs` §1c — the same claim with a **real engine** behind the
   modules. One guest, one span, `engine = "interp"` and `engine = "jit-wasm"`,
   one hash — and the count of blocks entered inside host-compiled modules
   printed beside it, because a hash that matched by quietly falling back to
   the interpreter would be the interpreter agreeing with itself.

Layers 1–3 all run the generated modules on `jit::wasm::exec`, so on their own
they say the backend agrees with `Interp` about *this project's reading* of the
core specification. Layer 4 is what removes that qualifier, and it is the
reason the browser path joins the determinism evidence rather than sitting
beside it. There is nothing the browser path can do that the others cannot
check, because there is no semantics in the JavaScript at all: every observable
thing a block does is an import, and every import routes straight back into
`Thunks`.

## `unsafe`

**None in `jit/`.** Not a block, not an `#[allow]`, and that is still true now
that the embedder exists. Both native backends opt into CLAUDE.md's second
sanctioned site — the JIT code buffer — because machine code has to be reached
through a raw pointer and has to reconstitute `&mut` references to call back. A
module is a byte vector, entering it is a safe function call, an import's
arguments are integers, and `jit::wasm::embed::Embedder` is a trait taking a
byte slice, a `&mut [u8]` and a `&mut dyn Env`.

The browser embedder uses the *existing* `ffi` site in `src/wasm.rs`, which
CLAUDE.md sanctions and which already carries a module-scoped
`#![allow(unsafe_code)]`. **Two blocks, and here they are in full.**

```rust
// SAFETY: `active.env` is the address of a `&mut dyn Env` local to the
// `Browser::enter` frame that pushed this activation, and `active.frame`/
// `active.len` describe the `&mut [u8]` that frame was given. That frame is
// blocked inside the `jit_enter` import call for the whole time this
// activation is reachable (`Pop` removes it on every path out), so both are
// alive; it derived raw pointers from each and never names the references
// again, so these reborrows are the only live `&mut` to either; and the stack
// is thread-local, so no other thread can reach them.
let (env, mem) = unsafe {
    (&mut *(active.env as *mut &mut dyn Env),
     core::slice::from_raw_parts_mut(active.frame as *mut u8, active.len))
};
```

and the import calls themselves, whose safety argument is that `handle` came
from this embedder's own `compile` and has not been released, and that `ctx`
names the activation pushed on the line above.

**The seven sanctioned sites are untouched and there is no eighth.** The
question CLAUDE.md asks of a candidate eighth — *what exactly is lost by
refusing it?* — was never reached, because the existing `ffi` site is where an
`i32` from a foreign embedder has always been turned back into something
typed, and this is one more of those.

What a review of this *should* look at is not the count but the shape, and the
shape changed: the design this note originally specified would have
dereferenced a pointer the embedder chose, and what shipped dereferences
nothing until a token has been checked against a table rsemu owns. That is the
difference between "sound if the JavaScript is right" and "sound".

## The measurement

§11.4 asks one question about this backend — **is a wasm module faster than the
IR interpreter in a browser?** — and it now has an answer.

### The workload, and why it is one workload

`crate::wasm::rsemu_jit_guest_run(engine, quanta, budget)` builds a RISC-V
hart, runs it, and returns a hash of everything a guest can see. Both halves of
the measurement call that one function: `benches/wasm_jit_embedder.rs` on a
native host, `web/check.mjs` §1c inside node. A benchmark here and a harness
there, each with its own fixture, would have been two workloads with one name.

The guest is seven RV64I instructions in a loop with its scratch word on the
page *after* its code. That separation decides what gets measured: a loop that
stores into its own page invalidates its block every pass, so nothing is ever
chained, every pass re-lifts, and the number is about the lifter. Moved one
page away, the block is compiled once and entered once per pass — which is the
shape an embedder's instantiation cost has to be amortised over, and therefore
the shape this question is about.

It is *not* the `riscv-virt` board the earlier table used, because the browser
harness cannot boot one: the demo build has no RISC-V machine and the JIT build
has no console. Both columns below were re-taken on the new workload for that
reason, and the old board numbers are kept underneath so the change of fixture
is visible rather than quietly folded in.

### The numbers

2000 quanta of 20000 ticks, 16 000 001 guest instructions, best of three after
a warm-up, one x86-64 Linux machine, node 26 (V8):

| Engine | native — `jit::wasm::exec` | in V8 |
| --- | --- | --- |
| `interp` | 1064 ms, 1.00× | 922 ms, 1.00× |
| `jit` (portable IR backend) | 517 ms, 2.06× | 840 ms, 1.10× |
| `jit-host` (`jit::x86`) | 85 ms, 12.50× | — there is no `mmap` |
| `jit-wasm` | 1392 ms, **0.76×** | 613 ms, **1.50×** |

One table is one run, and the browser column moves: five runs on this machine
gave 1.48×, 1.49×, 1.50×, 1.59× and 1.60×, against a native column that is
stable to a few per cent. That spread is V8's tiering and the machine's other
load, and it is why `web/check.mjs` **prints** the ratio rather than gating on
it — a wall-clock gate is a gate on whoever's runner is busiest. The
determinism checks beside it are the ones that must not flake, and those are
asserted.

**So it wins, and it wins earlier than §11.4 expected.** That section predicted
the backend would pay "only on long-running superblocks, and may not win at
all". It pays per *basic block*, with no chaining, no superblocks and no
tiering heuristic — 1.50× the IR interpreter and 1.36× the portable IR backend
in the same engine.

Three things about the shape of that result:

* **The native column is the floor, not the backend.** 0.76× is a block
  interpreted twice over, and every one of those 1392 ms is `jit::wasm::exec`
  decoding bytecode a real engine compiles once. It is correctness evidence.
  Its value is that the translation is *executed* on the x86-64 runner that
  gates every commit, which `jit::arm64` cannot claim.
* **`jit` is slower in V8 than natively** (2.06× → 1.10×) while `interp` is
  about the same. The portable backend is a dispatch loop over IR, which is
  exactly the branch-heavy indirect-call shape a wasm engine gives up the most
  on — so part of `jit-wasm`'s browser advantage is the comparison getting
  worse, not only the backend getting better. Reported here rather than left
  for a reader to notice.
* **The instantiation estimate was pessimistic, and the reason is the cost
  model above.** "Tens of microseconds per module, so `n` in the thousands
  before compiling pays" is the right arithmetic, and this workload runs one
  block 250 000 times. A guest with a large cold working set would land
  differently, and the tiering heuristic §11.4 asks for is still the answer for
  that — but it is now a *measurable* answer rather than a guess, which is the
  condition this note put on building it.

### What would move it, now that there is something to move

In the order the numbers suggest, and none of them was worth building before
this table existed:

1. **An inlined software-TLB probe.** Every guest access is an import call
   today; §9.1's first mechanism is the biggest single win on a native host.
   The module already imports linear memory, so the probe can load from it
   directly for the pages the TLB will hand out. This is the one whose payoff
   the note previously said was "not knowable from inside the crate" — it is
   knowable now.
2. **A superblock-shaped module.** The frontend decides this
   (`cpu::*::lift`'s `Shape`), and this backend compiles whatever block it is
   given, so it arrives for free the day the frontend produces one.
3. **One module with an internal dispatch loop over a table.** Guest work stays
   inside wasm across block boundaries and §11.4's `call_indirect` becomes a
   real mechanism. This is the design that would beat all of the above, and it
   is also the one that needs the most: a table, a growth strategy, and an
   answer for invalidation that does not go through the block cache.

### The old table, for continuity

The same four engines on the `riscv-virt` board with a 12-instruction RV64I
firmware, 400 quanta after a 20-quantum warm-up, before the embedder existed:
`interp` 11.81 s (1.00×), `jit` 6.53 s (1.81×), `jit-host` 1.47 s (8.06×),
`jit-wasm` 21.89 s (0.54×). Consistent with the native column above, on a
different fixture.

One practical consequence, since this engine is in a per-commit gate:
`tests/riscv_virt_engines.rs` costs about twelve seconds more with `jit-wasm`
in it than without, over a sixty-second test. That is the price of the
determinism evidence and it is worth paying.
