// The browser half of rsemu's WebAssembly JIT backend.
//
// `src/jit/wasm/` turns an IR block into a complete `WebAssembly.Module`. On
// every other host the only thing that can run one is `jit::wasm::exec`, a
// wasm interpreter written from the core specification — correct, and slower
// than interpreting the IR, because it is a wasm interpreter running wasm
// generated from IR. Here there is a real engine, and this file is how it gets
// the modules.
//
// Read `docs/techniques/wasm-jit.md` for the design and `src/jit/wasm/abi.rs`
// for the contract. Two things are worth stating up front, because they are
// what makes this file as short as it is.
//
// **There are no semantics here.** Every observable thing a generated block
// does — a guest slot read, a load, a store, a tick, an instruction boundary,
// a fault — is a wasm import, and all four of them go straight back into rsemu
// through its own exports. What those do lives in `Thunks` in
// `src/jit/wasm/rt.rs`, which is a transcription of `ir::interp`. That is the
// only reason a browser run can be asserted to hash identically to an
// interpreted one: there is one implementation of the IR's meaning and the
// browser does not get its own.
//
// **`ctx` is a token, not a pointer.** A generated module receives one and
// hands it back unexamined to every import; this file never reads it, never
// computes with it and never invents one. rsemu checks it against its own
// activation stack, so a wrong value is refused rather than dereferenced. The
// design note originally specified a raw pointer here; `src/wasm.rs`'s
// `Activation` says why it is not one.
//
// **Memory is rsemu's, not ours.** A generated module imports
// `rsemu.exports.memory` — the module's own linear memory — because the
// temporary frame lives in rsemu's heap and `$frame` is its address there.
// Nothing in this file allocates a `WebAssembly.Memory`, and nothing caches a
// typed-array view of that buffer: a wasm memory that grows detaches every
// existing view, and an allocation inside any rsemu call can grow it.

/**
 * Build the `rsemu.*` import object a jit-wasm build asks for.
 *
 * The exports are read through a getter rather than captured, because the
 * import object has to be handed to `WebAssembly.instantiate` *before* there
 * is an instance to read them from. That circularity is inherent — the
 * generated modules import the instance that generates them — and a late-bound
 * lookup is the whole of the trick.
 *
 * @param {() => WebAssembly.Exports} exports the instantiated module's exports
 * @returns {{rsemu: Record<string, Function>}} the import object
 */
export function jitImports(exports) {
  // Handles are 1-based indices into this array; 0 is rsemu's REFUSED, so it
  // can never name an instance. A released slot becomes null and is not
  // reused, which keeps a stale handle harmless rather than aliasing.
  const instances = [null];

  /** The four imports every generated module declares, in module "e". */
  const blockImports = () => {
    const e = exports();
    return {
      e: {
        m: e.memory,
        g: e.rsemu_jit_slot,
        l: e.rsemu_jit_load,
        s: e.rsemu_jit_store,
        n: e.rsemu_jit_note,
      },
    };
  };

  return {
    rsemu: {
      /**
       * Compile and instantiate the module at `ptr`, `len` bytes long.
       * @returns {number} a handle, or 0 if the engine would not take it
       */
      jit_compile(ptr, len) {
        const e = exports();
        // Sliced, not viewed: `new WebAssembly.Module` may keep the buffer
        // alive, and rsemu's memory can grow and detach it under us.
        const bytes = new Uint8Array(e.memory.buffer, ptr, len).slice();
        try {
          const instance = new WebAssembly.Instance(
            new WebAssembly.Module(bytes),
            blockImports(),
          );
          instances.push(instance);
          return instances.length - 1;
        } catch {
          // An engine out of memory, or past a module-count limit, or one that
          // rejects something this encoder emits. rsemu treats a 0 as "run it
          // on the reference executor", so this is a slowdown and never a
          // failure — which is exactly why it is caught rather than thrown.
          return 0;
        }
      },

      /**
       * Enter the block `handle` names.
       * @returns {bigint} a status from `src/jit/wasm/abi.rs`, or -1 for
       *   "this did not run", which rsemu answers by using the reference
       *   executor instead
       */
      jit_enter(handle, ctx, frame) {
        const instance = instances[handle];
        // Unreachable: rsemu only enters a handle it compiled and has not
        // released, and it bumps a generation on release so a stale one cannot
        // come back. Checked anyway because the alternative is a `TypeError`
        // thrown *into* a wasm frame, which is a trap that takes the whole
        // emulator down — and -1 is a slower block rather than a dead tab.
        if (!instance) return -1n;
        return instance.exports.b(ctx, frame);
      },

      /**
       * Drop the instance behind `handle`; rsemu evicted it.
       *
       * The slot is emptied rather than reused. A handle is never handed out
       * twice, so a stale one names nothing instead of naming somebody else's
       * module — and the cost is one array slot per compile for the tab's
       * life, which against `WebAssembly.Instance` is nothing.
       */
      jit_release(handle) {
        instances[handle] = null;
      },
    },
  };
}

/**
 * Instantiate `bytes` with the JIT imports bound to the resulting instance.
 *
 * The knot `jitImports` describes, tied: the import object is built first with
 * a getter that reads a variable this fills in afterwards.
 *
 * @param {BufferSource|Response|Promise<Response>} source bytes or a response
 * @returns {Promise<WebAssembly.Instance>}
 */
export async function instantiateWithJit(source) {
  let instance = null;
  const imports = jitImports(() => instance.exports);
  const result =
    source instanceof Uint8Array || source instanceof ArrayBuffer
      ? await WebAssembly.instantiate(source, imports)
      : await WebAssembly.instantiateStreaming(source, imports);
  instance = result.instance;
  return instance;
}
