# rsemu in the browser

The demo `ROADMAP.md` §11.7 asks for: load a ROM, play it, take a save state,
all client-side with nothing uploaded. It is also the harness that keeps the
wasm target *exercised* rather than merely compiled — a target that builds but
is never run is a target that quietly stops working.

| File | What it is |
| --- | --- |
| `index.html` | the page: a canvas, a machine picker, a ROM picker, save states |
| `rsemu.js` | the entire JavaScript side of the boundary — one class, no framework |
| `app.js` | the page's own logic: the frame loop, the keyboard, the buttons |
| `check.mjs` | the same module verified headlessly, under node or deno |

## Build and serve

```sh
# the demo build: the wasm boundary plus the machines the page offers
cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
    --no-default-features --features demo --release
cp target/wasm32-unknown-unknown/release/rsemu.wasm web/
python3 -m http.server -d web 8080
```

Then open <http://localhost:8080/>. `file://` will not work — the module is
fetched, and fetch has no `file://` scheme.

`--features wasm` alone also builds and loads; it just has no machines in it, and
the page says so. That narrower build is what CI compiles every commit, because
it is the boundary that must never rot.

## Verify without a browser

```sh
node web/check.mjs target/wasm32-unknown-unknown/release/rsemu.wasm [cartridge.nes]
```

It parses the module's export section and checks it against the functions
`rsemu.js` actually calls, asserts the module imports nothing, checks that every
element `app.js` reaches for exists in `index.html`, and then *runs* the thing:
boots a NES, renders 120 frames, checks the framebuffer is opaque and has more
than one colour, takes a save state, runs on, restores it and compares state
hashes — then boots an Apple 1 with no file at all and reads its monitor's
output back. Everything except the DOM, in other words.

## What is wired and what is not

* **NES** — picture, yes: 256×240 RGBA at the machine's own frame rate. The
  keyboard is mapped to controller 1 and reaches the console's controller port
  at `$4016` through the `pads` seam, which the guest samples when it strobes.
  Sound is not wired to the page at all yet: the APU runs, but nothing carries
  its samples to a `WebAudio` node.
* **Apple 1** — a real console: type at it and the monitor answers. It needs no
  file; leave the picker empty and it boots rsemu's own monitor ROM, exactly as
  `rsemu run apple1` does.
* **Save states** — `Save state` writes a file this tab produced; `Load state`
  reads one back into an identically configured machine. Nothing is uploaded.

## No wasm-bindgen

The dependency policy (`ROADMAP.md` §0) rules it out, and the boundary does not
need it. `src/wasm.rs` has the ABI in full; the three rules it follows are worth
repeating here:

1. **Nothing crosses as a pointer JavaScript made.** The page writes into a
   buffer rsemu owns (`rsemu_input_reserve`) and reads out of ones rsemu owns
   (`rsemu_output_ptr`, `rsemu_frame_ptr`). There is no `from_raw_parts` on a
   caller-supplied address anywhere in the module.
2. **Machines are named by index**, because a build is a feature set and the
   page has to ask what this one contains anyway. No string ever crosses in.
3. **One machine at a time.** A second instance is a second module.

## Threads, and why the page does not need them

Threaded execution needs `SharedArrayBuffer`, which needs cross-origin isolation
(`Cross-Origin-Opener-Policy: same-origin` and
`Cross-Origin-Embedder-Policy: require-corp`). That is often unavailable — a
GitHub Pages default, an embedded iframe, a corporate proxy — so the
non-threaded path is a *supported target, not a fallback* (`ROADMAP.md` §11.3),
and it is the one this page uses: `requestAnimationFrame` calls
`rsemu_run_frame`, which advances virtual time by exactly one video frame and
returns. No worker, no `Atomics.wait`, no headers.

## Determinism

Virtual time is computed inside the emulator and never read from the host clock,
so the same session produces the same state hash here as it does natively
(`ROADMAP.md` §11.6). The page prints the hash; `rsemu run` prints the same one.
