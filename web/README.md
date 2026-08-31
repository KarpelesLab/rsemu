# rsemu in the browser

The demo `ROADMAP.md` §11.7 asks for: load a ROM, play it, take a save state,
all client-side with nothing uploaded. It is also the harness that keeps the
wasm target *exercised* rather than merely compiled — a target that builds but
is never run is a target that quietly stops working.

It is a **Vue 3 application built by Vite**, published to
<https://karpeleslab.github.io/rsemu/> by `.github/workflows/pages.yml`.

| Path | What it is |
| --- | --- |
| `index.html` | Vite's entry document: a mount point and one module script |
| `src/main.js` | mounts the app |
| `src/App.vue` | the chrome — picker, transport, media, save states, prose |
| `src/session.js` | the frame loop, the canvas, the keyboard: **no Vue in here** |
| `src/rsemu.js` | the entire JavaScript side of the wasm boundary — one class |
| `src/components/` | `ScreenView`, `TerminalView`, `StatGrid`, `PadLegend` |
| `src/styles.css` | the design tokens and the handful of shared primitives |
| `public/rsemu.wasm` | *not committed* — a cargo build product you copy in |
| `check.mjs` | the same module and the same built site, verified headlessly |
| `scripts/licenses.mjs` | asserts every bundled npm package is permissive |

## Build and serve

```sh
# 1. the demo build: the wasm boundary plus the machines the page offers
cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
    --no-default-features --features demo --release
cp target/wasm32-unknown-unknown/release/rsemu.wasm web/public/

# 2. the page
cd web
npm ci            # not `npm install` — the lockfile is the build's input
npm run dev       # http://localhost:8080, hot reload
npm run build     # dist/, which is exactly what gets published
```

To look at the built site the way a visitor will:

```sh
python3 -m http.server -d web/dist 8080
```

`file://` will not work — the module is fetched, and fetch has no `file://`
scheme.

`--features wasm` alone also builds and loads; it just has no machines in it,
and the page says so out loud. That narrower build is what CI compiles every
commit, because it is the boundary that must never rot.

### Why the `.wasm` lives in `public/`

Vite copies `public/` to the site root **verbatim** — not hashed, not inlined,
not transformed. That is exactly the treatment a 900 KB cargo build product
wants. Nothing imports it, so Rollup never sees it, and the page fetches
`./rsemu.wasm` relative to itself.

That relative fetch is also why `vite.config.js` sets `base: "./"`. The site is
published under the `/rsemu/` subpath on GitHub Pages and served from `/` by a
local `http.server`; a relative base is the only one correct in both places.

## Verify without a browser

```sh
node web/check.mjs web/dist/rsemu.wasm --site web/dist
```

Three layers, and it is what gates the Pages deploy:

1. **The module.** Parses the export section and checks it against the
   functions `src/rsemu.js` actually calls, and asserts the module imports
   nothing.
2. **The built site.** `dist/index.html` has the mount point, is built rather
   than the raw Vite entry, and references only *relative* assets that exist;
   `rsemu.wasm` sits beside it byte-identical to the module under test and with
   its header intact; and the shipped bundle still names every one of those
   exports and still fetches `./rsemu.wasm`. Pass `--site DIR` explicitly, or
   it falls back to `web/dist` when that exists — both CI workflows always pass
   it, so it is never skipped where it matters.
3. **The ABI and the driver, running.** Boots a NES, renders 120 frames, checks
   the framebuffer is opaque and has more than one colour, takes a save state,
   runs on, restores it and compares state hashes; boots an Apple 1 with no file
   at all and reads its monitor back. Then it drives `src/session.js` itself
   under a stub canvas and a fake clock: two seconds of loop time, ~120 whole
   256×240 blits, pause actually stopping the machine, step advancing exactly
   one frame, `Z` becoming controller bit `$80`, and `FF00`↵ typed at the
   monitor coming back as a memory dump.

Everything except the DOM, in other words. It does **not** prove Vue renders,
that the layout works, or that anything is legible — nothing in this repository
has a browser. That part still needs a human.

> This check used to cross-reference every element id in `index.html` against
> `app.js`. Single-file components have no such ids to compare, so layer 2
> replaces it: it reads the *built* bundle instead of the source, which
> protects the same thing (page and module drifting apart) against the artifact
> that actually ships. Nothing else was dropped.

## What is wired and what is not

* **NES** — picture, yes: 256×240 RGBA at the machine's own frame rate, scaled
  by an integer factor and left hard-edged, with a 4:3 / 1:1 toggle because the
  ABI reports no pixel aspect. The keyboard is mapped to controller 1 and
  reaches the console's controller port at `$4016` through the `pads` seam,
  which the guest samples when it strobes; the on-screen pad lights up with it,
  since the seam is a *level* and a stuck button is otherwise invisible.
* **Apple 1** — a real console, and a different view from the NES rather than a
  framebuffer pretending: type at it and RSMON answers. It needs no file; leave
  the picker empty and it boots rsemu's own monitor ROM, exactly as
  `rsemu run apple1` does.
* **Sound** — yes, through `WebAudio`. The APU produces one sample per APU
  cycle at 894 886.36… Hz on an NTSC console; `src/host/audio` applies the
  board's own RC network (90 Hz and 440 Hz high-pass, 14 kHz low-pass — NESdev,
  "APU Mixer") and resamples to whatever this browser's `AudioContext` runs at,
  with an **exact integer phase** and no float anywhere near a duration. The
  page copies interleaved `f32` straight out of the module's memory into an
  `AudioBuffer` and schedules it against a playhead (`src/audio.js`).

  A context may only be opened from a user gesture, so sound starts with
  **Boot** and the panel has a mute and a volume. **Muting changes nothing about
  the machine**: rsemu still produces every sample and the page throws them
  away, and the state hash is identical either way — `check.mjs` asserts exactly
  that, headlessly.
* **Save states** — `Save to a file` writes a file this tab produced;
  `Restore` reads one back into an identically configured machine. Nothing is
  uploaded, and a save state can also be dropped onto the page.
* **Step** is a *frame* step. `rsemu_run_frame` is the finest advance the ABI
  offers; there is no instruction- or cycle-step export, and the button says
  "Step frame" rather than implying one.

## No wasm-bindgen

The dependency policy (`ROADMAP.md` §0) rules it out, and the boundary does not
need it. `src/wasm.rs` has the ABI in full; the three rules it follows are worth
repeating here:

1. **Nothing crosses as a pointer JavaScript made.** The page writes into a
   buffer rsemu owns (`rsemu_input_reserve`) and reads out of ones rsemu owns
   (`rsemu_output_ptr`, `rsemu_frame_ptr`, `rsemu_audio_ptr`). There is no
   `from_raw_parts` on a caller-supplied address anywhere in the module.
2. **Machines are named by index**, because a build is a feature set and the
   page has to ask what this one contains anyway. No string ever crosses in.
3. **One machine at a time.** A second instance is a second module.

`src/rsemu.js` is unchanged by the move to Vue except for its location: it is
plain ESM with no framework in it, which is what lets `check.mjs` import the
same file under node with no build step.

## Vue and the frame loop

Vue owns the *chrome* and nothing else. The emulator lives in `src/session.js`
as a plain object — deliberately not a `ref`, not `reactive`, not even a
`shallowRef` — because a framebuffer must never become reactive: 256×240×4
bytes handed to `putImageData` sixty times a second would otherwise sit behind
a `Proxy` that revalidates on every read, and it would fight the renderer for
no benefit at all. The canvas reaches the session as a bare DOM node through
one `@ready` event, and what crosses back into Vue is a handful of numbers and
one string, batched: stats twice a second, console text at 25 Hz.

## Third-party JavaScript

Unlike the crate — whose `cargo tree` is still empty by default, and whose
dependency policy in `CLAUDE.md` is untouched by any of this — **the built site
bundles third-party JavaScript**. The installed tree is about three dozen npm
packages — the exact count moves with the platform, because rollup and esbuild
ship a prebuilt binary per host — and every one of them is MIT, BSD-2-Clause,
BSD-3-Clause or ISC:

* **Vue 3** (MIT) — © 2013-present Yuxi (Evan) You and Vue contributors.
* **Vite** (MIT) and **@vitejs/plugin-vue** (MIT) — © 2019-present VoidZero Inc.
  and Vite contributors. Build-time only; nothing of Vite ships in `dist/`.
* Their transitive dependencies: `@babel/*`, `@jridgewell/sourcemap-codec`,
  `@rollup/*`, `@types/estree`, `csstype`, `esbuild`, `estree-walker`, `fdir`,
  `magic-string`, `nanoid`, `picomatch`, `postcss`, `rollup`, `tinyglobby`
  (MIT); `entities` (BSD-2-Clause); `source-map-js` (BSD-3-Clause);
  `picocolors` (ISC).

`npm run licenses` walks `node_modules` and fails on anything outside that set;
both workflows run it before building. The reason is the provenance rule in
`CLAUDE.md`: it is written about Rust, but a copyleft package inside the bundle
we publish would be the same violation in a different language. **Check the
licence before adding a dependency, and keep the tree small.**

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
(`ROADMAP.md` §11.6). The page prints the hash and will copy it to the
clipboard; `rsemu run` prints the same one.
