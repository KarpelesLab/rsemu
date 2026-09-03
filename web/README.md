# rsemu in the browser

The demo `ROADMAP.md` §11.7 asks for: load a ROM, play it, take a save state,
all client-side with nothing uploaded. It is also the harness that keeps the
wasm target *exercised* rather than merely compiled — a target that builds but
is never run is a target that quietly stops working.

**Four of its machines need no file at all.** The module carries their images —
rsemu's own monitors, one board's demonstration firmware, the Woz Monitor of
1976, and rsemu's own legacy PC BIOS — so the first thing on the page is a row
of buttons rather than a file picker, and a visitor is typing at a 1976 monitor,
or watching a PC/AT post, a few seconds after landing. The other three want a
cartridge, which is the visitor's to supply.

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
not transformed. That is exactly the treatment a three-megabyte cargo build
product wants. Nothing imports it, so Rollup never sees it, and the page fetches
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
   runs on, restores it and compares state hashes; does the same for the Game
   Boy and the Master System on cartridges it **generates**, since neither ships
   one and neither has a built-in image to fall back on; boots an Apple 1 with
   no file at all and reads its monitor back. Then **every built-in image every
   machine offers** is booted with nothing uploaded, and each has to have a
   console or a picture — a picture being checked for four-byte pixels, opacity
   and more than one colour. The Woz Monitor gets its own transcript: it must greet with a
   backslash and answer `FF00.FF0F` with the manual's own bytes, which is the
   same assertion `src/dev/wdc/tests.rs` and `src/wasm.rs`'s own tests make one
   layer down. Then it drives `src/session.js` itself under a stub canvas and a
   fake clock: two seconds of loop time, ~120 whole 256×240 blits, pause actually
   stopping the machine, step advancing exactly one frame, `Z` becoming
   controller bit `$80`, `FF00`↵ typed at RSMON coming back as a memory dump, and
   the same at Wozmon through the page's own one-click path. The PC/AT gets a
   transcript of its own, read off the *picture*: after 240 frames the screen
   has three lines of ink on it and nothing below them, and the nine-by-sixteen
   block of pixels that is the `B` of `BIOS` on the banner is byte-identical to
   the `B` of `Booting.` on the next line, and the two `o`s of `Booting.` to
   each other. There is no font table and no OCR here — glyph *identity* is
   what a screen full of words has and a test pattern does not. And
   `decodeGuest` is checked on its own for all three line endings, because the
   two it does not currently meet are the ones the next board will.

Everything except the DOM, in other words. It does **not** prove Vue renders,
that the layout works, or that anything is legible — nothing in this repository
has a browser. That part still needs a human.

> This check used to cross-reference every element id in `index.html` against
> `app.js`. Single-file components have no such ids to compare, so layer 2
> replaces it: it reads the *built* bundle instead of the source, which
> protects the same thing (page and module drifting apart) against the artifact
> that actually ships. Nothing else was dropped.

## The machines, and why these

`demo` is a feature set (`ROADMAP.md` §3) and the page reads the catalog out of
the module it fetched, so this table describes the build rather than the page:

| Machine | What a visitor gets | Needs a file? |
| --- | --- | --- |
| `nes-ntsc`, `nes-pal` | picture and sound, controller on the keyboard | **yes** — a `.nes` cartridge, read here and never uploaded |
| `gameboy` | a 160×144 picture in the DMG's four shades, eight buttons | **yes** — a `.gb` |
| `sms-ntsc`, `sms-pal` | a 256×192 mode-4 picture, two pads and the Pause switch | **yes** — a `.sms` |
| `apple1` | a console: RSMON, rsemu's own monitor | no |
| `beneater-6502` | a serial console, and a choice of *two* monitors: RSMON, or the Woz Monitor of 1976 | no |
| `spi-panel` | a picture with nothing uploaded: an RV32 board configures an ST7272A over SPI and paints a gradient | no |
| `pc-at` | **firmware, running**: rsemu's own BIOS posts on the board's VGA at 720×400 | no — the `bios` slot takes your own image if you have one |

The three that want a file sit in the same quadrant as each other and a
different one from the rest: there is nothing to press until the visitor opens
a cartridge, because rsemu ships none and never will (`ROADMAP.md` §1). The
page says so — the picker puts the two quadrants in their own `<optgroup>`s,
`bootHint` names the slot, and the file picker is the only route in — and
`check.mjs` proves the path anyway by generating a cartridge of its own for
each of them.

The module is **3.05 MB** (≈775 KB over the wire, gzipped). The Game Boy and
the Master System cost **406 KB** of that between them — two CPU cores (SM83
and Z80) and two video chips, which is most of a console each; measured as
1 416 362 bytes before and 1 822 234 after, both `--release` and unstripped.
`pc-at` costs three times that again, and the next two sections are why.

### `pc-at`, and what it costs

This is the one board here that runs **firmware** rather than a monitor, and the
only firmware rsemu ships or will: `fw-pcbios` assembles a 64 KiB real-mode ROM
out of `src/fw/asm16`, with the MP, ACPI and SMBIOS tables generated from the
machine description it is about to be put in. Every legacy PC BIOS anyone could
otherwise reach for is GPL, and running one is fine while shipping one is not
(`ROADMAP.md` phase 6a, and §1).

With nothing uploaded, in a browser, it prints:

```text
rsemu BIOS, 639K base, 15360K extended
Booting.
No bootable device._
```

720×400 in two colours at the CRTC's own 70.09 Hz — `rsemu_frame_period_ns`
reports 14 268 060 ns, so the page paces off the card rather than off the 60 Hz
fallback. That is POST, a video BIOS handing back a mode, the tables being
published, and `INT 19h` looking for a boot sector and not finding one, because
nothing was uploaded. Bind your own image to the `bios` slot and it displaces
rsemu's.

**It costs 1 204 854 bytes**, and that number is what decided this whole list:
1 844 822 before and 3 049 676 after, both `--release` and unstripped; 496 323
and 774 438 gzipped, so ≈272 KB more over the wire. Roughly 520 KB of it is
`cpu-x86`, which no cheaper x86 board avoids; about 350 KB is the chipset the
machine file names — two 8259As, an 8254, an MC146818, an 8042, two 8237s, a
VGA, a floppy controller, an IDE channel and a PCI host bridge, none of them
optional, because `machines/pc-at.machine` names them all; and ~340 KB is the
firmware assembler and its table generators, which enter the module only
because something now calls them (`--features demo` without the wiring in
`src/wasm.rs` measured 2 708 985 — a build that carried the board and could not
have booted it).

That is a 65% module and a 56% wire increase for one board, and it was still
the right trade: it is the only entry in this catalog that boots firmware, and
"a whole PC posting, client-side, with nothing uploaded" is a different claim
from "a monitor prompt". One caveat worth knowing before pressing **Save to a
file**: a `pc-at` snapshot is **16.7 MB**, because 16 MiB of guest RAM is in
it. It works; it is just not a 200 KB NES state.

### What is deliberately not here

Measured the same way, each on top of the same baseline, `--release` and
unstripped. (That baseline was 1 841 847 rather than the 1 844 822 above:
these were taken before `src/wasm.rs` grew the two-armed `Builtin` and the
empty-bay list, 2 975 bytes between them. Nothing here turns on that.)

| Not added | Cost | Why not |
| --- | --- | --- |
| `riscv-virt` | +262 243 | wants a kernel or an SBI build, and rsemu ships neither |
| `a64-mini` | +165 692 | wants a firmware image, and has neither a screen nor a console without one |
| `m68k-mini` | +77 182 | same |
| `mips-mini` | +60 648 | same |
| `z80-mini` | +4 578 | same — and being nearly free is not a reason for a catalog row that cannot do anything |
| `arm926`, `stm32f407` | — | same |
| `q35` | +190 802 *on top of `pc-at`* | a second POST screen. Its `bios` slot does **not** in fact default to rsemu's image, whatever `Cargo.toml` used to say: neither `builtin_bios` in `src/bin/rsemu.rs` nor `builtin_media` in `src/ffi/abi.rs` has a `q35` arm |
| `pc64`, `q35-linux` | +523 702 | they do reach a shell prompt on an uploaded `bzImage` — after several hundred *guest* seconds (`docs/platforms/pc64.md`), and with an initrd in a second media slot, which this ABI cannot bind |
| `nvme-mini` | +358 717 | wants a disk image, and has neither a screen nor a console to show you it read it |
| `xhci-pci-mini` | +466 147 | has no processor at all: it would realize, run, and be a black rectangle |
| `ahci-mini`, `usb-mini`, `hub-mini`, `xhci-mini` | — | a firmware *and* a disk image, and nothing to look at either way |

Two of those correct things this file used to say. `riscv-virt` is not "the
largest single thing this build could add" any more — `pc-at` is, by more than
three times. And the PC boards' BIOS is not the user's to supply any more; it
is rsemu's, which is the entire reason `pc-at` is on the page.

**What would change the answer.** A board that can reach a prompt in a browser
without a file it does not have. Three specific things would do it, and none of
them is in `web/`:

* **A multi-slot media binding in the ABI.** `rsemu_boot` binds the uploaded
  bytes to `entry.media.first()` and nothing else, so `pc-at` can take a BIOS
  but not a boot floppy, and `pc64` cannot take a kernel *and* an initrd. A
  `rsemu_stage_media(slot, len)` accumulating before `rsemu_boot` would give the
  page "boot this floppy on rsemu's BIOS", which is the obvious next thing to
  want from this board.
* **A keyboard path.** `pc.kbc` opens a character port carrying **raw AT scan
  codes**, so the page cannot type at a PC the way it types at an Apple 1.
  `host::input`'s `KeyMap` (keysym → set-2 scan codes) and `KeyboardSink`
  already exist for the VNC front end; one `rsemu_key(keysym, down)` export
  would reuse both, and X11 keysyms for printable ASCII *are* the ASCII codes,
  so the JavaScript side is nearly free. Until then `rsemu_has_console`
  deliberately answers `0` for `pc-at` rather than putting a terminal pane in
  front of a keyboard.
* **A built-in demonstration program for one of the bare boards**, the way
  `spi-panel` has one. `z80-mini` is 4.5 KB of module away from being on this
  page; what it lacks is thirty bytes of Z80 to run.

## What is wired and what is not

* **Built-in images** — `rsemu_machine_builtin_count` / `_name` / `_summary` /
  `_slot` list what the module carries for each machine, and `rsemu_boot_builtin`
  binds one with **nothing crossing into the module at all**: no
  `rsemu_input_reserve`, no file, no bytes. It is `rsemu run beneater-6502
  --monitor wozmon` for a page that has no command line. The images are rsemu's
  own (MIT) except Wozmon, which is the 1976 listing from the *Apple-1 Operation
  Manual*, published without a copyright notice and therefore public domain —
  `src/dev/wdc/wozmon.rs` and `docs/platforms/apple1.md` record the
  determination. Nothing of unclear provenance is shipped, and nothing is
  fetched at build time.

  One of them is not a `&'static [u8]`. rsemu's BIOS is **assembled for the
  board it is about to run in**, because its MP and ACPI tables describe that
  machine's processors, so `machine::catalog::BuiltinImage` cannot hold it:
  `src/wasm.rs` has a two-armed `Builtin` — static bytes, or this one — exactly
  as `builtin_bios` in the CLI and `builtin_media` in the C ABI each do.
* **Empty bays** — `machine::realize` refuses an *unbound* media slot, so a
  PC's diskette drive, its video option-ROM socket and its second IDE bay have
  to be bound as empty. `src/wasm.rs` binds the same eight slot names the CLI
  does, for the same reason: a board that would not assemble without a floppy
  in it describes no machine anyone owned. (`src/ffi/abi.rs` does not do this,
  which is why building `pc-at` through the C ABI still fails on `vgabios`. Not
  this page's bug, but the same one.)
* **NES** — picture, yes: 256×240 RGBA at the machine's own frame rate, scaled
  by an integer factor and left hard-edged, with a 4:3 / 1:1 toggle because the
  ABI reports no pixel aspect. The keyboard is mapped to controller 1 and
  reaches the console's controller port at `$4016` through the `pads` seam,
  which the guest samples when it strobes; the on-screen pad lights up with it,
  since the seam is a *level* and a stuck button is otherwise invisible.
  `rsemu_has_pad` is what decides whether that legend is drawn and whether the
  arrow keys belong to the guest at all — a display panel with no controller
  port is an ordinary machine, and it used to get a d-pad anyway.
* **Apple 1 and `beneater-6502`** — real consoles, and a different view from
  the NES rather than a framebuffer pretending: type at one and a monitor
  answers. Neither needs a file. The Ben Eater board is the interesting one,
  because it offers a choice: RSMON, or Woz's monitor, whose prompt is a
  backslash and whose `FF00.FF0F` prints the bytes the *Apple-1 Operation
  Manual* prints. The console pane's help text follows whichever is running.
* **`pc-at`** — the third display path, and the only one that is a *PC*:
  `host::display::pc::capture` hands the module the VGA's scanout, which
  reshapes itself when the guest sets a mode, so the page reads
  `rsemu_frame_width` back after a frame and resizes its canvas from it.

  It is also the machine that made the aspect toggle's *default* a decision.
  4:3 is a **stretch** on every console here — a 256-pixel NES line becomes 320
  CSS pixels and nothing is lost. On 720×400 it is a **squeeze**: a period VGA
  monitor really did show those pixels narrower than they were tall, but it did
  it in analogue, and a browser at integer scale 1 does it by throwing away two
  columns in nine, which is most of a nine-pixel-wide glyph — the banner came
  out as `rsevu ЗIOЁ`. So `defaultAspect` starts a picture at 1:1 when 4:3
  would come out narrower than its own pixel count, and at 4:3 otherwise. Every
  machine that was on this page before is unchanged by that rule (a Master
  System is exactly 256 either way); the toggle still offers both and still
  says which is which.

  It has no console, and that is deliberate. `pc.kbc` opens a character port,
  but every byte on it is a raw AT scan code in set 2 rather than a character
  (`dev::pc::kbc`), so `src/wasm.rs` skips a port named `keyboard` when it looks
  for one — the same line `src/bin/rsemu.rs` draws, by the same name. A terminal
  pane in front of it would show an empty screen and send `0x41` for `A` meaning
  the `9` key.
* **`spi-panel`** — the second display path in the page, and the reason
  `rsemu_frame_ptr` is documented as RGBA rather than "whatever the adapter
  prefers": this board's scanout engine would rather hand out `RGB888`, and an
  `ImageData` built over three-byte pixels is a sheared picture. The module
  fixes the surface format and every adapter converts on capture.
* **Game Boy and Master System** — pictures, and this is what changed: the DMG
  had no `host::display` adapter at all, and the Master System's was written but
  reached neither the CLI nor this module. Both are wired now, so `--screenshot`,
  the VNC server and this page all see the same frame. The Game Boy is 160×144
  in four greys — the panel drives four evenly spaced levels and a DMG's green
  tint belongs to its glass rather than to the chip, so `host::display::gb` emits
  the greyscale and says why. The Master System is 256×192, or 224 or 240 lines
  when a game asks for them, and the adapter reports the height the VDP is
  actually in.

  Neither has sound here: `host::audio` has an adapter for the RP2A03 and not
  yet for the DMG's four channels or the SN76489, so `rsemu_has_audio` answers
  `0` and the page says the machine has no audio device. That is honest rather
  than silent, and it is the next thing either console wants.

  Both are on the same eight-bit pad the NES is, translated per console in
  `src/wasm.rs` — a Game Boy's matrix reads its columns out in the opposite
  order, a Master System pad has six lines and no Select, and its Start is the
  console's Pause switch on `/NMI`. `rsemu_has_pad` is true for both, so the
  d-pad is drawn and the arrow keys belong to the guest.
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
   page has to ask what this one contains anyway. No string ever crosses in —
   and that holds for the built-in images too, which are an index within a
   machine.
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
