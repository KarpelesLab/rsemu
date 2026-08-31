#!/usr/bin/env node
// Verify the module the page loads, and the site that loads it, without a
// browser.
//
//   node web/check.mjs [path/to/rsemu.wasm] [path/to/cartridge.nes] [--site DIR]
//
// Three things a browser would find out the hard way, and one it cannot:
//
//   1. the module exports every symbol the glue calls, and imports nothing — a
//      module that loads but is missing a function fails silently at the
//      moment somebody clicks Boot;
//   2. the *built* site still asks for exactly those symbols, still fetches
//      ./rsemu.wasm, and references only assets that exist and only by
//      relative path — the site is published at /rsemu/ and served from / in
//      development, and an absolute asset URL is a 404 in one of the two;
//   3. the ABI works: boot a machine, run frames, take a save state, put it
//      back, read the console;
//   4. and the *picture* is a picture — the framebuffer is checked for more
//      than one distinct colour, which no export list can tell you.
//
// Check 2 replaces the element-id cross-reference this file used to do against
// the hand-written index.html. Single-file components have no stable element
// ids to cross-reference, but the thing that check was really protecting — the
// page and the module drifting apart without anyone noticing — is protected
// better by reading the built bundle, because that is the artifact that ships.
//
// It is deliberately runnable under node or deno, with no dependencies, so CI
// can gate on it the way it already gates on the export section.

import { readFileSync, existsSync, statSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join, resolve } from "node:path";
import { Rsemu } from "./src/rsemu.js";

const here = dirname(fileURLToPath(import.meta.url));

const argv = process.argv.slice(2);
const siteFlag = argv.indexOf("--site");
const siteArg = siteFlag >= 0 ? argv.splice(siteFlag, 2)[1] : null;
const positional = argv.filter((a) => !a.startsWith("--"));

const wasmPath = positional[0] ?? "target/wasm32-unknown-unknown/release/rsemu.wasm";
const romPath = positional[1] ?? process.env.RSEMU_NES_TEST_ROM ?? null;
// Without --site, fall back to web/dist when it happens to be there, so a
// local `node web/check.mjs` after a build gets the site checks for free.
const sitePath = siteArg ?? (existsSync(join(here, "dist", "index.html")) ? join(here, "dist") : null);

let failures = 0;
const ok = (what) => console.log(`  ok   ${what}`);
const bad = (what) => {
  failures += 1;
  console.log(`  FAIL ${what}`);
};
const check = (cond, what) => (cond ? ok(what) : bad(what));

// ---------------------------------------------------------------------------
// 1. The module's own sections
// ---------------------------------------------------------------------------

const wasm = readFileSync(wasmPath);
console.log(`${wasmPath}: ${wasm.length} bytes`);
check(wasm.subarray(0, 4).toString("latin1") === "\0asm", "is a wasm module");

/** Minimal section walk: exports (id 7) and imports (id 2). */
function sections(bytes) {
  let i = 8;
  const uleb = () => {
    let r = 0,
      s = 0;
    for (;;) {
      const b = bytes[i++];
      r |= (b & 0x7f) << s;
      s += 7;
      if (!(b & 0x80)) return r;
    }
  };
  const exports = new Set();
  const imports = [];
  while (i < bytes.length) {
    const id = bytes[i++];
    const size = uleb();
    const end = i + size;
    if (id === 7) {
      for (let n = uleb(); n > 0; n--) {
        const len = uleb();
        exports.add(bytes.subarray(i, i + len).toString("utf8"));
        i += len;
        i += 1; // kind
        uleb(); // index
      }
    } else if (id === 2) {
      for (let n = uleb(); n > 0; n--) {
        const ml = uleb();
        const module = bytes.subarray(i, i + ml).toString("utf8");
        i += ml;
        const nl = uleb();
        const name = bytes.subarray(i, i + nl).toString("utf8");
        i += nl;
        imports.push(`${module}.${name}`);
        i = end; // descriptors vary; the names are all we need
        break;
      }
    }
    i = end;
  }
  return { exports, imports };
}

const { exports, imports } = sections(wasm);
console.log(`  ${exports.size} exports, ${imports.length} imports`);

// What the glue actually calls, read out of the glue rather than duplicated
// here — a list maintained by hand is a list that goes stale.
const glue = readFileSync(join(here, "src", "rsemu.js"), "utf8");
const wanted = new Set([...glue.matchAll(/\bthis\.e\.(\w+)/g)].map((m) => m[1]));
check(wanted.size > 20, `rsemu.js calls ${wanted.size} exports`);
for (const name of [...wanted].sort()) {
  if (!exports.has(name)) bad(`export missing: ${name}`);
}
check(
  [...wanted].every((n) => exports.has(n)),
  "every export the page calls exists",
);
check(imports.length === 0, "the module imports nothing (the page passes {})");

// ---------------------------------------------------------------------------
// 1b. The built site
// ---------------------------------------------------------------------------
//
// There is still no DOM here, so this cannot prove the page *behaves*. It can
// prove the things that silently half-work in a browser: a bundle that no
// longer talks to this ABI, an asset that did not get emitted, an absolute URL
// that only resolves at the domain root, and a .wasm that the bundler mangled
// or the assembly step forgot.

if (!sitePath) {
  console.log(
    "  SKIP the built site — pass --site DIR (or run `npm run build` in web/ first).\n" +
      "       Both CI workflows always pass it, so it is never skipped where it matters.",
  );
} else {
  const site = resolve(sitePath);
  console.log(`site: ${site}`);
  const indexPath = join(site, "index.html");
  check(existsSync(indexPath), "the site has an index.html");
  const html = existsSync(indexPath) ? readFileSync(indexPath, "utf8") : "";

  check(html.includes('id="app"'), "index.html has the mount point Vue asks for");
  check(
    !html.includes("/src/main.js"),
    "index.html is built, not the raw Vite entry (no /src/main.js)",
  );

  // Every local URL the document names must exist on disk, and must be
  // relative: the site lives at https://karpeleslab.github.io/rsemu/, so a
  // leading slash points at someone else's site root.
  const refs = [...html.matchAll(/\b(?:src|href)="([^"]+)"/g)]
    .map((m) => m[1])
    .filter((u) => !/^(?:https?:|data:|mailto:|#)/.test(u));
  check(refs.length > 0, `index.html references ${refs.length} local assets`);
  for (const url of refs) {
    if (url.startsWith("/")) {
      bad(`absolute asset URL "${url}" — it would 404 under the /rsemu/ subpath`);
    } else if (!existsSync(join(site, url))) {
      bad(`index.html references ${url}, which the site does not contain`);
    }
  }
  check(
    refs.every((u) => !u.startsWith("/") && existsSync(join(site, u))),
    "every asset it references is relative and present",
  );

  // The module has to sit next to index.html under exactly that name, because
  // the page fetches "./rsemu.wasm" and nothing rewrites it.
  const siteWasm = join(site, "rsemu.wasm");
  check(existsSync(siteWasm), "rsemu.wasm sits beside index.html");
  if (existsSync(siteWasm)) {
    const shipped = readFileSync(siteWasm);
    check(
      shipped.length === wasm.length && shipped.equals(wasm),
      "and it is byte-identical to the module checked above",
    );
    check(
      shipped.subarray(0, 4).toString("latin1") === "\0asm",
      "the bundler passed the .wasm through untouched",
    );
  }

  // The bundle is the artifact that ships, so ask *it* what ABI it speaks.
  // Property accesses survive minification, which is what makes this work.
  const scripts = refs.filter((u) => u.endsWith(".js"));
  check(scripts.length > 0, "the site ships a script bundle");
  const bundle = scripts.map((u) => readFileSync(join(site, u), "utf8")).join("\n");
  const called = new Set([...bundle.matchAll(/\brsemu_\w+/g)].map((m) => m[0]));
  console.log(`  the bundle names ${called.size} rsemu_* exports`);
  // `memory` is the one export that is not a function and not prefixed, so it
  // is checked by the shape of its only use rather than by name.
  const abi = [...wanted].filter((n) => n.startsWith("rsemu_")).sort();
  for (const name of abi) {
    if (!called.has(name)) bad(`the built bundle no longer calls ${name}`);
  }
  check(abi.every((n) => called.has(n)), `the built bundle calls all ${abi.length} of them`);
  check(bundle.includes(".memory.buffer"), "and still reads the module's exported memory");
  for (const name of [...called].sort()) {
    if (!exports.has(name)) bad(`the built bundle calls ${name}, which the module does not export`);
  }
  check(
    [...called].every((n) => exports.has(n)),
    "and calls nothing the module does not export",
  );
  check(bundle.includes("./rsemu.wasm"), "the bundle fetches ./rsemu.wasm relative to the page");

  const css = refs.filter((u) => u.endsWith(".css"));
  check(css.length > 0, "the site ships a stylesheet");
  const bytes = refs.reduce((n, u) => n + statSync(join(site, u)).size, 0);
  console.log(`  ${refs.length} assets, ${(bytes / 1024).toFixed(1)} KiB before the module`);
}

// ---------------------------------------------------------------------------
// 2. The ABI, exercised
// ---------------------------------------------------------------------------

const { instance } = await WebAssembly.instantiate(wasm, {});
const emu = new Rsemu(instance);

console.log(`  build: ${emu.version()}`);
check(emu.echo(0xdeadbeef) === 0xdeadbeef, "echo round-trips");

const machines = emu.machines();
console.log(`  machines: ${machines.map((m) => m.name).join(", ") || "(none)"}`);
if (machines.length === 0) {
  // The `--features wasm` build is the boundary and nothing else, and it is
  // the one CI compiles every commit. Everything above still applies to it;
  // there is simply no machine to run, which the page also says out loud.
  ok("boundary-only build: exports and ABI check out, no machines to run");
  console.log(
    failures === 0
      ? "\nall checks passed (build --features demo for the machines)"
      : `\n${failures} check(s) failed`,
  );
  process.exit(failures === 0 ? 0 : 1);
}

const nes = machines.find((m) => m.name.startsWith("nes"));
const apple1 = machines.find((m) => m.name === "apple1");

if (nes) {
  const image = romPath ? new Uint8Array(readFileSync(romPath)) : minimalNrom();
  emu.boot(nes.index, image);
  check(emu.running, "the NES boots");
  check(emu.hasVideo, "it has a picture");
  check(emu.width === 256 && emu.height === 240, "256x240");

  let drawn = 0;
  for (let i = 0; i < 120; i++) if (emu.runFrame()) drawn++;
  check(drawn > 100, `${drawn} of 120 frames produced a picture`);

  const pixels = emu.bytes(instance.exports.rsemu_frame_ptr(), instance.exports.rsemu_frame_len());
  const colours = new Set();
  for (let i = 0; i < pixels.length; i += 4) {
    colours.add((pixels[i] << 16) | (pixels[i + 1] << 8) | pixels[i + 2]);
  }
  check(pixels.every((b, i) => i % 4 !== 3 || b === 255), "every pixel is opaque");
  console.log(`  ${colours.size} distinct colours in the last frame`);
  if (romPath) check(colours.size > 1, "a real cartridge draws more than one colour");

  // Sound. A headless check cannot listen, but it can prove there is something
  // to listen to and that listening changes nothing.
  check(emu.hasAudio, "it has an audio device");
  check(emu.audioChannels === 1, "mono, as an RP2A03 is");
  check(emu.audioSetRate(44100) && emu.audioRate === 44100, "the page can set the output rate");
  emu.audioConsume(emu.audioFrames());
  const before = emu.stateHash();
  emu.runFrames(30);
  const heard = emu.audioFrames();
  check(heard > 20000, `half a second produced ${heard} frames of audio at 44.1 kHz`);
  const pcm = emu.audioView(heard);
  check(
    pcm.every((v) => v >= -1 && v <= 1),
    "every sample is a normalised float in [-1, 1]",
  );
  check(emu.audioConsume(heard) === heard, "and the page drains what it copied");
  check(emu.audioDropped() === 0, "nothing was dropped");

  // The same thirty frames again with nobody reading the queue: the state hash
  // must be identical, or the audio path is moving guest state.
  emu.boot(nes.index, image);
  emu.runFrames(30);
  const ignored = emu.stateHash();
  emu.boot(nes.index, image);
  emu.audioSetRate(44100);
  emu.runFrames(30);
  let queued = emu.audioFrames();
  emu.audioConsume(queued);
  check(emu.stateHash() === ignored, "the state hash does not depend on the audio path");
  check(before !== ignored, "and the machine did advance, so that was not a tautology");

  const hash = emu.stateHash();
  const state = emu.save();
  check(state.length > 0, `save state is ${state.length} bytes`);
  emu.runFrames(5);
  check(emu.stateHash() !== hash, "running changes the state hash");
  emu.load(state);
  check(emu.stateHash() === hash, "and loading the snapshot restores it");

  emu.setButtons(0, Rsemu.BUTTONS.start | Rsemu.BUTTONS.a);
  check(emu.buttons(0) === (Rsemu.BUTTONS.start | Rsemu.BUTTONS.a), "buttons are recorded");
  console.log(`  state hash: ${hash}, ${(emu.nowNs() / 1e6).toFixed(1)} ms of virtual time`);
  emu.shutdown();
}

if (apple1) {
  emu.boot(apple1.index, null);
  check(emu.running, "the Apple 1 boots with no file at all");
  check(emu.hasConsole, "it has a console");
  emu.runFrames(30);
  let text = decode(emu.consoleRead());
  emu.consoleWrite("\r");
  emu.runFrames(30);
  text += decode(emu.consoleRead());
  check(text.length > 0, `the monitor says something (${JSON.stringify(text.slice(0, 40))})`);
  emu.shutdown();
}

// ---------------------------------------------------------------------------
// 3. The page's own driver
// ---------------------------------------------------------------------------
//
// `session.js` is the half of the page Vue does not own: the frame loop, the
// canvas blit, the console pump and the keyboard. None of it needs a DOM to be
// worth testing — it needs a canvas context, a clock and a
// requestAnimationFrame — so it gets a handful of stubs and is driven for two
// seconds of loop time here.
//
// This is not a browser test and does not pretend to be one: Vue never renders,
// nothing is laid out, and `image-rendering: pixelated` is the browser's
// business alone. What it does prove is that the loop paces itself off the
// machine's own frame period, that whole frames reach putImageData, that a
// console machine's output reaches the pane, and that a key becomes a
// controller bit — every one of which used to be provable only by opening the
// page and looking at it.

globalThis.ImageData ??= class ImageData {
  constructor(data, width, height) {
    Object.assign(this, { data, width, height });
  }
};

let frameCallback = null;
globalThis.requestAnimationFrame = (cb) => ((frameCallback = cb), 1);
globalThis.cancelAnimationFrame = () => (frameCallback = null);
globalThis.addEventListener ??= () => {};
globalThis.removeEventListener ??= () => {};

// The real path through Rsemu.load, with its two browser-only halves supplied.
globalThis.fetch = async (url) =>
  new Response(readFileSync(url.startsWith(".") ? wasmPath : url), {
    headers: { "content-type": "application/wasm" },
  });
WebAssembly.instantiateStreaming = async (source) =>
  WebAssembly.instantiate(await (await source).arrayBuffer(), {});

const blits = [];
const fakeCanvas = {
  width: 0,
  height: 0,
  getContext: () => ({
    fillStyle: "",
    fillRect() {},
    putImageData: (image) => blits.push(image),
  }),
};

/** An event as the session reads one: a code, a key, and a target to ignore. */
const press = (fields) => ({ target: {}, preventDefault() {}, ...fields });

let clock = 1000;
/** Run the rAF loop for `ms` of make-believe wall time, one display frame at a time. */
function spin(ms, step = 1000 / 60) {
  const start = clock;
  while (clock - start < ms) {
    clock += step;
    frameCallback?.(clock);
  }
}

const { Session } = await import("./src/session.js");
const driver = new Session();
let consoleSeen = "";
driver.on({ console: (text) => (consoleSeen = text) });
await driver.load("./rsemu.wasm");
driver.attach(fakeCanvas);
check(driver.machines.length === machines.length, "the driver reads the same catalog");

if (nes) {
  driver.boot(nes, minimalNrom());
  check(fakeCanvas.width === 256 && fakeCanvas.height === 240, "the driver sizes the canvas");
  const before = blits.length;
  spin(2000);
  const drew = blits.length - before;
  // NTSC runs at 60.098 Hz against a 60 Hz display, so two seconds is ~120.
  check(drew > 100 && drew < 140, `${drew} blits in 2 s of loop time`);
  const last = blits[blits.length - 1];
  check(
    last?.width === 256 && last?.height === 240 && last?.data.length === 256 * 240 * 4,
    "each blit is a whole 256x240 RGBA frame",
  );
  check(driver.emu.nowNs() > 1.9e9, "and virtual time kept up with real time");

  // Pausing must stop the machine, not merely stop drawing it.
  driver.pause();
  const parked = driver.emu.frameSerial();
  spin(500);
  check(driver.emu.frameSerial() === parked, "pause stops the machine, not just the picture");
  driver.step();
  check(driver.emu.frameSerial() === parked + 1, "step advances exactly one frame");

  driver.key(press({ code: "KeyZ", key: "z" }), true);
  check(driver.emu.buttons(0) === Rsemu.BUTTONS.a, "Z presses A on controller 1");
  driver.key(press({ code: "KeyZ", key: "z" }), false);
  check(driver.emu.buttons(0) === 0, "and releasing it lets go");
  driver.key(press({ code: "ArrowLeft", key: "ArrowLeft", target: { tagName: "SELECT" } }), true);
  check(driver.emu.buttons(0) === 0, "but a key aimed at the machine picker is left alone");
  driver.key(press({ code: "ArrowLeft", key: "ArrowLeft" }), true);
  driver.releaseAll();
  check(driver.emu.buttons(0) === 0, "and losing focus never leaves a button stuck down");
}

if (apple1) {
  driver.boot(apple1, null);
  driver.consoleFocused = true;
  spin(500);
  for (const key of ["F", "F", "0", "0", "Enter"]) driver.key(press({ key }), true);
  spin(700);
  check(consoleSeen.includes("RSMON"), "the monitor's banner reaches the console pane");
  check(
    consoleSeen.includes("FF00:"),
    `typing FF00 and Return dumps that address (${JSON.stringify(consoleSeen.slice(-30))})`,
  );
  driver.shutdown();
}

console.log(failures === 0 ? "\nall checks passed" : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);

// ---------------------------------------------------------------------------

/** 7-bit ASCII with the era's bare carriage returns, as a host would show it. */
function decode(bytes) {
  let out = "";
  for (const b of bytes) {
    const c = b & 0x7f;
    if (c === 0x0d) out += "\n";
    else if (c >= 0x20 && c < 0x7f) out += String.fromCharCode(c);
  }
  return out;
}

/** 16 KiB of PRG, 8 KiB of CHR, and `JMP $C000` at the reset vector. */
function minimalNrom() {
  const image = new Uint8Array(16 + 16384 + 8192);
  image.set([0x4e, 0x45, 0x53, 0x1a, 1, 1], 0);
  image[16] = 0x4c;
  image[17] = 0x00;
  image[18] = 0xc0;
  image[16 + 0x3ffc] = 0x00;
  image[16 + 0x3ffd] = 0xc0;
  return image;
}
