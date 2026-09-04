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
const gameboy = machines.find((m) => m.name === "gameboy");
const sms = machines.find((m) => m.name.startsWith("sms"));

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

  check(emu.hasPad, "the NES has controllers to press");
  emu.setButtons(0, Rsemu.BUTTONS.start | Rsemu.BUTTONS.a);
  check(emu.buttons(0) === (Rsemu.BUTTONS.start | Rsemu.BUTTONS.a), "buttons are recorded");
  console.log(`  state hash: ${hash}, ${(emu.nowNs() / 1e6).toFixed(1)} ms of virtual time`);
  emu.shutdown();
}

// The other two cartridge machines. Neither carries a built-in image — a
// cartridge is the visitor's to supply — so the picture is checked against a
// ROM generated right here, exactly as the NES's is. Nothing is vendored.
for (const [machine, image, label, shape] of [
  [gameboy, gameboy && minimalGb(), "Game Boy", [160, 144]],
  [sms, sms && minimalSms(), "Master System", [256, 192]],
]) {
  if (!machine) continue;
  emu.boot(machine.index, image);
  check(emu.running, `the ${label} boots`);
  check(emu.hasVideo, `the ${label} has a picture`);
  check(
    emu.width === shape[0] && emu.height === shape[1],
    `${label}: ${emu.width}x${emu.height}, wanted ${shape[0]}x${shape[1]}`,
  );

  let drawn = 0;
  for (let i = 0; i < 120; i++) if (emu.runFrame()) drawn++;
  check(drawn > 100, `${drawn} of 120 ${label} frames produced a picture`);

  const pixels = emu.bytes(instance.exports.rsemu_frame_ptr(), instance.exports.rsemu_frame_len());
  check(
    pixels.length === shape[0] * shape[1] * 4,
    `${label}: four bytes a pixel, which is what ImageData holds`,
  );
  check(
    pixels.every((b, i) => i % 4 !== 3 || b === 255),
    `${label}: every pixel is opaque`,
  );
  const colours = new Set();
  for (let i = 0; i < pixels.length; i += 4) {
    colours.add((pixels[i] << 16) | (pixels[i + 1] << 8) | pixels[i + 2]);
  }
  check(colours.size > 1, `${label}: ${colours.size} distinct colours, so it drew something`);

  check(emu.hasPad, `the ${label} has controllers to press`);
  emu.setButtons(0, Rsemu.BUTTONS.start | Rsemu.BUTTONS.a);
  check(
    emu.buttons(0) === (Rsemu.BUTTONS.start | Rsemu.BUTTONS.a),
    `${label}: buttons are recorded`,
  );
  emu.setButtons(0, 0);

  const hash = emu.stateHash();
  const state = emu.save();
  check(state.length > 0, `${label}: save state is ${state.length} bytes`);
  emu.runFrames(5);
  check(emu.stateHash() !== hash, `${label}: running changes the state hash`);
  emu.load(state);
  check(emu.stateHash() === hash, `${label}: and loading the snapshot restores it`);
  emu.shutdown();
}

if (apple1) {
  emu.boot(apple1.index, null);
  check(emu.running, "the Apple 1 boots with no file at all");
  check(emu.hasConsole, "it has a console");
  check(!emu.hasPad, "and no controllers, so the page draws none");
  emu.runFrames(30);
  let text = decode(emu.consoleRead());
  emu.consoleWrite("\r");
  emu.runFrames(30);
  text += decode(emu.consoleRead());
  check(text.length > 0, `the monitor says something (${JSON.stringify(text.slice(0, 40))})`);
  emu.shutdown();
}

// ---------------------------------------------------------------------------
// 2b. The images the module carries, and the machines that boot on them
// ---------------------------------------------------------------------------
//
// This is the headline claim of the page — "press one button and you are
// typing at a monitor" — so it is checked rather than asserted. Every built-in
// image every machine offers is booted with **nothing uploaded**: no
// `rsemu_input_reserve`, no file, no bytes crossing in at all.

const withImages = machines.filter((m) => m.builtins.length > 0);
console.log(
  `  built-in images: ${
    withImages.map((m) => `${m.name} [${m.builtins.map((b) => b.name).join(", ")}]`).join("; ") ||
    "(none)"
  }`,
);
check(withImages.length > 0, "at least one machine runs with no file at all");

for (const m of withImages) {
  for (const b of m.builtins) {
    check(Boolean(b.name && b.summary && b.slot), `${m.name}/${b.name} describes itself`);
    check(
      m.media.length === 0 || m.builtins.every((i) => i.slot.length > 0),
      `${m.name}/${b.name} names the slot it fills`,
    );
    emu.bootBuiltin(m.index, b.index);
    check(emu.running, `${m.name} boots on ${b.name} with nothing uploaded`);
    check(
      emu.hasVideo || emu.hasConsole,
      `${m.name} on ${b.name} has something to look at (video ${emu.hasVideo}, console ${emu.hasConsole})`,
    );
    if (emu.hasVideo) {
      // The ABI promises RGBA whatever the display adapter would rather
      // produce, and `ImageData` would tear the picture apart otherwise.
      check(
        instance.exports.rsemu_frame_len() === emu.width * emu.height * 4,
        `${m.name}'s framebuffer is four bytes a pixel`,
      );
      let drew = 0;
      for (let i = 0; i < 240; i++) if (emu.runFrame()) drew++;
      const px = emu.bytes(
        instance.exports.rsemu_frame_ptr(),
        instance.exports.rsemu_frame_len(),
      );
      const seen = new Set();
      for (let i = 0; i < px.length; i += 4) seen.add((px[i] << 16) | (px[i + 1] << 8) | px[i + 2]);
      check(drew > 0, `${m.name} on ${b.name} produced ${drew} frames`);
      check(px.every((v, i) => i % 4 !== 3 || v === 255), `${m.name}'s picture is opaque`);
      check(seen.size > 1, `${m.name} on ${b.name} drew ${seen.size} colours, not one flat one`);
    }
    emu.shutdown();
  }
}

// ---------------------------------------------------------------------------
// 2c. The PC/AT, posting on rsemu's own BIOS
// ---------------------------------------------------------------------------
//
// The other machines with a built-in image carry a *monitor*: a couple of
// hundred bytes of 6502 that the catalog holds as a `&'static [u8]`. This one
// carries **firmware**, assembled by the module for this board at the moment it
// is asked for (`fw::pcbios::image_for_machine`), and it is the only entry in
// the catalog that boots real firmware with nothing uploaded.
//
// There is no OCR here and no font table, so what is checked is the *shape* of
// a POST screen and the identity of glyphs across it: the `B` of "BIOS" on the
// banner is the same nine-by-sixteen block of ink as the `B` of "Booting.", and
// the two `o`s of "Booting." are each other. A machine that hung, or drew a
// test pattern, or scrolled, fails every one of them.

// A build with `machine-pc-at` but not `fw-pcbios` is legal and carries no
// image, so the board and the firmware are two separate questions here.
const pcat = machines.find((m) => m.name === "pc-at");
const pcbios = pcat?.builtins.find((b) => b.name === "rsemu-bios") ?? null;
if (pcat && !pcbios) {
  console.log("  SKIP pc-at is in this build without fw-pcbios, so it carries no firmware");
}
if (pcat && pcbios) {
  check(pcbios.slot === "bios", "pc-at carries rsemu's own BIOS, for the `bios` slot");
  emu.bootBuiltin(pcat.index, pcbios.index);
  check(emu.running, "the PC/AT boots on it with nothing uploaded at all");
  check(emu.hasVideo, "it has a picture");
  // `pc.kbc` opens a character port, and every byte on it is a raw AT scan
  // code rather than text — so it is a keyboard, not a console, and a page
  // that put a terminal pane in front of it would show an empty screen.
  check(!emu.hasConsole, "and no console: its one character port is a keyboard");
  check(!emu.hasPad, "and no controllers");
  check(emu.width === 720 && emu.height === 400, `VGA text: ${emu.width}x${emu.height}`);
  const periodMs = Number(instance.exports.rsemu_frame_period_ns()) / 1e6;
  check(
    periodMs > 14 && periodMs < 15,
    `${periodMs.toFixed(3)} ms a frame — the CRTC's own 70 Hz, not the 60 Hz fallback`,
  );

  for (let i = 0; i < 240; i++) emu.runFrame();

  const W = emu.width;
  const px = emu.bytes(instance.exports.rsemu_frame_ptr(), instance.exports.rsemu_frame_len());
  const ink = (x, y) => {
    const i = (y * W + x) * 4;
    return Boolean(px[i] | px[i + 1] | px[i + 2]);
  };
  /** One 9x16 text cell's ink, as a string — a glyph fingerprint. */
  const cell = (cx, cy) => {
    let bits = "";
    for (let y = 0; y < 16; y++) {
      for (let x = 0; x < 9; x++) bits += ink(cx * 9 + x, cy * 16 + y) ? "1" : "0";
    }
    return bits;
  };
  const rowHasInk = (cy) => {
    for (let cx = 0; cx < 80; cx++) if (cell(cx, cy).includes("1")) return true;
    return false;
  };

  check(rowHasInk(0) && rowHasInk(1) && rowHasInk(2), "three lines of POST output");
  // Row 3 is the cursor, which blinks, so it is deliberately not asserted on.
  let below = 0;
  for (let cy = 4; cy < 25; cy++) if (rowHasInk(cy)) below++;
  check(below === 0, "and nothing below them — it posted, it did not scroll");

  // "rsemu BIOS, ...K base, ...K extended" / "Booting." / "No bootable device."
  check(cell(6, 0) === cell(0, 1), "the B of BIOS is the B of Booting.");
  check(cell(1, 1) === cell(2, 1), "the two o's of Booting. are the same glyph");
  check(cell(1, 2) === cell(1, 1), "and so is the o of No");
  check(cell(0, 2) !== cell(0, 1), "N and B are not");
  check(cell(0, 0) !== cell(0, 1), "nor r and B");

  const hash = emu.stateHash();
  const state = emu.save();
  check(state.length > 0, `pc-at: save state is ${state.length} bytes`);
  emu.runFrames(5);
  check(emu.stateHash() !== hash, "pc-at: running changes the state hash");
  emu.load(state);
  check(emu.stateHash() === hash, "pc-at: and loading the snapshot restores it");
  console.log(`  state hash after POST: ${hash}`);
  emu.shutdown();
}

// ---------------------------------------------------------------------------
// 2f. The second bay, and the keyboard
// ---------------------------------------------------------------------------
//
// `rsemu_boot` binds one uploaded image to one slot, so until `rsemu_stage_media`
// existed a PC could be handed rsemu's own BIOS *or* a diskette and never both
// — which is the only interesting case, since the BIOS is the one the module
// carries. This is that path end to end: a diskette staged into the `floppy`
// slot, the firmware bound from the module, and the guest's own boot sector
// running.
//
// The assertion is the *negative* of the POST screen's last one above. With
// every drive empty the third line is "No bootable device.", so `cell(0, 2)`
// is an N. With a bootable diskette in the drive the BIOS jumps to `0000:7c00`
// and the sector's own `INT 10h` prints a `B` there — the same glyph as the
// `B` of "Booting." on the line above it. No font table, at either end.

if (pcat && pcbios) {
  const floppy = pcat.slots.find((s) => s.name === "floppy");
  check(
    Boolean(floppy) && pcat.slots.length === 5,
    `pc-at declares ${pcat.slots.length} media slots, "floppy" among them`,
  );

  // 1.44 MB, and eleven bytes of real mode in the first sector: teletype a
  // `B` through the BIOS's own INT 10h, halt, and loop. Hand-encoded, because
  // a fixture assembled by the module under test would agree with itself.
  const diskette = new Uint8Array(1_474_560);
  diskette.set([0xb4, 0x0e, 0xb0, 0x42, 0xb7, 0x00, 0xcd, 0x10, 0xf4, 0xeb, 0xfe], 0);
  diskette[510] = 0x55;
  diskette[511] = 0xaa;

  emu.stageMedia(pcat.index, floppy.index, diskette);
  emu.bootBuiltin(pcat.index, pcbios.index);
  check(emu.running, "the PC/AT boots on the module's BIOS with a staged diskette");
  for (let i = 0; i < 240; i++) emu.runFrame();

  const W2 = emu.width;
  const px2 = emu.bytes(instance.exports.rsemu_frame_ptr(), instance.exports.rsemu_frame_len());
  const cell2 = (cx, cy) => {
    let bits = "";
    for (let y = 0; y < 16; y++) {
      for (let x = 0; x < 9; x++) {
        const i = ((cy * 16 + y) * W2 + cx * 9 + x) * 4;
        bits += px2[i] | px2[i + 1] | px2[i + 2] ? "1" : "0";
      }
    }
    return bits;
  };
  check(
    cell2(0, 2) === cell2(0, 1),
    "INT 19h found the diskette and its boot sector printed a B",
  );

  // The keyboard, which is the other half of what this board could not do.
  // A key it has, both directions; a key it has not, no bytes at all.
  check(emu.hasKeyboard, "and the PC has an AT keyboard to type at");
  check(!emu.hasConsole, "which is not a console: on this board they are opposites");
  check(emu.key(0x41, true) && emu.key(0x41, false), "A goes down and comes back up");
  check(!emu.key(0xdeadbeef, true), "a key this keyboard has not got puts nothing on the wire");
  check(Rsemu.keysym("a") === 0x61, "printable ASCII keysyms are their own character codes");
  check(Rsemu.keysym("Enter") === 0xff0d, "and the named ones are X11's");
  check(Rsemu.keysym("Unidentified") === 0, "a key with no keysym is refused rather than guessed");

  // Staging is checked against the machine that boots, not ignored.
  let refused = false;
  try {
    emu.stageMedia(pcat.index, 99, new Uint8Array(4));
  } catch {
    refused = true;
  }
  check(refused, "a slot index the machine has not got is refused");

  emu.clearMedia();
  emu.bootBuiltin(pcat.index, pcbios.index);
  for (let i = 0; i < 240; i++) emu.runFrame();
  const px3 = emu.bytes(instance.exports.rsemu_frame_ptr(), instance.exports.rsemu_frame_len());
  const cell3 = (cx, cy) => {
    let bits = "";
    for (let y = 0; y < 16; y++) {
      for (let x = 0; x < 9; x++) {
        const i = ((cy * 16 + y) * emu.width + cx * 9 + x) * 4;
        bits += px3[i] | px3[i + 1] | px3[i + 2] ? "1" : "0";
      }
    }
    return bits;
  };
  check(cell3(0, 2) !== cell3(0, 1), "and ejecting it puts No bootable device. back");
  emu.shutdown();
}

// And the one that is the whole point: Woz's monitor of 1976, answering in the
// browser build. The bytes it prints are the ones the Apple-1 Operation
// Manual's own listing holds at $FF00 — nobody at rsemu chose them — and
// `src/dev/wdc/tests.rs` asserts the same transcript one layer down.
const beneater = machines.find((m) => m.name === "beneater-6502");
const wozmon = beneater?.builtins.find((b) => b.name === "wozmon");
if (beneater && wozmon) {
  emu.bootBuiltin(beneater.index, wozmon.index);
  check(emu.running && emu.hasConsole, "beneater-6502 boots the Woz Monitor, nothing uploaded");
  emu.runFrames(30);
  check(decode(emu.consoleRead()) === "\\\n", "it greets with a backslash, as it did in 1976");
  emu.consoleWrite("FF00.FF0F\r");
  emu.runFrames(90);
  const dump = decode(emu.consoleRead());
  check(
    dump.includes("FF00: D8 58 A0 7F A9 1F 8D 03"),
    `examining $FF00 prints the manual's own bytes (${JSON.stringify(dump.slice(-30))})`,
  );
  emu.consoleWrite("0300: AA BB CC\r");
  emu.runFrames(90);
  emu.consoleWrite("0300.0302\r");
  emu.runFrames(90);
  check(
    decode(emu.consoleRead()).includes("0300: AA BB CC"),
    "and depositing three bytes reads them back",
  );
  console.log(`  state hash after the Wozmon session: ${emu.stateHash()}`);
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

// The page's own path to a built-in image: `session.boot(entry, null, image)`,
// which is what the idle screen's one-click buttons call.
if (beneater && wozmon) {
  driver.boot(beneater, null, wozmon);
  driver.consoleFocused = true;
  spin(500);
  for (const key of ["F", "F", "0", "0", ".", "F", "F", "0", "F", "Enter"]) {
    driver.key(press({ key }), true);
  }
  spin(1200);
  check(
    consoleSeen.includes("FF00: D8 58 A0 7F A9 1F 8D 03"),
    `the driver types at Wozmon and it answers (${JSON.stringify(consoleSeen.slice(-32))})`,
  );
  driver.shutdown();
}

// The PC/AT through the same driver, because its picture is not 256x240 and
// the canvas has to follow: a board whose geometry the session got wrong would
// blit a 720x400 frame into a 256x240 element and show a corner of a BIOS.
if (pcat && pcbios) {
  driver.boot(pcat, null, pcbios);
  check(
    fakeCanvas.width === 720 && fakeCanvas.height === 400,
    `the driver sizes the canvas to the VGA: ${fakeCanvas.width}x${fakeCanvas.height}`,
  );
  const before = blits.length;
  spin(2000);
  const drew = blits.length - before;
  // 70.09 Hz against a 60 Hz display and a four-frame catch-up cap, so two
  // seconds is a hundred and twenty-odd whole frames rather than 140.
  check(drew > 100 && drew < 160, `${drew} PC/AT blits in 2 s of loop time`);
  const last = blits[blits.length - 1];
  check(
    last?.width === 720 && last?.height === 400 && last?.data.length === 720 * 400 * 4,
    "each blit is a whole 720x400 RGBA frame",
  );
  check(!driver.hasConsole, "and the page draws no terminal pane for its keyboard port");

  // Keys reach a PC only while the *picture* has focus, and for a stronger
  // reason than the console's: the guest wants Tab and the arrow keys, which
  // are how the rest of the page is navigated.
  check(driver.hasKeyboard, "the driver knows this machine takes keys");
  driver.key(press({ key: "a" }), true);
  check(driver.keysDown.size === 0, "an unfocused picture does not swallow the page's keys");
  driver.screenFocused = true;
  driver.key(press({ key: "a" }), true);
  check(driver.keysDown.has(0x61), "a focused picture sends the keysym down");
  driver.key(press({ key: "Enter" }), true);
  check(driver.keysDown.has(0xff0d), "and Return by its X11 name");
  driver.key(press({ key: "a" }), false);
  check(!driver.keysDown.has(0x61), "releasing it takes it off the wire");
  driver.key(press({ key: "b", target: { tagName: "SELECT" } }), true);
  check(!driver.keysDown.has(0x62), "a key aimed at the machine picker is still left alone");
  driver.releaseAll();
  check(driver.keysDown.size === 0, "and losing the window releases every key still down");
  driver.screenFocused = false;
  driver.shutdown();
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

// The console pane's decoder, on its own. Every board on this page today ends a
// line with a bare CR, so the other two endings have no machine here to
// exercise them — which is exactly why they are asserted directly: the first
// board with a 16550 on it would otherwise print as one unbroken line.
const { decodeGuest } = await import("./src/session.js");
const guest = (text) => Uint8Array.from(text, (c) => c.charCodeAt(0));
check(decodeGuest(guest("A\rB")) === "A\nB", "a bare CR is a newline (an Apple 1)");
check(decodeGuest(guest("A\nB")) === "A\nB", "a bare LF is a newline (a 16550)");
check(decodeGuest(guest("A\r\nB")) === "A\nB", "and CRLF is one newline, not two");
check(decodeGuest(guest("a\x00b\x07c")) === "abc", "control bytes are dropped");
check(decodeGuest(guest("\xc1\xc2")) === "AB", "and bit 7 is the keyboard's, not the text's");

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

// A two-bank Game Boy cartridge whose program fills tile data and the tile map,
// turns the LCD on and then scrolls — the same shape as the one
// `tests/workload/mod.rs` generates for the frame-hash regression, so the
// browser is looking at the machine the native tests pin. The header checksum
// at $014D is computed rather than baked: the boot ROM is not emulated here,
// but `gb.cart` validates the header the way the console's does.
function minimalGb() {
  const rom = new Uint8Array(2 * 16384);
  rom.set([0x00, 0xc3, 0x50, 0x01], 0x100); // NOP ; JP $0150
  rom.set(
    [
      0x21, 0x00, 0x80, 0x0e, 0x00, 0x3e, 0x00, 0x77, 0x3c, 0x23, 0x0d, 0x20, 0xfa, // tile data
      0x21, 0x00, 0x98, 0x0e, 0x00, 0x3e, 0x00, 0x77, 0x3c, 0x23, 0x0d, 0x20, 0xfa, // tile map
      0x3e, 0xe4, 0xe0, 0x47, // BGP  = $E4
      0x3e, 0x91, 0xe0, 0x40, // LCDC = $91: on, background on, tiles at $8000
      0xf0, 0x42, 0x3c, 0xe0, 0x42, 0x18, 0xf9, // SCY++ forever
    ],
    0x150,
  );
  rom[0x147] = 0x00; // ROM only
  rom[0x148] = 0x00; // two banks
  rom[0x149] = 0x00; // no cartridge RAM
  let sum = 0;
  for (let i = 0x134; i < 0x14d; i++) sum = (sum - rom[i] - 1) & 0xff;
  rom[0x14d] = sum;
  return rom;
}

// And a two-bank Sega cartridge that sets mode 4, writes a palette, sixteen
// tile patterns and a full name table through the VDP's own ports, then moves
// the horizontal scroll once a frame. Also the workload the native regression
// pins, for the same reason.
function minimalSms() {
  const rom = new Uint8Array(2 * 16384).fill(0xff);
  rom.set(
    [
      0xf3, 0x31, 0xf0, 0xdf, // di ; ld sp,$dff0
      0x3e, 0x04, 0xd3, 0xbf, 0x3e, 0x80, 0xd3, 0xbf, // R0 = mode 4
      0x3e, 0x40, 0xd3, 0xbf, 0x3e, 0x81, 0xd3, 0xbf, // R1 = display on
      0x3e, 0xff, 0xd3, 0xbf, 0x3e, 0x82, 0xd3, 0xbf, // R2 = name table $3800
      0xaf, 0xd3, 0xbf, 0x3e, 0xc0, 0xd3, 0xbf, // CRAM address 0
      0x06, 0x20, 0x0e, 0x00, 0x79, 0xd3, 0xbe, 0x0c, 0x10, 0xfa, // 32 colours
      0xaf, 0xd3, 0xbf, 0x3e, 0x40, 0xd3, 0xbf, // VRAM address 0
      0x21, 0x00, 0x02, 0x0e, 0x00,
      0x79, 0xd3, 0xbe, 0xc6, 0x4d, 0x4f, 0x2b, 0x7c, 0xb5, 0x20, 0xf5, // 16 tiles
      0xaf, 0xd3, 0xbf, 0x3e, 0x78, 0xd3, 0xbf, // VRAM $3800
      0x21, 0x80, 0x03, 0x0e, 0x00,
      0x79, 0xe6, 0x0f, 0xd3, 0xbe, 0xaf, 0xd3, 0xbe, 0x0c, 0x2b, 0x7c, 0xb5, 0x20, 0xf2,
      0x0e, 0x00,
      0xdb, 0xbf, 0xe6, 0x80, 0x28, 0xfa, // wait for the frame flag
      0x79, 0xd3, 0xbf, 0x3e, 0x88, 0xd3, 0xbf, 0x0c, 0x18, 0xf0, // R8 = scroll
    ],
    0,
  );
  return rom;
}
