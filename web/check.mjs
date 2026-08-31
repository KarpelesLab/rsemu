#!/usr/bin/env node
// Verify the module the page loads, without a browser.
//
//   node web/check.mjs [path/to/rsemu.wasm] [path/to/cartridge.nes]
//
// Two things a browser would find out the hard way, and one it cannot:
//
//   1. the module exports every symbol `rsemu.js` calls, and imports nothing —
//      a module that loads but is missing a function fails silently at the
//      moment somebody clicks Boot;
//   2. the ABI works: boot a machine, run frames, take a save state, put it
//      back, read the console;
//   3. and the *picture* is a picture — the framebuffer is checked for more
//      than one distinct colour, which no export list can tell you.
//
// It is deliberately runnable under node or deno, so CI can gate on it the way
// it already gates on the export section.

import { readFileSync } from "node:fs";
import { Rsemu } from "./rsemu.js";

const wasmPath = process.argv[2] ?? "target/wasm32-unknown-unknown/release/rsemu.wasm";
const romPath = process.argv[3] ?? process.env.RSEMU_NES_TEST_ROM ?? null;

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
    let r = 0, s = 0;
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
const glue = readFileSync(new URL("./rsemu.js", import.meta.url), "utf8");
const wanted = new Set([...glue.matchAll(/\bthis\.e\.(\w+)/g)].map((m) => m[1]));
check(wanted.size > 20, `rsemu.js calls ${wanted.size} exports`);
for (const name of [...wanted].sort()) {
  if (!exports.has(name)) bad(`export missing: ${name}`);
}
check([...wanted].every((n) => exports.has(n)), "every export the page calls exists");
check(imports.length === 0, "the module imports nothing (the page passes {})");

// ---------------------------------------------------------------------------
// 1b. The page's own wiring
// ---------------------------------------------------------------------------
//
// There is no DOM here, so this cannot prove the page *behaves*. It can prove
// the one thing that silently half-works in a browser: every element `app.js`
// reaches for exists in the markup, and the other way round.

const html = readFileSync(new URL("./index.html", import.meta.url), "utf8");
const app = readFileSync(new URL("./app.js", import.meta.url), "utf8");
const markupIds = new Set([...html.matchAll(/\bid="([^"]+)"/g)].map((m) => m[1]));
const scriptIds = new Set([...app.matchAll(/\$\("([^"]+)"\)/g)].map((m) => m[1]));
for (const id of scriptIds) {
  if (!markupIds.has(id)) bad(`app.js wants #${id}, which the page does not have`);
}
check(
  [...scriptIds].every((id) => markupIds.has(id)),
  `every one of app.js's ${scriptIds.size} elements is in the markup`,
);
check(html.includes('src="./app.js"'), "the page loads app.js");
check(app.includes('from "./rsemu.js"'), "app.js loads the glue");

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
  console.log("\nall checks passed (build --features demo for the machines)");
  process.exit(0);
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
