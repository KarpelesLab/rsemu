// The demo page: pick a machine, give it a ROM, run it, look at it.
//
// Everything here is DOM work; the boundary itself is `rsemu.js`. The loop is
// `requestAnimationFrame` driving `rsemu_run_frame`, which is the non-threaded
// browser configuration ROADMAP.md §11.3 calls a supported target rather than a
// fallback: no SharedArrayBuffer, no worker, no COOP/COEP, and the page stays
// responsive because the module returns after one frame.

import { Rsemu } from "./rsemu.js";

const $ = (id) => document.getElementById(id);

const ui = {
  status: $("status"),
  build: $("build"),
  machine: $("machine"),
  rom: $("rom"),
  romLabel: $("rom-label"),
  boot: $("boot"),
  reset: $("reset"),
  pause: $("pause"),
  saveState: $("save-state"),
  loadState: $("load-state"),
  screen: $("screen"),
  console: $("console"),
  screenBox: $("screen-box"),
  consoleBox: $("console-box"),
  stats: $("stats"),
  keys: $("keys"),
};

/** @type {Rsemu} */
let emu = null;
let machines = [];
let romBytes = null;
let romName = "";
let paused = false;
let pending = 0; // frames of virtual time owed to real time
let last = 0;
let drawnFrames = 0;
let fpsAt = 0;
let fps = 0;

const ctx = ui.screen.getContext("2d", { alpha: false });

function say(message, isError) {
  ui.status.textContent = message;
  ui.status.className = isError ? "err" : "";
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

async function start() {
  try {
    emu = await Rsemu.load("./rsemu.wasm");
  } catch (e) {
    say(`${e}. Serve this directory over HTTP — file:// cannot fetch a module.`, true);
    return;
  }

  ui.build.textContent = emu.version();
  if (emu.echo(0xdeadbeef) !== 0xdeadbeef) {
    say("the module loaded but does not run correctly", true);
    return;
  }

  machines = emu.machines();
  if (machines.length === 0) {
    say(
      "this build has no machines — rebuild with --features demo (see web/README.md)",
      true,
    );
    return;
  }
  for (const m of machines) {
    const option = document.createElement("option");
    option.value = String(m.index);
    option.textContent = `${m.name} — ${m.summary}`;
    ui.machine.append(option);
  }
  ui.machine.addEventListener("change", describeSelection);
  describeSelection();

  ui.boot.disabled = false;
  say(`ready — ${machines.length} machine(s) in this build`);
  requestAnimationFrame(loop);
}

function selected() {
  return machines[Number(ui.machine.value)] ?? machines[0];
}

function describeSelection() {
  const m = selected();
  const needsMedia = m.media !== "";
  ui.rom.disabled = !needsMedia;
  ui.romLabel.textContent = needsMedia
    ? `${m.name} loads a file into its “${m.media}” slot`
    : `${m.name} needs no file`;
}

ui.rom.addEventListener("change", async () => {
  const file = ui.rom.files?.[0];
  if (!file) return;
  romBytes = new Uint8Array(await file.arrayBuffer());
  romName = file.name;
  say(`${romName}: ${romBytes.length} bytes ready`);
});

ui.boot.addEventListener("click", () => {
  const m = selected();
  try {
    emu.boot(m.index, m.media ? romBytes : null);
  } catch (e) {
    say(String(e.message ?? e), true);
    return;
  }
  ui.screen.width = emu.width || 256;
  ui.screen.height = emu.height || 240;
  ui.screenBox.hidden = !emu.hasVideo;
  ui.consoleBox.hidden = !emu.hasConsole;
  ui.console.textContent = "";
  ui.keys.hidden = !emu.hasVideo;
  ui.reset.disabled = false;
  ui.pause.disabled = false;
  ui.saveState.disabled = false;
  ui.loadState.disabled = false;
  paused = false;
  ui.pause.textContent = "Pause";
  pending = 0;
  last = performance.now();
  say(`${m.name} running${romName ? ` — ${romName}` : ""}`);
});

ui.reset.addEventListener("click", () => {
  try {
    emu.reset();
    say("reset");
  } catch (e) {
    say(String(e.message ?? e), true);
  }
});

ui.pause.addEventListener("click", () => {
  paused = !paused;
  ui.pause.textContent = paused ? "Resume" : "Pause";
  last = performance.now();
});

// ---------------------------------------------------------------------------
// Save states — client-side, nothing uploaded (ROADMAP.md §11.7)
// ---------------------------------------------------------------------------

ui.saveState.addEventListener("click", () => {
  let bytes;
  try {
    bytes = emu.save();
  } catch (e) {
    say(String(e.message ?? e), true);
    return;
  }
  const url = URL.createObjectURL(new Blob([bytes], { type: "application/octet-stream" }));
  const a = document.createElement("a");
  a.href = url;
  a.download = `${selected().name}-${Date.now()}.rsemustate`;
  a.click();
  URL.revokeObjectURL(url);
  say(`saved ${bytes.length} bytes — state hash ${emu.stateHash()}`);
});

ui.loadState.addEventListener("change", async () => {
  const file = ui.loadState.files?.[0];
  if (!file) return;
  try {
    emu.load(new Uint8Array(await file.arrayBuffer()));
    say(`restored — state hash ${emu.stateHash()}`);
  } catch (e) {
    say(String(e.message ?? e), true);
  }
});

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

const B = Rsemu.BUTTONS;

/** Keyboard to controller 1, in the usual emulator arrangement. */
const PAD = {
  KeyZ: B.a,
  KeyX: B.b,
  ShiftRight: B.select,
  ShiftLeft: B.select,
  Enter: B.start,
  ArrowUp: B.up,
  ArrowDown: B.down,
  ArrowLeft: B.left,
  ArrowRight: B.right,
};

let held = 0;

function padEvent(event, down) {
  const bit = PAD[event.code];
  if (bit === undefined) return false;
  held = down ? held | bit : held & ~bit;
  emu?.setButtons(0, held);
  event.preventDefault();
  return true;
}

addEventListener("keydown", (event) => {
  if (!emu?.running) return;
  if (emu.hasVideo && padEvent(event, true)) return;
  if (!emu.hasConsole) return;

  // A console machine gets characters. The device does the rest: the Apple 1's
  // keyboard is upper case with bit 7 strapped high, and that belongs to the
  // keyboard rather than to this page (see `dev::apple1::pia`).
  let text = null;
  if (event.key.length === 1) text = event.key;
  else if (event.key === "Enter") text = "\r";
  else if (event.key === "Backspace") text = "\x7f";
  if (text !== null) {
    emu.consoleWrite(text);
    event.preventDefault();
  }
});

addEventListener("keyup", (event) => {
  if (emu?.running && emu.hasVideo) padEvent(event, false);
});

// A page that loses focus must not leave a button stuck down.
addEventListener("blur", () => {
  held = 0;
  emu?.setButtons(0, 0);
});

// ---------------------------------------------------------------------------
// The frame loop
// ---------------------------------------------------------------------------

function loop(now) {
  requestAnimationFrame(loop);
  if (!emu?.running || paused) return;

  // Virtual time is chased to real time in whole frames. The cap keeps a
  // backgrounded tab from trying to catch up on minutes of arrears when it
  // comes back — it runs slow instead, which is the honest failure.
  const period = emu.frameMs();
  pending += (now - last) / period;
  last = now;
  const frames = Math.min(Math.floor(pending), 4);
  pending -= frames;
  if (frames <= 0) return;

  let drew = false;
  try {
    drew = emu.runFrames(frames) > 0;
  } catch (e) {
    say(String(e.message ?? e), true);
    paused = true;
    return;
  }

  if (drew) {
    const image = emu.imageData();
    if (image) ctx.putImageData(image, 0, 0);
    drawnFrames += 1;
  }

  if (emu.hasConsole) {
    const out = emu.consoleRead();
    if (out.length > 0) appendConsole(out);
  }

  if (now - fpsAt > 500) {
    fps = Math.round((drawnFrames * 1000) / (now - fpsAt));
    drawnFrames = 0;
    fpsAt = now;
    ui.stats.textContent =
      `${fps} fps · frame ${emu.frameSerial()} · ` +
      `${(emu.nowNs() / 1e9).toFixed(2)} s of virtual time · ` +
      `state ${emu.stateHash()}`;
  }
}

/**
 * Guest output into the console pane.
 *
 * A 1970s console ends its lines with a bare carriage return, and it has no
 * lower case; translating that is the *backend's* job (`host::terminal` does
 * the same thing for a real terminal), never the stream's.
 */
function appendConsole(bytes) {
  let text = "";
  for (const byte of bytes) {
    const c = byte & 0x7f;
    if (c === 0x0d) text += "\n";
    else if (c >= 0x20 && c < 0x7f) text += String.fromCharCode(c);
  }
  ui.console.textContent += text;
  // Keep the pane bounded; a monitor session can print for a long time.
  if (ui.console.textContent.length > 8000) {
    ui.console.textContent = ui.console.textContent.slice(-6000);
  }
  ui.console.scrollTop = ui.console.scrollHeight;
}

start();
