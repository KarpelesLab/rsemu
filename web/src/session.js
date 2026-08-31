// The running machine, and nothing that Vue is allowed to touch.
//
// This is the imperative half of the page. It owns the wasm instance, the
// canvas 2D context, the requestAnimationFrame loop and the keyboard, and it
// pushes *summaries* out to the UI through the `on` hooks — never pixels.
//
// The rule that shapes this file: a framebuffer must not become reactive. 256
// x 240 x 4 bytes handed to `putImageData` sixty times a second is a
// `Uint8ClampedArray` view straight over the module's memory, and a Vue `ref`
// wrapping that would be a `Proxy` around 245 KB of pixels re-validated on
// every read. So the canvas element arrives here as a bare DOM node, the loop
// writes to it directly, and Vue learns only that the frame counter moved.
//
// The loop itself is the non-threaded browser configuration ROADMAP.md §11.3
// calls a supported target rather than a fallback: `rsemu_run_frame` advances
// virtual time by exactly one video frame and returns, so there is no worker,
// no SharedArrayBuffer and no COOP/COEP anywhere in the page.

import { Rsemu } from "./rsemu.js";

/** Re-exported so components can name a button without importing the glue. */
export const BUTTONS = Rsemu.BUTTONS;

/** Keyboard to controller 1, in the arrangement emulators have used forever. */
export const PAD = {
  KeyZ: BUTTONS.a,
  KeyX: BUTTONS.b,
  ShiftLeft: BUTTONS.select,
  ShiftRight: BUTTONS.select,
  Enter: BUTTONS.start,
  ArrowUp: BUTTONS.up,
  ArrowDown: BUTTONS.down,
  ArrowLeft: BUTTONS.left,
  ArrowRight: BUTTONS.right,
};

const FPS_WINDOW_MS = 500;
const CONSOLE_FLUSH_MS = 40;
// A backgrounded tab comes back owing minutes of virtual time. Capping the
// catch-up makes it run *slow*, which is the honest failure; the alternative
// is a multi-second freeze while it sprints through the arrears.
const MAX_CATCHUP_FRAMES = 4;
// A monitor session prints for as long as you let it. Keep the pane bounded.
const CONSOLE_LIMIT = 24000;
const CONSOLE_TRIM = 18000;

/** True when a key event belongs to the page's own widgets, not the guest. */
function isFormControl(target) {
  if (!target || !target.tagName) return false;
  return ["INPUT", "SELECT", "TEXTAREA", "BUTTON", "A"].includes(target.tagName);
}

export class Session {
  constructor() {
    /** @type {Rsemu|null} */
    this.emu = null;
    /** @type {HTMLCanvasElement|null} */
    this.canvas = null;
    /** @type {CanvasRenderingContext2D|null} */
    this.ctx = null;

    this.machines = [];
    this.version = "";
    /** The catalog entry currently booted, if any. */
    this.machine = null;

    this.paused = true;
    this.held = 0;
    this.consoleFocused = false;
    this.consoleText = "";

    // Loop bookkeeping. All plain numbers, none of it reactive.
    this.pending = 0;
    this.last = 0;
    this.drawn = 0;
    this.fpsAt = 0;
    this.consoleAt = 0;
    this.consoleDirty = false;
    this.fps = 0;
    this.raf = 0;

    /** Everything the UI is told. Replaced wholesale by `on()`. */
    this.hooks = {
      stats: () => {},
      console: () => {},
      status: () => {},
      buttons: () => {},
    };
  }

  /** Subscribe. One listener per channel is all a single page needs. */
  on(hooks) {
    Object.assign(this.hooks, hooks);
  }

  // -- boot ------------------------------------------------------------------

  /**
   * Fetch and instantiate the module, then read its catalog.
   *
   * The catalog is a property of the `.wasm` that was fetched, not of rsemu: a
   * machine is a feature set, so a build compiled `--features wasm` alone
   * honestly reports zero machines. Nothing here hard-codes a machine list.
   */
  async load(url = "./rsemu.wasm") {
    this.emu = await Rsemu.load(url);
    this.version = this.emu.version();
    if (this.emu.echo(0xdeadbeef) !== 0xdeadbeef) {
      throw new Error("the module loaded but does not run correctly");
    }
    this.machines = this.emu.machines();
    return this.machines;
  }

  /** Hand the loop the canvas to draw on. A DOM node, never a ref's value. */
  attach(canvas) {
    this.canvas = canvas;
    this.ctx = canvas ? canvas.getContext("2d", { alpha: false }) : null;
  }

  /**
   * Build a machine and start running it.
   * @param {object} entry a catalog entry from `load()`
   * @param {Uint8Array|null} image the media image, or null for none
   */
  boot(entry, image) {
    this.emu.boot(entry.index, entry.media ? image : null);
    this.machine = entry;
    this.consoleText = "";
    this.hooks.console("");
    this.held = 0;
    this.hooks.buttons(0);

    if (this.canvas) {
      this.canvas.width = this.emu.width || 256;
      this.canvas.height = this.emu.height || 240;
      // A fresh canvas is transparent black; with `alpha: false` that reads as
      // black anyway, but be explicit so a paused boot shows a real screen.
      this.ctx.fillStyle = "#000";
      this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    }

    this.pending = 0;
    this.drawn = 0;
    this.last = performance.now();
    this.fpsAt = this.last;
    this.paused = false;
    this.draw();
    this.pushStats();
    this.start();
  }

  shutdown() {
    this.stop();
    if (this.emu) this.emu.shutdown();
    this.machine = null;
    this.paused = true;
    this.consoleText = "";
    this.hooks.console("");
    this.pushStats();
  }

  // -- transport -------------------------------------------------------------

  get running() {
    return Boolean(this.emu && this.emu.running);
  }

  get hasVideo() {
    return Boolean(this.emu && this.emu.hasVideo);
  }

  get hasConsole() {
    return Boolean(this.emu && this.emu.hasConsole);
  }

  start() {
    if (this.raf) return;
    this.last = performance.now();
    this.fpsAt = this.last;
    this.raf = requestAnimationFrame(this.tick);
  }

  stop() {
    if (this.raf) cancelAnimationFrame(this.raf);
    this.raf = 0;
  }

  pause() {
    this.paused = true;
    this.pushStats();
  }

  resume() {
    if (!this.running) return;
    this.paused = false;
    this.pending = 0;
    this.last = performance.now();
    this.fpsAt = this.last;
    this.drawn = 0;
    this.pushStats();
  }

  /**
   * Advance exactly one video frame while paused.
   *
   * A frame is the finest step the ABI offers — `rsemu_run_frame` is the only
   * advance export — so this is a frame-step, not an instruction-step, and the
   * button says so.
   */
  step() {
    if (!this.running) return;
    this.paused = true;
    try {
      if (this.emu.runFrame()) this.draw();
    } catch (e) {
      this.fail(e);
      return;
    }
    this.pumpConsole(true);
    this.pushStats(true);
  }

  reset() {
    this.emu.reset();
    this.pushStats(true);
  }

  // -- the loop --------------------------------------------------------------

  // An arrow so it can be handed to requestAnimationFrame unbound.
  tick = (now) => {
    this.raf = requestAnimationFrame(this.tick);
    if (!this.running || this.paused) return;

    // Virtual time chases real time in whole frames, paced by the machine's
    // own frame period — an NTSC NES frame is 16 639 356 ns and a PAL one is
    // not, and neither is the 16.67 ms a display would assume.
    const period = this.emu.frameMs();
    this.pending += (now - this.last) / period;
    this.last = now;
    const frames = Math.min(Math.floor(this.pending), MAX_CATCHUP_FRAMES);
    this.pending -= frames;
    if (frames <= 0) return;

    let drew = false;
    try {
      drew = this.emu.runFrames(frames) > 0;
    } catch (e) {
      this.fail(e);
      return;
    }
    if (drew) {
      this.draw();
      this.drawn += 1;
    }

    this.pumpConsole(false, now);

    if (now - this.fpsAt > FPS_WINDOW_MS) {
      this.fps = Math.round((this.drawn * 1000) / (now - this.fpsAt));
      this.drawn = 0;
      this.fpsAt = now;
      this.pushStats();
    }
  };

  /**
   * Blit the current framebuffer.
   *
   * `imageData()` is a view over the module's memory rather than a copy — the
   * pixels are laid out as RGBA for exactly this call — so it is rebuilt every
   * time: a wasm memory that grows detaches every existing view.
   */
  draw() {
    if (!this.ctx) return;
    const image = this.emu.imageData();
    if (image) this.ctx.putImageData(image, 0, 0);
  }

  /** Drain guest console output into the pane, batched so Vue is not spammed. */
  pumpConsole(force, now = performance.now()) {
    if (!this.hasConsole) return;
    const bytes = this.emu.consoleRead();
    if (bytes.length > 0) {
      this.consoleText += decodeGuest(bytes);
      if (this.consoleText.length > CONSOLE_LIMIT) {
        this.consoleText = this.consoleText.slice(-CONSOLE_TRIM);
      }
      this.consoleDirty = true;
    }
    if (this.consoleDirty && (force || now - this.consoleAt > CONSOLE_FLUSH_MS)) {
      this.consoleDirty = false;
      this.consoleAt = now;
      this.hooks.console(this.consoleText);
    }
  }

  /** Hand the UI a fresh snapshot of everything worth showing. */
  pushStats(force = false) {
    if (!this.emu) return;
    this.hooks.stats({
      running: this.running,
      paused: this.paused,
      fps: this.paused ? 0 : (this.fps ?? 0),
      frame: this.running ? this.emu.frameSerial() : 0,
      seconds: this.running ? this.emu.nowNs() / 1e9 : 0,
      hash: this.running ? this.emu.stateHash() : "—",
      width: this.running ? this.emu.width : 0,
      height: this.running ? this.emu.height : 0,
      framePeriodMs: this.running ? this.emu.frameMs() : 0,
      force,
    });
  }

  fail(e) {
    this.paused = true;
    this.hooks.status({ text: String(e?.message ?? e), error: true });
    this.pushStats(true);
  }

  // -- save states -----------------------------------------------------------

  /** A snapshot. Nothing is uploaded: the bytes exist only in this tab. */
  save() {
    return this.emu.save();
  }

  restore(bytes) {
    this.emu.load(bytes);
    this.draw();
    this.pushStats(true);
  }

  // -- keyboard --------------------------------------------------------------

  /**
   * Install the global key handlers.
   *
   * Global rather than per-element because a d-pad that only works while the
   * canvas has focus is a d-pad that appears broken. Form controls keep their
   * keys — you must still be able to tab to the machine picker and use it —
   * and the console only takes characters while its own pane has focus, so
   * typing at an Apple 1 never fights the rest of the page.
   */
  listen() {
    this.onKeyDown = (event) => this.key(event, true);
    this.onKeyUp = (event) => this.key(event, false);
    this.onBlur = () => this.releaseAll();
    addEventListener("keydown", this.onKeyDown);
    addEventListener("keyup", this.onKeyUp);
    addEventListener("blur", this.onBlur);
  }

  unlisten() {
    removeEventListener("keydown", this.onKeyDown);
    removeEventListener("keyup", this.onKeyUp);
    removeEventListener("blur", this.onBlur);
  }

  key(event, down) {
    if (!this.running) return;
    if (event.ctrlKey || event.metaKey || event.altKey) return;

    if (this.hasVideo && !isFormControl(event.target)) {
      const bit = PAD[event.code];
      if (bit !== undefined) {
        this.held = down ? this.held | bit : this.held & ~bit;
        this.emu.setButtons(0, this.held);
        this.hooks.buttons(this.held);
        event.preventDefault();
        return;
      }
    }

    if (!down || !this.hasConsole || !this.consoleFocused) return;

    // A console machine gets characters, not scancodes. The device does the
    // rest: the Apple 1's keyboard is upper case with bit 7 strapped high, and
    // that belongs to the keyboard rather than to this page — see
    // `dev::apple1::pia`.
    let text = null;
    if (event.key.length === 1) text = event.key;
    else if (event.key === "Enter") text = "\r";
    else if (event.key === "Backspace") text = "\x7f";
    else if (event.key === "Escape") text = "\x1b";
    if (text !== null) {
      this.emu.consoleWrite(text);
      event.preventDefault();
    }
  }

  /** Press or release a button from the on-screen pad. */
  setButton(bit, down) {
    if (!this.running || !this.hasVideo) return;
    this.held = down ? this.held | bit : this.held & ~bit;
    this.emu.setButtons(0, this.held);
    this.hooks.buttons(this.held);
  }

  /** Losing focus must never leave a button stuck down. */
  releaseAll() {
    this.held = 0;
    if (this.emu) this.emu.setButtons(0, 0);
    this.hooks.buttons(0);
  }
}

/**
 * Guest bytes as a host would show them.
 *
 * A 1970s console ends its lines with a bare carriage return and has no lower
 * case. Translating that is the backend's job — `host::terminal` does the same
 * thing for a real terminal — never the stream's.
 */
export function decodeGuest(bytes) {
  let out = "";
  for (const byte of bytes) {
    const c = byte & 0x7f;
    if (c === 0x0d) out += "\n";
    else if (c >= 0x20 && c < 0x7f) out += String.fromCharCode(c);
  }
  return out;
}
