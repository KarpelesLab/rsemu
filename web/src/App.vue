<script setup>
// The page's chrome: picker, transport, media, save states, stats.
//
// Vue owns everything in this file and nothing outside it. The emulator lives
// in `session.js` as a plain object — deliberately not a `ref`, not `reactive`,
// not `shallowRef` — because making it reactive would put a Proxy between the
// frame loop and the framebuffer sixty times a second. What crosses back into
// Vue is a handful of numbers and one string.

import { computed, nextTick, onBeforeUnmount, onMounted, ref, shallowRef } from "vue";
import { Session } from "./session.js";
import ScreenView from "./components/ScreenView.vue";
import TerminalView from "./components/TerminalView.vue";
import StatGrid from "./components/StatGrid.vue";
import PadLegend from "./components/PadLegend.vue";

const session = new Session();

// -- reactive chrome state --------------------------------------------------

const phase = ref("loading"); // loading | ready | dead
const fatal = ref("");
const version = ref("");
const machines = shallowRef([]);
const chosen = ref(0);

// The ROM image is a Uint8Array of up to a few megabytes and must never become
// a Proxy; only its name and size are of any interest to the UI.
let romBytes = null;
const rom = ref(null); // { name, size } | null

const booted = ref(false);
const paused = ref(true);
const hasVideo = ref(false);
const hasConsole = ref(false);
const consoleText = ref("");
const held = ref(0);
const status = ref({ text: "Loading the module…", error: false });
const aspect = ref("tv");
const dragging = ref(false);

const stats = ref({
  running: false,
  paused: true,
  fps: 0,
  frame: 0,
  seconds: 0,
  hash: "—",
  width: 256,
  height: 240,
  framePeriodMs: 0,
});

const terminal = ref(null);

// -- derived ----------------------------------------------------------------

// Catalog indices happen to match array positions today, but the ABI hands
// out an index and that is what the option value carries — look it up.
const entry = computed(() => machines.value.find((m) => m.index === chosen.value) ?? null);
const needsMedia = computed(() => Boolean(entry.value?.media));
const canBoot = computed(
  () => phase.value === "ready" && entry.value !== null && (!needsMedia.value || rom.value !== null),
);

const bootHint = computed(() => {
  const m = entry.value;
  if (!m) return "";
  if (!m.media) return `${m.name} needs no file — it boots rsemu's own ROM.`;
  return rom.value
    ? `${rom.value.name} · ${bytes(rom.value.size)} ready for the “${m.media}” slot.`
    : `${m.name} loads a file into its “${m.media}” slot. Choose one, or drop it on the page.`;
});

function bytes(n) {
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KiB`;
  return `${(n / 1048576).toFixed(2)} MiB`;
}

function say(text, error = false) {
  status.value = { text, error };
}

// -- lifecycle --------------------------------------------------------------

onMounted(async () => {
  session.on({
    stats: (s) => {
      stats.value = s;
      paused.value = s.paused;
    },
    console: (text) => (consoleText.value = text),
    status: (s) => (status.value = s),
    buttons: (mask) => (held.value = mask),
  });

  try {
    // Relative to the page, which is what makes the same `dist/` work at
    // https://karpeleslab.github.io/rsemu/ and at http://localhost:8000/.
    await session.load("./rsemu.wasm");
  } catch (e) {
    phase.value = "dead";
    fatal.value =
      `${e}. The module is fetched, and fetch has no file:// scheme — ` +
      `serve this directory over HTTP.`;
    return;
  }

  version.value = session.version;
  machines.value = session.machines;

  if (machines.value.length === 0) {
    phase.value = "dead";
    fatal.value =
      "This build contains no machines. A machine is a feature set, so a " +
      "module built with --features wasm alone is the boundary and nothing " +
      "else; rebuild with --features demo (see web/README.md).";
    return;
  }

  // Default to a machine that needs no file, so a first-time visitor has
  // something to press rather than a file picker to satisfy.
  const free = machines.value.find((m) => !m.media);
  chosen.value = (free ?? machines.value[0]).index;

  session.listen();
  phase.value = "ready";
  say(`Ready — ${machines.value.length} machines in this build.`);
});

onBeforeUnmount(() => {
  session.unlisten();
  session.stop();
});

// -- actions ----------------------------------------------------------------

function onCanvas(canvas) {
  session.attach(canvas);
}

async function boot() {
  const m = entry.value;
  try {
    session.boot(m, romBytes);
  } catch (e) {
    say(String(e?.message ?? e), true);
    return;
  }
  booted.value = true;
  hasVideo.value = session.hasVideo;
  hasConsole.value = session.hasConsole;
  say(`${m.name} running${rom.value ? ` — ${rom.value.name}` : ""}.`);
  // A console machine is useless until it has focus, and asking the visitor to
  // discover that is a worse page than just giving it to them.
  if (session.hasConsole && !session.hasVideo) {
    await nextTick();
    terminal.value?.focus();
  }
}

function toggleRun() {
  if (paused.value) session.resume();
  else session.pause();
}

function step() {
  session.step();
}

function reset() {
  try {
    session.reset();
    say("Reset — the machine came up as if the button had been pressed.");
  } catch (e) {
    say(String(e?.message ?? e), true);
  }
}

function eject() {
  session.shutdown();
  booted.value = false;
  hasVideo.value = false;
  hasConsole.value = false;
  say("Machine shut down.");
}

// -- media ------------------------------------------------------------------

async function takeFile(file) {
  if (!file) return;
  romBytes = new Uint8Array(await file.arrayBuffer());
  rom.value = { name: file.name, size: romBytes.length };
  say(`${file.name}: ${bytes(romBytes.length)} read. Nothing was uploaded.`);
}

function onRomPicked(event) {
  takeFile(event.target.files?.[0]);
}

function clearRom() {
  romBytes = null;
  rom.value = null;
}

function onDrop(event) {
  dragging.value = false;
  const file = event.dataTransfer?.files?.[0];
  if (!file) return;
  // A dropped file is a cartridge unless it is obviously a save state.
  if (file.name.endsWith(".rsemustate")) restoreFile(file);
  else takeFile(file);
}

// -- save states ------------------------------------------------------------

function saveState() {
  let snapshot;
  try {
    snapshot = session.save();
  } catch (e) {
    say(String(e?.message ?? e), true);
    return;
  }
  const url = URL.createObjectURL(new Blob([snapshot], { type: "application/octet-stream" }));
  const a = document.createElement("a");
  a.href = url;
  a.download = `${entry.value.name}-${new Date().toISOString().replace(/[:.]/g, "-")}.rsemustate`;
  a.click();
  URL.revokeObjectURL(url);
  say(`Saved ${bytes(snapshot.length)} at state hash ${stats.value.hash}. Nothing was uploaded.`);
}

async function restoreFile(file) {
  try {
    session.restore(new Uint8Array(await file.arrayBuffer()));
    say(`Restored — state hash ${session.emu.stateHash()}.`);
  } catch (e) {
    say(String(e?.message ?? e), true);
  }
}

function onStatePicked(event) {
  const file = event.target.files?.[0];
  if (file) restoreFile(file);
  event.target.value = ""; // so the same file can be loaded twice
}
</script>

<template>
  <div
    class="shell"
    :class="{ dragging }"
    @dragover.prevent="dragging = true"
    @dragleave="dragging = false"
    @drop.prevent="onDrop"
  >
    <header class="masthead">
      <div class="brand">
        <h1>rsemu<span class="brand-dim"> in the browser</span></h1>
        <p class="tagline">
          A whole emulated machine, client-side. Nothing is uploaded: the ROM you pick is
          read by the page, and a save state is a file this tab writes.
        </p>
      </div>
      <p class="build mono" :title="version || 'loading'">{{ version || "loading…" }}</p>
    </header>

    <p v-if="phase === 'dead'" class="fatal" role="alert">{{ fatal }}</p>

    <main v-else class="layout">
      <!-- ------------------------------------------------ the viewport -->
      <section class="viewport panel" aria-label="Machine display">
        <div class="panel-title">
          <span>{{ booted ? entry.name : "display" }}</span>
          <span v-if="booted && hasVideo" class="aspect">
            <label class="sr-only" for="aspect">Pixel aspect</label>
            <select id="aspect" v-model="aspect" class="mini">
              <option value="tv">4:3 &mdash; as a TV showed it</option>
              <option value="pixel">1:1 &mdash; square pixels</option>
            </select>
          </span>
        </div>

        <div class="viewport-body">
          <ScreenView
            v-show="booted && hasVideo"
            :width="stats.width || 256"
            :height="stats.height || 240"
            :aspect="aspect"
            :paused="paused"
            :live="booted"
            @ready="onCanvas"
          />

          <TerminalView
            v-if="booted && hasConsole"
            ref="terminal"
            :text="consoleText"
            :live="booted && !paused"
            @focus="session.consoleFocused = true"
            @blur="session.consoleFocused = false"
          />

          <div v-if="!booted" class="idle">
            <p class="idle-mark" aria-hidden="true">▮</p>
            <p class="hint">
              No machine is running. Pick one and press <strong>Boot</strong>.
            </p>
          </div>
        </div>

        <p class="status" :class="{ err: status.error }" role="status" aria-live="polite">
          {{ status.text }}
        </p>
      </section>

      <!-- ---------------------------------------------------- the rail -->
      <aside class="rail">
        <section class="panel">
          <h2 class="panel-title"><span>machine</span></h2>
          <div class="panel-body">
            <div class="field">
              <label for="machine">Catalog</label>
              <select id="machine" v-model.number="chosen" :disabled="phase !== 'ready'">
                <option v-for="m in machines" :key="m.index" :value="m.index">
                  {{ m.name }} — {{ m.summary }}
                </option>
              </select>
            </div>

            <p class="hint">{{ bootHint }}</p>

            <div class="row">
              <label class="file" :class="{ off: !needsMedia }">
                <span>{{ rom ? "Change file" : "Choose file" }}</span>
                <input
                  type="file"
                  accept=".nes,.bin,.rom,application/octet-stream"
                  :disabled="!needsMedia"
                  aria-label="Cartridge or firmware image"
                  @change="onRomPicked"
                />
              </label>
              <button v-if="rom" class="btn" type="button" @click="clearRom">Clear</button>
            </div>

            <button class="btn btn-primary boot" type="button" :disabled="!canBoot" @click="boot">
              {{ booted ? "Reboot" : "Boot" }}
            </button>
          </div>
        </section>

        <section class="panel">
          <h2 class="panel-title"><span>transport</span></h2>
          <div class="panel-body">
            <div class="row transport">
              <button class="btn" type="button" :disabled="!booted" @click="toggleRun">
                {{ paused ? "Run" : "Pause" }}
              </button>
              <button
                class="btn"
                type="button"
                :disabled="!booted"
                title="Advance exactly one video frame — the finest step the ABI offers"
                @click="step"
              >
                Step frame
              </button>
              <button class="btn" type="button" :disabled="!booted" @click="reset">Reset</button>
              <button class="btn" type="button" :disabled="!booted" @click="eject">
                Shut down
              </button>
            </div>
            <StatGrid :stats="stats" :live="booted" />
          </div>
        </section>

        <section class="panel">
          <h2 class="panel-title"><span>save state</span></h2>
          <div class="panel-body">
            <div class="row">
              <button class="btn" type="button" :disabled="!booted" @click="saveState">
                Save to a file
              </button>
              <label class="file">
                <span>Restore</span>
                <input
                  type="file"
                  accept=".rsemustate,application/octet-stream"
                  :disabled="!booted"
                  aria-label="Save state file to restore"
                  @change="onStatePicked"
                />
              </label>
            </div>
            <p class="hint">
              A snapshot restores into an identically configured machine — same description,
              same cartridge. The bytes never leave this tab.
            </p>
          </div>
        </section>

        <section v-if="hasVideo || !booted" class="panel">
          <h2 class="panel-title"><span>controller 1</span></h2>
          <div class="panel-body">
            <PadLegend
              :held="held"
              :live="booted && hasVideo"
              @press="(bit) => session.setButton(bit, true)"
              @release="(bit) => session.setButton(bit, false)"
            />
          </div>
        </section>
      </aside>
    </main>

    <footer class="about">
      <h2>What this is</h2>
      <p>
        The same emulator that runs natively, built for
        <code>wasm32-unknown-unknown</code> and driven from
        <code>requestAnimationFrame</code>. There is no <code>wasm-bindgen</code>: the
        module exports plain C functions and one file of glue is the entire boundary.
      </p>
      <p>
        The page deliberately does <em>not</em> need cross-origin isolation. Threaded
        execution needs <code>SharedArrayBuffer</code> and therefore COOP/COEP, which is
        often unavailable — so the non-threaded path is a supported target rather than a
        fallback, and it is the one shipped here.
      </p>
      <p>
        Virtual time is computed inside the emulator, never read from the host clock, so a
        run here produces the same state hash as the same run under a native debugger. The
        hash is on screen; compare it with what <code>rsemu run</code> prints.
      </p>
      <p class="colophon">
        <a href="https://github.com/KarpelesLab/rsemu">github.com/KarpelesLab/rsemu</a>
        · MIT · the page is Vue 3 and Vite, both MIT
      </p>
    </footer>

    <div v-if="dragging" class="dropzone" aria-hidden="true">Drop a ROM or a save state</div>
  </div>
</template>

<style scoped>
.shell {
  max-width: 82rem;
  margin: 0 auto;
  padding: clamp(1rem, 3vw, 2.25rem) clamp(0.85rem, 3vw, 2rem) 3rem;
}

/* -- masthead -------------------------------------------------------------- */

.masthead {
  display: flex;
  flex-wrap: wrap;
  align-items: flex-end;
  justify-content: space-between;
  gap: 0.75rem 1.5rem;
  padding-bottom: 1rem;
  margin-bottom: 1.25rem;
  border-bottom: 1px solid var(--line);
}

h1 {
  font-size: clamp(1.35rem, 3.4vw, 1.85rem);
}

.brand-dim {
  font-weight: 400;
  color: var(--fg-faint);
}

.tagline {
  max-width: 46ch;
  margin: 0.35rem 0 0;
  font-size: 0.88rem;
  color: var(--fg-dim);
}

.build {
  max-width: 26rem;
  margin: 0;
  font-size: 0.7rem;
  line-height: 1.4;
  text-align: right;
  color: var(--fg-faint);
  overflow-wrap: anywhere;
}

/* -- layout ---------------------------------------------------------------- */

.layout {
  display: grid;
  grid-template-columns: minmax(0, 1fr) 21.5rem;
  align-items: start;
  gap: 1.1rem;
}

@media (max-width: 62rem) {
  .layout {
    grid-template-columns: minmax(0, 1fr);
  }
}

.rail {
  display: grid;
  gap: 1.1rem;
}

/* -- viewport -------------------------------------------------------------- */

.viewport {
  overflow: hidden;
}

.viewport-body {
  display: grid;
  gap: 1rem;
  padding: 1rem;
  background: var(--panel-2);
}

.idle {
  display: grid;
  place-items: center;
  gap: 0.5rem;
  min-height: 18rem;
  text-align: center;
}

.idle-mark {
  margin: 0;
  font-size: 2.5rem;
  line-height: 1;
  color: var(--line-strong);
  animation: pulse 2.4s ease-in-out infinite;
}

@keyframes pulse {
  50% {
    opacity: 0.3;
  }
}

.status {
  margin: 0;
  padding: 0.6rem 0.9rem;
  min-height: 2.5rem;
  font-size: 0.85rem;
  color: var(--fg-dim);
  border-top: 1px solid var(--line);
}

.status.err {
  color: var(--danger);
  font-weight: 550;
}

.mini {
  width: auto;
  min-height: 1.7rem;
  padding: 0.05rem 0.35rem;
  font-size: 0.72rem;
  font-weight: 500;
  letter-spacing: 0;
  text-transform: none;
}

/* -- rail bits ------------------------------------------------------------- */

.boot {
  width: 100%;
  min-height: 2.4rem;
}

.transport {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 0.4rem;
}

.file.off {
  opacity: 0.42;
}

/* -- about ----------------------------------------------------------------- */

.about {
  max-width: 60ch;
  margin-top: 2.5rem;
  padding-top: 1.5rem;
  border-top: 1px solid var(--line);
  font-size: 0.88rem;
  color: var(--fg-dim);
}

.about h2 {
  margin-bottom: 0.5rem;
  font-size: 0.95rem;
  color: var(--fg);
}

.about p {
  margin: 0 0 0.75rem;
}

.colophon {
  font-size: 0.8rem;
  color: var(--fg-faint);
}

/* -- errors and drops ------------------------------------------------------ */

.fatal {
  max-width: 60ch;
  padding: 1rem 1.15rem;
  color: var(--danger);
  background: var(--panel);
  border: 1px solid var(--danger);
  border-radius: var(--radius);
}

.dropzone {
  position: fixed;
  inset: 1rem;
  display: grid;
  place-items: center;
  font: 600 1.1rem var(--sans);
  color: var(--accent);
  background: color-mix(in srgb, var(--bg) 82%, transparent);
  border: 2px dashed var(--accent);
  border-radius: var(--radius);
  pointer-events: none;
  z-index: 10;
}
</style>
