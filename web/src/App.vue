<script setup>
// The page's chrome: picker, transport, media, save states, stats.
//
// Vue owns everything in this file and nothing outside it. The emulator lives
// in `session.js` as a plain object — deliberately not a `ref`, not `reactive`,
// not `shallowRef` — because making it reactive would put a Proxy between the
// frame loop and the framebuffer sixty times a second. What crosses back into
// Vue is a handful of numbers and one string.

import { computed, nextTick, onBeforeUnmount, onMounted, ref, shallowRef, watch } from "vue";
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
// Which image the next boot binds: an index into this machine's built-in
// images, or FROM_FILE for one the visitor opens. A number rather than an
// object, because it is a `v-model` on a radio group.
const FROM_FILE = -1;
const image = ref(FROM_FILE);

// The ROM image is a Uint8Array of up to a few megabytes and must never become
// a Proxy; only its name and size are of any interest to the UI.
let romBytes = null;
const rom = ref(null); // { name, size } | null

const booted = ref(false);
// What is actually running, as opposed to what the picker is showing. The
// console pane needs it: RSMON and the Woz Monitor take different commands.
const running = ref(null);
const paused = ref(true);
const hasVideo = ref(false);
const hasConsole = ref(false);
// Whether the machine has controllers, which is not the same as having a
// picture: a display panel with no game pad would otherwise get a d-pad drawn
// for hardware it does not have.
const hasPad = ref(false);
const consoleText = ref("");
const held = ref(0);
const status = ref({ text: "Loading the module…", error: false });
const aspect = ref("tv");
const dragging = ref(false);

// Sound. `soundOn` is what is actually coming out of the speakers, which is not
// the same as what the visitor asked for: a browser may refuse until a gesture,
// and it is honest to show the difference.
const soundSupported = ref(false);
const soundWanted = ref(false);
const soundOn = ref(false);
const soundRate = ref(0);
const volume = ref(70);

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
  hasAudio: false,
  sound: false,
  soundRate: 0,
  audioDropped: 0,
});

const terminal = ref(null);

// -- derived ----------------------------------------------------------------

// Catalog indices happen to match array positions today, but the ABI hands
// out an index and that is what the option value carries — look it up.
const entry = computed(() => machines.value.find((m) => m.index === chosen.value) ?? null);
const needsMedia = computed(() => Boolean(entry.value?.media));
// The images the module carries for this machine. A machine with at least one
// runs with nothing uploaded, which is the only thing a first-time visitor
// really wants to know about it.
const builtins = computed(() => entry.value?.builtins ?? []);
const fromFile = computed(() => image.value === FROM_FILE);
const chosenImage = computed(() => (fromFile.value ? null : (builtins.value[image.value] ?? null)));
const canBoot = computed(() => {
  if (phase.value !== "ready" || entry.value === null) return false;
  if (!fromFile.value) return chosenImage.value !== null;
  return !needsMedia.value || rom.value !== null;
});

// Every (machine, image) pair in this build that runs with no file at all —
// read out of the ABI, never a list kept by hand here. This is what the idle
// screen offers, so the first thing a visitor can do is press one button.
const quickStarts = computed(() =>
  machines.value.flatMap((m) => m.builtins.map((b) => ({ machine: m, image: b }))),
);

// A machine picked from the catalog defaults to its own first image, so
// changing the selection never leaves the page asking for a file it could have
// supplied itself.
watch(entry, (m) => {
  image.value = m && m.builtins.length > 0 ? 0 : FROM_FILE;
});

const bootHint = computed(() => {
  const m = entry.value;
  if (!m) return "";
  const b = chosenImage.value;
  if (b) {
    return `${b.name} is compiled into this page — nothing to upload. It goes in the “${b.slot}” slot: ${b.summary}.`;
  }
  if (!m.media) return `${m.name} takes no image at all.`;
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
      soundOn.value = s.sound;
      soundRate.value = s.soundRate;
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

  // Default to a machine that runs with nothing uploaded, so a first-time
  // visitor has something to press rather than a file picker to satisfy.
  const free = machines.value.find((m) => m.builtins.length > 0 || !m.media);
  chosen.value = (free ?? machines.value[0]).index;
  image.value = entry.value?.builtins.length ? 0 : FROM_FILE;

  soundSupported.value = session.soundSupported;
  soundWanted.value = session.soundWanted;
  session.setVolume(volume.value / 100);

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

/** Boot the machine and image the picker is showing. */
function boot() {
  return bootWith(entry.value, chosenImage.value);
}

/**
 * Boot `m` on `b`, or on the file the visitor opened when `b` is null.
 *
 * Takes both explicitly rather than reading the picker, so the idle screen's
 * buttons do not depend on a `watch` having run first.
 */
async function bootWith(m, b) {
  try {
    session.boot(m, romBytes, b);
  } catch (e) {
    say(String(e?.message ?? e), true);
    return;
  }
  booted.value = true;
  running.value = { machine: m, image: b };
  hasVideo.value = session.hasVideo;
  hasConsole.value = session.hasConsole;
  hasPad.value = session.hasPad;
  const on = b ? ` on ${b.name}` : rom.value ? ` — ${rom.value.name}` : "";
  say(`${m.name} running${on}.`);
  // A console machine is useless until it has focus, and asking the visitor to
  // discover that is a worse page than just giving it to them.
  if (session.hasConsole && !session.hasVideo) {
    await nextTick();
    terminal.value?.focus();
  }
}

/** One of the idle screen's "nothing to upload" buttons. */
async function quickStart(start) {
  await bootWith(start.machine, start.image);
  // Leave the picker showing what is running, so Reboot means what it says.
  chosen.value = start.machine.index;
  await nextTick();
  image.value = start.image.index;
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

function toggleSound() {
  soundWanted.value = session.toggleSound();
  soundOn.value = session.soundOn;
  soundRate.value = session.speaker.rate;
  say(
    soundWanted.value
      ? `Sound on — resampled from the console's own crystal to ${session.speaker.rate || "?"} Hz.`
      : "Muted. The machine runs identically either way — the state hash is the proof.",
  );
}

function onVolume(event) {
  volume.value = Number(event.target.value);
  session.setVolume(volume.value / 100);
}

function eject() {
  session.shutdown();
  booted.value = false;
  running.value = null;
  hasVideo.value = false;
  hasConsole.value = false;
  hasPad.value = false;
  say("Machine shut down.");
}

// -- media ------------------------------------------------------------------

async function takeFile(file) {
  if (!file) return;
  romBytes = new Uint8Array(await file.arrayBuffer());
  rom.value = { name: file.name, size: romBytes.length };
  // Opening a file is a choice: switch the picker to it, or the next Boot
  // would quietly run a built-in image instead of what was just dropped.
  if (needsMedia.value) image.value = FROM_FILE;
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
  const name = running.value?.machine.name ?? "rsemu";
  a.download = `${name}-${new Date().toISOString().replace(/[:.]/g, "-")}.rsemustate`;
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
          A whole emulated machine, client-side. Several boot on ROMs compiled into this
          page &mdash; including the Woz Monitor of 1976 &mdash; so there is something to
          press before there is anything to open. Nothing is uploaded either way: a
          cartridge you pick is read here, and a save state is a file this tab writes.
        </p>
      </div>
      <p class="build mono" :title="version || 'loading'">{{ version || "loading…" }}</p>
    </header>

    <p v-if="phase === 'dead'" class="fatal" role="alert">{{ fatal }}</p>

    <main v-else class="layout">
      <!-- ------------------------------------------------ the viewport -->
      <section class="viewport panel" aria-label="Machine display">
        <div class="panel-title">
          <span>{{ booted ? running.machine.name : "display" }}</span>
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
            :monitor="running?.image?.name ?? ''"
            @focus="session.consoleFocused = true"
            @blur="session.consoleFocused = false"
          />

          <!-- A machine with neither a picture nor a console would otherwise
               be a black rectangle claiming to be a computer. -->
          <div v-if="booted && !hasVideo && !hasConsole" class="idle">
            <p class="idle-mark" aria-hidden="true">▮</p>
            <p class="hint">
              {{ running.machine.name }} is running, and this build has no way to look at it: no
              display device and no console. The state hash and the clock on the right are
              still the machine's own.
            </p>
          </div>

          <div v-if="!booted" class="idle">
            <p class="idle-mark" aria-hidden="true">▮</p>
            <template v-if="quickStarts.length">
              <p class="hint">
                Nothing to upload &mdash; these boot on images compiled into this page.
              </p>
              <ul class="starts">
                <li v-for="s in quickStarts" :key="`${s.machine.name}/${s.image.name}`">
                  <button
                    class="btn start"
                    type="button"
                    :disabled="phase !== 'ready'"
                    @click="quickStart(s)"
                  >
                    <span class="start-name mono">{{ s.machine.name }} · {{ s.image.name }}</span>
                    <span class="start-note">{{ s.image.summary }}</span>
                  </button>
                </li>
              </ul>
              <p class="hint">
                Or pick any machine on the right &mdash; a cartridge is a file you open, and
                it is read here rather than sent anywhere.
              </p>
            </template>
            <p v-else class="hint">
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

            <fieldset v-if="builtins.length || needsMedia" class="images">
              <legend>Boot on</legend>
              <label v-for="b in builtins" :key="b.name" class="choice">
                <input type="radio" :value="b.index" v-model.number="image" />
                <span class="choice-name mono">{{ b.name }}</span>
                <span class="choice-note">{{ b.summary }}</span>
              </label>
              <label v-if="needsMedia" class="choice">
                <input type="radio" :value="-1" v-model.number="image" />
                <span class="choice-name">a file of your own</span>
                <span class="choice-note">
                  read here, never uploaded &mdash; it fills the &ldquo;{{ entry.media }}&rdquo;
                  slot
                </span>
              </label>
            </fieldset>

            <p class="hint">{{ bootHint }}</p>

            <div v-if="fromFile && needsMedia" class="row">
              <label class="file">
                <span>{{ rom ? "Change file" : "Choose file" }}</span>
                <input
                  type="file"
                  accept=".nes,.bin,.rom,application/octet-stream"
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
          <h2 class="panel-title"><span>sound</span></h2>
          <div class="panel-body">
            <div class="row sound">
              <button
                class="btn"
                type="button"
                :disabled="!soundSupported"
                :aria-pressed="soundWanted"
                @click="toggleSound"
              >
                {{ soundWanted ? "Mute" : "Unmute" }}
              </button>
              <label class="sr-only" for="volume">Volume</label>
              <input
                id="volume"
                class="volume"
                type="range"
                min="0"
                max="100"
                :value="volume"
                :disabled="!soundSupported"
                @input="onVolume"
              />
              <span class="mono volume-read">{{ volume }}%</span>
            </div>
            <p class="hint">
              <template v-if="!soundSupported">
                This browser has no WebAudio, so there is nothing to play into.
              </template>
              <template v-else-if="!booted">
                Sound starts with the machine — a page may only open an audio context from a
                click, and Boot is one.
              </template>
              <template v-else-if="!stats.hasAudio">
                {{ running.machine.name }} has no audio device, so there is nothing to hear.
              </template>
              <template v-else-if="soundOn">
                Playing at {{ soundRate }} Hz, resampled from the console's own crystal —
                894 886.36… Hz on an NTSC NES — with exact integer phase.
                <template v-if="stats.audioDropped">
                  {{ stats.audioDropped }} frames dropped because this tab could not keep up;
                  the machine ran identically anyway.
                </template>
              </template>
              <template v-else>
                Muted. The emulator still produces every sample and throws them away here, so
                the state hash is the same either way.
              </template>
            </p>
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

        <section v-if="hasPad || !booted" class="panel">
          <h2 class="panel-title"><span>controller 1</span></h2>
          <div class="panel-body">
            <PadLegend
              :held="held"
              :live="booted && hasPad"
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
        The machine list is not written here &mdash; a machine is a feature set, so this
        page asks the module it fetched what is in it, and the same question gets the
        images it carries. Those are rsemu's own monitors and one board's demonstration
        firmware, plus Steve Wozniak's monitor of 1976, whose listing was published
        without a copyright notice and is public domain. A cartridge is yours to supply,
        which is why none is shipped here.
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

/* -- the idle screen's one-click starts ------------------------------------ */

.starts {
  display: grid;
  gap: 0.5rem;
  width: min(34rem, 100%);
  margin: 0;
  padding: 0;
  list-style: none;
}

.start {
  display: grid;
  gap: 0.15rem;
  width: 100%;
  padding: 0.6rem 0.8rem;
  text-align: left;
}

.start-name {
  font-size: 0.9rem;
  font-weight: 650;
  color: var(--fg);
}

.start-note {
  font-size: 0.78rem;
  font-weight: 400;
  letter-spacing: 0;
  text-transform: none;
  color: var(--fg-dim);
}

/* -- the image chooser ----------------------------------------------------- */

.images {
  display: grid;
  gap: 0.4rem;
  margin: 0;
  padding: 0.6rem 0.7rem;
  border: 1px solid var(--line);
  border-radius: var(--radius-sm);
}

.images legend {
  padding: 0 0.35rem;
  font-size: 0.68rem;
  font-weight: 650;
  letter-spacing: 0.07em;
  text-transform: uppercase;
  color: var(--fg-faint);
}

.choice {
  display: grid;
  grid-template-columns: auto minmax(0, 1fr);
  align-items: baseline;
  gap: 0.1rem 0.5rem;
  cursor: pointer;
}

.choice input {
  grid-row: span 2;
  margin: 0;
  accent-color: var(--accent);
}

.choice-name {
  font-size: 0.85rem;
  font-weight: 600;
  color: var(--fg);
}

.choice-note {
  grid-column: 2;
  font-size: 0.76rem;
  color: var(--fg-dim);
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
