<script setup>
// What is interesting about a running machine.
//
// The state hash earns its place here: virtual time is computed inside the
// emulator and never read from the host clock, so this number is the same one
// `rsemu run` prints for the same session (ROADMAP.md §11.6). It is the page's
// evidence that the browser is not a different emulator.

import { computed, ref } from "vue";

const props = defineProps({
  stats: { type: Object, required: true },
  live: { type: Boolean, default: false },
});

const hz = computed(() =>
  props.stats.framePeriodMs ? (1000 / props.stats.framePeriodMs).toFixed(3) : "—",
);

const elapsed = computed(() => {
  const s = props.stats.seconds ?? 0;
  if (s < 60) return `${s.toFixed(2)} s`;
  const m = Math.floor(s / 60);
  return `${m}m ${(s - m * 60).toFixed(1)}s`;
});

const copied = ref(false);
async function copyHash() {
  try {
    await navigator.clipboard.writeText(props.stats.hash);
    copied.value = true;
    setTimeout(() => (copied.value = false), 1400);
  } catch {
    // Clipboard permission is the user's to withhold; the hash is on screen
    // either way, so this is not worth an error message.
  }
}
</script>

<template>
  <dl class="stats">
    <div class="stat">
      <dt>Frames / s</dt>
      <dd class="mono big" :class="{ idle: !live || stats.paused }">
        {{ live ? (stats.paused ? "—" : stats.fps) : "—" }}
      </dd>
    </div>
    <div class="stat">
      <dt>Frame</dt>
      <dd class="mono">{{ live ? stats.frame.toLocaleString() : "—" }}</dd>
    </div>
    <div class="stat">
      <dt>Virtual time</dt>
      <dd class="mono">{{ live ? elapsed : "—" }}</dd>
    </div>
    <div class="stat">
      <dt>Refresh</dt>
      <dd class="mono">{{ live ? `${hz} Hz` : "—" }}</dd>
    </div>
    <div class="stat wide">
      <dt>State hash</dt>
      <dd class="mono hash">
        <span>{{ stats.hash }}</span>
        <button
          v-if="live"
          class="copy"
          type="button"
          @click="copyHash"
          :aria-label="`Copy state hash ${stats.hash} to the clipboard`"
        >
          {{ copied ? "copied" : "copy" }}
        </button>
      </dd>
    </div>
  </dl>
</template>

<style scoped>
.stats {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(7rem, 1fr));
  gap: 1px;
  margin: 0;
  background: var(--line);
  border: 1px solid var(--line);
  border-radius: var(--radius-sm);
  overflow: hidden;
}

.stat {
  padding: 0.55rem 0.7rem;
  background: var(--panel);
}

.wide {
  grid-column: 1 / -1;
}

dt {
  font-size: 0.68rem;
  font-weight: 650;
  letter-spacing: 0.07em;
  text-transform: uppercase;
  color: var(--fg-faint);
}

dd {
  margin: 0.15rem 0 0;
  font-size: 0.92rem;
  color: var(--fg);
}

.big {
  font-size: 1.15rem;
  font-weight: 600;
  color: var(--accent);
}

.big.idle {
  color: var(--fg-faint);
}

.hash {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 0.5rem;
  font-size: 0.86rem;
}

.copy {
  padding: 0.1rem 0.45rem;
  font: 600 0.68rem/1.6 var(--sans);
  letter-spacing: 0.05em;
  text-transform: uppercase;
  color: var(--fg-dim);
  background: var(--panel-2);
  border: 1px solid var(--line-strong);
  border-radius: 4px;
  cursor: pointer;
}

.copy:hover {
  color: var(--accent);
  border-color: var(--accent);
}
</style>
