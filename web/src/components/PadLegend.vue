<script setup>
// The controller map, lit up as it is used.
//
// Showing the live mask is not decoration: the seam is a *level*, not an
// event. The console samples $4016 whenever the guest strobes it, so a button
// stays held until the page clears it — which is exactly what makes a recorded
// session replayable, and exactly the thing that looks like a bug when a key
// sticks and nothing on screen says so.

import { BUTTONS } from "../session.js";

defineProps({
  held: { type: Number, default: 0 },
  live: { type: Boolean, default: false },
});

const emit = defineEmits(["press", "release"]);

/** Laid out the way the pad is, not the way the shift register is. */
const DPAD = [
  { bit: BUTTONS.up, glyph: "↑", name: "Up", area: "u" },
  { bit: BUTTONS.left, glyph: "←", name: "Left", area: "l" },
  { bit: BUTTONS.down, glyph: "↓", name: "Down", area: "d" },
  { bit: BUTTONS.right, glyph: "→", name: "Right", area: "r" },
];

const FACE = [
  { bit: BUTTONS.select, glyph: "Select", name: "Select", key: "Shift" },
  { bit: BUTTONS.start, glyph: "Start", name: "Start", key: "Enter" },
  { bit: BUTTONS.b, glyph: "B", name: "B", key: "X" },
  { bit: BUTTONS.a, glyph: "A", name: "A", key: "Z" },
];

function hex(mask) {
  return "0x" + mask.toString(16).padStart(2, "0");
}
</script>

<template>
  <div class="pad">
    <div class="cluster dpad">
      <button
        v-for="b in DPAD"
        :key="b.name"
        type="button"
        class="key"
        :class="[`area-${b.area}`, { on: (held & b.bit) !== 0 }]"
        :disabled="!live"
        :aria-pressed="(held & b.bit) !== 0"
        :aria-label="`${b.name} — arrow key`"
        @pointerdown.prevent="emit('press', b.bit)"
        @pointerup.prevent="emit('release', b.bit)"
        @pointerleave="emit('release', b.bit)"
        @pointercancel="emit('release', b.bit)"
      >
        {{ b.glyph }}
      </button>
    </div>

    <div class="cluster face">
      <button
        v-for="b in FACE"
        :key="b.name"
        type="button"
        class="key wide"
        :class="{ on: (held & b.bit) !== 0 }"
        :disabled="!live"
        :aria-pressed="(held & b.bit) !== 0"
        :aria-label="`${b.name} — ${b.key} key`"
        @pointerdown.prevent="emit('press', b.bit)"
        @pointerup.prevent="emit('release', b.bit)"
        @pointerleave="emit('release', b.bit)"
        @pointercancel="emit('release', b.bit)"
      >
        <span>{{ b.glyph }}</span>
        <kbd>{{ b.key }}</kbd>
      </button>
    </div>
  </div>

  <p class="hint">
    Arrow keys are the d-pad. The mask reaches the console's controller port at
    <code>$4016</code> as a level the guest samples when it strobes — currently
    <code class="mask">{{ hex(held) }}</code
    >.
  </p>
</template>

<style scoped>
.pad {
  display: flex;
  flex-wrap: wrap;
  gap: 1rem;
  align-items: center;
  justify-content: space-between;
}

.dpad {
  display: grid;
  grid-template-areas: ". u ." "l . r" ". d .";
  gap: 3px;
}

.area-u {
  grid-area: u;
}
.area-l {
  grid-area: l;
}
.area-d {
  grid-area: d;
}
.area-r {
  grid-area: r;
}

.face {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 4px;
  flex: 1 1 11rem;
}

.key {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: 0.35rem;
  min-width: 2.1rem;
  min-height: 2.1rem;
  padding: 0.2rem 0.4rem;
  font: 600 0.8rem/1 var(--sans);
  color: var(--fg-dim);
  background: var(--panel-2);
  border: 1px solid var(--line-strong);
  border-radius: var(--radius-sm);
  cursor: pointer;
  user-select: none;
  touch-action: none;
  transition:
    background 90ms ease,
    color 90ms ease,
    border-color 90ms ease;
}

.key.wide {
  justify-content: space-between;
  padding-inline: 0.55rem;
}

.key:disabled {
  opacity: 0.4;
  cursor: not-allowed;
}

.key.on {
  color: var(--accent-fg);
  background: var(--accent);
  border-color: var(--accent);
}

.key.on kbd {
  color: var(--accent-fg);
  background: transparent;
  border-color: currentColor;
}

kbd {
  pointer-events: none;
}

.mask {
  color: var(--accent);
}
</style>
