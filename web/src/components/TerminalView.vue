<script setup>
// The Apple 1 is a console, not a framebuffer, so it gets its own view.
//
// The distinction is not cosmetic: `rsemu_has_video` is false for this machine
// and `rsemu_has_console` is true, there are no pixels to blit at all, and the
// keyboard carries characters rather than controller bits. Rendering a
// terminal as a 40x24 canvas would be inventing hardware the Apple 1 did not
// have — it drove a television through a shift register and the host decides
// what a "line" looks like.

import { nextTick, ref, watch } from "vue";

const props = defineProps({
  text: { type: String, default: "" },
  live: { type: Boolean, default: false },
});

const emit = defineEmits(["focus", "blur"]);

const pane = ref(null);
const focused = ref(false);

// Follow the output, but only when the reader has not scrolled up to look at
// something — yanking the viewport away mid-read is the classic terminal bug.
let pinned = true;
function onScroll() {
  const el = pane.value;
  if (!el) return;
  pinned = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
}

watch(
  () => props.text,
  async () => {
    if (!pinned) return;
    await nextTick();
    const el = pane.value;
    if (el) el.scrollTop = el.scrollHeight;
  },
);

function focus() {
  pane.value?.focus();
}

defineExpose({ focus });
</script>

<template>
  <div class="terminal-wrap">
    <div
      ref="pane"
      class="terminal"
      :class="{ focused }"
      tabindex="0"
      role="log"
      aria-label="Machine console output. Focus this pane to type at the machine."
      @scroll="onScroll"
      @focus="((focused = true), emit('focus'))"
      @blur="((focused = false), emit('blur'))"
    >
      <span class="out">{{ text }}</span
      ><span class="cursor" :class="{ on: focused && live }" aria-hidden="true">&#9608;</span>
    </div>

    <p class="hint">
      <template v-if="focused">
        <strong>Typing goes to the machine.</strong> RSMON is a hex examine/deposit
        monitor: type <kbd>F</kbd><kbd>F</kbd><kbd>0</kbd><kbd>0</kbd> then
        <kbd>Enter</kbd> to dump eight bytes, <kbd>Enter</kbd> again to walk on, or
        <kbd>0</kbd><kbd>3</kbd><kbd>0</kbd><kbd>0</kbd><kbd>:</kbd><kbd>A</kbd><kbd>A</kbd>
        to deposit one. The upper case is the keyboard's doing, not the page's.
      </template>
      <template v-else>
        Click the screen, or tab to it, to type at the machine. Keystrokes only
        reach the guest while this pane has focus.
      </template>
    </p>
  </div>
</template>

<style scoped>
.terminal-wrap {
  display: grid;
  gap: 0.6rem;
}

.terminal {
  height: clamp(16rem, 52vh, 34rem);
  padding: 1rem 1.1rem;
  overflow-y: auto;
  overflow-wrap: anywhere;
  white-space: pre-wrap;
  font: 14px/1.45 var(--mono);
  color: var(--phosphor);
  background:
    repeating-linear-gradient(
      to bottom,
      rgb(255 255 255 / 0.028) 0 1px,
      transparent 1px 3px
    ),
    radial-gradient(120% 100% at 50% 0%, #1c2620, var(--bezel) 70%);
  border: 1px solid var(--bezel-line);
  border-radius: 12px;
  box-shadow: inset 0 0 60px -20px rgb(143 240 164 / 0.25);
  text-shadow: 0 0 6px rgb(143 240 164 / 0.35);
  cursor: text;
  transition: box-shadow 150ms ease;
}

.terminal.focused {
  box-shadow:
    inset 0 0 60px -20px rgb(143 240 164 / 0.4),
    0 0 0 2px var(--accent);
  outline: none;
}

.out {
  color: inherit;
}

.cursor {
  color: var(--phosphor-dim);
}

.cursor.on {
  color: var(--phosphor);
  animation: blink 1.06s step-end infinite;
}

@keyframes blink {
  50% {
    opacity: 0;
  }
}

kbd {
  margin-right: 1px;
}
</style>
