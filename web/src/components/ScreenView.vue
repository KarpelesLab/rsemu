<script setup>
// The picture.
//
// The canvas element is handed straight to the session on mount and never
// touched by Vue again: the emulator writes pixels with putImageData, and this
// component's only job is deciding how big the element is on screen. Nothing
// reactive here is larger than a number.

import { computed, onBeforeUnmount, onMounted, ref, watch } from "vue";

const props = defineProps({
  width: { type: Number, default: 256 },
  height: { type: Number, default: 240 },
  /** "pixel" = square guest pixels; "tv" = the 4:3 a console was wired for. */
  aspect: { type: String, default: "tv" },
  paused: { type: Boolean, default: false },
  live: { type: Boolean, default: false },
  /** Whether the guest takes keys, which is what makes the picture focusable. */
  keyboard: { type: Boolean, default: false },
});

const emit = defineEmits(["ready", "focus", "blur"]);

// A picture is not focusable unless there is something to type at. Giving
// every canvas a tab stop would put a dead stop in the page's tab order for
// the machines that have no keyboard, which is most of them.
const focused = ref(false);

const stage = ref(null);
const canvas = ref(null);
const scale = ref(1);

/** Widest the picture may get before it stops being a picture and starts
 *  being wallpaper. Six times a NES frame is already 1536 px. */
const MAX_SCALE = 8;

function measure() {
  const box = stage.value;
  if (!box || !props.width || !props.height) return;
  // Width comes from the layout; height is bounded by the viewport instead of
  // by the stage, because the stage's own height is *derived* from the canvas
  // and observing it would be a feedback loop.
  const availableWidth = box.clientWidth;
  const availableHeight = Math.max(200, window.innerHeight * 0.74);

  // Integer scaling is the whole point: a NES pixel is a square of identical
  // host pixels or it is a shimmering mess. In "tv" the *vertical* scale stays
  // integral and the width is stretched to 4:3, which is the compromise a
  // non-square pixel aspect forces on anyone.
  const displayWidth = props.aspect === "tv" ? (props.height * 4) / 3 : props.width;
  const fit = Math.min(availableWidth / displayWidth, availableHeight / props.height);
  scale.value = Math.max(1, Math.min(MAX_SCALE, Math.floor(fit)));
}

const style = computed(() => {
  const h = props.height * scale.value;
  const w = props.aspect === "tv" ? Math.round((h * 4) / 3) : props.width * scale.value;
  return { width: `${w}px`, height: `${h}px` };
});

let observer = null;
onMounted(() => {
  emit("ready", canvas.value);
  observer = new ResizeObserver(measure);
  observer.observe(stage.value);
  addEventListener("resize", measure);
  measure();
});
onBeforeUnmount(() => {
  observer?.disconnect();
  removeEventListener("resize", measure);
});
watch(() => [props.width, props.height, props.aspect], measure);

defineExpose({ canvas });
</script>

<template>
  <div class="stage" ref="stage">
    <div class="bezel" :style="style">
      <canvas
        ref="canvas"
        class="screen"
        :width="width || 256"
        :height="height || 240"
        role="img"
        :class="{ focused: keyboard && focused }"
        :tabindex="keyboard ? 0 : undefined"
        :aria-label="`Emulated display, ${width} by ${height} pixels`"
        @focus="((focused = true), emit('focus'))"
        @blur="((focused = false), emit('blur'))"
      ></canvas>
      <div v-if="live && paused" class="paused-badge">Paused</div>
    </div>
    <p class="readout mono">
      {{ width }}&times;{{ height }} &middot; &times;{{ scale }}
      {{ aspect === "tv" ? "at 4:3" : "square pixels" }}
    </p>
    <p v-if="keyboard" class="readout">
      <template v-if="focused">
        Keys reach the guest's keyboard &mdash; make and break codes, as the wire carries
        them.
      </template>
      <template v-else>Click the picture to type at it.</template>
    </p>
  </div>
</template>

<style scoped>
.stage {
  display: grid;
  justify-items: center;
  gap: 0.5rem;
}

.bezel {
  position: relative;
  padding: 10px;
  background: var(--bezel);
  border: 1px solid var(--bezel-line);
  border-radius: 12px;
  box-shadow:
    inset 0 0 0 1px rgb(255 255 255 / 0.04),
    0 18px 40px -24px rgb(0 0 0 / 0.9);
  /* The declared size is the picture; the padding is the plastic around it. */
  box-sizing: content-box;
}

.screen {
  display: block;
  width: 100%;
  height: 100%;
  /* A NES picture is not a photograph, and smoothing it is a lie about the
     hardware. Every vendor prefix that ever meant this, because the one that
     is standard today is not the one older engines answer to. */
  image-rendering: pixelated;
  image-rendering: crisp-edges;
  background: #000;
  border-radius: 2px;
}

.screen:focus-visible,
.screen.focused {
  outline: 2px solid var(--phosphor);
  outline-offset: 3px;
}

.paused-badge {
  position: absolute;
  top: 18px;
  right: 18px;
  padding: 0.15rem 0.5rem;
  font: 650 0.7rem/1.6 var(--mono);
  letter-spacing: 0.08em;
  text-transform: uppercase;
  color: var(--bezel);
  background: var(--phosphor);
  border-radius: 4px;
}

.readout {
  margin: 0;
  font-size: 0.74rem;
  color: var(--fg-faint);
}
</style>
