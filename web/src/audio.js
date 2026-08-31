// The speaker: WebAudio's half of the audio seam.
//
// rsemu hands out interleaved `f32` frames already resampled to whatever rate
// it was told the page runs at (`src/host/audio`), so there is nothing to
// convert here. What is left is the one thing only a browser can do: decide
// *when* each block is heard.
//
// The scheduling model is a playhead. Every animation frame the page copies
// whatever has accumulated into an `AudioBuffer` and starts it at
// `this.playhead`, then advances the playhead by the block's own duration. The
// blocks therefore butt up against each other exactly, with no gap and no
// overlap, and the only clock involved is the `AudioContext`'s.
//
// Two things can go wrong and both are handled by moving the playhead rather
// than by touching the emulator:
//
//   * **Underrun.** The tab was backgrounded, or a frame took too long, and the
//     playhead is now in the past. Restart it a little ahead of `currentTime` —
//     the audio has a hole in it, which is honest, rather than being scheduled
//     into a moment that has gone.
//   * **Drift.** The machine is running slightly faster than real time, so the
//     playhead creeps ahead and latency grows without bound. Past a quarter of
//     a second, drop the block instead of queueing it.
//
// **None of this reaches the guest.** A dropped block is a host that could not
// keep up; the machine ran exactly as it would have with the tab muted, and its
// state hash says so. That is the property `src/host/audio` exists to keep.

// How far ahead of `currentTime` a restarted playhead is placed. Enough to
// cover one animation frame plus the browser's own block size; less than this
// and every hitch is audible, more and the sound lags the picture.
const LEAD_SECONDS = 0.06;

// Past this much queued audio the machine is outrunning the clock, so blocks
// are discarded rather than queued. A quarter of a second of latency is already
// more than a player will accept.
const MAX_AHEAD_SECONDS = 0.25;

/** Whether this browser has WebAudio at all. */
export function audioSupported() {
  return typeof globalThis.AudioContext === "function";
}

export class Speaker {
  constructor() {
    /** @type {AudioContext|null} */
    this.ctx = null;
    /** @type {GainNode|null} */
    this.gain = null;
    /** When the next block should start, on the context's own timeline. */
    this.playhead = 0;
    /** Blocks discarded because the queue had run too far ahead. */
    this.starved = 0;
    this.volume = 0.7;
  }

  /** True once there is a running context to schedule into. */
  get playing() {
    return this.ctx !== null && this.ctx.state === "running";
  }

  /**
   * The rate the context actually runs at, which is the browser's choice and
   * not ours — 48 000 on most desktops, 44 100 on plenty of others.
   */
  get rate() {
    return this.ctx ? this.ctx.sampleRate : 0;
  }

  /**
   * Create and resume the context.
   *
   * **Must be called from inside a user gesture.** A page may not start audio
   * on its own, so this is wired to the Boot and Sound buttons and to nothing
   * that runs by itself.
   */
  enable() {
    if (!audioSupported()) return false;
    if (!this.ctx) {
      this.ctx = new AudioContext();
      this.gain = this.ctx.createGain();
      this.gain.gain.value = this.volume;
      this.gain.connect(this.ctx.destination);
    }
    // Fire and forget: the promise resolves after the gesture has ended, and
    // `playing` reports the truth in the meantime.
    this.ctx.resume().catch(() => {});
    this.playhead = 0;
    return true;
  }

  /** Stop scheduling and let the context go. */
  disable() {
    if (!this.ctx) return;
    const ctx = this.ctx;
    this.ctx = null;
    this.gain = null;
    this.playhead = 0;
    ctx.close().catch(() => {});
  }

  /** 0 to 1. Kept across a disable, so unmuting comes back where it was. */
  setVolume(value) {
    this.volume = Math.min(1, Math.max(0, value));
    if (this.gain) this.gain.gain.value = this.volume;
  }

  /** Forget the playhead, so the next block starts fresh. After a pause. */
  silence() {
    this.playhead = 0;
  }

  /**
   * Copy everything rsemu has queued and schedule it.
   *
   * @param {import("./rsemu.js").Rsemu} emu
   * @returns {number} frames scheduled
   */
  push(emu) {
    const frames = emu.audioFrames();
    if (frames === 0) return 0;
    if (!this.playing) {
      // Muted, or the context has not started yet. Drop the frames rather than
      // letting them pile up: rsemu counts a queue it had to trim as a *loss*,
      // and a muted tab has not lost anything.
      emu.audioConsume(frames);
      return 0;
    }

    const ctx = this.ctx;
    const now = ctx.currentTime;
    if (this.playhead > now + MAX_AHEAD_SECONDS) {
      this.starved += 1;
      emu.audioConsume(frames);
      return 0;
    }

    const channels = Math.max(1, emu.audioChannels);
    const block = ctx.createBuffer(channels, frames, this.rate);
    // The view is built and consumed with no allocation in between, because a
    // wasm memory that grows detaches it. `copyToChannel` copies, so once this
    // returns the module may do whatever it likes with those bytes.
    const interleaved = emu.audioView(frames);
    if (channels === 1) {
      block.copyToChannel(interleaved, 0);
    } else {
      for (let ch = 0; ch < channels; ch++) {
        const plane = new Float32Array(frames);
        for (let i = 0; i < frames; i++) plane[i] = interleaved[i * channels + ch];
        block.copyToChannel(plane, ch);
      }
    }
    emu.audioConsume(frames);

    const source = ctx.createBufferSource();
    source.buffer = block;
    source.connect(this.gain);
    if (this.playhead < now + LEAD_SECONDS / 2) this.playhead = now + LEAD_SECONDS;
    source.start(this.playhead);
    this.playhead += frames / this.rate;
    return frames;
  }
}
