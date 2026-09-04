// The whole JavaScript side of the rsemu boundary, in one class.
//
// There is no wasm-bindgen here and there never will be (ROADMAP.md §0): the
// module exports plain C functions, and this file is the entire glue. Read
// `src/wasm.rs` for the ABI's three rules — nothing crosses as a pointer the
// embedder made, machines are named by index, one machine at a time.

/** Decodes UTF-8 out of the module's exported memory. */
const utf8 = new TextDecoder();

export class Rsemu {
  /**
   * @param {WebAssembly.Instance} instance an instantiated rsemu module
   */
  constructor(instance) {
    this.e = instance.exports;
  }

  /**
   * Fetch, instantiate and wrap a module.
   * @param {string} url where the .wasm lives
   */
  static async load(url) {
    // No imports: the module is std-on-wasm32-unknown-unknown and asks the
    // host for nothing yet. When it does (ROADMAP.md §11.5 — now, random_get,
    // compile, log) they arrive as a second argument here and nowhere else.
    const { instance } = await WebAssembly.instantiateStreaming(fetch(url), {});
    return new Rsemu(instance);
  }

  // -- memory ---------------------------------------------------------------

  // Every view is created fresh from `memory.buffer`, never cached: a wasm
  // memory that grows detaches every existing view, and an allocation inside
  // any call below can grow it.

  /** @returns {Uint8Array} a view of `len` bytes at `ptr`. */
  bytes(ptr, len) {
    return new Uint8Array(this.e.memory.buffer, ptr, len);
  }

  /** The output buffer's current contents, as bytes. */
  output(len) {
    return len === 0 ? new Uint8Array(0) : this.bytes(this.e.rsemu_output_ptr(), len).slice();
  }

  /** The output buffer's current contents, as a string. */
  text(len) {
    return len === 0 ? "" : utf8.decode(this.bytes(this.e.rsemu_output_ptr(), len));
  }

  /** Copy `data` into the module's input buffer. */
  input(data) {
    const ptr = this.e.rsemu_input_reserve(data.length);
    this.bytes(ptr, data.length).set(data);
    return data.length;
  }

  // -- identity -------------------------------------------------------------

  /** The build string: version and enabled features. */
  version() {
    return utf8.decode(this.bytes(this.e.rsemu_version_ptr(), this.e.rsemu_version_len()));
  }

  /** Prove the module actually runs, not merely loads. */
  echo(value) {
    return this.e.rsemu_echo(value) >>> 0;
  }

  /** The message from the last call that failed. */
  error() {
    return this.text(this.e.rsemu_error());
  }

  // -- the catalog ----------------------------------------------------------

  /**
   * Every machine this build can run. A machine is a feature set, so this is a
   * property of the .wasm that was fetched.
   *
   * `builtins` is what makes a machine runnable with nothing uploaded: images
   * compiled into the module for its own media slot — RSMON, the Woz Monitor,
   * a board's demonstration firmware. An empty one means the visitor has to
   * supply a file, which is every cartridge and every BIOS.
   */
  machines() {
    const out = [];
    for (let i = 0; i < this.e.rsemu_machine_count(); i++) {
      out.push({
        index: i,
        name: this.text(this.e.rsemu_machine_name(i)),
        summary: this.text(this.e.rsemu_machine_summary(i)),
        media: this.text(this.e.rsemu_machine_media(i)),
        slots: this.slots(i),
        builtins: this.builtins(i),
      });
    }
    return out;
  }

  /**
   * Every media slot machine `index` declares, in the module's order.
   *
   * `media` above is the first of them, which is the one `boot` fills. This is
   * the whole list, because a PC has five — `bios`, `vgabios`, `floppy`,
   * `hd0`, `hd1` — and `stageMedia` fills the ones a boot does not.
   */
  slots(index) {
    const out = [];
    for (let s = 0; s < this.e.rsemu_machine_media_count(index); s++) {
      out.push({ index: s, name: this.text(this.e.rsemu_machine_media_name(index, s)) });
    }
    return out;
  }

  /** The images this build carries for machine `index`, in the module's order. */
  builtins(index) {
    const out = [];
    for (let b = 0; b < this.e.rsemu_machine_builtin_count(index); b++) {
      out.push({
        index: b,
        name: this.text(this.e.rsemu_machine_builtin_name(index, b)),
        summary: this.text(this.e.rsemu_machine_builtin_summary(index, b)),
        slot: this.text(this.e.rsemu_machine_builtin_slot(index, b)),
      });
    }
    return out;
  }

  // -- lifecycle ------------------------------------------------------------

  /**
   * Build machine `index`, optionally with an image bound to its media slot.
   * @param {number} index
   * @param {Uint8Array|null} image
   */
  boot(index, image) {
    const len = image ? this.input(image) : 0;
    if (!this.e.rsemu_boot(index, len)) {
      throw new Error(this.error() || "boot failed");
    }
  }

  /**
   * Build machine `index` on one of the images the module carries, uploading
   * nothing at all.
   *
   * `rsemu run beneater-6502 --monitor wozmon` for a page with no command
   * line: the bytes are already in the module, so this is one click from a
   * cold load.
   * @param {number} index
   * @param {number} builtin
   */
  bootBuiltin(index, builtin) {
    if (!this.e.rsemu_boot_builtin(index, builtin)) {
      throw new Error(this.error() || "boot failed");
    }
  }

  /**
   * Put `image` in machine `index`'s media slot `slot` for the next boot.
   *
   * The second bay. `boot` binds one uploaded image to one slot, so before
   * this existed a PC could be handed a firmware or a disk and never both —
   * and since the firmware is the one the module carries, "both" is the only
   * interesting case: `stageMedia(pc, floppySlot, img)` then
   * `bootBuiltin(pc, bios)` is a diskette booting on rsemu's own BIOS.
   *
   * Staging survives a boot, so Reboot reboots the same media. It is keyed by
   * slot *name* on the module's side, so a slot the next machine has not got
   * is refused at boot rather than ignored — call `clearMedia` when the
   * machine changes.
   * @param {number} index
   * @param {number} slot
   * @param {Uint8Array} image
   */
  stageMedia(index, slot, image) {
    const len = this.input(image);
    if (!this.e.rsemu_stage_media(index, slot, len)) {
      throw new Error(this.error() || "staging failed");
    }
  }

  /** Forget everything `stageMedia` staged. */
  clearMedia() {
    this.e.rsemu_clear_media();
  }

  shutdown() {
    this.e.rsemu_shutdown();
  }

  reset() {
    if (!this.e.rsemu_reset()) throw new Error(this.error() || "reset failed");
  }

  get running() {
    return this.e.rsemu_is_running() !== 0;
  }

  // -- running --------------------------------------------------------------

  /** Advance one video frame. True when there is a new picture to draw. */
  runFrame() {
    return this.e.rsemu_run_frame() !== 0;
  }

  /** Advance `n` frames, returning how many produced a new picture. */
  runFrames(n) {
    return this.e.rsemu_run_frames(n);
  }

  /** Milliseconds of virtual time one frame takes. */
  frameMs() {
    return Number(this.e.rsemu_frame_period_ns()) / 1e6;
  }

  /** Virtual nanoseconds run so far, as a Number (good to 104 days). */
  nowNs() {
    return Number(this.e.rsemu_now_ns());
  }

  /**
   * The deterministic state hash, as a hex string.
   *
   * A wasm `i64` arrives as a *signed* BigInt, so it is reinterpreted here —
   * otherwise half of all hashes would print with a minus sign and would not
   * match what `rsemu run` prints for the same run.
   */
  stateHash() {
    const hash = BigInt.asUintN(64, this.e.rsemu_state_hash());
    return "0x" + hash.toString(16).padStart(16, "0");
  }

  // -- the picture ----------------------------------------------------------

  get hasVideo() {
    return this.e.rsemu_has_video() !== 0;
  }

  get width() {
    return this.e.rsemu_frame_width();
  }

  get height() {
    return this.e.rsemu_frame_height();
  }

  /** Which frame the buffer holds, as the display device counts them. */
  frameSerial() {
    return Number(this.e.rsemu_frame_serial());
  }

  /**
   * The framebuffer as `ImageData`, RGBA, ready for `putImageData`.
   *
   * A view over the module's memory rather than a copy — the pixels are laid
   * out for exactly this — so it must be rebuilt every frame.
   */
  imageData() {
    const len = this.e.rsemu_frame_len();
    if (len === 0) return null;
    const pixels = new Uint8ClampedArray(this.e.memory.buffer, this.e.rsemu_frame_ptr(), len);
    return new ImageData(pixels, this.width, this.height);
  }

  // -- the sound ------------------------------------------------------------

  // Interleaved `f32` in [-1, 1] at whatever rate was last announced, which is
  // exactly what an `AudioBuffer` holds — the conversion from the console's own
  // crystal-derived rate happened in Rust, where the exact ratio lives.

  get hasAudio() {
    return this.e.rsemu_has_audio() !== 0;
  }

  /**
   * Tell rsemu what rate the page's `AudioContext` runs at.
   *
   * The browser picks that rate, not us, and it differs between machines. Say
   * it once per context; anything queued at the old rate is discarded.
   */
  audioSetRate(hz) {
    return this.e.rsemu_audio_set_rate(hz) !== 0;
  }

  get audioRate() {
    return this.e.rsemu_audio_rate();
  }

  get audioChannels() {
    return this.e.rsemu_audio_channels();
  }

  /** How many frames are waiting. A frame is one sample per channel. */
  audioFrames() {
    return this.e.rsemu_audio_frames();
  }

  /**
   * A view over the queued frames.
   *
   * Like `imageData()`, a view rather than a copy, and like it the view must be
   * rebuilt every time: this queue grows, and a wasm memory that grows detaches
   * every existing view. Copy out of it before calling anything else.
   */
  audioView(frames) {
    const count = frames * Math.max(1, this.audioChannels);
    return new Float32Array(this.e.memory.buffer, this.e.rsemu_audio_ptr(), count);
  }

  /** Say the frames have been taken. Nothing drops them on its own. */
  audioConsume(frames) {
    return this.e.rsemu_audio_consume(frames);
  }

  /** Frames lost because the page did not keep up. A diagnostic, never state. */
  audioDropped() {
    return Number(this.e.rsemu_audio_dropped());
  }

  // -- input ----------------------------------------------------------------

  /**
   * NES controller bits, in the shift register's own output order: A is the
   * first bit out, so it is the high bit. Same constants as
   * `dev::nes::input::buttons`.
   */
  static BUTTONS = {
    a: 0x80,
    b: 0x40,
    select: 0x20,
    start: 0x10,
    up: 0x08,
    down: 0x04,
    left: 0x02,
    right: 0x01,
  };

  setButtons(port, mask) {
    this.e.rsemu_set_buttons(port, mask);
  }

  buttons(port) {
    return this.e.rsemu_buttons(port);
  }

  /**
   * Whether this machine has controllers at all.
   *
   * Not the same question as `hasVideo`: a display panel with no game pad is
   * an ordinary machine, and a page that drew a d-pad for one would be
   * inventing hardware.
   */
  get hasPad() {
    return this.e.rsemu_has_pad() !== 0;
  }

  get hasConsole() {
    return this.e.rsemu_has_console() !== 0;
  }

  /** Type at the machine. Returns how many bytes it accepted. */
  consoleWrite(text) {
    const bytes = new TextEncoder().encode(text);
    this.input(bytes);
    return this.e.rsemu_console_write(bytes.length);
  }

  /** Everything the machine has said since the last call. */
  consoleRead() {
    return this.output(this.e.rsemu_console_read());
  }

  /**
   * Whether this machine has an AT keyboard rather than a character console.
   *
   * On a PC the two are opposites: `pc.kbc`'s port carries set-2 scan codes,
   * so `hasConsole` is false and this is true. An Apple 1 is the other way
   * round. No machine in this build has both, but nothing forbids one — a PC
   * with a serial console would — so ask both rather than inferring.
   */
  get hasKeyboard() {
    return this.e.rsemu_has_keyboard() !== 0;
  }

  /**
   * Press or release one key, named by X11 keysym the way RFB names it.
   *
   * A *transition*, not a character: the make and break codes are what an AT
   * keyboard puts on the wire, and a guest that watches for a key coming up
   * can tell. Printable ASCII keysyms are their own character codes, so the
   * letters need no table; `KEYSYMS` below has the named ones.
   *
   * Returns false for a key this keyboard has not got, which puts no bytes on
   * the wire at all.
   * @param {number} keysym
   * @param {boolean} down
   */
  key(keysym, down) {
    return this.e.rsemu_key(keysym >>> 0, down ? 1 : 0) !== 0;
  }

  /**
   * The named X11 keysyms a browser's `KeyboardEvent.key` maps onto.
   *
   * X11's, not ours — the same numbers `host::input::Keysym` names and the
   * same ones a VNC client sends, so a browser and a VNC session type at a PC
   * through one table on the Rust side.
   */
  static KEYSYMS = {
    Enter: 0xff0d,
    Backspace: 0xff08,
    Tab: 0xff09,
    Escape: 0xff1b,
    Home: 0xff50,
    ArrowLeft: 0xff51,
    ArrowUp: 0xff52,
    ArrowRight: 0xff53,
    ArrowDown: 0xff54,
    PageUp: 0xff55,
    PageDown: 0xff56,
    End: 0xff57,
    Insert: 0xff63,
    Shift: 0xffe1,
    Control: 0xffe3,
    CapsLock: 0xffe5,
    Alt: 0xffe9,
    Delete: 0xffff,
    F1: 0xffbe,
    F2: 0xffbf,
    F3: 0xffc0,
    F4: 0xffc1,
    F5: 0xffc2,
    F6: 0xffc3,
    F7: 0xffc4,
    F8: 0xffc5,
    F9: 0xffc6,
    F10: 0xffc7,
    F11: 0xffc8,
    F12: 0xffc9,
  };

  /**
   * The keysym for a browser `KeyboardEvent.key`, or 0 for one with none.
   *
   * A one-character `key` is the character, and for printable ASCII that *is*
   * the keysym — X11 chose Latin-1 for the low range and never moved it. So
   * this table is only the keys with names.
   * @param {string} key
   */
  static keysym(key) {
    if (key.length === 1) {
      const code = key.charCodeAt(0);
      return code >= 0x20 && code < 0x7f ? code : 0;
    }
    return Rsemu.KEYSYMS[key] ?? 0;
  }

  // -- save states ----------------------------------------------------------

  /** A snapshot. Nothing is uploaded: the bytes exist only in this page. */
  save() {
    const len = this.e.rsemu_save();
    if (len === 0) throw new Error(this.error() || "save failed");
    return this.output(len);
  }

  /** Restore a snapshot taken from an identically configured machine. */
  load(bytes) {
    const len = this.input(bytes);
    if (!this.e.rsemu_load(len)) throw new Error(this.error() || "load failed");
  }
}
