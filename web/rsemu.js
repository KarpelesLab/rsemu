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
   */
  machines() {
    const out = [];
    for (let i = 0; i < this.e.rsemu_machine_count(); i++) {
      out.push({
        index: i,
        name: this.text(this.e.rsemu_machine_name(i)),
        summary: this.text(this.e.rsemu_machine_summary(i)),
        media: this.text(this.e.rsemu_machine_media(i)),
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
