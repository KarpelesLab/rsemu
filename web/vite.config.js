import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

export default defineConfig({
  // Relative, not "/rsemu/". The site is published at a subpath on GitHub
  // Pages but is also opened straight out of `dist/` with `python3 -m
  // http.server`, and the page fetches `./rsemu.wasm` relative to itself. A
  // relative base is the only one that is correct in both places — an absolute
  // "/rsemu/" would 404 locally and an absolute "/" would 404 on Pages.
  base: "./",

  plugins: [vue()],

  // `public/` is copied to the site root verbatim: no hashing, no inlining, no
  // transform. That is deliberately where rsemu.wasm goes. It is a *build
  // product of cargo*, not a bundler input — nothing imports it, so Rollup
  // never sees it, and `./rsemu.wasm` resolves next to index.html both in
  // `vite dev` and in `dist/`.
  publicDir: "public",

  build: {
    outDir: "dist",
    emptyOutDir: true,
    // The module is instantiated with WebAssembly.instantiateStreaming and the
    // page uses top-level await nowhere, but BigInt literals and `??=` do want
    // a recent baseline. Every browser that can run wasm can run this.
    target: "es2022",
    // Small enough that a separate vendor chunk buys nothing; one request for
    // the app beats two.
    chunkSizeWarningLimit: 700,
  },

  server: {
    port: 8080,
    strictPort: false,
  },
});
