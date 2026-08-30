# rsemu web harness

A deliberately minimal page that loads the wasm build and calls it. It is not
the eventual UI — it exists so the WebAssembly target is *exercised* from the
first commit, not merely compiled. A target that builds but is never run is a
target that quietly stops working.

## Build and serve

```sh
cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
    --no-default-features --features wasm --release
cp target/wasm32-unknown-unknown/release/rsemu.wasm web/
python3 -m http.server -d web 8080
```

Then open <http://localhost:8080/>. `file://` will not work — the module is
fetched, and fetch has no `file://` scheme.

## No wasm-bindgen

The dependency policy (`ROADMAP.md` §0) rules it out, and the boundary does not
need it: the module is instantiated directly, and strings cross as a
pointer/length pair read out of the exported memory. This is purecrypto's
browser convention, and it keeps the JS glue small enough to read in one sitting.

## Threads, later

Threaded execution needs `SharedArrayBuffer`, which needs cross-origin isolation
(`Cross-Origin-Opener-Policy: same-origin` and
`Cross-Origin-Embedder-Policy: require-corp`). The page reports whether it got
it. The non-threaded path must keep working regardless — COOP/COEP is often
unavailable, so it is a supported configuration rather than a fallback
(`ROADMAP.md` §11.3).
