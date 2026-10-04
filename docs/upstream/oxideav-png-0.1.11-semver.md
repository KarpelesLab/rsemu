# oxideav-png 0.1.11 broke its 0.1 API in a semver-compatible release

**Status:** worked around in rsemu (the dependency floor is now `0.1.11` and
`host::display::png` is written against it). The upstream question is open.

This is a note for `oxideav-png`, which is a *separate repository*. rsemu's
working rule is that we do not push to sibling repos, so an upstream problem
becomes a written note and somebody carries it across deliberately.

## What happened

rsemu depended on `oxideav-png = "0.1"`. Its `Cargo.lock` is not committed, so
CI resolves the newest compatible version on every run. Between the locally
locked `0.1.8` and `0.1.11` — both of which Cargo considers compatible with
`"0.1"` — the public API changed shape:

| 0.1.8 | 0.1.11 |
|---|---|
| `PngImage { width, height, pixel_format, stride, data, palette }` | `#[non_exhaustive] PngImage { width, height, format, planes, color, metadata, palette, transparency }` |
| `pixel_format: PngPixelFormat` | `format: PixelFormat` (`PngPixelFormat` kept as the enum; `PixelFormat` is now the alias) |
| `stride` and `data` on the image | `planes: Vec<Plane>`, each `Plane { stride, data }`, also `#[non_exhaustive]` |
| `palette: Vec<…>` | `palette: Option<Palette>` |
| `encode_png_image(&image)` | still present, now `#[deprecated]` in favour of `encode(&image, &EncodeOptions)` |
| `decode_png(&bytes)` | still present, now `#[deprecated]` in favour of `decode(&bytes)` |

Each row on its own breaks a downstream crate that builds with
`-D warnings`: a struct expression no longer compiles (`E0639`), a renamed
field no longer exists (`E0609`), and a deprecated call is an error.

## What it cost

rsemu's `master` went red on **every job that builds `display-png`** — which,
because `--all-features` includes it, was clippy, rustdoc, MSRV, the test job
on all three operating systems, the feature sweep and the wasm jobs — on the
same day an unrelated `no_std` slip landed, which made the two look like one
failure. Locally everything was green, because the local lock still said
`0.1.8`. Nothing in rsemu changed to cause it.

## What would have prevented it

Under Cargo's rules a `0.x.y` crate signals a breaking change by bumping `x`.
So any one of these:

1. **Release the reshape as `0.2.0`.** Downstream `"0.1"` requirements then
   keep resolving to the last 0.1 release until each one opts in.
2. **Or keep 0.1 additive**: add `PngImage::new` and `Plane`, add the
   `planes` field *alongside* `stride`/`data` for one release, and deprecate
   rather than remove — `#[non_exhaustive]` on an existing public struct is
   itself the breaking part and cannot be made additive.
3. **Deprecate in one release, remove or reshape in the next minor.** A
   deprecation that lands in the same release as the reshape gives nobody a
   release to migrate in.

## How to tell it is fixed

A crate pinned to `oxideav-png = "0.1"` that uses the 0.1.8 struct expression
keeps compiling under `-D warnings` after `cargo update`.

## The rsemu side, for whoever reads this from here

`Cargo.toml` names the floor and says why. Every use of the crate is in
`src/host/display/png.rs` (encode, APNG) and one round-trip test in
`src/host/display/tests.rs` (decode). The frame-hash goldens hash **machine
state**, not PNG bytes, so an encoder change cannot move them.
