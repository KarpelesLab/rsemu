# fstool: the `std` build should not demand a filesystem feature

**Status:** open upstream. rsemu pins `fstool = ">=0.4.26, <0.4.28"` until fixed.

This is a specification for a change to `fstool`, which is a *separate
repository*. It is written down here rather than committed there — rsemu's
working rule is that we do not push to sibling repos, so an upstream fix becomes
a written spec and somebody carries it across deliberately.

## What happens

`fstool` 0.4.28 introduced an unconditional `compile_error!` to its `std` build:

```
error: fstool: the `std` build needs at least one filesystem feature
       (`fat`, `ext`, … or `filesystems`); `inspect` has nothing to dispatch
       to otherwise
```

followed by a cascade of `E0004` non-exhaustive-match errors on `&AnyFs` and
`&mut AnyFs`, because with no filesystem feature that enum has no variants.

## Why it is wrong for a block-layer consumer

rsemu depends on `fstool` for **disk images**, not filesystems. Every use is in
the block layer:

- `fstool::BlockDevice`
- `fstool::block::open_image_with_password`, `open_image_read_only_with_password`
- `fstool::Error` and its variants

`AnyFs` and `inspect` are never named. The guard fires on a build that cannot
reach the code it is guarding, so the only way to satisfy it is to enable a
filesystem we will never call — dead weight in a crate whose dependency policy
(`CLAUDE.md`) is deliberately strict about what enters the tree.

## The fix

Gate the check on the thing that actually needs it. `inspect` — and whatever
else dispatches over `AnyFs` — should be behind a feature, and the
`compile_error!` should fire only when that feature is on with no filesystem
backing it. A consumer taking `default-features = false` and asking only for
`gzip`/`zstd` should build.

An acceptable alternative: make `AnyFs` inhabited (an empty/unsupported variant)
so the matches stay exhaustive, and downgrade the guard to a `deprecated` note.

## How to verify the fix from rsemu

```sh
cargo update -p fstool --precise <new version>
cargo check --features dev-blk
```

Then lift the upper bound in `Cargo.toml` back to a plain caret requirement and
delete this file.

## Which versions

Established by bisection from rsemu, not from release notes: **0.4.27 builds,
0.4.28 is the first that does not.** Two separate reports guessed 0.4.30 and
">= 0.4.27"; both were wrong, so check by building rather than by changelog.
