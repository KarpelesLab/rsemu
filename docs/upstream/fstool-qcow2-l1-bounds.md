# fstool: `ensure_mapping` indexes the qcow2 L1 table without a bounds check

**Status:** open upstream as of `fstool` 0.4.27. Worked around in
`fuzz/fuzz_targets/blk_image.rs` by opening every qcow2 read-only; nothing in
rsemu proper can guard it.

This is a specification for a change to `fstool`, which is a *separate
repository*. It is written down here rather than committed there — rsemu's
working rule is that we do not push to sibling repos, so an upstream fix becomes
a written spec and somebody carries it across deliberately.

## What happens

```
thread '<unnamed>' panicked at fstool-0.4.27/src/block/qcow2/mod.rs:952:36:
index out of bounds: the len is 0 but the index is 0
  <Qcow2Backend>::ensure_mapping        qcow2/mod.rs:952
  <Qcow2Backend>::write_virtual         qcow2/mod.rs:785
  <Qcow2Backend as BlockDevice>::write_at  qcow2/mod.rs:1266
```

`ensure_mapping` computes an L1 index from the guest offset and then indexes
`self.l1l2.l1[l1_idx]` directly. `l1l2.rs` deliberately does **not** require the
header's `l1_size` to cover the image's virtual `size` — its own comment says
so — so the two can disagree and nothing downstream re-checks.

The result: a qcow2 whose header declares a non-zero `size` and `l1_size = 0`

* opens cleanly — no validation rejects it,
* **reads** cleanly — the read path uses `get` and an absent mapping reads as
  zeros, which is correct for a sparse image,
* and **panics on the guest's first write**.

## Why it matters to a consumer

An image file is the one thing rsemu parses that the *user did not write*:
`rsemu run pc-at --drive hd0=downloaded.qcow2`. A crafted image plus one
`WRITE SECTOR(S)` from the guest aborts the emulator. It is a denial of service
reachable from a file a person was handed, and it is reached by ordinary use
rather than by any unusual API call.

## Reproducing it

The bytes are `fuzz/corpus/blk_image/qcow2-l1-size-zero` in this repository —
the libFuzzer artifact as found, kept verbatim. Its first nineteen bytes are the
fuzz target's own prologue (flags, then the opcode stream, which contains a
WRITE); the image begins at offset 19 and is an otherwise ordinary 64 KiB qcow2
v3 with 512-byte clusters whose header reads:

| offset | field           | value    |
| ------ | --------------- | -------- |
| 0      | magic           | `QFI\xfb` |
| 4      | version         | 3        |
| 20     | `cluster_bits`  | 9        |
| 24     | `size`          | 0x10000  |
| 36     | `l1_size`       | **0**    |
| 40     | `l1_table_offset` | 0      |
| 48     | `refcount_table_offset` | 0 |

Any write then panics. Field names and offsets are from the qcow2 format
specification; no QEMU source was consulted for this report (`CLAUDE.md`,
Provenance).

## The fix

`ensure_mapping` should return `Error::Corrupted` (or `OutOfBounds`) rather than
index, for an L1 index at or beyond `l1.len()`. That is the minimum, and it is
enough for a consumer: rsemu maps every non-`OutOfBounds` failure on a
bounds-checked access to an uncorrectable data error and tells the guest so.

Better, and independently worth doing: reject the inconsistency at **open**.
A header whose `l1_size` does not cover `size` describes an image that cannot
represent its own address space, and refusing it at the header parse means no
later operation has to be defensive about it. If `l1l2.rs` keeps its current
permissiveness on purpose — for a truncated image somebody is trying to
recover — then the write path must grow the mapping or fail, and must not index.

The read path already does the right thing and needs no change.

## How to verify the fix from rsemu

Delete the read-only gate in `fuzz/fuzz_targets/blk_image.rs` first — it is one
marked block, and while it stands no write reaches the backend, so the target
cannot tell a fixed `fstool` from an unfixed one. Then:

```sh
cargo update -p fstool --precise <new version>
cargo fuzz run blk_image fuzz/corpus/blk_image -- -runs=0
```

`qcow2-l1-size-zero` is in that corpus, so a single pass answers the question.
Afterwards delete this file and the `fuzz/README.md` entry that points at it,
and run a real campaign — `fuzz/README.md` has the command — because the qcow2
write path will have been unfuzzed for as long as the workaround stood.
