# SGX543 USSE and PDS encodings: method and measurements

This is the measurement record behind `src/dev/sgx/usse.rs` and
`src/dev/sgx/pds.rs`. The section numbers (`§n`) are the ones those files
cite. The first set of notes (`ENCODING.md`, `PDS_ENCODING.md`) was lost.
§1–§7 below rebuild what `usse.rs` already said from those notes. §8–§12
were measured again from scratch in November 2026 to decode the navi UI's
shader corpus.

## Provenance

No Imagination documentation, driver source (of any licence), Mesa, QEMU,
community PowerVR reverse-engineering or PVR SDK disassembler output was
consulted. Every fact here comes from one of three places:

- the navi's own binaries, run as black boxes;
- those binaries read as data (tables, assertion strings);
- the corpus of programs dumped from a running navi.

Field names are ours. Where a field has no known meaning, the mnemonic
carries the encoder's opcode or flag number (`op189`, `f3.20`).

Each claim carries a grade:

- **exact**: read off the encoder by flipping one input;
- **strong**: the encoder and an independent source agree (the corpus data
  flow, or the GL driver's own tables);
- **[G]**: a guess.

## The method

`libGLES_CM.so` (`usr/lib` in the hc59_2022 rootfs) contains the driver's
USSE assembler back end. Its source path survives as an assertion string,
`tools/intern/useasm/useasm.c`. The function that encodes one instruction
is at `0x77094`, found through `AssembleUSEASMInstructions` →
`0x7957c`, which walks the list at `inst+0xf4`. It is called as:

```
r0 = target descriptor   (0x9d354: core 7 = SGX543, revision 142 = 1.4.2;
                          table of 0x14-byte entries at 0x9d2dc, looked up
                          by 0x7d780; +0x10 points at the feature words)
r1 = instruction struct  r2 = output base  r3 = output pointer
[sp] = 0                 [sp+4] = context (+0x00 user, +0x18 error callback)
returns the number of 32-bit words written (2 per instruction), -1 on error
```

The harness loads the ELF's `PT_LOAD` segments at address 0 into rsemu's
own ARM core (`cpu-arm-aprofile`, Cortex-A9 configuration). It runs until
the function returns to a magic link address. It catches every PLT entry
and emulates the few imports the encoder reaches: `memcpy`, `memset`, the
allocators and `strlen`. Any other import, the error callback, or a wild PC
ends the call. The harness also returns the number of ARM instructions that
ran before the error callback. A search over operand types and values uses
that count as a progress measure, which finds a valid instruction for
opcodes whose operand rules are not yet known. With a valid instruction
in hand, every input field is flipped in turn and the change in the 64-bit
output is recorded.

The harness and its driver scripts are not committed. They hold no guest
bytes, but they are scaffolding. Rebuilding them takes about 300 lines:

- **harness:** a Rust binary using `rsemu::cpu::arm::aprofile`, which reads
  `call`/`mem`/`dump` commands on stdin;
- **driver:** a Python script that builds instruction structs and diffs the
  output words.

### Encoder input (exact)

The instruction struct has a fixed header and then argument slots:

| Offset | Field |
|---|---|
| `+0x00` | opcode |
| `+0x04` | flags1 |
| `+0x08` | flags2 |
| `+0x0c` | flags3 |
| `+0x10` | flags4 |
| `+0x14 + 0x14·i` | argument `i`, up to 11 |
| `+0xf4` | next instruction |

Each argument is five words: `{number, type, flags, index, ?}`.

Argument types:

| Type | Meaning |
|---|---|
| 0 | `r` |
| 1 | `o` |
| 2 | `pa` |
| 3 | `sa` |
| 4 | index register |
| 5 | `g` (number + 64) |
| 6 | `c` |
| 7 | internal register (`r124+n` in scalar fields) |
| 8 | immediate |
| 10 | DRC |
| 13 | small field |
| 15 | swizzle |
| 16 | enumerated selector |
| 17 | small enum |
| 21 | label |

Argument flags:

- bit 0: negate;
- bit 1: absolute value [G];
- bit 15: complement (selectors);
- bit 20 on a destination: an explicit byte mask in bits 10:7.

Index values 1 and 2 select `i1` and `i2`.

flags1:

- bit 0: skipinv;
- bit 2: nosched;
- bit 3: END;
- bits 8:4: repeat;
- bits 12:9: repeat count, or the write mask on vector forms;
- bits 15:13: predicate;
- bit 17: `syncs?`;
- bit 25: paired with the next instruction.

The function at `0x600e0` checks the flags words against a per-opcode mask
of permitted bits. Its constant arguments are a reliable list of the flags
each opcode accepts.

Opcodes whose handler first tests feature bit 27 (`0x7df2c`) and fails are
the SGX530/540 scalar float forms, absent on this core: useasm 0–78, 156,
157, 367 and 368. These include all the old scalar `fmad`, `fdp` and
`frcp` forms. The SGX543 does its floating point on the vector forms of §8.

## §1 Instruction word (strong)

One instruction is 64 bits, stored as two little-endian words with bits
31:0 at the lower address. The primary opcode is in bits 63:59.

## §2 Operands (exact, from the first notes; re-checked on `and`/`mov`)

- **§2.1 Predicates.**
  - Long form (3 bits): 0 none, 1–4 `p0`–`p3`, 5 `!p0`, 6 `!p1`, 7 `pN`.
  - Short form (2 bits): 0 none, 1 `p0`, 2 `p1`, 3 `!p0`.
- **§2.2 Flags.** END is b54 on the ALU forms and b43 on op 31 classes 0
  and 2.
- **§2.3 Register fields.**
  - Destination: number b27:21; bank `(b51, b33, b32)` gives 000 r, 001 o,
    010 pa, 100 sa, 110 i, 101 g/c, and 011/111 indexed through `i1`/`i2`.
  - `src1`: number b13:7, bank b31:30, extension b49.
  - `src2`: number b6:0, bank b29:28, extension b48.
  - With the extension bit set, bank 0/3 is indexed, bank 1 is `g`/`c` and
    bank 2 is an immediate.

## §3–§7

These cover the bitwise and shift family (§3), the integer multiply-adds
(§4), `limm`, `setl`/`savl` and the test form (§5), `ld`/`st`/`ldr`/`str`
(§6) and the flow, `phas` and `smlsi` forms (§7). The field positions in
`usse.rs` for these are unchanged from the first notes. Spot checks in
this round (`and`, `or`, `xor`, `shl`, `shr`, `asr`, `rol`, `mov`, `limm`,
`str`, `ld*`, `st*`, `imadd20`, `imae`, `imadd26`) reproduced them exactly.

## §8 Vector floating point (exact fields; semantics as graded)

These rules are common to the vector ops:

- **Register fields.** On ops 0–3 and 7 they are 6 bits wide and count
  64-bit registers: field `f` is register `2f`. In the temporaries, fields
  60–63 are the internal registers `v0`–`v3`, which hold four 32-bit
  components each. Constants and immediates are not halved.
- **Operand meaning.** A source names four consecutive 32-bit registers as
  `xyzw`, and a destination writes component `k` to the `k`-th (strong:
  the vertex shaders' matrix columns `sa10`, `sa14`, `sa18`, `sa22` are
  four words apart).
- **Predicate.** Ops 1–3 and 6–8 use 3 bits at b58:56: 0 none, 1 `p0`,
  2 `p1`, 3 `p2`, 4 `!p0`, 5 `!p1`, 6 unknown, 7 `pN`. Op 0 uses 2 bits at
  b57:56: 0 none, 1 `p0`, 2 `!p0`, 3 `pN`.
- **Write mask.** b42:39, component 0 at b39. An F32 destination in
  temporaries, `pa` or `sa` accepts only the low two bits; `v0`–`v3`
  accept four.
- **b43 is not END.** The encoder puts its END flag at b43, but the vertex
  shaders set it on almost every vector instruction with live code after
  it. It is printed as `b43` and ignored [G: meaning unknown]. `pck`'s END
  slot, b54, behaves the same way.

The 16-entry swizzle table (T16), used by `vop` `src2`, `vmov` and op 3:

| Index | Swizzle | Index | Swizzle | Index | Swizzle | Index | Swizzle |
|---|---|---|---|---|---|---|---|
| 0 | xxxx | 4 | xyzw | 8 | xyxy | 12 | yzxz |
| 1 | yyyy | 5 | yzww | 9 | xywz | 13 | xxyy |
| 2 | zzzz | 6 | xyzz | 10 | zxyw | 14 | xzww |
| 3 | wwww | 7 | xxyz | 11 | zwzw | 15 | xyz + code 5 |

Code 5 is probably the constant 1.0 [G].

Op 3 uses a 32-entry table: entries 0–15 are T16, then 16–31 are:

`yzxw zwxy xzwy yyww wyzw wzwz xyzx zzww xwzx yyyx yyyz xzyw xxxy zyxw yyzz zzzy`

### §8.2 Op 0, `vmad` (useasm 278; 288 sets b58)

| Operand | Register field | Swizzle | Modifiers |
|---|---|---|---|
| dst | b27:22 | — | — |
| src0 | b17:12, bank b34 (r or pa) | `{b53, b19:18}`: xxxx yyyy zzzz wwww xyzw yzxw xyww zwxy | b50 abs |
| src1 | b11:6, `src1` bank bits | `{b44, b21:20}`: xxxx yyyy zzzz wwww xyzw xyyz yyww wyzw | b38 abs |
| src2 | b5:0, `src2` bank bits | b47:45: xxxx yyyy zzzz wwww xyzw xzww xxyz xyzz | b35 negate, b36 abs |

b37 negates the product (a negate on either src0 or src1 sets it).

The result is `src0·src1 + src2` (strong). b58 changes how the mask pairs
up components and may select F16 [G]; it is not executed.

### §8.4 Ops 1 and 2, `vop` (useasm 279–286 and 289–296)

Op 1 is F32: it takes a two-bit mask on a 64-bit register (strong). Op 2
is F16: it takes a four-bit mask on the same register (strong).

- **dst:** b27:22.
- **src1:** b11:6, `src1` bank bits. Its swizzle is 3 bits per component,
  low bit first:
  - x = b17:15
  - y = b20:18
  - z = `{b35, b34, b21}`
  - w = `{b54, b53, b50}`
  - Codes 4–7 are accepted; 4 and 5 are probably 0.0 and 1.0 [G].
- **src2:** b5:0, `src2` bank bits; swizzle is a T16 index at b47:44.
- **Sub-op:** b14:12. The encoder's opcodes map to it as 279→0, 280→1,
  281→2, 284→3, 285→4, 282→5, 283→6, 286→7.
  - 0 is multiply (strong: the matrix chain).
  - 1 is add [G].
  - 2–7 are unknown.
- **Modifiers:** b37 negates src1 (on sub-op 0 it negates the product);
  b38 is src1 abs; b36 is src2 abs.

### §8.3 Op 3 (useasm 261–264)

- **Sub-op:** b53:52 when that is 2 or 3. In that case b54 is the high
  bit of C's swizzle. Otherwise the sub-op is b54:52 (useasm 261 → 4,
  262 → 5).
- **Fields:**
  - dst: b27:22.
  - A: b5:0 with the `src1` bank bits; swizzle b10:6 indexes the 32-entry
    table.
  - B: an internal register at b29:28; swizzle `{b34, b21:18}`.
  - C: an internal register at b13:12; swizzle `{b54, b17:14}`.
  - Selector: b48:47 (encoder values 70–73; always 3 in the corpus;
    meaning unknown).
  - Mask: b42:39.
- **Modifiers:** b38 negates A (shared with B's negate); b37 is A abs; b46
  is B abs; b36 negates C; b35 is C abs.
- Sub-op 3 is `A·B + C` (strong). Sub-op 2 has the same operand shape but
  an unknown operation.

### §8.5 Op 7 (useasm 275, 276, 277)

- **Variant:** b47:46. 0 is a move (strong); 1 and 2 take three sources.
- **Repeat:** b45:44 holds count − 1.
- **Other fields:**
  - b41:40: format;
  - b39: set by default;
  - b38:35: swizzle (T16);
  - b27:24: mask;
  - dst: b23:18, in 64-bit registers (strong from the corpus: the state
    programs fill `o0..o5` contiguously that way);
  - src0: b17:12;
  - src1: b11:6;
  - src2: b5:0.
- Each repeat advances by one 64-bit register [G].

### §8.6 Op 8, `pck` (useasm 302–355)

The encoder opcode is `302 + 7·src + dst` for formats 0–6, plus 344, 352
and 353–355 for format 7.

- **Formats:** src at b43:41, dst at b40:38 [strong: first written the
  other way round; the UI vertex shader's `s16` → `f32` position unpack at
  `fe008368` only works this way]. The structure is exact; the names are a
  guess: 0 u8, 1 s8, 2 o8, 3 u16, 4 s16, 5 f16, 6 f32, 7 c10.
- **Lane mask:** b37:34.
- **dst:** b27:21, the 7-bit standard destination. `r124`–`r127` are
  `v0`–`v3`.
- **src:** the 7-bit `src1` slot.
- **Per-lane source select:** lane 0 at b1:0, lane 1 at b17:16, lane 2 at
  b15:14, lane 3 at b20:19.
  With b54 set, lane 0 reads x whatever b1:0 holds [G: the two corpus
  uses, `f32 <- f32` position copies at `fe0083c8` and its twin, have
  b1:0 = 2 and must copy `xyzw`].
- **Other bits:**
  - b18 is flags2 bit 8, executed as the normalising scale [G];
  - b47:44 is a thermometer of the repeat count;
  - b53, b48 and b50 appear only in the corpus, with no known meaning.
- **Lane layout used in execution [G]:** 8-bit lanes are the bytes of one
  word; 16-bit lanes are packed two to a word; f32 lanes are one word each.
  c10 and o8 are not executed.

### §8 corpus check

`fe0083a0..fe0083f0` is `v0 ← position`, then
`v1 = sa26·v0.x; v0 = sa22·v0.y + v1; ...`, ending
`o0.xy = sa32·v0.w + v1; o2.xy = sa34·v0.w + v1.zw`. That is a 4×4 matrix
times the vertex.

## §9 Op 31 class 1 (exact fields)

Sub-op b58:56 with class b54:52 = 1. END is b50 (flags1.3).

| Sub-op | useasm | Fields |
|---|---|---|
| 1 | 189 | four 12-bit unsigned fields: arguments 0–3 at b47:36, b35:24, b23:12, b11:0 |
| 2 | 190 | `smlsi` (§7) |
| 3 | 191 | the same four fields |
| 4 | 192 | four 12-bit two's-complement fields (encoder range about ±127) |
| 5 | 193 | two single bits, b0 and b8 |

**Sub-op 3 is `setbase` (strong):** its fields are offsets added to every
register number in the destination, `src0`, `src1` and `src2` slots until
the next `setbase`. The evidence is in the format-conversion library at
`f9002000`:

- its swap sequences only work this way (`o129`/`o130` swapped through
  `o209`, with offset `0x81`);
- `setbase #0x6f, #0, #7, #0` followed by `mov o78, r0` moves `r7` into
  `o189`, the library's scratch register, so temporaries move too.

Whether the offsets apply to `g`, `c`, the index registers or indexed
operands is not known; the interpreter leaves those alone [G].

## §10 EMIT (op 31 class 2 sub-op 3; exact fields)

b47:46 select the form:

| b47:46 | useasm | Notes |
|---|---|---|
| 00 | 158 | |
| 01 | 159, 160, 161 | b45:44 = 00, 01, 10 |
| 10 | 162 | the microkernel's; b45/b44 are flags2.4/.5 |

Every form has the same fields:

- target: b33:32;
- src0: b20:14, banks via b51 (o) and b34 (pa);
- src1: the standard `src1` slot;
- src2: the standard `src2` slot;
- a 14-bit immediate: b27:22, b40:35, b55:54.

Register fields hold `n/2` and name a pair. The encoder leaves the
operands a form does not take at fixed values (`#0`). Only flags1.3 (END,
b43), flags1.17 (b50, `syncs?`) and flags3.20 (b21) flip.

Corpus uses:

- **End-of-tile programs:** `emit158 #0, r0, r2, r4, #4` after loading
  `r0..r5` with what looks like pixel-back-end state.
- **Vertex and state programs:** they end in
  `emit159.syncs?.f3.20 #0, o0, ...` or `emit160` after filling `o0..oN`.
  The burst length is not in the word [G: the program's declared output
  size].

## §11 The blend family, ops 16–25 (exact fields; semantics strong where noted)

The meanings come from libGLES_CM's GL ES1 framebuffer-blend generator
(function `0x44554`). Its RGB and alpha factor tables at `0x855b0` and
`0x85610`, and its src↔dst swap table for REVERSE_SUBTRACT at `0x85584`,
put each GL blend factor onto an encoder selector value.

**Selector values (strong).** Names are relative to the operand slots:
`src1` is the source and `src2` the destination.

| Value | Factor |
|---|---|
| 0 | zero |
| 2 | saturate, `min(src1.a, 1 − src2.a)`; one on alpha |
| 5 | `src1` colour |
| 6 | `src2` colour |
| 8 | `src1` alpha |
| 9 | `src2` alpha |
| 4, 7 | `src0` colour / alpha (op 17 only) [G] |
| 24, 25 | op 16 only, unknown |

The complement flag turns `f` into `1 − f`.

**Operations.**

- 18: add, `src1·f1 + src2·f2` (strong).
- 19: subtract, `src1·f1 − src2·f2` (strong).
- 22 and 23: never produced by the GL code; taken as min and max, which
  ignore the factors as GL's do [G].

The arithmetic is unorm8 per byte, byte 3 is alpha, and products are
rounded to nearest. All three are guesses.

**Common header.**

- predicate: short form at b58:57;
- skipinv: b55;
- END: b54 (but on op 23, b54 is set mid-program);
- `syncs?`: b50;
- dst: standard;
- `src0` (ops 17, 19, 23, 25): b20:14, with b34 for pa;
- `src1` and `src2`: standard.

**Op 16, `blend2`** (useasm 87 paired with 227). Colour and alpha have
separate factors:

| Field | Complement | Code | Code → value |
|---|---|---|---|
| RGB f1 | b56 | b40:38 | 0, 5, 6, 8, 9, 2, 24 |
| RGB f2 | b47 | b37:35 | 0, 5, 6, 8, 9, 2, 25 |
| alpha f1 | b43 | b53:52 | 0, 8, 9, 24 |
| alpha f2 | b34 | b42:41 | 0, 8, 9, 25 |

- RGB op: b19:18. Alpha op: b17:16. Op code → value: 0 → 18, 1 → 19,
  2 → 22, 3 → 23.
- b14 reverses the alpha subtract (strong).
- b20 and b15 select an unknown alternative mode; not executed.
- Repeat: b46:44.

**Op 18, `blend`** (useasm 88; 250 is a fixed variant). One factor pair
for all bytes:

- f1: complement b56, code b40:38. f2: complement b47, code b37:35. Both
  use the code → value table 0, 2, 5, 8, 4, 7, 6, 9.
- Colour op: b53:52. Alpha op: b42:41.
- Byte write mask: b46:43, where b43+k is byte k [G].
- No repeat.

**Op 17, `blend3`** (useasm 225 + 222): the same factor fields with a
`src0`. b41 picks add or subtract; b53:52, b45, b44, b46 and b43 are
further selectors. Not executed.

**Op 23** (useasm 108):

- a4: b41:40
- a5: b39:38
- a6: b37:35
- a7: bits b42, b43, b46, b47
- a8: b53:52
- repeat: b45:44

**Ops 19 and 25** (useasm 98, 244), **22** (103–107) and **24** (90–97):
fields are known only partially. They decode, but are not executed.

**Corpus checks.**

- `blend2 o0, pa0, pa1, s1.a, 1-s1.a, add, one, 1-s1.a, add` is exactly GL
  SRC_ALPHA / ONE_MINUS_SRC_ALPHA.
- The UI's pixel programs `blend o0, pa0, pa0, zero, zero, max?, max?,
  #mask` (mask 0–14) emulate the colour write mask.
- `blend o0, pa2, pa64, s2, zero, add, add, #0xf` is a texture modulate.

## §12 Op 28, texture sample (useasm 111–122; exact fields)

| Bits | Field |
|---|---|
| b43:42 | variant (useasm 111/112/113 → 0/1/2) [G: dimensionality] |
| b27:21 | dst (b39 for pa) |
| b20:14 | coordinates, a pair (banks b50 o, b34 pa) |
| `src1` slot | sampler state, a pair |
| b32 | DRC |
| b56:58 | long predicate |
| b55 | skipinv |
| b54 | END |
| b53 | flags1.4 |
| b52 | nosched |
| b51 | flags3.16 |
| b38:37 | flags3.3/.2 |
| b45:44 | repeat |

Execution sends four registers from each source to `UsseBus::sample`. The
UI corpus has no op-28 word: its texturing goes through the PDS (§13).

## §13 PDS additions

**Conditions.** `[26:24]` value 7 means always and 0 means `P0`. Codes 1–6
have no known meaning; the interpreter asks `PdsBus::condition`. Where
they appear:

- code 3 is on the `DOUTU` that ends every vertex PDS program. The
  driver's generator (libGLES_CM `0x4ed84`, and the same code in
  libsrv_um) ORs in `0x0300_0005` unconditionally, so the condition is
  presumably true whenever a vertex task should run [G].
- codes 3 and 4 are on the branches of the event-dispatch programs in the
  pixel PDS heap.

**DOUT types.**

- Type 2 (iterators): two window words.
- Type 4 (texture): one window word.

Every UI pixel PDS program is `doutu; dout2; dout4; halt`. A bus that
declares these types through `PdsBus::accepts_dout` receives the window
words unchanged. Their contents are the data segment's and are filled in
per draw.

## Open questions

- The `vop` sub-ops 2–7, op 3 sub-ops 2/4/5, op 7 variants 1/2, op 17's
  extra selectors, op 23, and the `pck` bits b53/b48/b50.
- The c10 and o8 lane layouts.
- The EMIT target meanings and the burst length.
- What b43 means on the vector forms.
- PDS condition codes 1–6.
- The contents of the iterator and texture DOUT words.
