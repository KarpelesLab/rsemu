//! The bodies of the built test corpus.
//!
//! Each constant is the *body* of one test: [`super::conformance::PROLOGUE`]
//! supplies the vector table, the handler trampoline and the `CHECK` macros,
//! and the epilogue supplies the pass/fail tail. See that module for the
//! conventions.
//!
//! Every expected value here was derived from DDI 0403's operation
//! pseudocode for the instruction concerned and worked out by hand; a
//! disagreement between one of these numbers and the core is a bug in exactly
//! one of the two, which is the whole point.

/// Data processing: modified immediates, plain immediates, shifted register
/// operands, `ORN`, `MOVW`/`MOVT`, `RRX`, and the flag rules.
pub(super) const DATAPROC: &str = r#"
    @ The four replicated forms of a modified immediate (A5.3.2).
    mov.w r1, #0x000000ab
    CHECK 1, r1, 0x000000ab
    mov.w r1, #0x00ab00ab
    CHECK 2, r1, 0x00ab00ab
    mov.w r1, #0xab00ab00
    CHECK 3, r1, 0xab00ab00
    mov.w r1, #0xabababab
    CHECK 4, r1, 0xabababab
    @ ...and the rotated form, whose top bit is forced set.
    mov.w r1, #0x00ff0000
    CHECK 5, r1, 0x00ff0000

    @ A rotated immediate sets C from bit 31 of the result; a replicated one
    @ leaves C alone.
    movs.w r1, #0x80000000
    CHECKF 6, 0xa0000000
    movs.w r1, #0x00000001
    CHECKF 7, 0x20000000

    @ ADDW / SUBW: a twelve-bit plain immediate, no flags.
    LOADC r1, 0x00001000
    addw r2, r1, #0xfff
    CHECK 8, r2, 0x00001fff
    subw r2, r1, #0xabc
    CHECK 9, r2, 0x00000544

    @ MOVW and MOVT.
    movw r3, #0x1234
    CHECK 10, r3, 0x00001234
    movt r3, #0xfeed
    CHECK 11, r3, 0xfeed1234

    @ ORN and the wide MVN, neither of which ARMv5TE encodes.
    LOADC r1, 0x0f0f0f0f
    orn r2, r1, #0x000000ff
    CHECK 12, r2, 0xffffff0f
    mvn.w r2, #0x0000ff00
    CHECK 13, r2, 0xffff00ff

    @ Shifted-register operands.
    LOADC r1, 0x00000001
    LOADC r2, 0x00000010
    add.w r3, r1, r2, lsl #4
    CHECK 14, r3, 0x00000101
    LOADC r1, 0x80000000
    add.w r3, r2, r1, asr #28
    CHECK 15, r3, 0x00000008

    @ RRX rotates through the carry.
    LOADC r1, 0x00000003
    lsrs r2, r1, #1
    LOADC r1, 0x00000002
    rrx r3, r1
    CHECK 16, r3, 0x80000001

    @ SUBS overflowing: C set because there was no borrow, V set because a
    @ negative minus a positive came out positive.
    LOADC r1, 0x80000000
    LOADC r2, 0x00000001
    subs r3, r1, r2
    CHECKF 17, 0x30000000
    CHECK 18, r3, 0x7fffffff

    @ ADCS then ADC: the carry chains, and the second does not disturb it.
    LOADC r1, 0xffffffff
    LOADC r2, 0x00000001
    adds r3, r1, r2
    adc r4, r2, r2
    CHECK 19, r3, 0x00000000
    CHECK 20, r4, 0x00000003

    @ RSB, which in T32 takes a full operand rather than only #0.
    LOADC r1, 0x00000005
    rsb r2, r1, #0x0000000f
    CHECK 21, r2, 0x0000000a

    @ TST and TEQ discard their result. The flags are cleared first, because
    @ a replicated immediate leaves C alone and the check would otherwise be
    @ measuring whatever the previous CMP left behind.
    movs r0, #0
    msr apsr_nzcvq, r0
    LOADC r1, 0xa5a5a5a5
    LOADC r2, 0xa5a5a5a5
    teq r1, r2
    CHECKF 22, 0x40000000
    movs r0, #0
    msr apsr_nzcvq, r0
    tst r1, #0x0000000a
    CHECKF 23, 0x40000000

    @ ADR, forwards and backwards, is PC-relative and word-aligned.
    adr r1, adr_target
    ldr r2, [r1]
    CHECK 24, r2, 0x0badf00d
    b adr_past
    .align 2
adr_target:
    .word 0x0badf00d
adr_past:

    @ CMN of a value and its negation is zero with C set.
    LOADC r1, 0x00001000
    LOADC r2, 0xfffff000
    cmn r1, r2
    CHECKF 25, 0x60000000
"#;

/// The barrel shifter: every type, immediate and register forms, the
/// amounts the architecture treats specially, and the carry each produces.
pub(super) const SHIFT: &str = r#"
    @ LSL by an immediate, and the carry out of bit 31.
    LOADC r1, 0x80000001
    lsls r2, r1, #1
    CHECKF 1, 0x20000000
    CHECK 2, r2, 0x00000002

    @ LSR #32 is what `LSR #0` encodes; the assembler writes it as #32.
    LOADC r1, 0x80000000
    lsrs r2, r1, #32
    CHECKF 3, 0x60000000
    CHECK 4, r2, 0x00000000

    @ ASR #32 leaves the sign in every bit.
    LOADC r1, 0x80000000
    asrs r2, r1, #32
    CHECKF 5, 0xa0000000
    CHECK 6, r2, 0xffffffff

    @ ROR by an immediate.
    LOADC r1, 0x00000003
    rors r2, r1, #1
    CHECKF 7, 0xa0000000
    CHECK 8, r2, 0x80000001

    @ Register-controlled shifts use only the low byte of Rs: a shift amount
    @ of 0x100 is a shift of nothing, not a shift of 256.
    LOADC r1, 0x00000001
    LOADC r2, 0x00000100
    lsl r3, r1, r2
    CHECK 9, r3, 0x00000001
    LOADC r2, 0x00000104
    lsl r3, r1, r2
    CHECK 10, r3, 0x00000010

    @ A register shift of exactly 32 zeroes the result and puts the last bit
    @ shifted out in the carry; 33 leaves nothing at all.
    LOADC r1, 0x00000001
    LOADC r2, 0x00000020
    lsls r3, r1, r2
    CHECKF 11, 0x60000000
    CHECK 12, r3, 0x00000000
    LOADC r2, 0x00000021
    lsls r3, r1, r2
    CHECKF 13, 0x40000000
    CHECK 14, r3, 0x00000000

    @ ASR by 32 or more with a negative operand.
    LOADC r1, 0x80000000
    LOADC r2, 0x000000ff
    asrs r3, r1, r2
    CHECKF 15, 0xa0000000
    CHECK 16, r3, 0xffffffff

    @ ROR by a multiple of 32 is the identity but still reports bit 31.
    LOADC r1, 0x80000001
    LOADC r2, 0x00000020
    rors r3, r1, r2
    CHECKF 17, 0xa0000000
    CHECK 18, r3, 0x80000001

    @ A register shift of zero changes nothing, carry included.
    LOADC r1, 0x00000003
    lsrs r2, r1, #1
    LOADC r2, 0x00000000
    lsls r3, r1, r2
    CHECKF 19, 0x20000000
    CHECK 20, r3, 0x00000003
"#;

/// Loads and stores: every width, every addressing mode, `LDRD`/`STRD`, the
/// block transfers, the exclusive monitor, and unaligned access.
pub(super) const MEMORY: &str = r#"
    @ A word out and back, and the byte and halfword views of it.
    LOADC r0, 0x20000100
    LOADC r1, 0x12345678
    str r1, [r0]
    ldr r2, [r0]
    CHECK 1, r2, 0x12345678
    ldrb r2, [r0]
    CHECK 2, r2, 0x00000078
    ldrb r2, [r0, #3]
    CHECK 3, r2, 0x00000012
    ldrh r2, [r0, #2]
    CHECK 4, r2, 0x00001234

    @ The sign-extending loads.
    LOADC r1, 0x80009000
    str r1, [r0]
    ldrsh r2, [r0, #2]
    CHECK 5, r2, 0xffff8000
    ldrsb r2, [r0, #1]
    CHECK 6, r2, 0xffffff90

    @ Pre-indexed with writeback, then post-indexed.
    LOADC r0, 0x20000200
    LOADC r1, 0xaabbccdd
    str r1, [r0, #4]!
    CHECK 7, r0, 0x20000204
    ldr r2, [r0]
    CHECK 8, r2, 0xaabbccdd
    ldr r3, [r0], #-4
    CHECK 9, r3, 0xaabbccdd
    CHECK 10, r0, 0x20000200

    @ A negative immediate offset, which only the imm8 encoding can express.
    LOADC r0, 0x20000210
    ldr r4, [r0, #-12]
    CHECK 11, r4, 0xaabbccdd

    @ Register offset with a shift.
    LOADC r0, 0x20000200
    movs r1, #1
    ldr r5, [r0, r1, lsl #2]
    CHECK 12, r5, 0xaabbccdd

    @ LDRD and STRD, which are always word-aligned.
    LOADC r0, 0x20000300
    LOADC r1, 0x11112222
    LOADC r2, 0x33334444
    strd r1, r2, [r0]
    ldrd r3, r4, [r0]
    CHECK 13, r3, 0x11112222
    CHECK 14, r4, 0x33334444
    strd r1, r2, [r0, #8]!
    CHECK 15, r0, 0x20000308
    ldrd r5, r6, [r0, #-8]
    CHECK 16, r5, 0x11112222
    CHECK 17, r6, 0x33334444

    @ STMIA and LDMDB.
    LOADC r0, 0x20000400
    LOADC r1, 0x00000001
    LOADC r2, 0x00000002
    LOADC r3, 0x00000003
    stmia r0!, {r1, r2, r3}
    CHECK 18, r0, 0x2000040c
    ldmdb r0!, {r4, r5, r6}
    CHECK 19, r0, 0x20000400
    CHECK 20, r4, 0x00000001
    CHECK 21, r5, 0x00000002
    CHECK 22, r6, 0x00000003

    @ PUSH and POP reaching the high registers.
    LOADC r1, 0xdeadbeef
    mov r8, r1
    LOADC r1, 0xfeedface
    mov r9, r1
    push {r8, r9}
    LOADC r1, 0x00000000
    mov r8, r1
    mov r9, r1
    pop {r8, r9}
    mov r1, r8
    CHECK 23, r1, 0xdeadbeef
    mov r1, r9
    CHECK 24, r1, 0xfeedface

    @ Unaligned access, which ARMv7-M performs rather than rotating whenever
    @ CCR.UNALIGN_TRP is clear — and it is, out of reset.
    LOADC r0, 0x20000500
    LOADC r1, 0x11223344
    str r1, [r0]
    LOADC r1, 0x55667788
    str r1, [r0, #4]
    ldr.w r2, [r0, #1]
    CHECK 25, r2, 0x88112233
    ldrh.w r2, [r0, #3]
    CHECK 26, r2, 0x00008811
    LOADC r1, 0xa5a5a5a5
    str.w r1, [r0, #1]
    ldr.w r2, [r0, #1]
    CHECK 27, r2, 0xa5a5a5a5
    ldrb r2, [r0]
    CHECK 28, r2, 0x00000044

    @ LDR (literal), backwards.
    b lit_past
    .align 2
lit_word:
    .word 0xcafebabe
lit_past:
    ldr r3, lit_word
    CHECK 29, r3, 0xcafebabe

    @ The local exclusive monitor: a tagged store succeeds once.
    LOADC r0, 0x20000600
    LOADC r1, 0x0000abcd
    ldrex r2, [r0]
    strex r3, r1, [r0]
    CHECK 30, r3, 0x00000000
    ldr r4, [r0]
    CHECK 31, r4, 0x0000abcd
    LOADC r1, 0x00001234
    strex r3, r1, [r0]
    CHECK 32, r3, 0x00000001
    ldr r4, [r0]
    CHECK 33, r4, 0x0000abcd

    @ CLREX drops the tag.
    ldrex r2, [r0]
    clrex
    strex r3, r1, [r0]
    CHECK 34, r3, 0x00000001
"#;

/// The multipliers and the dividers.
pub(super) const MULTIPLY: &str = r#"
    LOADC r1, 0x00010001
    LOADC r2, 0x00000003
    mul r3, r1, r2
    CHECK 1, r3, 0x00030003
    LOADC r4, 0x00000005
    mla r5, r1, r2, r4
    CHECK 2, r5, 0x00030008
    mls r5, r1, r2, r4
    CHECK 3, r5, 0xfffd0002

    @ The 64-bit multiplies, signed and unsigned over the same operands.
    LOADC r1, 0xffffffff
    LOADC r2, 0x00000002
    umull r3, r4, r1, r2
    CHECK 4, r3, 0xfffffffe
    CHECK 5, r4, 0x00000001
    smull r3, r4, r1, r2
    CHECK 6, r3, 0xfffffffe
    CHECK 7, r4, 0xffffffff

    @ UMLAL accumulates the 64-bit pair.
    LOADC r3, 0x00000001
    LOADC r4, 0x00000000
    LOADC r1, 0x00000002
    LOADC r2, 0x00000003
    umlal r3, r4, r1, r2
    CHECK 8, r3, 0x00000007
    CHECK 9, r4, 0x00000000

    @ UMAAL accumulates *both* halves as separate addends.
    LOADC r3, 0x00000005
    LOADC r4, 0x00000007
    umaal r3, r4, r1, r2
    CHECK 10, r3, 0x00000012
    CHECK 11, r4, 0x00000000

    @ SMLAL over a carry boundary.
    LOADC r3, 0xffffffff
    LOADC r4, 0x00000000
    LOADC r1, 0x00000001
    LOADC r2, 0x00000001
    smlal r3, r4, r1, r2
    CHECK 12, r3, 0x00000000
    CHECK 13, r4, 0x00000001

    @ SDIV truncates toward zero; UDIV sees the same bits as a huge positive.
    LOADC r1, 0xfffffff6
    LOADC r2, 0x00000003
    sdiv r3, r1, r2
    CHECK 14, r3, 0xfffffffd
    udiv r4, r1, r2
    CHECK 15, r4, 0x55555552

    @ INT_MIN / -1 wraps rather than trapping.
    LOADC r1, 0x80000000
    LOADC r2, 0xffffffff
    sdiv r3, r1, r2
    CHECK 16, r3, 0x80000000

    @ Division by zero gives zero while CCR.DIV_0_TRP is clear.
    LOADC r1, 0x00000010
    LOADC r2, 0x00000000
    sdiv r3, r1, r2
    CHECK 17, r3, 0x00000000
    udiv r3, r1, r2
    CHECK 18, r3, 0x00000000
"#;

/// Bitfields, the one-operand bit manipulations, and the extending moves.
pub(super) const BITFIELD: &str = r#"
    LOADC r1, 0x12345678
    ubfx r2, r1, #4, #8
    CHECK 1, r2, 0x00000067
    sbfx r3, r1, #4, #8
    CHECK 2, r3, 0x00000067
    LOADC r1, 0x0000f000
    sbfx r3, r1, #12, #4
    CHECK 3, r3, 0xffffffff

    LOADC r2, 0xffffffff
    LOADC r1, 0x00000005
    bfi r2, r1, #8, #4
    CHECK 4, r2, 0xfffff5ff
    bfc r2, #0, #8
    CHECK 5, r2, 0xfffff500

    LOADC r1, 0x00008000
    clz r3, r1
    CHECK 6, r3, 0x00000010
    LOADC r1, 0x00000000
    clz r3, r1
    CHECK 7, r3, 0x00000020

    LOADC r1, 0x12345678
    rbit r3, r1
    CHECK 8, r3, 0x1e6a2c48
    rev r3, r1
    CHECK 9, r3, 0x78563412
    rev16 r3, r1
    CHECK 10, r3, 0x34127856
    LOADC r1, 0x0000f0ab
    revsh r3, r1
    CHECK 11, r3, 0xffffabf0

    LOADC r1, 0x000000ff
    sxtb r3, r1
    CHECK 12, r3, 0xffffffff
    uxtb r3, r1
    CHECK 13, r3, 0x000000ff
    LOADC r1, 0x0000ff00
    sxth r3, r1
    CHECK 14, r3, 0xffffff00
    uxth r3, r1
    CHECK 15, r3, 0x0000ff00

    @ The rotate is part of the base encoding, not of the DSP extension.
    LOADC r1, 0xaabbccdd
    uxtb r3, r1, ror #8
    CHECK 16, r3, 0x000000cc
    sxtb r3, r1, ror #24
    CHECK 17, r3, 0xffffffaa
"#;

/// Branches: the wide forms, `BL`, `BX`, `CBZ`/`CBNZ`, `TBB` and `TBH`.
pub(super) const BRANCH: &str = r#"
    b main

    .thumb_func
subr:
    movs r1, #0x55
    bx lr

    .thumb_func
main:
    @ A wide unconditional branch.
    b.w after_wide
    movs r0, #90
    b fail
after_wide:

    @ BL and the return through LR.
    movs r1, #0
    bl subr
    CHECK 1, r1, 0x00000055

    @ CBZ and CBNZ, neither of which ARMv5TE has.
    movs r2, #0
    cbz r2, after_cbz
    movs r0, #91
    b fail
after_cbz:
    movs r2, #1
    cbnz r2, after_cbnz
    movs r0, #92
    b fail
after_cbnz:

    @ CBZ must *not* branch when the condition fails.
    movs r2, #1
    cbz r2, cbz_wrong
    b cbz_right
cbz_wrong:
    movs r0, #93
    b fail
cbz_right:

    @ TBB: a byte table of halfword offsets from the table's own address.
    movs r2, #1
    tbb [pc, r2]
tbb_table:
    .byte (tbb_0 - tbb_table) / 2
    .byte (tbb_1 - tbb_table) / 2
    .byte (tbb_2 - tbb_table) / 2
    .byte 0
tbb_0:
    movs r0, #94
    b fail
tbb_1:
    b tbb_done
tbb_2:
    movs r0, #95
    b fail
tbb_done:

    @ TBH: the same with halfword entries.
    movs r2, #2
    tbh [pc, r2, lsl #1]
tbh_table:
    .hword (tbh_0 - tbh_table) / 2
    .hword (tbh_1 - tbh_table) / 2
    .hword (tbh_2 - tbh_table) / 2
    .hword 0
tbh_0:
    movs r0, #96
    b fail
tbh_1:
    movs r0, #97
    b fail
tbh_2:

    @ BX to a Thumb address computed with ADR.
    adr r3, after_bx
    orr r3, r3, #1
    bx r3
    movs r0, #98
    b fail
after_bx:

    @ BLX (register) returns through LR, which carries the Thumb bit.
    movs r5, #0
    adr r4, blx_target
    orr r4, r4, #1
    blx r4
    b after_blx
    .thumb_func
blx_target:
    mov r5, lr
    bx lr
after_blx:
    and r6, r5, #1
    CHECK 2, r6, 0x00000001
"#;

/// `IT` blocks: how ARMv7-M does conditional execution at all.
pub(super) const IT: &str = r#"
    @ ITTEE with the condition true.
    movs r1, #0
    movs r2, #0
    movs r3, #0
    movs r4, #0
    cmp r1, #0
    ittee eq
    moveq r1, #1
    moveq r2, #2
    movne r3, #3
    movne r4, #4
    CHECK 1, r1, 0x00000001
    CHECK 2, r2, 0x00000002
    CHECK 3, r3, 0x00000000
    CHECK 4, r4, 0x00000000

    @ The same block with the condition false: the two `E` slots run instead.
    movs r1, #0
    movs r2, #0
    movs r3, #0
    movs r4, #0
    cmp r1, #1
    ittee eq
    moveq r1, #1
    moveq r2, #2
    movne r3, #3
    movne r4, #4
    CHECK 5, r1, 0x00000000
    CHECK 6, r2, 0x00000000
    CHECK 7, r3, 0x00000003
    CHECK 8, r4, 0x00000004

    @ Four `T` slots, all taken.
    movs r1, #0
    cmp r1, #0
    itttt eq
    addeq r1, r1, #1
    addeq r1, r1, #1
    addeq r1, r1, #1
    addeq r1, r1, #1
    CHECK 9, r1, 0x00000004

    @ A skipped memory access must not happen at all.
    LOADC r0, 0x20000700
    movs r1, #0
    str r1, [r0]
    LOADC r3, 0x00001234
    movs r2, #0
    cmp r2, #1
    it eq
    streq r3, [r0]
    ldr r4, [r0]
    CHECK 10, r4, 0x00000000

    @ ...and a taken one must.
    cmp r2, #0
    it eq
    streq r3, [r0]
    ldr r4, [r0]
    CHECK 11, r4, 0x00001234

    @ ITSTATE is cleared when the block ends, so the next instruction runs
    @ unconditionally whatever the flags say.
    movs r1, #0
    cmp r1, #1
    ite eq
    moveq r1, #1
    movne r1, #2
    movs r5, #7
    CHECK 12, r1, 0x00000002
    CHECK 13, r5, 0x00000007

    @ MRS of xPSR reads the EPSR bits — the `T` bit and ITSTATE — as zero.
    movs r1, #0
    cmp r1, #0
    itt eq
    mrseq r6, xpsr
    moveq r7, #1
    LOADC r0, 0x0600fc00
    and r6, r6, r0
    CHECK 14, r6, 0x00000000
    CHECK 15, r7, 0x00000001

    @ A wide instruction inside an IT block.
    movs r1, #0
    cmp r1, #0
    it eq
    addweq r2, r1, #0x123
    CHECK 16, r2, 0x00000123
"#;

/// The SIMD half of the DSP extension, and the `GE` bits it feeds `SEL`.
pub(super) const DSP_SIMD: &str = r#"
    @ SADD16, and the GE bits set by two non-negative lanes.
    LOADC r1, 0x00010002
    LOADC r2, 0x00030004
    sadd16 r3, r1, r2
    mrs r4, apsr
    LOADC r5, 0x000f0000
    and r4, r4, r5
    CHECK 1, r3, 0x00040006
    CHECK 2, r4, 0x000f0000

    @ SSUB16 with one negative lane clears that lane's two GE bits.
    LOADC r1, 0x00010001
    LOADC r2, 0x00020000
    ssub16 r3, r1, r2
    mrs r4, apsr
    LOADC r5, 0x000f0000
    and r4, r4, r5
    CHECK 3, r3, 0xffff0001
    CHECK 4, r4, 0x00030000

    @ SEL picks bytes from Rn where GE is set and Rm where it is not.
    LOADC r1, 0xaaaaaaaa
    LOADC r2, 0xbbbbbbbb
    sel r6, r1, r2
    CHECK 5, r6, 0xbbbbaaaa

    @ QADD16 saturates each halfword.
    LOADC r1, 0x7fff8000
    LOADC r2, 0x00018000
    qadd16 r3, r1, r2
    CHECK 6, r3, 0x7fff8000

    @ UQADD8 clamps at 0xff; UHADD8 halves without clamping.
    LOADC r1, 0xff01ff01
    LOADC r2, 0x02ff02ff
    uqadd8 r3, r1, r2
    CHECK 7, r3, 0xffffffff
    uhadd8 r3, r1, r2
    CHECK 8, r3, 0x80808080

    @ SHADD16 is exact: the extra bit the sum needs is the one the shift
    @ takes away again.
    LOADC r1, 0x7fff7fff
    LOADC r2, 0x7fff7fff
    shadd16 r3, r1, r2
    CHECK 9, r3, 0x7fff7fff

    @ UQSUB8 clamps at zero.
    LOADC r1, 0x01020304
    LOADC r2, 0x04030201
    uqsub8 r3, r1, r2
    CHECK 10, r3, 0x00000103

    @ SASX and SSAX cross Rm's halves in opposite directions.
    LOADC r1, 0x00050003
    LOADC r2, 0x00020001
    sasx r3, r1, r2
    CHECK 11, r3, 0x00060001
    ssax r3, r1, r2
    CHECK 12, r3, 0x00040005

    @ USAD8 and USADA8.
    LOADC r1, 0x01020304
    LOADC r2, 0x04030201
    usad8 r3, r1, r2
    CHECK 13, r3, 0x00000008
    movs r4, #10
    usada8 r3, r1, r2, r4
    CHECK 14, r3, 0x00000012

    @ PKHBT keeps Rn's bottom half, PKHTB keeps Rn's top.
    LOADC r1, 0x11112222
    LOADC r2, 0x33334444
    pkhbt r3, r1, r2, lsl #16
    CHECK 15, r3, 0x44442222
    pkhtb r3, r1, r2, asr #16
    CHECK 16, r3, 0x11113333

    @ UXTAB16 and SXTAB16 accumulate each halfword separately, so the low
    @ half's carry does not reach the high one.
    LOADC r1, 0x00010001
    LOADC r2, 0x00ff00ff
    uxtab16 r3, r1, r2
    CHECK 17, r3, 0x01000100
    sxtab16 r3, r1, r2
    CHECK 18, r3, 0x00000000

    @ SXTB16 on its own.
    LOADC r2, 0x00800080
    sxtb16 r3, r2
    CHECK 19, r3, 0xff80ff80
"#;

/// The DSP multiplies, the saturating arithmetic, and the `Q` flag.
pub(super) const DSP_MULTIPLY: &str = r#"
    @ The four halfword multiplies: the first suffix picks Rn's half.
    LOADC r1, 0x00020003
    LOADC r2, 0x00040005
    smulbb r3, r1, r2
    CHECK 1, r3, 0x0000000f
    smulbt r3, r1, r2
    CHECK 2, r3, 0x0000000c
    smultb r3, r1, r2
    CHECK 3, r3, 0x0000000a
    smultt r3, r1, r2
    CHECK 4, r3, 0x00000008

    movs r4, #100
    smlabb r3, r1, r2, r4
    CHECK 5, r3, 0x00000073

    @ SMULWB takes the top 32 bits of a 48-bit product.
    LOADC r5, 0x00010000
    LOADC r6, 0x00000002
    smulwb r3, r5, r6
    CHECK 6, r3, 0x00000002

    @ SMUAD, its exchanging form, and SMUSD.
    smuad r3, r1, r2
    CHECK 7, r3, 0x00000017
    smuadx r3, r1, r2
    CHECK 8, r3, 0x00000016
    smusd r3, r1, r2
    CHECK 9, r3, 0x00000007

    movs r4, #10
    smlad r3, r1, r2, r4
    CHECK 10, r3, 0x00000021
    smlsd r3, r1, r2, r4
    CHECK 11, r3, 0x00000011

    @ SMMUL, SMMLA and SMMLS keep the top word of a 64-bit product.
    LOADC r5, 0x40000000
    LOADC r6, 0x40000000
    smmul r3, r5, r6
    CHECK 12, r3, 0x10000000
    movs r4, #1
    smmla r3, r5, r6, r4
    CHECK 13, r3, 0x10000001
    smmls r3, r5, r6, r4
    CHECK 14, r3, 0xf0000001

    @ SMLALD accumulates the dual sum into a 64-bit pair.
    LOADC r3, 0x00000001
    LOADC r4, 0x00000000
    smlald r3, r4, r1, r2
    CHECK 15, r3, 0x00000018
    CHECK 16, r4, 0x00000000

    @ Clear Q, then make QADD set it.
    movs r5, #0
    msr apsr_nzcvq, r5
    LOADC r1, 0x7fffffff
    LOADC r2, 0x00000001
    qadd r3, r1, r2
    mrs r4, apsr
    LOADC r5, 0x08000000
    and r4, r4, r5
    CHECK 17, r3, 0x7fffffff
    CHECK 18, r4, 0x08000000

    @ QDADD doubles its second source with its own saturation first.
    movs r5, #0
    msr apsr_nzcvq, r5
    LOADC r1, 0x00000010
    LOADC r2, 0x00000003
    qdadd r3, r1, r2
    CHECK 19, r3, 0x00000016
    qdsub r3, r1, r2
    CHECK 20, r3, 0x0000000a
    qsub r3, r1, r2
    CHECK 21, r3, 0x0000000d

    @ SSAT and USAT, both ends.
    LOADC r1, 0x00000100
    ssat r3, #8, r1
    CHECK 22, r3, 0x0000007f
    usat r3, #8, r1
    CHECK 23, r3, 0x000000ff
    LOADC r1, 0xffffff00
    ssat r3, #8, r1
    CHECK 24, r3, 0xffffff80
    usat r3, #8, r1
    CHECK 25, r3, 0x00000000

    @ SSAT with a shift applied first.
    LOADC r1, 0x00000040
    ssat r3, #8, r1, lsl #2
    CHECK 26, r3, 0x0000007f

    @ The halfword saturates.
    LOADC r1, 0x01000100
    ssat16 r3, #8, r1
    CHECK 27, r3, 0x007f007f
    usat16 r3, #8, r1
    CHECK 28, r3, 0x00ff00ff
"#;

/// The exception model: entry, the stacked frame, `EXC_RETURN`, the two
/// stack pointers, `CONTROL`, and the masking registers.
pub(super) const EXCEPTIONS: &str = r#"
    b exc_main

    @ An SVCall handler that checks its own context, then edits the stacked
    @ frame so the interrupted code can see that it ran.
    .thumb_func
svc_handler:
    mrs r4, ipsr
    CHECK 60, r4, 0x0000000b
    CHECK 61, lr, 0xfffffff9
    ldr r4, [sp, #12]
    str r4, [sp, #4]
    bx lr

    @ A handler that only records EXC_RETURN, in a register the frame does
    @ not restore.
    .thumb_func
lr_handler:
    mov r5, lr
    bx lr

    @ A handler that records the exception number, likewise.
    .thumb_func
num_handler:
    mrs r6, ipsr
    bx lr

    .thumb_func
exc_main:
    @ SVC from Thread mode on the main stack.
    LOADC r0, HANDLER_PTR
    adr r1, svc_handler
    orr r1, r1, #1
    str r1, [r0]
    movs r1, #0
    movs r3, #0x22
    svc #7
    CHECK 1, r1, 0x00000022

    @ CONTROL.SPSEL moves Thread mode onto the process stack.
    LOADC r0, HANDLER_PTR
    adr r1, lr_handler
    orr r1, r1, #1
    str r1, [r0]
    LOADC r0, 0x20008000
    msr psp, r0
    movs r1, #2
    msr control, r1
    isb
    mov r2, sp
    CHECK 2, r2, 0x20008000

    @ An exception taken from Thread/PSP returns with 0xFFFFFFFD.
    movs r5, #0
    svc #0
    movs r1, #0
    msr control, r1
    isb
    CHECK 3, r5, 0xfffffffd
    mov r2, sp
    CHECK 4, r2, 0x20010000

    @ ...and one taken from Thread/MSP returns with 0xFFFFFFF9.
    movs r5, #0
    svc #0
    CHECK 5, r5, 0xfffffff9

    @ NVIC: enable IRQ0, pend it, and watch it arrive as exception 16.
    LOADC r0, HANDLER_PTR
    adr r1, num_handler
    orr r1, r1, #1
    str r1, [r0]
    LOADC r0, 0xe000e100
    movs r1, #1
    str r1, [r0]
    movs r6, #0
    LOADC r0, 0xe000e200
    movs r1, #1
    str r1, [r0]
    nop
    nop
    CHECK 6, r6, 0x00000010

    @ PRIMASK holds it off, and releasing PRIMASK lets it through.
    cpsid i
    movs r6, #0
    LOADC r0, 0xe000e200
    movs r1, #1
    str r1, [r0]
    nop
    nop
    CHECK 7, r6, 0x00000000
    cpsie i
    nop
    nop
    CHECK 8, r6, 0x00000010

    @ PendSV, pended through ICSR.
    movs r6, #0
    LOADC r0, 0xe000ed04
    LOADC r1, 0x10000000
    str r1, [r0]
    nop
    nop
    CHECK 9, r6, 0x0000000e

    @ BASEPRI blocks an interrupt whose priority is numerically no lower.
    LOADC r0, 0xe000e400
    movs r1, #0x40
    strb r1, [r0]
    movs r1, #0x40
    msr basepri, r1
    movs r6, #0
    LOADC r0, 0xe000e200
    movs r1, #1
    str r1, [r0]
    nop
    nop
    CHECK 10, r6, 0x00000000
    movs r1, #0
    msr basepri, r1
    nop
    nop
    CHECK 11, r6, 0x00000010

    @ The stacked frame holds R0-R3, R12, LR, the return address and xPSR,
    @ in that order, and the handler can read all eight.
    LOADC r0, HANDLER_PTR
    adr r1, frame_handler
    orr r1, r1, #1
    str r1, [r0]
    b past_frame_handler
    .thumb_func
frame_handler:
    ldr r4, [sp, #0]
    CHECK 62, r4, 0x000000a0
    ldr r4, [sp, #4]
    CHECK 63, r4, 0x000000a1
    ldr r4, [sp, #8]
    CHECK 64, r4, 0x000000a2
    ldr r4, [sp, #12]
    CHECK 65, r4, 0x000000a3
    @ The stacked xPSR carries the T bit and no exception number.
    ldr r4, [sp, #28]
    LOADC r11, 0x010001ff
    and r4, r4, r11
    CHECK 66, r4, 0x01000000
    bx lr
past_frame_handler:
    movs r0, #0xa0
    movs r1, #0xa1
    movs r2, #0xa2
    movs r3, #0xa3
    svc #0
    CHECK 12, r0, 0x000000a0
    CHECK 13, r1, 0x000000a1
    CHECK 14, r2, 0x000000a2
    CHECK 15, r3, 0x000000a3
"#;

/// The fault taxonomy: which fault, which status bit, and when a fault
/// escalates to HardFault.
pub(super) const FAULTS: &str = r#"
    b fault_main

    @ A handler that records the exception number and steps the interrupted
    @ code past the instruction that faulted; R7 says how long that
    @ instruction was.
    .thumb_func
fault_handler:
    mrs r6, ipsr
    ldr r0, [sp, #24]
    add r0, r0, r7
    str r0, [sp, #24]
    bx lr

    .thumb_func
fault_main:
    LOADC r0, HANDLER_PTR
    adr r1, fault_handler
    orr r1, r1, #1
    str r1, [r0]

    @ Enable UsageFault so it is taken rather than escalated.
    LOADC r0, 0xe000ed24
    LOADC r1, 0x00040000
    str r1, [r0]

    @ An undefined instruction.
    movs r6, #0
    movs r7, #2
    udf #0
    CHECK 1, r6, 0x00000006
    LOADC r0, 0xe000ed28
    ldr r1, [r0]
    LOADC r2, 0x00010000
    and r1, r1, r2
    CHECK 2, r1, 0x00010000

    @ CFSR is write-one-to-clear.
    LOADC r0, 0xe000ed28
    LOADC r1, 0xffffffff
    str r1, [r0]
    ldr r1, [r0]
    CHECK 3, r1, 0x00000000

    @ Divide by zero with CCR.DIV_0_TRP set.
    LOADC r0, 0xe000ed14
    ldr r1, [r0]
    orr r1, r1, #0x10
    str r1, [r0]
    movs r6, #0
    movs r7, #4
    LOADC r1, 0x00000010
    movs r2, #0
    sdiv r3, r1, r2
    CHECK 4, r6, 0x00000006
    LOADC r0, 0xe000ed28
    ldr r1, [r0]
    LOADC r2, 0x02000000
    and r1, r1, r2
    CHECK 5, r1, 0x02000000
    LOADC r0, 0xe000ed28
    LOADC r1, 0xffffffff
    str r1, [r0]

    @ An unaligned access with CCR.UNALIGN_TRP set.
    LOADC r0, 0xe000ed14
    ldr r1, [r0]
    orr r1, r1, #0x08
    str r1, [r0]
    movs r6, #0
    movs r7, #4
    LOADC r0, 0x20000501
    ldr.w r1, [r0]
    CHECK 6, r6, 0x00000006
    LOADC r0, 0xe000ed28
    ldr r1, [r0]
    LOADC r2, 0x01000000
    and r1, r1, r2
    CHECK 7, r1, 0x01000000
    @ Put CCR back and check the same access now succeeds.
    LOADC r0, 0xe000ed14
    ldr r1, [r0]
    bic r1, r1, #0x18
    str r1, [r0]
    LOADC r0, 0xe000ed28
    LOADC r1, 0xffffffff
    str r1, [r0]
    movs r6, #0
    LOADC r0, 0x20000501
    ldr.w r1, [r0]
    CHECK 8, r6, 0x00000000

    @ A BusFault: nothing is mapped above the SRAM.
    LOADC r0, 0xe000ed24
    LOADC r1, 0x00060000
    str r1, [r0]
    movs r6, #0
    movs r7, #4
    LOADC r0, 0x40000000
    ldr.w r1, [r0]
    CHECK 9, r6, 0x00000005
    LOADC r0, 0xe000ed28
    ldr r1, [r0]
    LOADC r2, 0x00008200
    and r1, r1, r2
    CHECK 10, r1, 0x00008200
    LOADC r0, 0xe000ed38
    ldr r1, [r0]
    CHECK 11, r1, 0x40000000

    @ With UsageFault disabled, an undefined instruction escalates to
    @ HardFault and HFSR.FORCED says so.
    LOADC r0, 0xe000ed28
    LOADC r1, 0xffffffff
    str r1, [r0]
    LOADC r0, 0xe000ed2c
    LOADC r1, 0xffffffff
    str r1, [r0]
    LOADC r0, 0xe000ed24
    movs r1, #0
    str r1, [r0]
    movs r6, #0
    movs r7, #2
    udf #0
    CHECK 12, r6, 0x00000003
    LOADC r0, 0xe000ed2c
    ldr r1, [r0]
    LOADC r2, 0x40000000
    and r1, r1, r2
    CHECK 13, r1, 0x40000000
"#;

/// The NVIC's register map, SysTick, and the identification registers.
pub(super) const NVIC_SYSTICK: &str = r#"
    b nvic_main

    .thumb_func
tick_handler:
    mrs r6, ipsr
    bx lr

    .thumb_func
nvic_main:
    @ CPUID names a real part.
    LOADC r0, 0xe000ed00
    ldr r1, [r0]
    LOADC r2, 0xff00fff0
    and r1, r1, r2
    CHECK 1, r1, 0x4100c240

    @ Only the implemented priority bits stick, which is how CMSIS counts
    @ them.
    LOADC r0, 0xe000e400
    movs r1, #0xff
    strb r1, [r0]
    ldrb r2, [r0]
    CHECK 2, r2, 0x000000e0
    movs r1, #0
    strb r1, [r0]

    @ ISER and ICER are separate set and clear registers over one bitmap.
    LOADC r0, 0xe000e100
    movs r1, #3
    str r1, [r0]
    ldr r2, [r0]
    CHECK 3, r2, 0x00000003
    LOADC r0, 0xe000e180
    movs r1, #1
    str r1, [r0]
    LOADC r0, 0xe000e100
    ldr r2, [r0]
    CHECK 4, r2, 0x00000002

    @ ICSR reports the highest pending exception while PRIMASK holds it off.
    cpsid i
    LOADC r0, 0xe000e200
    movs r1, #2
    str r1, [r0]
    LOADC r0, 0xe000ed04
    ldr r2, [r0]
    LOADC r3, 0x001ff000
    and r2, r2, r3
    CHECK 5, r2, 0x00011000
    LOADC r0, 0xe000e280
    movs r1, #2
    str r1, [r0]
    cpsie i

    @ VTOR reads back what it is given, with the low seven bits fixed at
    @ zero.
    LOADC r0, 0xe000ed08
    LOADC r1, 0x2000007f
    str r1, [r0]
    ldr r2, [r0]
    CHECK 6, r2, 0x20000000
    movs r1, #0
    str r1, [r0]

    @ AIRCR ignores a write with the wrong key.
    LOADC r0, 0xe000ed0c
    LOADC r1, 0x00000700
    str r1, [r0]
    ldr r2, [r0]
    LOADC r3, 0x00000700
    and r2, r2, r3
    CHECK 7, r2, 0x00000000
    LOADC r1, 0x05fa0500
    str r1, [r0]
    ldr r2, [r0]
    LOADC r3, 0x00000700
    and r2, r2, r3
    CHECK 8, r2, 0x00000500
    LOADC r1, 0x05fa0000
    str r1, [r0]

    @ SysTick counts the processor clock down and pends its exception.
    LOADC r0, HANDLER_PTR
    adr r1, tick_handler
    orr r1, r1, #1
    str r1, [r0]
    movs r6, #0
    @ A reload large enough that the interrupted code makes progress: an
    @ entry-and-return round trip is a couple of dozen cycles, so a reload of
    @ sixteen would leave Thread mode no time to run at all.
    LOADC r0, 0xe000e014
    LOADC r1, 0x00000400
    str r1, [r0]
    LOADC r0, 0xe000e018
    movs r1, #0
    str r1, [r0]
    LOADC r0, 0xe000e010
    movs r1, #3
    str r1, [r0]
systick_wait:
    cmp r6, #0
    beq systick_wait
    CHECK 9, r6, 0x0000000f
    LOADC r0, 0xe000e010
    movs r1, #0
    str r1, [r0]
"#;

/// The memory protection unit.
pub(super) const MPU: &str = r#"
    b mpu_main

    .thumb_func
mpu_handler:
    mrs r6, ipsr
    ldr r0, [sp, #24]
    add r0, r0, r7
    str r0, [sp, #24]
    bx lr

    .thumb_func
mpu_main:
    @ MPU_TYPE reports eight regions.
    LOADC r0, 0xe000ed90
    ldr r1, [r0]
    LOADC r2, 0x0000ff00
    and r1, r1, r2
    CHECK 1, r1, 0x00000800

    @ Region 0: the whole address space, full access. Without it the
    @ background region would be what the next check measured.
    LOADC r0, 0xe000ed9c
    LOADC r1, 0x00000010
    str r1, [r0]
    LOADC r0, 0xe000eda0
    LOADC r1, 0x0300003f
    str r1, [r0]

    @ Region 1: thirty-two bytes at 0x20000800, privileged read-only.
    LOADC r0, 0xe000ed9c
    LOADC r1, 0x20000811
    str r1, [r0]
    LOADC r0, 0xe000eda0
    LOADC r1, 0x05000009
    str r1, [r0]

    LOADC r0, HANDLER_PTR
    adr r1, mpu_handler
    orr r1, r1, #1
    str r1, [r0]
    LOADC r0, 0xe000ed24
    LOADC r1, 0x00010000
    str r1, [r0]

    @ Enable the MPU, with the default map still available to privileged
    @ code.
    LOADC r0, 0xe000ed94
    movs r1, #5
    str r1, [r0]
    dsb
    isb

    @ A write into the read-only region faults; MMFAR names the address.
    movs r6, #0
    movs r7, #4
    LOADC r0, 0x20000800
    movs r1, #1
    str.w r1, [r0]
    CHECK 2, r6, 0x00000004
    LOADC r0, 0xe000ed34
    ldr r1, [r0]
    CHECK 3, r1, 0x20000800
    LOADC r0, 0xe000ed28
    ldr r1, [r0]
    LOADC r2, 0x00000082
    and r1, r1, r2
    CHECK 4, r1, 0x00000082

    @ A read of the same address is allowed.
    LOADC r0, 0xe000ed28
    LOADC r1, 0xffffffff
    str r1, [r0]
    movs r6, #0
    LOADC r0, 0x20000800
    ldr.w r2, [r0]
    CHECK 5, r6, 0x00000000

    @ Just outside the thirty-two-byte region, the write succeeds.
    movs r6, #0
    LOADC r0, 0x20000820
    movs r1, #1
    str.w r1, [r0]
    CHECK 6, r6, 0x00000000

    @ With the MPU off, the write inside the region succeeds too.
    LOADC r0, 0xe000ed94
    movs r1, #0
    str r1, [r0]
    dsb
    isb
    movs r6, #0
    LOADC r0, 0x20000800
    movs r1, #1
    str.w r1, [r0]
    CHECK 7, r6, 0x00000000
"#;
