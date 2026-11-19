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

// ---------------------------------------------------------------------------
// The FPv4-SP floating-point unit
//
// Every expected bit pattern below is IEEE 754-2019's result for the
// operation at binary32, with DDI 0403E A7.7's Arm-specific choices where
// IEEE leaves one open. Nothing here was read off a run of this core: the
// values were worked out from the formats and written down first, which is
// what makes the corpus evidence rather than a recording.
// ---------------------------------------------------------------------------

/// Enable `CP10` and `CP11`, which every test below has to do first because
/// `CPACR` is zero out of reset.
#[cfg(feature = "cpu-arm-v7m-fp")]
pub(super) const ENABLE_FP: &str = r#"
    LOADC r0, 0xe000ed88
    ldr r1, [r0]
    LOADC r2, 0x00f00000
    orr r1, r1, r2
    str r1, [r0]
    dsb
    isb
"#;

/// The single-precision arithmetic, its rounding modes and its flags.
#[cfg(feature = "cpu-arm-v7m-fp")]
pub(super) const FP_ARITH: &str = r#"
    @ 1.5 + 2.25 = 3.75, exactly, with no flag raised.
    LOADC r0, 0x3fc00000
    LOADC r1, 0x40100000
    vmov s0, r0
    vmov s1, r1
    vadd.f32 s2, s0, s1
    vmov r2, s2
    CHECK 1, r2, 0x40700000
    vmrs r2, fpscr
    CHECK 2, r2, 0x00000000

    @ 7.0 / 2.0 = 3.5, and 2.0 * 3.0 = 6.0.
    LOADC r0, 0x40e00000
    LOADC r1, 0x40000000
    vmov s0, r0
    vmov s1, r1
    vdiv.f32 s2, s0, s1
    vmov r2, s2
    CHECK 3, r2, 0x40600000
    LOADC r0, 0x40000000
    LOADC r1, 0x40400000
    vmov s0, r0
    vmov s1, r1
    vmul.f32 s2, s0, s1
    vmov r2, s2
    CHECK 4, r2, 0x40c00000

    @ sqrt(9.0) = 3.0, and VABS/VNEG are bit operations.
    LOADC r0, 0x41100000
    vmov s0, r0
    vsqrt.f32 s1, s0
    vmov r2, s1
    CHECK 5, r2, 0x40400000
    vneg.f32 s2, s1
    vmov r2, s2
    CHECK 6, r2, 0xc0400000
    vabs.f32 s3, s2
    vmov r2, s3
    CHECK 7, r2, 0x40400000

    @ 1/3 in each of the four rounding modes. Round-to-nearest and
    @ round-toward-positive give the value above, the other two the value
    @ below, and inexact is raised every time.
    LOADC r0, 0x3f800000
    LOADC r1, 0x40400000
    vmov s0, r0
    vmov s1, r1
    movs r3, #0
    movs r4, #0
    vmsr fpscr, r3
    vdiv.f32 s2, s0, s1
    vmov r2, s2
    CHECK 8, r2, 0x3eaaaaab
    LOADC r3, 0x00400000
    vmsr fpscr, r3
    vdiv.f32 s2, s0, s1
    vmov r2, s2
    CHECK 9, r2, 0x3eaaaaab
    LOADC r3, 0x00800000
    vmsr fpscr, r3
    vdiv.f32 s2, s0, s1
    vmov r2, s2
    CHECK 10, r2, 0x3eaaaaaa
    LOADC r3, 0x00c00000
    vmsr fpscr, r3
    vdiv.f32 s2, s0, s1
    vmov r2, s2
    CHECK 11, r2, 0x3eaaaaaa
    vmrs r2, fpscr
    LOADC r3, 0x0000001f
    and r2, r2, r3
    CHECK 12, r2, 0x00000010

    @ 1.0 / 0.0 raises divide-by-zero and gives +inf; 0.0 / 0.0 raises
    @ invalid and gives the default NaN.
    movs r3, #0
    vmsr fpscr, r3
    LOADC r0, 0x3f800000
    movs r1, #0
    vmov s0, r0
    vmov s1, r1
    vdiv.f32 s2, s0, s1
    vmov r2, s2
    CHECK 13, r2, 0x7f800000
    vmrs r2, fpscr
    LOADC r3, 0x0000001f
    and r2, r2, r3
    CHECK 14, r2, 0x00000002
    movs r3, #0
    vmsr fpscr, r3
    vmov s0, s1
    vdiv.f32 s2, s0, s1
    vmov r2, s2
    CHECK 15, r2, 0x7fc00000
    vmrs r2, fpscr
    LOADC r3, 0x0000001f
    and r2, r2, r3
    CHECK 16, r2, 0x00000001

    @ VMLA is chained and VFMA is fused, and here they differ: the product
    @ (1 + 2^-23)^2 rounds to 1 + 2^-22, which the addend cancels exactly,
    @ but the unrounded product leaves 2^-46 behind.
    movs r3, #0
    vmsr fpscr, r3
    LOADC r0, 0x3f800001
    LOADC r1, 0xbf800002
    vmov s0, r0
    vmov s1, r0
    vmov s2, r1
    vmla.f32 s2, s0, s1
    vmov r2, s2
    CHECK 17, r2, 0x00000000
    vmov s2, r1
    vfma.f32 s2, s0, s1
    vmov r2, s2
    CHECK 18, r2, 0x28800000

    @ VCMP writes FPSCR's own flags, and VMRS APSR_nzcv moves them across.
    @ Unordered sets C and V both, which is what makes BVS the "was a NaN
    @ involved" test.
    LOADC r0, 0x3f800000
    LOADC r1, 0x40000000
    vmov s0, r0
    vmov s1, r1
    vcmp.f32 s0, s1
    vmrs APSR_nzcv, fpscr
    CHECKF 19, 0x80000000
    vcmp.f32 s1, s0
    vmrs APSR_nzcv, fpscr
    CHECKF 20, 0x20000000
    vcmp.f32 s0, s0
    vmrs APSR_nzcv, fpscr
    CHECKF 21, 0x60000000
    LOADC r0, 0x7fc00000
    vmov s0, r0
    vcmp.f32 s0, s1
    vmrs APSR_nzcv, fpscr
    CHECKF 22, 0x30000000

    @ A quiet NaN propagates its payload with DN clear, and is replaced by
    @ the default NaN with DN set.
    movs r3, #0
    vmsr fpscr, r3
    LOADC r0, 0x7fc01234
    LOADC r1, 0x3f800000
    vmov s0, r0
    vmov s1, r1
    vadd.f32 s2, s0, s1
    vmov r2, s2
    CHECK 23, r2, 0x7fc01234
    LOADC r3, 0x02000000
    vmsr fpscr, r3
    vadd.f32 s2, s0, s1
    vmov r2, s2
    CHECK 24, r2, 0x7fc00000

    @ With FZ set a subnormal operand is replaced by a zero of its own sign
    @ and IDC records it; with FZ clear it is used exactly.
    movs r3, #0
    vmsr fpscr, r3
    movs r0, #1
    LOADC r1, 0x3f800000
    vmov s0, r0
    vmov s1, r1
    vmul.f32 s2, s0, s1
    vmov r2, s2
    CHECK 25, r2, 0x00000001
    LOADC r3, 0x01000000
    vmsr fpscr, r3
    vmul.f32 s2, s0, s1
    vmov r2, s2
    CHECK 26, r2, 0x00000000
    vmrs r2, fpscr
    LOADC r3, 0x0000009f
    and r2, r2, r3
    CHECK 27, r2, 0x00000080

    b pass
"#;

/// The conversions: integer, fixed point and half precision.
#[cfg(feature = "cpu-arm-v7m-fp")]
pub(super) const FP_CONVERT: &str = r#"
    movs r3, #0
    vmsr fpscr, r3

    @ VCVT truncates whatever FPSCR.RMode says; VCVTR uses the mode.
    LOADC r0, 0x40600000
    vmov s0, r0
    vcvt.s32.f32 s1, s0
    vmov r2, s1
    CHECK 1, r2, 0x00000003
    vcvtr.s32.f32 s1, s0
    vmov r2, s1
    CHECK 2, r2, 0x00000004

    @ Negative, and the unsigned form of a negative saturates to zero with
    @ invalid raised.
    LOADC r0, 0xc0600000
    vmov s0, r0
    vcvt.s32.f32 s1, s0
    vmov r2, s1
    CHECK 3, r2, 0xfffffffd
    movs r3, #0
    vmsr fpscr, r3
    vcvt.u32.f32 s1, s0
    vmov r2, s1
    CHECK 4, r2, 0x00000000
    vmrs r2, fpscr
    LOADC r3, 0x0000001f
    and r2, r2, r3
    CHECK 5, r2, 0x00000001

    @ Out of range saturates to the end of the range, and a NaN gives zero.
    movs r3, #0
    vmsr fpscr, r3
    LOADC r0, 0x7f800000
    vmov s0, r0
    vcvt.s32.f32 s1, s0
    vmov r2, s1
    CHECK 6, r2, 0x7fffffff
    LOADC r0, 0xff800000
    vmov s0, r0
    vcvt.s32.f32 s1, s0
    vmov r2, s1
    CHECK 7, r2, 0x80000000
    LOADC r0, 0x7fc00000
    vmov s0, r0
    vcvt.s32.f32 s1, s0
    vmov r2, s1
    CHECK 8, r2, 0x00000000

    @ Integer to float, signed and unsigned, and the case where the two
    @ differ: 0xffffffff is -1 signed and 2^32-1 unsigned.
    movs r3, #0
    vmsr fpscr, r3
    LOADC r0, 0xffffffff
    vmov s0, r0
    vcvt.f32.s32 s1, s0
    vmov r2, s1
    CHECK 9, r2, 0xbf800000
    vcvt.f32.u32 s1, s0
    vmov r2, s1
    CHECK 10, r2, 0x4f800000

    @ Fixed point: 1.0 with sixteen fraction bits is 0x00010000, and back.
    LOADC r0, 0x3f800000
    vmov s0, r0
    vcvt.s32.f32 s0, s0, #16
    vmov r2, s0
    CHECK 11, r2, 0x00010000
    vcvt.f32.s32 s0, s0, #16
    vmov r2, s0
    CHECK 12, r2, 0x3f800000

    @ Half precision, both halves of the register. 1.0 is 0x3c00 as a half.
    LOADC r0, 0x3f800000
    vmov s0, r0
    LOADC r1, 0xdeadbeef
    vmov s1, r1
    vcvtb.f16.f32 s1, s0
    vmov r2, s1
    CHECK 13, r2, 0xdead3c00
    vmov s1, r1
    vcvtt.f16.f32 s1, s0
    vmov r2, s1
    CHECK 14, r2, 0x3c00beef

    @ And back out of each half.
    LOADC r0, 0x11113c00
    vmov s0, r0
    vcvtb.f32.f16 s1, s0
    vmov r2, s1
    CHECK 15, r2, 0x3f800000
    LOADC r0, 0x3c001111
    vmov s0, r0
    vcvtt.f32.f16 s1, s0
    vmov r2, s1
    CHECK 16, r2, 0x3f800000

    @ A subnormal half is a normal single, which is why flush-to-zero must
    @ not reach a half-precision operand.
    LOADC r3, 0x01000000
    vmsr fpscr, r3
    LOADC r0, 0x00000001
    vmov s0, r0
    vcvtb.f32.f16 s1, s0
    vmov r2, s1
    CHECK 17, r2, 0x33800000

    @ VMOV with an immediate, and the two-register core transfer.
    movs r3, #0
    vmsr fpscr, r3
    vmov.f32 s0, #1.0
    vmov r2, s0
    CHECK 18, r2, 0x3f800000
    LOADC r0, 0x11112222
    LOADC r1, 0x33334444
    vmov s4, s5, r0, r1
    vmov r2, s4
    CHECK 19, r2, 0x11112222
    vmov r2, s5
    CHECK 20, r2, 0x33334444
    vmov r4, r5, s4, s5
    CHECK 21, r4, 0x11112222
    CHECK 22, r5, 0x33334444

    b pass
"#;

/// The extension-register loads and stores.
#[cfg(feature = "cpu-arm-v7m-fp")]
pub(super) const FP_MEMORY: &str = r#"
    @ VSTR then VLDR, with a positive and a negative offset.
    LOADC r0, 0x20000200
    LOADC r1, 0x12345678
    vmov s0, r1
    vstr s0, [r0]
    ldr r2, [r0]
    CHECK 1, r2, 0x12345678
    vstr s0, [r0, #8]
    ldr r2, [r0, #8]
    CHECK 2, r2, 0x12345678
    LOADC r3, 0x9abcdef0
    str r3, [r0, #-4]
    vldr s1, [r0, #-4]
    vmov r2, s1
    CHECK 3, r2, 0x9abcdef0

    @ VSTMIA with writeback, then VLDMDB back to where it started.
    LOADC r0, 0x20000300
    LOADC r1, 0x00000011
    vmov s0, r1
    LOADC r1, 0x00000022
    vmov s1, r1
    LOADC r1, 0x00000033
    vmov s2, r1
    vstmia r0!, {s0-s2}
    CHECK 4, r0, 0x2000030c
    ldr r2, [r0, #-12]
    CHECK 5, r2, 0x00000011
    ldr r2, [r0, #-4]
    CHECK 6, r2, 0x00000033
    vldmdb r0!, {s8-s10}
    CHECK 7, r0, 0x20000300
    vmov r2, s8
    CHECK 8, r2, 0x00000011
    vmov r2, s10
    CHECK 9, r2, 0x00000033

    @ VPUSH and VPOP are the SP spelling of the same two encodings.
    LOADC r1, 0x0000abcd
    vmov s0, r1
    LOADC r1, 0x0000ef01
    vmov s15, r1
    mov r4, sp
    vpush {s0-s15}
    mov r5, sp
    subs r5, r4, r5
    CHECK 10, r5, 0x00000040
    ldr r2, [sp]
    CHECK 11, r2, 0x0000abcd
    ldr r2, [sp, #60]
    CHECK 12, r2, 0x0000ef01
    movs r1, #0
    vmov s0, r1
    vmov s15, r1
    vpop {s0-s15}
    CHECK 13, sp, 0x20010000
    vmov r2, s0
    CHECK 14, r2, 0x0000abcd
    vmov r2, s15
    CHECK 15, r2, 0x0000ef01

    @ A literal VLDR, which is how a compiler gets a constant into an S
    @ register without a core register.
    vldr s2, one_point_five
    vmov r2, s2
    CHECK 16, r2, 0x3fc00000

    b pass_fp_memory

    .align 2
one_point_five:
    .word 0x3fc00000

    .thumb_func
pass_fp_memory:
    b pass
"#;

/// `CONTROL.FPCA`, the extended frame, and lazy state preservation.
///
/// The two handlers record what they saw in `R5`, `R6` and `R7`: those are
/// not in the exception frame, so they survive the return, and they are low
/// registers, so every `CHECK` against them assembles narrow.
#[cfg(feature = "cpu-arm-v7m-fp")]
pub(super) const FP_EXCEPTIONS: &str = r#"
    b fp_exc_main

    @ Records the frame pointer in r5, EXC_RETURN in r6 and FPCCR in r7,
    @ and touches nothing else -- in particular, no floating-point register,
    @ so a lazy reservation is still outstanding when it returns.
    .thumb_func
fp_handler:
    mov r5, sp
    mov r6, lr
    LOADC r7, 0xe000ef34
    ldr r7, [r7]
    bx lr

    @ The same, but its first act is a floating-point instruction, which is
    @ what has to resolve the reservation before it clobbers S0.
    .thumb_func
fp_handler_uses_fp:
    movs r3, #0
    vmov s0, r3
    mov r5, sp
    mov r6, lr
    bx lr

    .thumb_func
fp_exc_main:
    @ CONTROL.FPCA is clear until a floating-point instruction sets it.
    mrs r2, control
    movs r3, #4
    ands r2, r2, r3
    CHECK 1, r2, 0x00000000
    LOADC r0, 0xcafe0001
    vmov s0, r0
    mrs r2, control
    movs r3, #4
    ands r2, r2, r3
    CHECK 2, r2, 0x00000004

    @ Install the recording handler, and fill the frame area with a sentinel
    @ so "the core did not write here" is something the test can see.
    LOADC r0, HANDLER_PTR
    adr r1, fp_handler
    orr r1, r1, #1
    str r1, [r0]

    mov r4, sp
    sub r2, r4, #0x68
    movs r3, #0
    LOADC r1, 0x5a5a5a5a
fill_frame:
    str r1, [r2, r3]
    adds r3, r3, #4
    cmp r3, #0x68
    blt fill_frame

    svc #0

    @ The frame was the extended one -- twenty-six words -- and EXC_RETURN
    @ had bit 4 clear to say so.
    subs r2, r4, r5
    CHECK 3, r2, 0x00000068
    movs r3, #0x10
    ands r2, r6, r3
    CHECK 4, r2, 0x00000000

    @ Lazy stacking left the S-register slots untouched...
    ldr r2, [r5, #0x20]
    CHECK 5, r2, 0x5a5a5a5a
    @ ...with LSPACT set while the handler ran...
    movs r3, #1
    ands r2, r7, r3
    CHECK 6, r2, 0x00000001
    @ ...and FPCAR pointing at the slot S0 would have gone in. The check is
    @ written out rather than using CHECK, which compares against a constant.
    LOADC r0, 0xe000ef38
    ldr r2, [r0]
    adds r3, r5, #0x20
    cmp r2, r3
    beq 8000f
    movs r0, #7
    b fail
8000:

    @ Nothing in the handler used the FPU, so S0 came back untouched, the
    @ reservation was discarded, and the whole frame was popped.
    vmov r2, s0
    CHECK 8, r2, 0xcafe0001
    LOADC r0, 0xe000ef34
    ldr r2, [r0]
    movs r3, #1
    ands r2, r2, r3
    CHECK 9, r2, 0x00000000
    mov r2, sp
    CHECK 10, r2, 0x20010000

    @ Now a handler that does use the FPU: the deferred push has to happen
    @ before it clobbers S0, so the interrupted value reaches the frame and
    @ comes back on return.
    LOADC r0, HANDLER_PTR
    adr r1, fp_handler_uses_fp
    orr r1, r1, #1
    str r1, [r0]
    LOADC r0, 0xcafe0002
    vmov s0, r0
    svc #0
    vmov r2, s0
    CHECK 11, r2, 0xcafe0002

    @ With FPCCR.ASPEN clear a floating-point instruction no longer claims a
    @ context, so the next exception gets the basic eight-word frame.
    LOADC r0, 0xe000ef34
    ldr r1, [r0]
    LOADC r2, 0x80000000
    bics r1, r1, r2
    str r1, [r0]
    mrs r1, control
    movs r2, #4
    bics r1, r1, r2
    msr control, r1
    isb
    LOADC r0, HANDLER_PTR
    adr r1, fp_handler
    orr r1, r1, #1
    str r1, [r0]
    LOADC r0, 0x0000beef
    vmov s0, r0
    mov r4, sp
    svc #0
    subs r2, r4, r5
    CHECK 12, r2, 0x00000020
    movs r3, #0x10
    ands r2, r6, r3
    CHECK 13, r2, 0x00000010

    b pass
"#;

/// Armv8-M Mainline on a Cortex-M33: load-acquire/store-release, `TT` against
/// a PMSAv8 region, the v8-M `EXC_RETURN` values, a stack-limit violation
/// caught and stepped over, and every condition of the wide conditional
/// branch over a gap wide enough that `imm6` is non-zero in both directions.
///
/// Expected values are DDI 0553's (`TT_RESP`, `EXC_RETURN`, `UFSR.STKOF`)
/// as `super::v8m` states them.
#[cfg(feature = "cpu-arm-v8m")]
pub(super) const V8M_MAINLINE: &str = r#"
    .cpu cortex-m33
    b v8_main

    .thumb_func
lr_handler:
    mov r5, lr
    bx lr

    @ Record CFSR, clear it, and resume past the sixteen-bit instruction
    @ that overflowed.
    .thumb_func
stkof_handler:
    LOADC r4, 0xe000ed28
    ldr r6, [r4]
    str r6, [r4]
    ldr r4, [sp, #24]
    adds r4, r4, #2
    str r4, [sp, #24]
    bx lr

    .thumb_func
v8_main:
    @ Load-acquire and store-release, every size.
    LOADC r1, 0x20000100
    LOADC r0, 0x8badf00d
    stl r0, [r1]
    lda r2, [r1]
    CHECK 1, r2, 0x8badf00d
    stlb r0, [r1]
    ldab r2, [r1]
    CHECK 2, r2, 0x0000000d
    stlh r0, [r1]
    ldah r2, [r1]
    CHECK 3, r2, 0x0000f00d

    @ The exclusive forms: one store through, the next refused.
    ldaex r2, [r1]
    stlex r3, r0, [r1]
    CHECK 4, r3, 0
    stlex r3, r0, [r1]
    CHECK 5, r3, 1
    ldaexb r2, [r1]
    CHECK 6, r2, 0x0000000d
    stlexb r3, r2, [r1]
    CHECK 7, r3, 0

    @ TT with the MPU off: no region, readable and writable.
    tt r2, r1
    CHECK 8, r2, 0x000c0000
    sg
    csdb
    ssbb
    pssbb

    @ A PMSAv8 region 3 over 0x20000100-0x200001FF, read/write for anyone;
    @ TT now names it.
    LOADC r4, 0xe000ed98
    movs r0, #3
    str r0, [r4]
    LOADC r0, 0x20000102
    str r0, [r4, #4]
    LOADC r0, 0x200001e1
    str r0, [r4, #8]
    LOADC r4, 0xe000ed94
    movs r0, #5
    str r0, [r4]
    dsb
    isb
    tt r2, r1
    CHECK 9, r2, 0x000d0003
    movs r0, #0
    str r0, [r4]
    dsb
    isb

    @ EXC_RETURN in the Armv8-M format: Thread/PSP, then Thread/MSP.
    LOADC r0, HANDLER_PTR
    adr r1, lr_handler
    orr r1, r1, #1
    str r1, [r0]
    LOADC r0, 0x20008000
    msr psp, r0
    movs r1, #2
    msr control, r1
    isb
    movs r5, #0
    svc #0
    movs r1, #0
    msr control, r1
    isb
    CHECK 10, r5, 0xffffffbc
    movs r5, #0
    svc #0
    CHECK 11, r5, 0xffffffb8

    @ MSPLIM: a MOV to SP below it is a STKOF UsageFault and SP stays put.
    LOADC r0, 0x2000f000
    msr msplim, r0
    mrs r2, msplim
    CHECK 12, r2, 0x2000f000
    LOADC r0, 0xe000ed24
    ldr r1, [r0]
    orr r1, r1, #0x40000
    str r1, [r0]
    LOADC r0, HANDLER_PTR
    adr r1, stkof_handler
    orr r1, r1, #1
    str r1, [r0]
    movs r6, #0
    LOADC r0, 0x2000e000
    mov sp, r0
    mov r2, sp
    CHECK 13, r2, 0x20010000
    CHECK 14, r6, 0x00100000
    movs r0, #0
    msr msplim, r0

    @ B<cond>.W, every condition, forward over a 4 KiB gap and back again,
    @ so imm6 is non-zero both ways. First N=1 Z=0 C=1 V=0.
    LOADC r0, 0xa0000000
    msr APSR_nzcvq, r0
    bne.w f1
b1: bcs.w f2
b2: bmi.w f3
b3: bvc.w f4
b4: bhi.w f5
b5: blt.w f6
b6: ble.w f7
b7: beq.w cond_fail
    bcc.w cond_fail
    bpl.w cond_fail
    bvs.w cond_fail
    bls.w cond_fail
    bge.w cond_fail
    bgt.w cond_fail
    @ N=0 Z=1 C=0 V=1.
    LOADC r0, 0x50000000
    msr APSR_nzcvq, r0
    beq.w f8
b8: bcc.w f9
b9: bpl.w f10
b10: bvs.w f11
b11: bls.w f12
b12: bne.w cond_fail
    bhi.w cond_fail
    bge.w cond_fail
    @ All clear: GE and GT.
    LOADC r0, 0x00000000
    msr APSR_nzcvq, r0
    bge.w f13
b13: bgt.w f14
b14: blt.w cond_fail
    ble.w cond_fail
    b.w cond_done

    .space 4200

f1: bne.w b1
f2: bcs.w b2
f3: bmi.w b3
f4: bvc.w b4
f5: bhi.w b5
f6: blt.w b6
f7: ble.w b7
f8: beq.w b8
f9: bcc.w b9
f10: bpl.w b10
f11: bvs.w b11
f12: bls.w b12
f13: bge.w b13
f14: bgt.w b14

cond_fail:
    movs r0, #20
    b fail

cond_done:
    b pass
"#;

// ---------------------------------------------------------------------------
// ARMv6-M
//
// These bodies are assembled with `--target=thumbv6m-none-eabi` against
// `super::conformance::V6M_PROLOGUE`, whose macros differ from the v7-M ones
// (low registers only for `LOADC`; `R6`/`R7` are the scratch pair). Every
// expected value is DDI 0419E's: the A6.7 operation pseudocode for an
// instruction, B1.5 for the exception model, B3 for the system control
// space. No emulator source of any licence was consulted (`ROADMAP.md` §1).
// ---------------------------------------------------------------------------

/// The whole ARMv6-M sixteen-bit data-processing set with its flag results
/// (DDI 0419E A5.2.1-A5.2.4, A6.7): what each writes, and — as often the
/// point — what it leaves alone.
#[cfg(feature = "cpu-arm-v6m")]
pub(super) const V6M_ALU: &str = r#"
    @ MSR APSR writes N, Z, C and V; there is no Q to write (B4.2.3), so
    @ bit 27 reads zero whatever was written to it.
    SETF 0xf8000000
    CHECKF 1, 0xf0000000

    @ MOVS #imm8 sets N and Z, keeps C and V (A6.7.39).
    SETF 0x30000000
    movs r0, #0
    CHECKF 2, 0x70000000
    SETF 0x30000000
    movs r0, #0x80
    CHECKF 3, 0x30000000
    CHECK 4, r0, 0x00000080

    @ MOVS Rd, Rm is LSLS #0: N and Z from the value, C kept (A6.7.40).
    LOADC r1, 0x80000000
    SETF 0x20000000
    movs r2, r1
    CHECKF 5, 0xa0000000
    CHECK 6, r2, 0x80000000

    @ MOV with a high register sets no flags at all.
    SETF 0x00000000
    mov r8, r1
    mov r3, r8
    CHECKF 7, 0x00000000
    CHECK 8, r3, 0x80000000
    CHECK 9, r8, 0x80000000

    @ ADDS, all three sixteen-bit forms (A6.7.2, A6.7.3).
    LOADC r1, 0x7fffffff
    adds r2, r1, #1
    CHECKF 10, 0x90000000
    CHECK 11, r2, 0x80000000
    movs r2, #0xff
    adds r2, #1
    CHECKF 12, 0x00000000
    CHECK 13, r2, 0x00000100
    LOADC r1, 0xffffffff
    movs r2, #1
    adds r3, r1, r2
    CHECKF 14, 0x60000000
    CHECK 15, r3, 0x00000000

    @ ADD Rdn, Rm with a high register: no flags.
    movs r1, #0x10
    mov r9, r1
    movs r2, #0x22
    SETF 0xf0000000
    add r2, r9
    CHECKF 16, 0xf0000000
    CHECK 17, r2, 0x00000032
    add r9, r2
    CHECK 18, r9, 0x00000042

    @ The SP forms: SUB SP, ADD Rd, SP, #imm, ADD SP, #imm, ADD Rd, SP, Rd
    @ and ADD SP, Rm (A6.7.5, A6.7.6, A6.7.66).
    sub sp, #16
    mov r5, sp
    CHECK 19, r5, 0x2000fff0
    add r1, sp, #8
    CHECK 20, r1, 0x2000fff8
    movs r1, #4
    add r1, sp, r1
    CHECK 21, r1, 0x2000fff4
    add sp, #16
    mov r5, sp
    CHECK 22, r5, 0x20010000
    movs r1, #0
    subs r1, #8
    add sp, r1
    mov r5, sp
    CHECK 23, r5, 0x2000fff8
    add sp, #8

    @ ADR is forward-only and word-aligned on ARMv6-M (A6.7.7).
    adr r1, alu_word
    ldr r2, [r1]
    CHECK 24, r2, 0x0badf00d
    b alu_past
    .align 2
alu_word:
    .word 0x0badf00d
alu_past:

    @ SUBS, all three forms, with the borrow and overflow cases.
    movs r1, #5
    subs r2, r1, #7
    CHECKF 25, 0x80000000
    CHECK 26, r2, 0xfffffffe
    movs r2, #200
    subs r2, #200
    CHECKF 27, 0x60000000
    LOADC r1, 0x80000000
    movs r2, #1
    subs r3, r1, r2
    CHECKF 28, 0x30000000
    CHECK 29, r3, 0x7fffffff

    @ RSBS Rd, Rn, #0 (NEG): zero has no borrow; the most negative number
    @ overflows (A6.7.55).
    movs r1, #0
    rsbs r2, r1, #0
    CHECKF 30, 0x60000000
    movs r1, #1
    rsbs r2, r1, #0
    CHECKF 31, 0x80000000
    CHECK 32, r2, 0xffffffff
    LOADC r1, 0x80000000
    rsbs r2, r1, #0
    CHECKF 33, 0x90000000
    CHECK 34, r2, 0x80000000

    @ ADCS and SBCS take the carry in.
    SETF 0x20000000
    movs r1, #1
    movs r2, #2
    adcs r1, r2
    CHECKF 35, 0x00000000
    CHECK 36, r1, 0x00000004
    LOADC r1, 0xffffffff
    SETF 0x20000000
    movs r2, #0
    adcs r1, r2
    CHECKF 37, 0x60000000
    CHECK 38, r1, 0x00000000
    movs r1, #5
    movs r2, #3
    SETF 0x00000000
    sbcs r1, r2
    CHECKF 39, 0x20000000
    CHECK 40, r1, 0x00000001
    movs r1, #3
    movs r2, #5
    SETF 0x20000000
    sbcs r1, r2
    CHECKF 41, 0x80000000
    CHECK 42, r1, 0xfffffffe
    LOADC r1, 0x80000000
    movs r2, #0
    SETF 0x00000000
    sbcs r1, r2
    CHECKF 43, 0x30000000
    CHECK 44, r1, 0x7fffffff

    @ CMP: immediate, low registers, and a high register; CMN.
    movs r1, #5
    cmp r1, #5
    CHECKF 45, 0x60000000
    movs r2, #6
    cmp r1, r2
    CHECKF 46, 0x80000000
    mov r8, r2
    cmp r8, r1
    CHECKF 47, 0x20000000
    movs r1, #1
    LOADC r2, 0xffffffff
    cmn r1, r2
    CHECKF 48, 0x60000000
    LOADC r1, 0x7fffffff
    movs r2, #1
    cmn r1, r2
    CHECKF 49, 0x90000000

    @ The logical operations set N and Z and keep C and V.
    LOADC r1, 0xf0f0f0f0
    LOADC r2, 0xff00ff00
    mov r3, r1
    SETF 0x30000000
    ands r3, r2
    CHECKF 50, 0xb0000000
    CHECK 51, r3, 0xf000f000
    SETF 0x30000000
    mov r3, r1
    eors r3, r2
    CHECKF 52, 0x30000000
    CHECK 53, r3, 0x0ff00ff0
    SETF 0x30000000
    mov r3, r1
    orrs r3, r2
    CHECKF 54, 0xb0000000
    CHECK 55, r3, 0xfff0fff0
    SETF 0x30000000
    mov r3, r1
    bics r3, r2
    CHECKF 56, 0x30000000
    CHECK 57, r3, 0x00f000f0
    SETF 0x30000000
    mvns r3, r1
    CHECKF 58, 0x30000000
    CHECK 59, r3, 0x0f0f0f0f
    movs r3, #0
    SETF 0x00000000
    tst r3, r1
    CHECKF 60, 0x40000000
    SETF 0x00000000
    tst r1, r2
    CHECKF 61, 0x80000000

    @ MULS sets N and Z and leaves C and V alone (A6.7.44).
    LOADC r1, 0x00010000
    mov r2, r1
    SETF 0x30000000
    muls r2, r1, r2
    CHECKF 62, 0x70000000
    CHECK 63, r2, 0x00000000
    movs r1, #3
    LOADC r2, 0xffffffff
    SETF 0x30000000
    muls r2, r1, r2
    CHECKF 64, 0xb0000000
    CHECK 65, r2, 0xfffffffd

    @ Shifts by an immediate, with the amounts the encoding treats
    @ specially: LSR #32 and ASR #32 are what an immediate of 0 means
    @ (A6.7.35, A6.7.37, A6.7.10). V is never touched.
    LOADC r1, 0x80000001
    SETF 0x10000000
    lsls r2, r1, #1
    CHECKF 66, 0x30000000
    CHECK 67, r2, 0x00000002
    LOADC r1, 0x80000000
    SETF 0x00000000
    lsrs r2, r1, #32
    CHECKF 68, 0x60000000
    CHECK 69, r2, 0x00000000
    SETF 0x00000000
    asrs r2, r1, #32
    CHECKF 70, 0xa0000000
    CHECK 71, r2, 0xffffffff
    SETF 0x00000000
    asrs r2, r1, #4
    CHECKF 72, 0x80000000
    CHECK 73, r2, 0xf8000000
    movs r1, #1
    SETF 0x00000000
    lsrs r2, r1, #1
    CHECKF 74, 0x60000000

    @ Shifts by a register use its bottom byte; zero leaves the value and C
    @ alone, 32 shifts the last bit into C, more than 32 clears it
    @ (A6.7.36, A6.7.38, A6.7.11, A6.7.54).
    movs r1, #1
    movs r3, #0
    SETF 0x20000000
    lsls r1, r3
    CHECKF 75, 0x20000000
    CHECK 76, r1, 0x00000001
    SETF 0x00000000
    movs r3, #32
    lsls r1, r3
    CHECKF 77, 0x60000000
    movs r1, #1
    SETF 0x00000000
    movs r3, #33
    lsls r1, r3
    CHECKF 78, 0x40000000
    movs r1, #1
    LOADC r3, 0x00000101
    lsls r1, r3
    CHECK 79, r1, 0x00000002
    LOADC r1, 0x80000000
    SETF 0x00000000
    movs r3, #32
    lsrs r1, r3
    CHECKF 80, 0x60000000
    LOADC r1, 0x80000000
    SETF 0x00000000
    movs r3, #40
    lsrs r1, r3
    CHECKF 81, 0x40000000
    LOADC r1, 0x80000000
    SETF 0x00000000
    movs r3, #40
    asrs r1, r3
    CHECKF 82, 0xa0000000
    CHECK 83, r1, 0xffffffff
    LOADC r1, 0x80000001
    SETF 0x00000000
    movs r3, #1
    rors r1, r3
    CHECKF 84, 0xa0000000
    CHECK 85, r1, 0xc0000000
    LOADC r1, 0x80000001
    movs r3, #32
    SETF 0x00000000
    rors r1, r3
    CHECKF 86, 0xa0000000
    CHECK 87, r1, 0x80000001
    movs r3, #0
    SETF 0x00000000
    rors r1, r3
    CHECKF 88, 0x80000000

    @ The extends and byte reversals: no flags (A6.7.69-75, A6.7.51-53).
    LOADC r1, 0x1234f680
    SETF 0x00000000
    sxtb r2, r1
    CHECK 89, r2, 0xffffff80
    sxth r2, r1
    CHECK 90, r2, 0xfffff680
    uxtb r2, r1
    CHECK 91, r2, 0x00000080
    uxth r2, r1
    CHECK 92, r2, 0x0000f680
    LOADC r1, 0x11223344
    rev r2, r1
    CHECK 93, r2, 0x44332211
    rev16 r2, r1
    CHECK 94, r2, 0x22114433
    LOADC r1, 0x000012f0
    revsh r2, r1
    CHECK 95, r2, 0xfffff012
"#;

/// Every ARMv6-M load and store addressing form (DDI 0419E A5.2.4, A6.7):
/// immediate, register and SP-relative offsets at every size, the literal
/// form, `LDM`/`STM` with their write-back rules, and `PUSH`/`POP`.
#[cfg(feature = "cpu-arm-v6m")]
pub(super) const V6M_MEMORY: &str = r#"
    LOADC r0, 0x20000100
    LOADC r1, 0x8badf00d

    @ Word, immediate offset: zero and the largest (imm5 * 4 = 124).
    str r1, [r0]
    str r1, [r0, #124]
    ldr r2, [r0, #124]
    CHECK 1, r2, 0x8badf00d

    @ Halfword and byte, immediate offset; little-endian.
    strh r1, [r0, #8]
    ldrh r2, [r0, #8]
    CHECK 2, r2, 0x0000f00d
    ldrb r2, [r0, #9]
    CHECK 3, r2, 0x000000f0
    strb r1, [r0, #31]
    ldrb r2, [r0, #31]
    CHECK 4, r2, 0x0000000d

    @ Register offsets, including the sign-extending loads that have no
    @ immediate form (A6.7.31, A6.7.32).
    movs r3, #8
    ldrsh r2, [r0, r3]
    CHECK 5, r2, 0xfffff00d
    movs r3, #9
    ldrsb r2, [r0, r3]
    CHECK 6, r2, 0xfffffff0
    movs r3, #31
    ldrsb r2, [r0, r3]
    CHECK 7, r2, 0x0000000d
    movs r3, #64
    str r1, [r0, r3]
    ldr r2, [r0, r3]
    CHECK 8, r2, 0x8badf00d
    movs r3, #68
    strh r1, [r0, r3]
    ldrh r2, [r0, r3]
    CHECK 9, r2, 0x0000f00d
    movs r3, #72
    strb r1, [r0, r3]
    ldrb r2, [r0, r3]
    CHECK 10, r2, 0x0000000d
    ldr r2, [r0, #72]
    CHECK 11, r2, 0x0000000d

    @ SP-relative.
    sub sp, #8
    str r1, [sp, #4]
    ldr r2, [sp, #4]
    CHECK 12, r2, 0x8badf00d
    add sp, #8

    @ The literal form, aligned down from PC (A6.7.27).
    ldr r2, mem_lit
    CHECK 13, r2, 0xcafef00d
    b mem_past
    .align 2
mem_lit:
    .word 0xcafef00d
mem_past:

    @ STMIA with write-back: lowest register at the lowest address.
    movs r1, #1
    movs r2, #2
    movs r3, #3
    mov r4, r0
    stmia r4!, {r1-r3}
    CHECK 14, r4, 0x2000010c
    ldr r5, [r0, #8]
    CHECK 15, r5, 0x00000003
    movs r1, #0
    movs r2, #0
    movs r3, #0
    mov r4, r0
    ldmia r4!, {r1-r3}
    CHECK 16, r4, 0x2000010c
    CHECK 17, r1, 0x00000001
    CHECK 18, r3, 0x00000003

    @ LDM with the base in the list does not write back: the loaded value
    @ wins (A6.7.25).
    mov r1, r0
    ldm r1, {r1, r2}
    CHECK 19, r1, 0x00000001
    CHECK 20, r2, 0x00000002

    @ STMIA with the base the lowest register stores its original value
    @ (A6.7.58).
    mov r1, r0
    movs r2, #0x55
    stmia r1!, {r1, r2}
    CHECK 21, r1, 0x20000108
    ldr r3, [r0]
    CHECK 22, r3, 0x20000100
    ldr r3, [r0, #4]
    CHECK 23, r3, 0x00000055

    @ PUSH and POP, with LR: full descending, lowest register lowest.
    movs r1, #0x11
    movs r2, #0x22
    LOADC r3, 0x33333333
    mov lr, r3
    push {r1, r2, lr}
    mov r5, sp
    CHECK 24, r5, 0x2000fff4
    ldr r5, [sp]
    CHECK 25, r5, 0x00000011
    ldr r5, [sp, #8]
    CHECK 26, r5, 0x33333333
    movs r1, #0
    movs r2, #0
    pop {r1, r2}
    pop {r4}
    CHECK 27, r1, 0x00000011
    CHECK 28, r2, 0x00000022
    CHECK 29, r4, 0x33333333
    mov r5, sp
    CHECK 30, r5, 0x20010000
"#;

/// Every branch ARMv6-M has: `B<cond>` under every condition, taken and
/// not, `B`, `BL`, `BX`, `BLX`, `MOV PC` and `POP {PC}` (DDI 0419E A6.7).
#[cfg(feature = "cpu-arm-v6m")]
pub(super) const V6M_BRANCH: &str = r#"
    .macro TAKEN cond, n
    b\cond 8000f
    movs r0, #\n
    bl fail
8000:
    .endm
    .macro NOTTAKEN cond, n
    b\cond 8001f
    b 8002f
8001:
    movs r0, #\n
    bl fail
8002:
    .endm

    @ N=1 Z=0 C=1 V=0.
    SETF 0xa0000000
    TAKEN ne, 1
    TAKEN cs, 2
    TAKEN mi, 3
    TAKEN vc, 4
    TAKEN hi, 5
    TAKEN lt, 6
    TAKEN le, 7
    NOTTAKEN eq, 8
    NOTTAKEN cc, 9
    NOTTAKEN pl, 10
    NOTTAKEN vs, 11
    NOTTAKEN ls, 12
    NOTTAKEN ge, 13
    NOTTAKEN gt, 14
    @ N=0 Z=1 C=0 V=1.
    SETF 0x50000000
    TAKEN eq, 15
    TAKEN cc, 16
    TAKEN pl, 17
    TAKEN vs, 18
    TAKEN ls, 19
    TAKEN lt, 20
    TAKEN le, 21
    NOTTAKEN ne, 22
    NOTTAKEN cs, 23
    NOTTAKEN mi, 24
    NOTTAKEN vc, 25
    NOTTAKEN hi, 26
    NOTTAKEN ge, 27
    NOTTAKEN gt, 28
    @ N=1 V=1, Z=0: GE and GT hold.
    SETF 0x90000000
    TAKEN ge, 29
    TAKEN gt, 30
    NOTTAKEN lt, 31
    NOTTAKEN le, 32

    @ BL sets LR to the next instruction with bit 0 set; BX LR returns.
    movs r4, #0
    bl br_sub
    CHECK 33, r4, 0x00000001
    @ ...and BLX Rm does the same through a register.
    movs r4, #0
    ADDR r2, br_sub
    blx r2
br_after_blx:
    CHECK 34, r4, 0x00000001
    mov r3, lr
    ADDR r2, br_after_blx
    CHECKR 35, r3, r2

    @ BX to an odd address stays in Thumb state.
    ADDR r2, br_bx_target
    bx r2
    movs r0, #36
    bl fail
    .align 2
br_bx_target:

    @ MOV PC, Rm is a branch.
    ADDR r2, br_mov_target
    mov pc, r2
    movs r0, #37
    bl fail
    .align 2
br_mov_target:

    @ POP {PC} returns, interworking on bit 0.
    movs r5, #0
    bl br_popper
    CHECK 38, r5, 0x00000005

    @ An unconditional B backwards.
    movs r5, #0
    b br_fwd
br_back:
    adds r5, #1
    b br_done
br_fwd:
    adds r5, #2
    b br_back
br_done:
    CHECK 39, r5, 0x00000003
    b pass

    .align 2
    .thumb_func
br_sub:
    mov r5, lr
    movs r4, #1
    ands r4, r5
    bx lr

    .thumb_func
br_popper:
    push {r4, lr}
    movs r5, #5
    pop {r4, pc}
"#;

/// The ARMv6-M system registers and system instructions on a Cortex-M0
/// (DDI 0419E B3.2, B3.4, B4.2): `MRS`/`MSR` over every special register,
/// `CPS`, the barriers and hints, `CPUID`, the fixed `CCR`, `AIRCR`, the
/// two-bit priority probe CMSIS does, the 32-interrupt NVIC, and the
/// registers a Cortex-M0 does not have.
#[cfg(feature = "cpu-arm-v6m")]
pub(super) const V6M_SYSTEM: &str = r#"
    @ Thread mode: IPSR is zero, EPSR reads as zero through MRS (B4.2.2),
    @ and xPSR is therefore just the flags.
    mrs r1, ipsr
    CHECK 1, r1, 0x00000000
    mrs r1, epsr
    CHECK 2, r1, 0x00000000
    SETF 0x90000000
    mrs r1, xpsr
    CHECK 3, r1, 0x90000000
    SETF 0x90000000
    mrs r1, iapsr
    CHECK 4, r1, 0x90000000

    @ PSP and MSP are word-aligned: bits [1:0] read zero (B1.4.2).
    LOADC r1, 0x20008007
    msr psp, r1
    mrs r2, psp
    CHECK 5, r2, 0x20008004
    mrs r2, msp
    CHECK 6, r2, 0x20010000

    @ CONTROL.SPSEL moves Thread mode onto the process stack; a Cortex-M0
    @ has no nPRIV, so bit 0 does not stick (B1.4.4).
    LOADC r1, 0x20008000
    msr psp, r1
    movs r1, #3
    msr control, r1
    isb
    mrs r2, control
    CHECK 7, r2, 0x00000002
    mov r2, sp
    CHECK 8, r2, 0x20008000
    movs r1, #0
    msr control, r1
    isb
    mov r2, sp
    CHECK 9, r2, 0x20010000

    @ PRIMASK is one bit, reached by MSR and by CPS (B4.2.1).
    movs r1, #0xff
    msr primask, r1
    mrs r2, primask
    CHECK 10, r2, 0x00000001
    cpsie i
    mrs r2, primask
    CHECK 11, r2, 0x00000000
    cpsid i
    mrs r2, primask
    CHECK 12, r2, 0x00000001
    cpsie i

    @ The barriers and the hints all execute; SEV sets the event register so
    @ that the WFE after it returns at once (B1.6.1).
    dmb
    dsb
    isb
    nop
    yield
    sev
    wfe

    @ CPUID: a Cortex-M0 r0p0 (Cortex-M0 TRM Table 4-1).
    LOADC r0, 0xe000ed00
    ldr r1, [r0]
    CHECK 13, r1, 0x410cc200

    @ CCR is read-only with STKALIGN and UNALIGN_TRP set (B3.2.8).
    LOADC r0, 0xe000ed14
    ldr r1, [r0]
    CHECK 14, r1, 0x00000208
    movs r1, #0
    str r1, [r0]
    ldr r1, [r0]
    CHECK 15, r1, 0x00000208

    @ AIRCR reads VECTKEYSTAT and nothing else: no PRIGROUP, little-endian
    @ (B3.2.6). A write with the wrong key is ignored.
    LOADC r0, 0xe000ed0c
    ldr r1, [r0]
    CHECK 16, r1, 0xfa050000
    LOADC r1, 0x00000700
    str r1, [r0]
    ldr r1, [r0]
    CHECK 17, r1, 0xfa050000

    @ The priority probe CMSIS does: write all ones, read back what stuck.
    @ Two bits, [7:6] (B1.5.4).
    LOADC r0, 0xe000ed20
    movs r1, #0
    mvns r1, r1
    str r1, [r0]
    ldr r2, [r0]
    CHECK 18, r2, 0xc0c00000
    movs r1, #0
    str r1, [r0]
    LOADC r0, 0xe000ed1c
    movs r1, #0
    mvns r1, r1
    str r1, [r0]
    ldr r2, [r0]
    CHECK 19, r2, 0xc0000000
    movs r1, #0
    str r1, [r0]
    LOADC r0, 0xe000e400
    movs r1, #0
    mvns r1, r1
    str r1, [r0]
    ldr r2, [r0]
    CHECK 20, r2, 0xc0c0c0c0
    LOADC r0, 0xe000e41c
    str r1, [r0]
    ldr r2, [r0]
    CHECK 21, r2, 0xc0c0c0c0
    movs r1, #0
    str r1, [r0]
    LOADC r0, 0xe000e400
    str r1, [r0]
    @ SHPR1 does not exist: there are no configurable faults (B3.2.10).
    LOADC r0, 0xe000ed18
    movs r1, #0
    mvns r1, r1
    str r1, [r0]
    ldr r2, [r0]
    CHECK 22, r2, 0x00000000

    @ The NVIC has one word of each register: 32 interrupts, and no IPR8
    @ (B3.4.2).
    LOADC r0, 0xe000e100
    movs r1, #0
    mvns r1, r1
    str r1, [r0]
    ldr r2, [r0]
    CHECK 23, r2, 0xffffffff
    LOADC r0, 0xe000e180
    str r1, [r0]
    LOADC r0, 0xe000e100
    ldr r2, [r0]
    CHECK 24, r2, 0x00000000
    LOADC r0, 0xe000e104
    str r1, [r0]
    ldr r2, [r0]
    CHECK 25, r2, 0x00000000
    LOADC r0, 0xe000e420
    str r1, [r0]
    ldr r2, [r0]
    CHECK 26, r2, 0x00000000

    @ ICSR: PendSV pending under PRIMASK shows in PENDSVSET and VECTPENDING,
    @ and PENDSVCLR takes it back (B3.2.4).
    cpsid i
    LOADC r0, 0xe000ed04
    LOADC r1, 0x10000000
    str r1, [r0]
    ldr r2, [r0]
    LOADC r3, 0x1001f000
    ands r2, r3
    CHECK 27, r2, 0x1000e000
    LOADC r1, 0x08000000
    str r1, [r0]
    ldr r2, [r0]
    ands r2, r3
    CHECK 28, r2, 0x00000000
    cpsie i

    @ What a Cortex-M0 does not have reads as zero: VTOR (an option the M0
    @ does not take), the MPU, the fault status registers, CPACR.
    LOADC r0, 0xe000ed08
    LOADC r1, 0x20000000
    str r1, [r0]
    ldr r2, [r0]
    CHECK 29, r2, 0x00000000
    LOADC r0, 0xe000ed90
    ldr r2, [r0]
    CHECK 30, r2, 0x00000000
    LOADC r0, 0xe000ed28
    ldr r2, [r0]
    CHECK 31, r2, 0x00000000
    LOADC r0, 0xe000ed2c
    ldr r2, [r0]
    CHECK 32, r2, 0x00000000
    LOADC r0, 0xe000ed88
    ldr r2, [r0]
    CHECK 33, r2, 0x00000000

    @ SysTick: RVR is 24 bits, and COUNTFLAG clears on read (B3.3).
    LOADC r0, 0xe000e014
    movs r1, #0
    mvns r1, r1
    str r1, [r0]
    ldr r2, [r0]
    CHECK 34, r2, 0x00ffffff
    movs r1, #0x40
    str r1, [r0]
    LOADC r0, 0xe000e018
    str r1, [r0]
    LOADC r0, 0xe000e010
    movs r1, #5
    str r1, [r0]
sys_tick_wait:
    ldr r2, [r0]
    lsrs r2, r2, #17
    bcc sys_tick_wait
    ldr r2, [r0]
    CHECK 35, r2, 0x00000005
    movs r1, #0
    str r1, [r0]
"#;

/// The ARMv6-M exception model (DDI 0419E B1.5): the stacked frame and its
/// alignment, the three `EXC_RETURN` values, preemption, tail-chaining,
/// PRIMASK, PendSV, SysTick, and return through `POP {PC}`.
#[cfg(feature = "cpu-arm-v6m")]
pub(super) const V6M_EXCEPTIONS: &str = r#"
    @ Append a low register to the log at 0x20000200; the count is at
    @ 0x200001fc. Clobbers r1, r2 and the flags.
    .macro LOGV reg
    LOADC r1, 0x200001fc
    ldr r2, [r1]
    adds r2, #1
    str r2, [r1]
    lsls r2, r2, #2
    adds r2, r2, r1
    str \reg, [r2]
    .endm
    .equ LOGBASE, 0x20000200

    @ ---- SVC: the frame, from Thread mode on the main stack ----
    INSTALL frame_handler
    LOADC r0, 0x10101010
    LOADC r1, 0x11111111
    LOADC r2, 0x12121212
    LOADC r3, 0x13131313
    LOADC r4, 0x1c1c1c1c
    mov r12, r4
    LOADC r4, 0x1e1e1e1e
    mov lr, r4
    SETF 0x90000000
    svc #0x5a
exc_after_svc:
    @ The frame restored r0-r3, r12, LR and the flags.
    CHECKF 1, 0x90000000
    CHECK 2, r0, 0x10101010
    CHECK 3, r3, 0x13131313
    CHECK 4, r12, 0x1c1c1c1c
    CHECK 5, lr, 0x1e1e1e1e
    LOADC r0, LOGBASE
    ldr r1, [r0, #0]
    CHECK 6, r1, 0x0000000b
    ldr r1, [r0, #4]
    CHECK 7, r1, 0xfffffff9
    ldr r1, [r0, #8]
    CHECK 8, r1, 0x2000ffe0
    ldr r1, [r0, #12]
    CHECK 9, r1, 0x10101010
    ldr r1, [r0, #16]
    CHECK 10, r1, 0x13131313
    ldr r1, [r0, #20]
    CHECK 11, r1, 0x1c1c1c1c
    ldr r1, [r0, #24]
    CHECK 12, r1, 0x1e1e1e1e
    @ The return address is the instruction after the SVC...
    ldr r1, [r0, #28]
    ADDR r2, exc_after_svc
    subs r2, #1
    CHECKR 13, r1, r2
    @ ...the stacked xPSR has T and the flags, bit 9 clear because SP was
    @ already 8-byte aligned...
    ldr r1, [r0, #32]
    CHECK 14, r1, 0x91000000
    @ ...and the immediate is the low byte of the SVC instruction.
    ldr r1, [r0, #36]
    CHECK 15, r1, 0x0000005a

    @ ---- An SP that is 4 mod 8 is realigned and bit 9 records it ----
    @ (B1.5.6: CCR.STKALIGN is always one on ARMv6-M.)
    bl reset_log
    sub sp, #4
    SETF 0x00000000
    svc #0
    mov r3, sp
    CHECK 16, r3, 0x2000fffc
    add sp, #4
    LOADC r0, LOGBASE
    ldr r1, [r0, #8]
    CHECK 17, r1, 0x2000ffd8
    ldr r1, [r0, #32]
    CHECK 18, r1, 0x01000200

    @ ---- From Thread mode on the process stack: 0xFFFFFFFD ----
    bl reset_log
    LOADC r1, 0x20008000
    msr psp, r1
    movs r1, #2
    msr control, r1
    isb
    svc #1
    movs r1, #0
    msr control, r1
    isb
    LOADC r0, LOGBASE
    ldr r1, [r0, #4]
    CHECK 19, r1, 0xfffffffd
    @ The handler runs on MSP, and the frame went on PSP.
    ldr r1, [r0, #8]
    CHECK 20, r1, 0x20010000
    mrs r1, psp
    CHECK 21, r1, 0x20008000

    @ ---- Preemption and tail-chaining ----
    @ SVCall at 0x80, IRQ0 at 0x40, PendSV at 0xc0. The SVC handler pends
    @ IRQ0, which preempts it at once (EXC_RETURN 0xFFFFFFF1, a second frame
    @ below the first), then pends PendSV, which cannot. When the SVC
    @ returns, PendSV is taken straight away on the same frame -- returning
    @ to Thread, 0xFFFFFFF9, at the SVC handler's SP -- before the
    @ instruction after the SVC runs (B1.5.12).
    bl reset_log
    INSTALL chain_handler
    LOADC r0, 0xe000ed1c
    LOADC r1, 0x80000000
    str r1, [r0]
    LOADC r0, 0xe000ed20
    LOADC r1, 0x00c00000
    str r1, [r0]
    LOADC r0, 0xe000e400
    movs r1, #0x40
    str r1, [r0]
    LOADC r0, 0xe000e100
    movs r1, #1
    str r1, [r0]
    svc #2
    movs r0, #0xee
    LOGV r0
    LOADC r0, LOGBASE
    ldr r1, [r0, #0]
    CHECK 22, r1, 0x0000000b
    ldr r1, [r0, #4]
    CHECK 23, r1, 0xfffffff9
    ldr r1, [r0, #8]
    CHECK 24, r1, 0x2000ffe0
    ldr r1, [r0, #12]
    CHECK 25, r1, 0x00000010
    ldr r1, [r0, #16]
    CHECK 26, r1, 0xfffffff1
    ldr r1, [r0, #20]
    CHECK 27, r1, 0x2000ffc0
    ldr r1, [r0, #24]
    CHECK 28, r1, 0x000000ff
    ldr r1, [r0, #28]
    CHECK 29, r1, 0x0000000e
    ldr r1, [r0, #32]
    CHECK 30, r1, 0xfffffff9
    ldr r1, [r0, #36]
    CHECK 31, r1, 0x2000ffe0
    ldr r1, [r0, #40]
    CHECK 32, r1, 0x000000ee
    LOADC r0, 0xe000e180
    movs r1, #1
    str r1, [r0]

    @ ---- PRIMASK holds PendSV off until CPSIE ----
    bl reset_log
    cpsid i
    LOADC r0, 0xe000ed04
    LOADC r1, 0x10000000
    str r1, [r0]
    isb
    LOADC r0, 0x200001fc
    ldr r1, [r0]
    CHECK 33, r1, 0x00000000
    cpsie i
    isb
    LOADC r0, LOGBASE
    ldr r1, [r0]
    CHECK 34, r1, 0x0000000e

    @ ---- SysTick, returning through POP {PC} ----
    INSTALL tick_handler
    movs r5, #0
    LOADC r0, 0xe000e014
    LOADC r1, 0x00000400
    str r1, [r0]
    LOADC r0, 0xe000e018
    movs r1, #0
    str r1, [r0]
    LOADC r0, 0xe000e010
    movs r1, #7
    str r1, [r0]
exc_tick_wait:
    cmp r5, #0
    beq exc_tick_wait
    CHECK 35, r5, 0x0000000f
    LOADC r0, 0xe000e010
    movs r1, #0
    str r1, [r0]
    LOADC r0, 0xe000ed1c
    str r1, [r0]
    LOADC r0, 0xe000ed20
    str r1, [r0]
    LOADC r0, 0xe000e400
    str r1, [r0]
    b pass

    @ Empty the log.
    .align 2
    .thumb_func
reset_log:
    LOADC r1, 0x200001fc
    movs r2, #0
    str r2, [r1]
    bx lr

    @ Log IPSR, EXC_RETURN, SP, then the stacked r0, r3, r12, LR, PC and
    @ xPSR, then the SVC immediate fetched through the stacked PC.
    .thumb_func
frame_handler:
    mrs r0, ipsr
    LOGV r0
    mov r0, lr
    LOGV r0
    mov r0, sp
    LOGV r0
    @ The frame is on whichever stack EXC_RETURN names.
    mov r3, sp
    LOADC r0, 0xfffffffd
    cmp lr, r0
    bne 1f
    mrs r3, psp
1:
    ldr r0, [r3, #0]
    LOGV r0
    ldr r0, [r3, #12]
    LOGV r0
    ldr r0, [r3, #16]
    LOGV r0
    ldr r0, [r3, #20]
    LOGV r0
    ldr r0, [r3, #24]
    LOGV r0
    ldr r0, [r3, #28]
    LOGV r0
    ldr r0, [r3, #24]
    subs r0, #2
    ldrb r0, [r0]
    LOGV r0
    bx lr

    @ Log IPSR, EXC_RETURN and SP; in SVCall, pend IRQ0 and PendSV.
    .thumb_func
chain_handler:
    mrs r0, ipsr
    LOGV r0
    mov r0, lr
    LOGV r0
    mov r0, sp
    LOGV r0
    mrs r0, ipsr
    cmp r0, #11
    bne 1f
    LOADC r1, 0xe000e200
    movs r0, #1
    str r0, [r1]
    dsb
    isb
    LOADC r1, 0xe000ed04
    LOADC r0, 0x10000000
    str r0, [r1]
    dsb
    isb
    movs r0, #0xff
    LOGV r0
1:  bx lr

    @ Record the exception number in a register no frame restores, and
    @ return through POP {PC} with EXC_RETURN (B1.5.8).
    .thumb_func
tick_handler:
    push {r4, lr}
    mrs r5, ipsr
    pop {r4, pc}
"#;

/// ARMv6-M HardFault (DDI 0419E B1.5.14): every fault there is lands in
/// exception 3 — an UNDEFINED sixteen- or thirty-two-bit encoding, every
/// v7-M-only encoding, an unaligned access, a branch to ARM state, `BKPT`
/// without a debugger, and an `SVC` that cannot be taken.
#[cfg(feature = "cpu-arm-v6m")]
pub(super) const V6M_FAULTS: &str = r#"
    @ Check that the last fault was a HardFault at \label.
    .macro FAULTED n, label
    movs r0, #3
    CHECKR \n, r4, r0
    ADDR r0, \label
    subs r0, #1
    CHECKR (\n + 100), r5, r0
    .endm

    INSTALL skip_handler

    @ The architecturally undefined UDF, and the UDF.W in the one
    @ thirty-two-bit class ARMv6-M allocates (A6.7.76).
f_udf:
    udf #0
    FAULTED 1, f_udf
f_udfw:
    .short 0xf7f0, 0xa000
    FAULTED 2, f_udfw

    @ CBZ, CBNZ and IT do not exist on ARMv6-M (A5.2).
f_cbz:
    .short 0xb100
    FAULTED 3, f_cbz
f_cbnz:
    .short 0xb900
    FAULTED 4, f_cbnz
f_it:
    .short 0xbf08
    FAULTED 5, f_it

    @ Neither does any thirty-two-bit encoding outside BL, MSR, MRS and the
    @ barriers: NOP.W, B.W, MOVW, LDR.W, CLREX, SDIV, LDREX (A5.1.1).
f_nopw:
    .short 0xf3af, 0x8000
    FAULTED 6, f_nopw
f_bw:
    .short 0xf000, 0xb800
    FAULTED 7, f_bw
f_movw:
    .short 0xf240, 0x0000
    FAULTED 8, f_movw
f_ldrw:
    .short 0xf8d0, 0x0000
    FAULTED 9, f_ldrw
f_clrex:
    .short 0xf3bf, 0x8f2f
    FAULTED 10, f_clrex
f_sdiv:
    .short 0xfb90, 0xf0f1
    FAULTED 11, f_sdiv
f_ldrex:
    .short 0xe850, 0x0f00
    FAULTED 12, f_ldrex

    @ Every unaligned halfword or word access faults: CCR.UNALIGN_TRP is
    @ fixed at one (B3.2.8). The store must not happen.
    LOADC r1, 0x20000100
    movs r2, #0
    str r2, [r1]
    str r2, [r1, #4]
    adds r1, #1
f_ldr:
    ldr r2, [r1]
    FAULTED 13, f_ldr
f_ldrh:
    ldrh r2, [r1]
    FAULTED 14, f_ldrh
f_str:
    str r1, [r1]
    FAULTED 15, f_str
f_strh:
    strh r1, [r1]
    FAULTED 16, f_strh
    subs r1, #1
    ldr r2, [r1]
    CHECK 17, r2, 0x00000000
    ldr r2, [r1, #4]
    CHECK 18, r2, 0x00000000
    adds r1, #2
f_ldm:
    ldm r1!, {r2}
    FAULTED 19, f_ldm
    @ A byte access is never unaligned.
    ldrb r2, [r1]
    CHECK 20, r2, 0x00000000

    @ BKPT with no debugger attached is a HardFault (C1.5).
f_bkpt:
    bkpt #0
    FAULTED 21, f_bkpt

    @ SVC with PRIMASK set cannot be taken at its own priority, so it
    @ escalates (B1.5.14). The handler's skip makes the NOP the landing
    @ site whichever return address the escalation stacks.
    movs r4, #0
    cpsid i
    svc #0
    nop
    cpsie i
    movs r0, #3
    CHECKR 22, r4, r0

    @ BX to an even address clears EPSR.T, and the next instruction fetch
    @ faults (B1.4.2): the stacked PC is the target, and the stacked xPSR
    @ has T clear. The handler sets T and resumes there.
    INSTALL thumb_handler
    movs r6, #0
    ADDR r2, f_arm
    subs r2, #1
    bx r2
    movs r0, #23
    bl fail
    .align 2
f_arm:
    FAULTED 24, f_arm
    CHECK 25, r6, 0x00000000

    b pass

    @ Record IPSR and the stacked PC, and step over the instruction there,
    @ sixteen or thirty-two bits.
    .align 2
    .thumb_func
skip_handler:
    mrs r4, ipsr
    ldr r5, [sp, #24]
    ldrh r0, [r5]
    lsrs r0, r0, #11
    movs r1, #2
    cmp r0, #0x1d
    blo 1f
    movs r1, #4
1:  adds r1, r1, r5
    str r1, [sp, #24]
    bx lr

    @ Record IPSR, the stacked PC and the stacked T bit, then set T.
    .thumb_func
thumb_handler:
    mrs r4, ipsr
    ldr r5, [sp, #24]
    ldr r0, [sp, #28]
    lsrs r6, r0, #24
    movs r1, #1
    ands r6, r1
    lsls r1, r1, #24
    orrs r0, r1
    str r0, [sp, #28]
    bx lr
"#;

/// What a Cortex-M0+ adds to a Cortex-M0, all options built: `VTOR`
/// (bits `[31:8]`), unprivileged Thread mode, and the eight-region PMSAv6
/// MPU (DDI 0419E B1.4.4, B3.2.5, B3.5; Cortex-M0+ TRM Table 1-1). An
/// unprivileged program that reaches for the PPB, the MPU's privileged-only
/// region, `MSR PRIMASK` or `CPSID` either faults to HardFault or is
/// ignored, and it climbs back up through SVC.
#[cfg(feature = "cpu-arm-v6m")]
pub(super) const V6M_M0PLUS: &str = r#"
    @ CPUID: a Cortex-M0+ r0p1 (Cortex-M0+ TRM Table 4-2).
    LOADC r0, 0xe000ed00
    ldr r1, [r0]
    CHECK 1, r1, 0x410cc601

    @ VTOR keeps bits [31:8].
    LOADC r0, 0xe000ed08
    LOADC r1, 0x200004ff
    str r1, [r0]
    ldr r2, [r0]
    CHECK 2, r2, 0x20000400

    @ A vector table in SRAM: every entry the trampoline, except SVCall,
    @ which goes straight to its own handler. VTOR moves the table
    @ (B1.5.3).
    LOADC r0, 0x20000400
    LOADC r1, 0x000000c1
    movs r2, #48
1:  str r1, [r0]
    adds r0, #4
    subs r2, #1
    bne 1b
    LOADC r0, 0x20000400
    ADDR r1, vt_svc
    str r1, [r0, #44]
    movs r4, #0
    svc #0
    CHECK 3, r4, 0x0000007e
    LOADC r0, 0xe000ed08
    movs r1, #0
    str r1, [r0]
    movs r4, #0
    INSTALL priv_handler
    svc #0
    CHECK 4, r4, 0x00000000

    @ The MPU: eight regions, unified (B3.5.4).
    LOADC r0, 0xe000ed90
    ldr r1, [r0]
    CHECK 5, r1, 0x00000800

    @ RBAR keeps ADDR [31:8] and reports the region in [3:0]; VALID writes
    @ RNR (B3.5.6).
    LOADC r0, 0xe000ed9c
    LOADC r1, 0x200003f3
    str r1, [r0]
    ldr r2, [r0]
    CHECK 6, r2, 0x20000303
    LOADC r0, 0xe000ed98
    ldr r2, [r0]
    CHECK 7, r2, 0x00000003

    @ Region 0 over the whole SRAM, full access for anyone; region 1 over
    @ 0x20000800-0x200008FF, privileged read/write and no unprivileged
    @ access (AP = 001). Code at 0 runs from the default map, which
    @ PRIVDEFENA does not give an unprivileged program -- so region 2
    @ covers the flash, read-only for anyone (AP = 110).
    LOADC r0, 0xe000ed9c
    LOADC r1, 0x20000010
    str r1, [r0]
    LOADC r0, 0xe000eda0
    LOADC r1, 0x0300001f
    str r1, [r0]
    LOADC r0, 0xe000ed9c
    LOADC r1, 0x20000811
    str r1, [r0]
    LOADC r0, 0xe000eda0
    LOADC r1, 0x1100000f
    str r1, [r0]
    LOADC r0, 0xe000ed9c
    LOADC r1, 0x00000012
    str r1, [r0]
    LOADC r0, 0xe000eda0
    LOADC r1, 0x0600001f
    str r1, [r0]
    LOADC r0, 0xe000ed94
    movs r1, #5
    str r1, [r0]
    dsb
    isb

    @ Privileged: region 1 is readable.
    LOADC r0, 0x20000800
    LOADC r1, 0x5a5a5a5a
    str r1, [r0]
    ldr r2, [r0]
    CHECK 8, r2, 0x5a5a5a5a

    @ Drop to unprivileged Thread mode on the process stack. nPRIV sticks
    @ on this part.
    INSTALL fault_handler
    LOADC r0, 0x20008000
    msr psp, r0
    movs r0, #3
    msr control, r0
    isb
    mrs r1, control
    CHECK 9, r1, 0x00000003

    @ Unprivileged: region 0 still works, region 1 is a HardFault and the
    @ load does not happen (B3.5.1).
    LOADC r0, 0x20000100
    ldr r2, [r0]
    movs r4, #0
    movs r2, #0
    LOADC r0, 0x20000800
m_mpu:
    ldr r2, [r0]
    movs r0, #3
    CHECKR 10, r4, r0
    ADDR r0, m_mpu
    subs r0, #1
    CHECKR 11, r5, r0
    CHECK 12, r2, 0x00000000

    @ An unprivileged store to region 1 is refused as well.
    movs r4, #0
    LOADC r0, 0x20000800
    movs r1, #0
m_store:
    str r1, [r0]
    movs r0, #3
    CHECKR 13, r4, r0

    @ Any PPB access is a HardFault unprivileged (B3.1.1).
    movs r4, #0
    LOADC r0, 0xe000ed00
m_ppb:
    ldr r2, [r0]
    movs r0, #3
    CHECKR 14, r4, r0
    ADDR r0, m_ppb
    subs r0, #1
    CHECKR 15, r5, r0

    @ MSR PRIMASK and CPSID are ignored unprivileged (B4.2.1, B4.2.3).
    movs r0, #1
    msr primask, r0
    mrs r1, primask
    CHECK 16, r1, 0x00000000
    cpsid i
    mrs r1, primask
    CHECK 17, r1, 0x00000000
    @ ...and so is an attempt to clear nPRIV.
    movs r0, #2
    msr control, r0
    isb
    mrs r1, control
    CHECK 18, r1, 0x00000003

    @ SVC is the way back up: its handler clears nPRIV.
    INSTALL priv_handler
    svc #0
    mrs r1, control
    CHECK 19, r1, 0x00000002
    movs r0, #0
    msr control, r0
    isb

    @ Privileged again: region 1 is reachable, the MPU switched off.
    LOADC r0, 0x20000800
    ldr r2, [r0]
    CHECK 20, r2, 0x5a5a5a5a
    LOADC r0, 0xe000ed94
    movs r1, #0
    str r1, [r0]
    dsb
    isb
    b pass

    .align 2
    .thumb_func
vt_svc:
    movs r4, #0x7e
    bx lr

    @ Clear nPRIV, the privileged half of CONTROL a handler may write.
    .thumb_func
priv_handler:
    mrs r0, control
    movs r1, #1
    bics r0, r1
    msr control, r0
    bx lr

    @ Record IPSR and the stacked PC (on PSP: the fault came from Thread
    @ mode on the process stack), and step over the instruction.
    .thumb_func
fault_handler:
    mrs r4, ipsr
    mrs r3, psp
    ldr r5, [r3, #24]
    adds r0, r5, #2
    str r0, [r3, #24]
    bx lr
"#;
