/*
 * rsemu's device-under-test macros for riscv-arch-test.
 *
 * Copyright (c) 2026 the rsemu authors. MIT.
 *
 * riscv-arch-test does not ship a `model_test.h`: it is the one file the suite
 * expects the DUT's owner to write, and `riscv-test-suite/env/arch_test.h`
 * (BSD-3-Clause, (c) RISC-V International) is the specification for it. Every
 * macro below is defined from that header's stated contract — the required
 * labels and the trap-handler layout it documents — rather than copied from
 * another DUT's plugin.
 *
 * The whole of what rsemu needs from the model is:
 *
 *   * a way for a test to say it is finished. Berkeley's HTIF convention: a
 *     non-zero store to a word called `tohost`, then spin. Both the reference
 *     model and tests/conformance/riscv.rs watch that word, so the two agree
 *     on when a test ended without either needing a UART.
 *   * the two symbols that delimit the signature.
 *
 * Everything else is deliberately empty:
 *
 *   * `RVMODEL_BOOT` — the linker script puts `.text.init` at the reset
 *     vector, so there is nothing to set up before the test's own prologue.
 *   * `RVMODEL_IO_*` — these write progress text to a console. rsemu's runner
 *     diffs signatures, so a console would be an extra device model whose only
 *     job is to be ignored. Note in particular that `RVMODEL_IO_ASSERT_GPR_EQ`
 *     is handed the `correctval` operand baked into each test: leaving it
 *     empty is what makes the *signature* the sole verdict, which is what
 *     conformance means here.
 *   * the interrupt macros — no test selected for this hart uses them, and a
 *     definition that poked a CLINT rsemu's runner does not map would be a
 *     silent hang rather than an unmapped-access report.
 */

#ifndef RSEMU_MODEL_TEST_H
#define RSEMU_MODEL_TEST_H

/* HTIF's `tohost`/`fromhost` pair, in their own section so the linker script
 * can place them on a page of their own where a model that scans for them will
 * find them. `.dword` for both regardless of XLEN: HTIF is a 64-bit protocol. */
#define RVMODEL_DATA_SECTION                                            \
        .pushsection .tohost,"aw",@progbits;                            \
        .align 8; .global tohost;   tohost:   .dword 0;                 \
        .align 8; .global fromhost; fromhost: .dword 0;                 \
        .popsection;

/* Store one to `tohost`, then spin. The spin matters: the store is what ends
 * the test, and a hart that ran off the end afterwards would be executing
 * whatever followed while the runner was still deciding it had finished.
 *
 * `tohost` is 64 bits wide whatever XLEN is, and the *whole* of it has to be
 * written: the reference model acts on the store that completes the
 * doubleword, so an RV32 halt that wrote only the low word left the model
 * spinning in this loop until its instruction limit — every RV32 test in the
 * corpus, silently, until the trace was read. Hence two stores on RV32, low
 * half first, and one on RV64. */
#if XLEN == 32
  #define RSEMU_STORE_TOHOST(_val, _base) sw _val, 0(_base); sw x0, 4(_base)
#else
  #define RSEMU_STORE_TOHOST(_val, _base) sd _val, 0(_base)
#endif

#define RVMODEL_HALT                                                    \
        li t5, 1;                                                       \
        la t4, tohost;                                                  \
        RSEMU_STORE_TOHOST(t5, t4);                                     \
  99:   j 99b;

#define RVMODEL_BOOT

#define RVMODEL_DATA_BEGIN                                              \
        RVMODEL_DATA_SECTION                                            \
        .align 4; .global begin_signature; begin_signature:

#define RVMODEL_DATA_END                                                \
        .align 4; .global end_signature; end_signature:

#define RVMODEL_IO_INIT
#define RVMODEL_IO_WRITE_STR(_R, _STR)
#define RVMODEL_IO_CHECK()
#define RVMODEL_IO_ASSERT_GPR_EQ(_S, _R, _I)
#define RVMODEL_IO_ASSERT_SFPR_EQ(_F, _R, _I)
#define RVMODEL_IO_ASSERT_DFPR_EQ(_D, _R, _I)

#define RVMODEL_SET_MSW_INT
#define RVMODEL_CLR_MSW_INT
#define RVMODEL_CLR_MTIMER_INT
#define RVMODEL_CLR_MEXT_INT
#define RVMODEL_SET_SSW_INT
#define RVMODEL_CLR_SSW_INT
#define RVMODEL_CLR_STIMER_INT
#define RVMODEL_CLR_SEXT_INT
#define RVMODEL_SET_VSW_INT
#define RVMODEL_CLR_VSW_INT
#define RVMODEL_CLR_VTIMER_INT
#define RVMODEL_CLR_VEXT_INT

#endif /* RSEMU_MODEL_TEST_H */
