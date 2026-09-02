/* Where an A64 conformance guest is linked.
 *
 * One flat image at the base of the board's DRAM, with `.text.start` kept
 * first so the entry point is the first instruction — which is what makes the
 * entry address in the ELF header a cross-check rather than the only thing the
 * loader trusts.
 *
 * Copyright (c) Karpeles Lab Inc. MIT.
 */

ENTRY(_start)

SECTIONS {
	. = 0x40000000;

	.text : {
		KEEP(*(.text.start))
		*(.text .text.*)
	}

	.rodata : {
		*(.rodata .rodata.*)
	}

	.data : {
		*(.data .data.*)
	}

	.bss : {
		*(.bss .bss.*)
		*(COMMON)
	}

	/* Nothing here reads its own unwind tables or its own debug info, and a
	 * loader that copied them into guest RAM would be loading megabytes of
	 * nothing. */
	/DISCARD/ : {
		*(.eh_frame)
		*(.eh_frame_hdr)
		*(.comment)
		*(.note*)
		*(.debug*)
	}
}
