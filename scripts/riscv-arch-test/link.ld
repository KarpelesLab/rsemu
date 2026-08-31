/*
 * Where a riscv-arch-test image lands.
 *
 * Copyright (c) 2026 the rsemu authors. MIT.
 *
 * 0x8000_0000 because that is where every RISC-V board rsemu models puts the
 * first byte of RAM, and because it is where tests/conformance/riscv.rs maps
 * its sixteen megabytes. The reference model's default memory region starts
 * there too, so one script builds an image both of them can run.
 *
 * The page alignment between sections is not tidiness. `.tohost` has to sit on
 * a page of its own so a model that watches it cannot be confused by traffic
 * to a neighbour, and putting the data and signature areas on page boundaries
 * is what lets the privileged tests map them with an identity page table
 * without the mapping cutting a section in half.
 */

OUTPUT_ARCH("riscv")
ENTRY(rvtest_entry_point)

SECTIONS
{
  . = 0x80000000;
  .text.init      : { *(.text.init)   }
  . = ALIGN(0x1000);
  .tohost         : { *(.tohost)      }
  . = ALIGN(0x1000);
  .text           : { *(.text)        }
  . = ALIGN(0x1000);
  .data           : { *(.data)        }
  .data.string    : { *(.data.string) }
  .bss            : { *(.bss)         }
  _end = .;
}
