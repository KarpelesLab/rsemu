/*
 * Linker script for the STM32WB BLE smoke firmware (rsemu, MIT).
 *
 * Sized for the smaller of the two boards, the WB35CE: 32 KiB of SRAM1 and
 * the application flash below its secure area. The mailbox lives at the
 * start of SRAM2a, where CPU2 looks for the reference table (IPCCDBA = 0,
 * RM0434 §3.10.16), and stays inside SRAM2a's first ten kilobytes, below the
 * part CPU2's stack reserves for itself.
 *
 * The symbols are the ones ST's startup file expects: _sidata/_sdata/_edata
 * for the data copy, _sbss/_ebss for the clear, _estack for the initial SP.
 */
ENTRY(Reset_Handler)
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
  MB    (rw)  : ORIGIN = 0x20030000, LENGTH = 10K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } >FLASH
  .text : { *(.text*) *(.rodata*) . = ALIGN(4); } >FLASH
  .ARM.exidx : { *(.ARM.exidx*) } >FLASH
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } >RAM AT>FLASH
  /* ST's startup file also copies an initialised SRAM2b section, MB_MEM2;
   * this app has none, so the range is empty. */
  _sMB_MEM2 = .; _eMB_MEM2 = .; _siMB_MEM2 = .;
  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } >RAM
  /* Not cleared by the startup file: main() initialises every field. */
  .mapping_table (NOLOAD) : { KEEP(*(.mapping_table)) } >MB
  .mailbox (NOLOAD) : { . = ALIGN(4); *(.mailbox) } >MB
}
