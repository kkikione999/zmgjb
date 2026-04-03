/*
 * STM32F411XE linker script for GCC
 * Flash: 512KB (0x08000000 - 0x08080000)
 * SRAM:  128KB (0x20000000 - 0x20020000)
 */

ENTRY(Reset_Handler)

/* Memory regions */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Stack and heap sizes */
_stack_size = 0x400;  /* 1KB stack */
_heap_size = 0x200;    /* 512B heap */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

ENTRY(Reset_Handler)

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.exidx :
  {
    *(.ARM.exidx*)
    *(.gnu.linkonce.armexidx.*)
  } >FLASH

  .ARM.attributes :
  {
    *(.ARM.attributes)
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >SRAM AT>FLASH

  _sidata = LOADADDR(.data);

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >SRAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    _end = .;
    . = . + _heap_size;
    . = . + _stack_size;
    . = ALIGN(8);
  } >SRAM

  /DISCARD/ :
  {
    libc.a (*)
    libm.a (*)
    libgcc.a (*)
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
