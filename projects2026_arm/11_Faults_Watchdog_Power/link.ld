/* link.ld - lesson 11's copy of _startup/c031c6/link.ld, with two additions
 * FLASH 32K @ 0x08000000, RAM 12K @ 0x20000000
 *
 * The build engine prefers a lesson's own link.ld, so this file replaces the
 * shared one for this lesson only.  Compared with the shared copy it adds:
 *
 *   1.  .noinit - RAM that startup.c neither copies nor zeroes, because it
 *       lies outside both .data and .bss.  A RESET (watchdog, software,
 *       NRST pin) does not clear SRAM; startup.c does, for .data and .bss.
 *       So a variable placed here survives a reset, and the crash record
 *       written by HardFault_Handler is still there when main() runs again.
 *       After POWER-ON it holds garbage, which is why crash.c guards it with
 *       a magic number and a checksum.  (NOLOAD: the section takes RAM but
 *       has no bytes in the image.)
 *
 *   2.  _stack_floor - the lowest address the main stack may reach.  RAM
 *       above it is the stack; below it, the heap and everything else.
 *       crash.c paints the stack from here up with 0xDEADBEEF at boot (lesson
 *       07's OS_STACK_FILL) and watches the bottom 16 words: a stack that has
 *       grown through them has overflowed.  _Min_Stack_Size is a MINIMUM: the
 *       stack actually gets everything from _stack_floor to _estack.
 */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
  RAM   (rw)  : ORIGIN = 0x20000000, LENGTH = 12K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x800;

SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH

  .text : {
    . = ALIGN(4);
    *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM       : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

  .preinit_array (READONLY) : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
  .init_array    (READONLY) : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
  .fini_array    (READONLY) : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH

  _sidata = LOADADDR(.data);

  .data : {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss : {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* NEW: survives a reset.  Outside _sdata.._edata and _sbss.._ebss, so
   * startup.c's two loops never touch it. */
  .noinit (NOLOAD) : {
    . = ALIGN(4);
    _snoinit = .;
    *(.noinit) *(.noinit.*)
    . = ALIGN(4);
    _enoinit = .;
  } >RAM

  ._user_heap_stack : {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
    _stack_floor = .;            /* NEW: the stack may not go below this */
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
