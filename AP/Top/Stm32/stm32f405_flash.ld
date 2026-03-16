/*
 * stm32f405_flash.ld
 * Linker script for STM32F405RG (1 MB Flash, 192 KB SRAM, 64 KB CCM)
 *
 * Memory layout:
 *   FLASH  – code + read-only data
 *   SRAM   – initialized + zero data, heap
 *   CCM    – FreeRTOS kernel + critical task stacks (CPU-local, fastest access)
 *
 * NOTE: CCM cannot be used for DMA.  Keep DMA buffers in SRAM.
 */

MEMORY
{
    FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    SRAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K   /* Main SRAM1 + SRAM2 */
    CCM     (xrw) : ORIGIN = 0x10000000, LENGTH = 64K    /* Core-coupled memory */
}

ENTRY(Reset_Handler)

_Min_Heap_Size  = 0x1000;
_Min_Stack_Size = 0x0800;

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
        KEEP(*(.init)) KEEP(*(.fini))
        . = ALIGN(4); _etext = .;
    } >FLASH

    .rodata  : { *(.rodata)  *(.rodata*)  } >FLASH
    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

    .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)); PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
    .init_array    : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))); KEEP(*(.init_array*)); PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
    .fini_array    : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))); KEEP(*(.fini_array*)); PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH

    _sidata = LOADADDR(.data);
    .data : { . = ALIGN(4); _sdata = .; *(.data) *(.data*) . = ALIGN(4); _edata = .; } >SRAM AT> FLASH

    .bss : { . = ALIGN(4); _sbss = .; *(.bss) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } >SRAM

    /* FreeRTOS kernel + task stacks in CCM */
    .ccm_data (NOLOAD) :
    {
        . = ALIGN(4);
        *(.freertos_tcb_data)
        *(.freertos_stack_data)
        . = ALIGN(4);
    } >CCM

    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .); PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >SRAM

    /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
