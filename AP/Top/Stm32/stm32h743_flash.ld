/*
 * stm32h743_flash.ld
 * Linker script for STM32H743 (2 MB Flash, 1 MB AXI SRAM + 128 KB DTCM)
 *
 * Memory layout strategy:
 *   FLASH  – code + read-only data
 *   DTCM   – FreeRTOS kernel data, task stacks, critical paths (zero-wait)
 *   AXI    – F' component state, queues, DMA buffers
 *   SRAM1  – overflow heap / additional buffers
 *
 * The FreeRTOS heap (configTOTAL_HEAP_SIZE = 256 KB) is placed in AXI SRAM.
 */

MEMORY
{
    FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
    DTCM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K    /* DTCM: zero-wait CPU access */
    AXI     (xrw) : ORIGIN = 0x24000000, LENGTH = 512K    /* AXI SRAM: general purpose   */
    SRAM1   (xrw) : ORIGIN = 0x30000000, LENGTH = 128K
    SRAM2   (xrw) : ORIGIN = 0x30020000, LENGTH = 128K
    BACKUP  (xrw) : ORIGIN = 0x38000000, LENGTH = 64K     /* Backup SRAM */
}

ENTRY(Reset_Handler)

_Min_Heap_Size  = 0x2000;    /* 8 KB minimum for newlib */
_Min_Stack_Size = 0x1000;    /* 4 KB minimum boot stack */

SECTIONS
{
    /* Vector table and code in FLASH */
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

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM        : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    /* Initialized data in AXI SRAM (copied from FLASH at startup) */
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >AXI AT> FLASH

    /* Zero-initialized data in AXI SRAM */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >AXI

    /* FreeRTOS task stacks and kernel state – placed in DTCM for performance */
    .freertos_data (NOLOAD) :
    {
        . = ALIGN(4);
        _freertos_start = .;
        *(.freertos_tcb_data)
        *(.freertos_stack_data)
        . = ALIGN(4);
        _freertos_end = .;
    } >DTCM

    /* Boot stack (used before FreeRTOS starts, then reused as MSP stack) */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >AXI

    /* Discard C++ exception handling tables if not used */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
