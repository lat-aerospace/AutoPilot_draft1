/*
 * stm32h745_flash.ld
 * Linker script for STM32H745ZIT6 — Cortex-M7 core only
 *
 * Dual-core device: M7 boots from Flash Bank 1 (0x0800_0000, 1 MB).
 *                   M4 is kept in reset (not used).
 *
 * Memory layout strategy:
 *   FLASH  – code + read-only data (Bank 1, M7 only)
 *   DTCM   – FreeRTOS kernel data, task stacks, critical paths (zero-wait)
 *   AXI    – F' component state, queues, DMA buffers
 *   SRAM1  – overflow heap / additional buffers (D2 domain, shared)
 *   SRAM2  – available for DMA (D2 domain, shared)
 *   SRAM3  – available for DMA (D2 domain, shared)
 *   SRAM4  – D3 domain (backup / low-power accessible)
 *
 * The FreeRTOS heap (configTOTAL_HEAP_SIZE = 256 KB) is placed in AXI SRAM.
 */

MEMORY
{
    FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K   /* Flash Bank 1 (M7) */
    DTCM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K    /* DTCM: zero-wait CPU access */
    AXI     (xrw) : ORIGIN = 0x24000000, LENGTH = 512K    /* AXI SRAM: general purpose   */
    SRAM1   (xrw) : ORIGIN = 0x30000000, LENGTH = 128K    /* D2 SRAM1 */
    SRAM2   (xrw) : ORIGIN = 0x30020000, LENGTH = 128K    /* D2 SRAM2 */
    SRAM3   (xrw) : ORIGIN = 0x30040000, LENGTH = 32K     /* D2 SRAM3 */
    SRAM4   (xrw) : ORIGIN = 0x38000000, LENGTH = 64K     /* D3 SRAM4 */
}

/* Initial stack pointer — top of AXI SRAM (stack grows downward) */
_estack = ORIGIN(AXI) + LENGTH(AXI);

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
        KEEP(*(.text.Reset_Handler))
        KEEP(*(.text.Default_Handler))
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
