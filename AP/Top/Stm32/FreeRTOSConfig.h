/*
 * FreeRTOSConfig.h
 * AP Autopilot – FreeRTOS kernel configuration for STM32H7/F7/F4
 *
 * Targets:
 *   STM32H7xx @ 480 MHz, 1 MB SRAM, ARM Cortex-M7
 *   STM32F7xx @ 216 MHz, 512 KB SRAM, ARM Cortex-M7
 *   STM32F4xx @ 168 MHz, 192 KB SRAM, ARM Cortex-M4
 *
 * Tune configCPU_CLOCK_HZ, configTOTAL_HEAP_SIZE, and the NVIC priority
 * macros for each target via compile-time defines set in the toolchain
 * cmake file:  -DSTM32H7 / -DSTM32F7 / -DSTM32F4
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* Target-specific clock / heap tuning */
#if defined(STM32H7)
  #define configCPU_CLOCK_HZ              480000000UL
  #define configTOTAL_HEAP_SIZE           ((size_t)(128 * 1024))   /* 128 KB */
#elif defined(STM32F7)
  #define configCPU_CLOCK_HZ              216000000UL
  #define configTOTAL_HEAP_SIZE           ((size_t)(128 * 1024))   /* 128 KB */
#elif defined(STM32F4)
  #define configCPU_CLOCK_HZ              168000000UL
  #define configTOTAL_HEAP_SIZE           ((size_t)(96 * 1024))    /* 96 KB  */
#else
  #error "No STM32 target defined. Add -DSTM32H7, -DSTM32F7, or -DSTM32F4."
#endif

/* -----------------------------------------------------------------------
 * Core kernel settings
 * --------------------------------------------------------------------- */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configTICK_RATE_HZ                      ((TickType_t)1000)  /* 1 ms tick */
#define configMAX_PRIORITIES                    8
#define configMINIMAL_STACK_SIZE                ((uint16_t)256)     /* words */
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1

/* -----------------------------------------------------------------------
 * Synchronisation primitives
 * --------------------------------------------------------------------- */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               16
#define configUSE_QUEUE_SETS                    0

/* -----------------------------------------------------------------------
 * Memory allocation
 * --------------------------------------------------------------------- */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
/* heap_4.c: best-fit, coalescing.  Configured in lib/freertos. */

/* -----------------------------------------------------------------------
 * Hook functions
 * --------------------------------------------------------------------- */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     1   /* drives HAL_IncTick() */
#define configUSE_MALLOC_FAILED_HOOK            1   /* pvPortMalloc failure */
#define configCHECK_FOR_STACK_OVERFLOW          2   /* Fill pattern check  */

/* -----------------------------------------------------------------------
 * Run-time statistics (disabled – DWT setup is done in Main.cpp)
 * --------------------------------------------------------------------- */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* -----------------------------------------------------------------------
 * Co-routines (not used)
 * --------------------------------------------------------------------- */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         2

/* -----------------------------------------------------------------------
 * Software timers (not used by F' but available if needed)
 * --------------------------------------------------------------------- */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE

/* -----------------------------------------------------------------------
 * Interrupt priorities (ARM Cortex-M)
 *
 * STM32H7/F7/F4 implement 4 bits of interrupt priority → 16 levels.
 * FreeRTOS requires configMAX_SYSCALL_INTERRUPT_PRIORITY to be set so
 * that ISRs using FreeRTOS API functions have a priority ≥ this value.
 *
 * Rule: hardware ISRs that call FreeRTOS API (*FromISR variants) must be
 * configured with NVIC priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 * ISRs that do NOT call FreeRTOS API can use any priority.
 * --------------------------------------------------------------------- */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* These are values in the priority register (shifted left by 4 on M4/M7) */
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - 4))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - 4))

/* -----------------------------------------------------------------------
 * F' / AP-specific: map vPortSVCHandler and friends to CMSIS names
 * --------------------------------------------------------------------- */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* -----------------------------------------------------------------------
 * Optional: task-aware debugging with FreeRTOS-aware IDEs
 * --------------------------------------------------------------------- */
#define INCLUDE_vTaskPrioritySet        1
#define INCLUDE_uxTaskPriorityGet       1
#define INCLUDE_vTaskDelete             1
#define INCLUDE_vTaskSuspend            1
#define INCLUDE_xResumeFromISR          1
#define INCLUDE_vTaskDelayUntil         1
#define INCLUDE_vTaskDelay              1
#define INCLUDE_xTaskGetSchedulerState  1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle  0
#define INCLUDE_eTaskGetState           1
#define INCLUDE_xEventGroupSetBitFromISR 1
#define INCLUDE_xTimerPendFunctionCall  0
#define INCLUDE_xTaskAbortDelay         0

#endif  /* FREERTOS_CONFIG_H */
