####
# cmake/platform/stm32/Platform/CMakeLists.txt
#
# STM32 platform types configuration.
# Re-uses the same PlatformTypes.fpp/.h as Unix (both are 32-bit, ILP32)
# but selects FreeRTOS-backed OS implementations instead of Posix.
####

register_fprime_config(
        Stm32PlatformTypes
    AUTOCODER_INPUTS
        "${FPRIME_FRAMEWORK_PATH}/cmake/platform/unix/Platform/PlatformTypes.fpp"
    HEADERS
        "${FPRIME_FRAMEWORK_PATH}/cmake/platform/unix/Platform/PlatformTypes.h"
    CHOOSES_IMPLEMENTATIONS
        Os_Task_FreeRtos
        Os_Mutex_FreeRtos
        Os_Generic_PriorityQueue
        Os_RawTime_FreeRtos
        Os_Console_FreeRtos
        Os_File_Stub
        Os_Cpu_Stub
        Os_Memory_Stub
        Fw_StringFormat_snprintf
    INTERFACE
    BASE_CONFIG
)
