####
# AP/Os/FreeRtos/CMakeLists.txt
#
# Registers FreeRTOS-backed implementations of the F' OS primitives.
# Each call to register_os_implementation() follows the same pattern as
# Os/Posix/CMakeLists.txt:
#
#   register_os_implementation("Primitive" FreeRtos [extra_deps...])
#
# Required files per primitive (checked by add_fprime_supplied_os_module):
#   Primitive.cpp / Primitive.hpp  – implementation class
#   DefaultPrimitive.cpp           – getDelegate() factory function
#
# The FreeRTOS kernel itself is provided by the 'freertos_kernel' INTERFACE
# target defined in lib/freertos/CMakeLists.txt (see that file for setup).
####

add_custom_target("${FPRIME_CURRENT_MODULE}")

# Task – FreeRTOS xTaskCreate / vTaskDelete
register_os_implementation("Task" FreeRtos freertos_kernel Fw_Time)

# Mutex + ConditionVariable – FreeRTOS recursive semaphore + task notifications
register_os_implementation("Mutex;ConditionVariable" FreeRtos freertos_kernel)

# RawTime – xTaskGetTickCount + DWT cycle counter
register_os_implementation("RawTime" FreeRtos freertos_kernel Fw_Buffer)

# Console – ITM SWO / overridable AP_ConsolePutChar
register_os_implementation("Console" FreeRtos freertos_kernel Fw_Logger)

# Queue – uses the framework's Generic PriorityQueue (backed by FreeRTOS Mutex/CondVar).
# The DefaultQueue.cpp factory is registered separately below since it only
# provides the delegate factory, not a full Queue implementation.
# The actual queue logic lives in Os/Generic/PriorityQueue (framework-supplied).
