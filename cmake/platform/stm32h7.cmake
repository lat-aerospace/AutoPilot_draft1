####
# cmake/platform/stm32h7.cmake
#
# F Prime platform file for STM32H7 targets (Cortex-M7 with FreeRTOS).
# Selects FreeRTOS-backed OS implementations instead of Posix,
# and stubs for file I/O, CPU, and memory (no OS filesystem on bare-metal).
#
# NOTE: We use stm32/Platform/ instead of unix/Platform/ because the
# Unix platform types config (UnixPlatformTypes) selects Posix OS
# implementations that don't exist in the cross-compile build.
# stm32/Platform/ provides the same PlatformTypes.fpp/h (32-bit ILP32)
# but only selects FreeRTOS + Stub implementations.
####

add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/stm32/Platform/")
target_compile_definitions(Stm32PlatformTypes INTERFACE -DTGT_OS_TYPE_FREERTOS)
