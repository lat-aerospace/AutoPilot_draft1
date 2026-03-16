####
# This sets up the build system for the 'AP' project, including
# components and deployments from project.cmake. In addition, it imports the core F Prime components.
####

cmake_minimum_required(VERSION 3.24.2)
project(AP C CXX ASM)

###
# F' Core Setup
# This includes all of the F prime core components, and imports the make-system.
###
include("${CMAKE_CURRENT_LIST_DIR}/lib/fprime/cmake/FPrime.cmake")
# NOTE: register custom targets between these two lines

# ---------------------------------------------------------------------------
# FreeRTOS kernel — compile sources once as a STATIC library (STM32 only)
# The toolchain creates freertos_kernel as INTERFACE (headers only).
# ---------------------------------------------------------------------------
if(TARGET freertos_kernel AND NOT TARGET freertos_kernel_lib)
    add_library(freertos_kernel_lib STATIC
        "${FREERTOS_PATH}/tasks.c"
        "${FREERTOS_PATH}/queue.c"
        "${FREERTOS_PATH}/list.c"
        "${FREERTOS_PATH}/timers.c"
        "${FREERTOS_PATH}/event_groups.c"
        "${FREERTOS_PATH}/stream_buffer.c"
        "${FREERTOS_PATH}/portable/GCC/ARM_CM7/r0p1/port.c"
        "${FREERTOS_PATH}/portable/MemMang/heap_4.c"
    )
    target_link_libraries(freertos_kernel_lib PUBLIC freertos_kernel)
    target_link_libraries(freertos_kernel INTERFACE freertos_kernel_lib)
endif()

# ---------------------------------------------------------------------------
# STM32 HAL — compile sources once as a STATIC library (STM32 only)
# The toolchain creates stm32_hal as INTERFACE (headers only).
# ---------------------------------------------------------------------------
if(TARGET stm32_hal AND NOT TARGET stm32_hal_lib)
    set(STM32_HAL_SRCS
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c"
        "${STM32_HAL_PATH}/Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c"
    )
    add_library(stm32_hal_lib STATIC ${STM32_HAL_SRCS})
    target_link_libraries(stm32_hal_lib PUBLIC stm32_hal)
    target_link_libraries(stm32_hal INTERFACE stm32_hal_lib)
endif()

# Eigen — header-only linear algebra library
add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE "${CMAKE_CURRENT_LIST_DIR}/lib/eigen")

# MAVLink v2 — header-only C library
add_library(mavlink INTERFACE)
target_include_directories(mavlink SYSTEM INTERFACE "${CMAKE_CURRENT_LIST_DIR}/lib/mavlink")

fprime_setup_included_code()

# Project configuration overrides (e.g. RateGroupDriverRateGroupPorts = 4)
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/config")


# This includes project-wide objects
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/AP")
