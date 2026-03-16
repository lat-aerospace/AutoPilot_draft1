####
# cmake/stm32f4_toolchain.cmake
#
# ARM GCC cross-compilation toolchain for STM32F4xx (Cortex-M4 with FPU).
# Same structure as stm32h7_toolchain.cmake but with different MCU flags
# and a tighter heap budget.
####

set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  arm)

find_program(ARM_GCC arm-none-eabi-gcc REQUIRED)
find_program(ARM_GXX arm-none-eabi-g++ REQUIRED)

set(CMAKE_C_COMPILER   ${ARM_GCC})
set(CMAKE_CXX_COMPILER ${ARM_GXX})

# ---------------------------------------------------------------------------
# Architecture flags (Cortex-M4F)
# ---------------------------------------------------------------------------
set(MCU_FLAGS
    "-mcpu=cortex-m4"
    "-mthumb"
    "-mfpu=fpv4-sp-d16"
    "-mfloat-abi=hard"
)
string(JOIN " " MCU_FLAGS_STR ${MCU_FLAGS})

set(CMAKE_C_FLAGS_INIT
    "${MCU_FLAGS_STR} \
     -DSTM32F4 \
     -DSTM32F405xx \
     -DUSE_HAL_DRIVER \
     -fdata-sections -ffunction-sections \
     -fno-exceptions \
     -Wall -Wextra -O2 -g"
)
set(CMAKE_CXX_FLAGS_INIT
    "${MCU_FLAGS_STR} \
     -DSTM32F4 \
     -DSTM32F405xx \
     -DUSE_HAL_DRIVER \
     -fdata-sections -ffunction-sections \
     -fno-exceptions -fno-rtti \
     -std=c++17 -Wall -Wextra -O2 -g"
)
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${MCU_FLAGS_STR} \
     -Wl,--gc-sections \
     -Wl,--print-memory-usage \
     -specs=nano.specs \
     -specs=nosys.specs \
     -T${CMAKE_CURRENT_LIST_DIR}/../AP/Top/Stm32/stm32f405_flash.ld"
)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(FPRIME_PLATFORM stm32f4)

if(NOT DEFINED FREERTOS_PATH)
    set(FREERTOS_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/freertos")
endif()
if(NOT DEFINED STM32_HAL_PATH)
    set(STM32_HAL_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/stm32f4_hal")
endif()

if(NOT TARGET freertos_kernel)
    add_library(freertos_kernel INTERFACE)
    target_include_directories(freertos_kernel INTERFACE
        "${FREERTOS_PATH}/include"
        "${FREERTOS_PATH}/portable/GCC/ARM_CM4F"
        "${CMAKE_CURRENT_LIST_DIR}/../AP/Top/Stm32"
    )
    target_sources(freertos_kernel INTERFACE
        "${FREERTOS_PATH}/tasks.c"
        "${FREERTOS_PATH}/queue.c"
        "${FREERTOS_PATH}/list.c"
        "${FREERTOS_PATH}/timers.c"
        "${FREERTOS_PATH}/event_groups.c"
        "${FREERTOS_PATH}/stream_buffer.c"
        "${FREERTOS_PATH}/portable/GCC/ARM_CM4F/port.c"
        "${FREERTOS_PATH}/portable/MemMang/heap_4.c"
    )
endif()

if(NOT TARGET stm32_hal)
    add_library(stm32_hal INTERFACE)
    target_include_directories(stm32_hal INTERFACE
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Inc"
        "${STM32_HAL_PATH}/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
        "${STM32_HAL_PATH}/Drivers/CMSIS/Include"
    )
    file(GLOB STM32_HAL_SRCS
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"
        "${STM32_HAL_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c"
    )
    target_sources(stm32_hal INTERFACE ${STM32_HAL_SRCS})
endif()
