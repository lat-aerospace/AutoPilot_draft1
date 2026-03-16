####
# cmake/stm32h7_toolchain.cmake
#
# ARM GCC cross-compilation toolchain for STM32H7xx (Cortex-M7 with FPU).
#
# Usage:
#   fprime-util generate --toolchain cmake/stm32h7_toolchain.cmake stm32h7
#
# Prerequisites:
#   arm-none-eabi-gcc >= 10.x  (from ARM embedded toolchain or package manager)
#   lib/freertos/              (FreeRTOS-Kernel submodule or download)
#   lib/stm32h7_hal/           (STM32CubeH7 HAL from ST Microelectronics)
####

# Cross-compilation target
set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  arm)

# ---------------------------------------------------------------------------
# Toolchain binaries
# ---------------------------------------------------------------------------
find_program(ARM_GCC arm-none-eabi-gcc REQUIRED)
find_program(ARM_GXX arm-none-eabi-g++ REQUIRED)
find_program(ARM_AR  arm-none-eabi-ar  REQUIRED)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy)
find_program(ARM_SIZE    arm-none-eabi-size)

set(CMAKE_C_COMPILER   ${ARM_GCC})
set(CMAKE_CXX_COMPILER ${ARM_GXX})
set(CMAKE_AR           ${ARM_AR})
set(CMAKE_RANLIB       arm-none-eabi-ranlib)

# ---------------------------------------------------------------------------
# Architecture flags (Cortex-M7, single + double precision FPU)
# ---------------------------------------------------------------------------
set(MCU_FLAGS
    "-mcpu=cortex-m7"
    "-mthumb"
    "-mfpu=fpv5-d16"
    "-mfloat-abi=hard"
)
string(JOIN " " MCU_FLAGS_STR ${MCU_FLAGS})

# ---------------------------------------------------------------------------
# Compile flags
# ---------------------------------------------------------------------------
set(CMAKE_C_FLAGS_INIT
    "${MCU_FLAGS_STR} \
     -DSTM32H7 \
     -DSTM32H745xx \
     -DCORE_CM7 \
     -DUSE_HAL_DRIVER \
     -DUSE_PWR_LDO_SUPPLY \
     -D__int64_t_defined=1 \
     -fdata-sections \
     -ffunction-sections \
     -fno-exceptions \
     -Wall -Wextra \
     -O2 -g"
)
set(CMAKE_CXX_FLAGS_INIT
    "${MCU_FLAGS_STR} \
     -DSTM32H7 \
     -DSTM32H745xx \
     -DCORE_CM7 \
     -DUSE_HAL_DRIVER \
     -DUSE_PWR_LDO_SUPPLY \
     -D__int64_t_defined=1 \
     -fdata-sections \
     -ffunction-sections \
     -fno-exceptions \
     -fno-rtti \
     -std=c++17 \
     -Wall -Wextra \
     -O2 -g"
)

# ---------------------------------------------------------------------------
# Linker flags
# ---------------------------------------------------------------------------
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${MCU_FLAGS_STR} \
     -Wl,--gc-sections \
     -Wl,--print-memory-usage \
     -specs=nano.specs \
     -specs=nosys.specs \
     -T${CMAKE_CURRENT_LIST_DIR}/../AP/Top/Stm32/stm32h745_flash.ld"
)

# ---------------------------------------------------------------------------
# Sysroot & search restrictions
# ---------------------------------------------------------------------------
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# ---------------------------------------------------------------------------
# F Prime platform
# ---------------------------------------------------------------------------
set(FPRIME_PLATFORM stm32h7)

# ---------------------------------------------------------------------------
# FreeRTOS kernel path (override with -DFREERTOS_PATH=...)
# ---------------------------------------------------------------------------
if(NOT DEFINED FREERTOS_PATH)
    set(FREERTOS_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/freertos")
endif()

# STM32 HAL path (override with -DSTM32_HAL_PATH=...)
if(NOT DEFINED STM32_HAL_PATH)
    set(STM32_HAL_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/stm32h7_hal")
endif()

# ---------------------------------------------------------------------------
# Create INTERFACE targets consumed by AP components
# ---------------------------------------------------------------------------

# FreeRTOS kernel target — headers only at toolchain time.
# The actual static library is created in lib/freertos/CMakeLists.txt
# (added via add_subdirectory after project()).
if(NOT TARGET freertos_kernel)
    add_library(freertos_kernel INTERFACE)
    target_include_directories(freertos_kernel INTERFACE
        "${FREERTOS_PATH}/include"
        "${FREERTOS_PATH}/portable/GCC/ARM_CM7/r0p1"
        "${CMAKE_CURRENT_LIST_DIR}/../AP/Top/Stm32"  # FreeRTOSConfig.h lives here
    )
endif()

# STM32H7 HAL target
if(NOT TARGET stm32_hal)
    add_library(stm32_hal INTERFACE)
    target_include_directories(stm32_hal INTERFACE
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Inc"
        "${STM32_HAL_PATH}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
        "${STM32_HAL_PATH}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
        "${STM32_HAL_PATH}/Drivers/CMSIS/Include"
        "${CMAKE_CURRENT_LIST_DIR}/../AP/Top/Stm32"  # stm32h7xx_hal_conf.h lives here
    )
    # HAL sources are compiled as a STATIC library in the root CMakeLists.txt
    # (after project()) to avoid duplicate symbols from INTERFACE propagation.
endif()
