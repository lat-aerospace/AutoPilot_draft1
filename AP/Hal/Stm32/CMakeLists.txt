####
# AP/Hal/Stm32/CMakeLists.txt
# STM32 hardware abstraction layer components.
# These are only compiled when targeting an STM32 platform.
####

add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/FreeRtosTime")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Stm32Timer")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Stm32SpiImu")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Stm32Uart")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Stm32I2cBaro")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Stm32GpsUart")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Stm32PwmOutput")
