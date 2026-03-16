####
# AP/Top/Stm32/CMakeLists.txt
# STM32 hardware deployment topology
####

add_compile_options(-Wno-shadow)

set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/instances.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/topology.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/Stm32Topology.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/bsp.c"
    "${CMAKE_CURRENT_LIST_DIR}/stm32h7xx_it.c"
    "${CMAKE_CURRENT_LIST_DIR}/stm32h7xx_hal_msp.c"
    "${CMAKE_CURRENT_LIST_DIR}/../../Os/FreeRtos/FreeRtosMemAllocator.cpp"
)
set(MOD_DEPS
    Fw/Logger
    AP/Hal/Stm32/FreeRtosTime
    AP/Hal/Stm32/Stm32Timer
    AP/Hal/Stm32/Stm32SpiImu
    AP/Hal/Stm32/Stm32Uart
    AP/Hal/Stm32/Stm32I2cBaro
    AP/Hal/Stm32/Stm32GpsUart
    AP/Hal/Stm32/Stm32PwmOutput
    AP/Components/FlightControl/StateEstimator
    AP/Components/FlightControl/Autonomy
    AP/Components/FlightControl/Controller
    stm32_hal
    freertos_kernel
)
register_fprime_module()
