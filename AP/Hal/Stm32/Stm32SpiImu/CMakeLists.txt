####
# AP/Hal/Stm32/Stm32SpiImu/CMakeLists.txt
# SPI IMU driver (ICM-42688-P)
####
set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/Stm32SpiImu.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/Stm32SpiImu.cpp"
)
set(MOD_DEPS
    AP_Types
    AP_Ports
    stm32_hal
    freertos_kernel
)
register_fprime_module()
