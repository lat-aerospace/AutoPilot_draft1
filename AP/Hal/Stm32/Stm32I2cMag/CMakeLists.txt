####
# AP/Hal/Stm32/Stm32I2cMag/CMakeLists.txt
# IST8310 I2C magnetometer driver
####
set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/Stm32I2cMag.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/Stm32I2cMag.cpp"
)
set(MOD_DEPS
    AP_Types
    AP_Ports
    stm32_hal
    freertos_kernel
)
register_fprime_module()
