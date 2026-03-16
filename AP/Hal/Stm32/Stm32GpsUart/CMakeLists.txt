####
# AP/Hal/Stm32/Stm32GpsUart/CMakeLists.txt
# UBLOX UBX GPS UART driver
####
set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/Stm32GpsUart.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/Stm32GpsUart.cpp"
)
set(MOD_DEPS
    AP_Types
    AP_Ports
    stm32_hal
    freertos_kernel
)
register_fprime_module()
