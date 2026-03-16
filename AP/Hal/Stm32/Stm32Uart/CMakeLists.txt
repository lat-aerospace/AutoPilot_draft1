####
# AP/Hal/Stm32/Stm32Uart/CMakeLists.txt
# UART MAVLink / GPS driver
####
set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/Stm32Uart.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/Stm32Uart.cpp"
)
set(MOD_DEPS
    AP_Types
    AP_Ports
    stm32_hal
    freertos_kernel
)
register_fprime_module()
target_link_libraries(${FPRIME_CURRENT_MODULE} PUBLIC mavlink)
