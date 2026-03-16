####
# AP/Hal/Stm32/Stm32Timer/CMakeLists.txt
# Hardware timer component (TIM6 -> 400 Hz base tick)
####
set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/Stm32Timer.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/Stm32Timer.cpp"
)
set(MOD_DEPS
    AP_Types
    AP_Ports
    Os
    stm32_hal
    freertos_kernel
)
register_fprime_module()
