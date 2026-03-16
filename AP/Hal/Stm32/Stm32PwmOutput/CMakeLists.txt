####
# AP/Hal/Stm32/Stm32PwmOutput/CMakeLists.txt
####
set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/Stm32PwmOutput.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/Stm32PwmOutput.cpp"
)
set(MOD_DEPS
    AP_Types
    AP_Ports
    stm32_hal
)
register_fprime_module()
