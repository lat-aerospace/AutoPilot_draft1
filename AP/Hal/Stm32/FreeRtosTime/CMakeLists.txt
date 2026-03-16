####
# AP/Hal/Stm32/FreeRtosTime/CMakeLists.txt
####
set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/FreeRtosTime.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/FreeRtosTime.cpp"
)
set(MOD_DEPS
    Fw_Time
    freertos_kernel
)
register_fprime_module()
