set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/SimBaro.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/SimBaro.cpp"
)

set(MOD_DEPS
    AP_Types
    AP_Ports
)

register_fprime_module()
