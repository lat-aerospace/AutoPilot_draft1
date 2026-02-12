set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/SimMag.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/SimMag.cpp"
)

set(MOD_DEPS
    AP_Types
    AP_Ports
)

register_fprime_module()
# Eigen + ApMath headers (for quaternion rotation)
target_link_libraries(${MODULE_NAME} PUBLIC Ap_Math)
