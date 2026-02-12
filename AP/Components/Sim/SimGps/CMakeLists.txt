set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/SimGps.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/SimGps.cpp"
)

set(MOD_DEPS
    AP_Types
    AP_Ports
)

register_fprime_module()
# Eigen + ApMath headers (for CoordTransforms)
target_link_libraries(${MODULE_NAME} PUBLIC Ap_Math)
