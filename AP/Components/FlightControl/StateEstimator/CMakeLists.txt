set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/StateEstimator.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/StateEstimator.cpp"
)
set(MOD_DEPS
    AP_Types
    AP_Ports
)
register_fprime_module()

# Eigen + ApMath headers
target_link_libraries(${MODULE_NAME} PUBLIC Ap_Math)
