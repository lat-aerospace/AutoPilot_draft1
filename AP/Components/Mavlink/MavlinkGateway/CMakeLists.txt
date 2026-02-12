set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/MavlinkGateway.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/MavlinkGateway.cpp"
)
set(MOD_DEPS
    AP_Types
    AP_Ports
)
register_fprime_module()

# MAVLink + ApMath headers
target_link_libraries(${MODULE_NAME} PUBLIC mavlink Ap_Math)
