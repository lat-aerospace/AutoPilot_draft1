register_fprime_library(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/SimDynamics.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/SimDynamics.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/RigidBody6DOF.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/AtmosphereISA.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/AeroModel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PropulsionModel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/WindModel.cpp"
    DEPENDS
        AP_Types
        AP_Ports
)
# Eigen + ApMath headers
target_link_libraries(${MODULE_NAME} PUBLIC Ap_Math)
