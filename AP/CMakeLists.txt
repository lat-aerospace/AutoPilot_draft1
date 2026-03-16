# This CMake file is intended to register project-wide objects.
# This allows for reuse between deployments, or other projects.

add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Types")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Ports")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Math")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Components")

# FreeRTOS OS abstraction (only registered when building for embedded targets)
if(DEFINED FPRIME_PLATFORM AND FPRIME_PLATFORM MATCHES "stm32")
    add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Os/FreeRtos")
    add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Hal/Stm32")
endif()

add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Top")
