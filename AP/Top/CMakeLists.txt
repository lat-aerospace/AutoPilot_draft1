####
# Deployments
# Sitl  – Linux software-in-the-loop (POSIX / native host)
# Stm32 – STM32H7/F7/F4 hardware with FreeRTOS (cross-compile only)
####

# SITL – only when NOT cross-compiling for STM32
if(NOT DEFINED FPRIME_PLATFORM OR NOT FPRIME_PLATFORM MATCHES "stm32")
    add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Sitl")
    register_fprime_deployment(
        SOURCES
            "${CMAKE_CURRENT_LIST_DIR}/Sitl/Main.cpp"
        DEPENDS
            AP_Top_Sitl
    )
endif()

# STM32 hardware deployment – only built when cross-compiling for STM32
if(DEFINED FPRIME_PLATFORM AND FPRIME_PLATFORM MATCHES "stm32")
    add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Stm32")
    register_fprime_deployment(
        SOURCES
            "${CMAKE_CURRENT_LIST_DIR}/Stm32/Main.cpp"
            "${CMAKE_CURRENT_LIST_DIR}/Stm32/startup_stm32h745xx.s"
        DEPENDS
            AP_Top_Stm32
    )
endif()
