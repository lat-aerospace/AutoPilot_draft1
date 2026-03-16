####
# cmake/platform/stm32f4.cmake
# F Prime platform file for STM32F4 – same OS selections as stm32h7
# See stm32h7.cmake for detailed comments.
####

add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/stm32/Platform/")
target_compile_definitions(Stm32PlatformTypes INTERFACE -DTGT_OS_TYPE_FREERTOS)
