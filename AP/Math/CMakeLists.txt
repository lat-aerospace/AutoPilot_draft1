# Header-only math utilities — plain CMake INTERFACE library
add_library(Ap_Math INTERFACE)
target_include_directories(Ap_Math INTERFACE "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(Ap_Math INTERFACE eigen)
