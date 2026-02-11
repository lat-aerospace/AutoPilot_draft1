####
# This sets up the build system for the 'AP' project, including
# components and deployments from project.cmake. In addition, it imports the core F Prime components.
####

cmake_minimum_required(VERSION 3.24.2)
project(AP C CXX)

###
# F' Core Setup
# This includes all of the F prime core components, and imports the make-system.
###
include("${CMAKE_CURRENT_LIST_DIR}/lib/fprime/cmake/FPrime.cmake")
# NOTE: register custom targets between these two lines

# Eigen — header-only linear algebra library
add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE "${CMAKE_CURRENT_LIST_DIR}/lib/eigen")

fprime_setup_included_code()

# Project configuration overrides (e.g. RateGroupDriverRateGroupPorts = 4)
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/config")


# This includes project-wide objects
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/AP")
