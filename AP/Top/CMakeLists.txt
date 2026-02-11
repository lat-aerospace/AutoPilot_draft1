####
# SITL Deployment
# Top/Sitl/ provides the topology module; this file builds the executable.
####

add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Sitl")

register_fprime_deployment(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/Sitl/Main.cpp"
    DEPENDS
        AP_Top_Sitl
)
