####
# SITL Topology module (autocoded from FPP)
####

add_compile_options(-Wno-shadow)

set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/instances.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/topology.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/SitlTopology.cpp"
)
set(MOD_DEPS
    Fw/Logger
    Svc/PosixTime
    Svc/LinuxTimer
)
register_fprime_module()
