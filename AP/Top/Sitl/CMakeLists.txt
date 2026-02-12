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
    AP/Components/Sim/SimDynamics
    AP/Components/Sim/SimServoDriver
    AP/Components/Sim/SimImu
    AP/Components/Sim/SimGps
    AP/Components/Sim/SimBaro
    AP/Components/Sim/SimMag
    AP/Components/FlightControl/StateEstimator
    AP/Components/FlightControl/Autonomy
    AP/Components/FlightControl/Controller
    AP/Components/Mavlink/MavlinkGateway
)
register_fprime_module()
