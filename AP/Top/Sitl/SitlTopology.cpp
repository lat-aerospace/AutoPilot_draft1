// ======================================================================
// \title  SitlTopology.cpp
// \brief  SITL deployment topology setup / teardown
// ======================================================================

#include <AP/Top/Sitl/SitlTopologyAc.hpp>
#include <Fw/Types/MallocAllocator.hpp>
#include <AP/Components/Mavlink/MavlinkGateway/MavlinkGateway.hpp>

using namespace Sitl;

// ---------------------------------------------------------------------------
// Rate group driver divisors
// ---------------------------------------------------------------------------
// Base tick = 400 Hz (2.5 ms).  Divisors produce:
//   RG1: 400 Hz  (÷1)
//   RG2: 100 Hz  (÷4)
//   RG3:  50 Hz  (÷8)
//   RG4:  10 Hz  (÷40)
Svc::RateGroupDriver::DividerSet rateGroupDivisorsSet{
    {{1, 0}, {4, 0}, {8, 0}, {40, 0}}
};

// Context arrays — unused, but required by ActiveRateGroup::configure()
U32 rateGroup1Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};
U32 rateGroup2Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};
U32 rateGroup3Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};
U32 rateGroup4Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};

enum TopologyConstants {
    COMM_PRIORITY = 34,
};

// ---------------------------------------------------------------------------
// Project-specific configuration
// ---------------------------------------------------------------------------
void configureTopology() {
    rateGroupDriverComp.configure(rateGroupDivisorsSet);

    rateGroup1Comp.configure(rateGroup1Context, FW_NUM_ARRAY_ELEMENTS(rateGroup1Context));
    rateGroup2Comp.configure(rateGroup2Context, FW_NUM_ARRAY_ELEMENTS(rateGroup2Context));
    rateGroup3Comp.configure(rateGroup3Context, FW_NUM_ARRAY_ELEMENTS(rateGroup3Context));
    rateGroup4Comp.configure(rateGroup4Context, FW_NUM_ARRAY_ELEMENTS(rateGroup4Context));
}

// ---------------------------------------------------------------------------
// Public API (called from Main.cpp)
// ---------------------------------------------------------------------------
namespace Sitl {

void setupTopology(const TopologyState& state) {
    initComponents(state);
    setBaseIds();
    connectComponents();
    regCommands();
    configComponents(state);

    if (state.hostname != nullptr && state.port != 0) {
        comDriver.configure(state.hostname, state.port);
    }

    configureTopology();

    // Open MAVLink UDP socket
    // Send telemetry to GCS (default 127.0.0.1:14550), receive on 0.0.0.0:14540
    const char* mavIp = (state.hostname != nullptr) ? state.hostname : "127.0.0.1";
    mavlinkGateway.configure(mavIp, 14550, 14540);

    loadParameters();
    startTasks(state);

    if (state.hostname != nullptr && state.port != 0) {
        Os::TaskString name("ReceiveTask");
        comDriver.start(name, COMM_PRIORITY, Default::STACK_SIZE);
    }
}

void startRateGroups(const Fw::TimeInterval& interval) {
    linuxTimer.startTimer(interval);
}

void stopRateGroups() {
    linuxTimer.quit();
}

void teardownTopology(const TopologyState& state) {
    stopTasks(state);
    freeThreads(state);

    comDriver.stop();
    (void)comDriver.join();

    tearDownComponents(state);
}

}  // namespace Sitl
