#ifndef SITL_SITLTOPOLOGY_HPP
#define SITL_SITLTOPOLOGY_HPP

#include <AP/Top/Sitl/SitlTopologyDefs.hpp>

namespace Sitl {

void setupTopology(const TopologyState& state);
void teardownTopology(const TopologyState& state);
void startRateGroups(const Fw::TimeInterval& interval);
void stopRateGroups();

}  // namespace Sitl

#endif
