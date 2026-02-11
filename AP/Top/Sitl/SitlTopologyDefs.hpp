#ifndef SITL_SITLTOPOLOGYDEFS_HPP
#define SITL_SITLTOPOLOGYDEFS_HPP

#include "AP/Top/Sitl/FppConstantsAc.hpp"

// Subtopology PingEntries
#include "Svc/Subtopologies/CdhCore/PingEntries.hpp"
#include "Svc/Subtopologies/ComCcsds/PingEntries.hpp"

// SubtopologyTopologyDefs
#include "Svc/Subtopologies/CdhCore/SubtopologyTopologyDefs.hpp"
#include "Svc/Subtopologies/ComCcsds/SubtopologyTopologyDefs.hpp"

// ComCcsds enum includes
#include "Svc/Subtopologies/ComCcsds/Ports_ComBufferQueueEnumAc.hpp"
#include "Svc/Subtopologies/ComCcsds/Ports_ComPacketQueueEnumAc.hpp"

// Ping entries for active components in this topology
namespace PingEntries {
namespace Sitl_rateGroup1Comp {
enum { WARN = 3, FATAL = 5 };
}
namespace Sitl_rateGroup2Comp {
enum { WARN = 3, FATAL = 5 };
}
namespace Sitl_rateGroup3Comp {
enum { WARN = 3, FATAL = 5 };
}
namespace Sitl_rateGroup4Comp {
enum { WARN = 3, FATAL = 5 };
}
}  // namespace PingEntries

namespace Sitl {

struct TopologyState {
    const char* hostname;
    U16 port;
    CdhCore::SubtopologyState cdhCore;
    ComCcsds::SubtopologyState comCcsds;
};

namespace PingEntries = ::PingEntries;

}  // namespace Sitl

#endif
