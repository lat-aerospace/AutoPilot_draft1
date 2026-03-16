// ======================================================================
// \title  AP/Top/Stm32/Stm32TopologyDefs.hpp
// \brief  Definitions required by the F' auto-generated topology
//         (Stm32TopologyAc.hpp / Stm32TopologyAc.cpp)
//
// This file is the STM32 equivalent of SitlTopologyDefs.hpp.
// It must be included before Stm32TopologyAc.hpp.
// ======================================================================
#ifndef STM32_TOPOLOGY_DEFS_HPP
#define STM32_TOPOLOGY_DEFS_HPP

#include "AP/Top/Stm32/FppConstantsAc.hpp"   // generated base-ID constants

// CdhCore subtopology support
#include "Svc/Subtopologies/CdhCore/PingEntries.hpp"
#include "Svc/Subtopologies/CdhCore/SubtopologyTopologyDefs.hpp"

// ---------------------------------------------------------------------------
// Health ping-entry constants for active components in this deployment.
// These are consumed by the CdhCore.health component via the
// "health connections" topology pattern.
// ---------------------------------------------------------------------------
namespace PingEntries {

namespace Stm32_rateGroup1Comp { enum { WARN = 50, FATAL = 200 }; }
namespace Stm32_rateGroup2Comp { enum { WARN = 50, FATAL = 200 }; }
namespace Stm32_rateGroup3Comp { enum { WARN = 50, FATAL = 200 }; }
namespace Stm32_rateGroup4Comp { enum { WARN = 50, FATAL = 200 }; }
namespace Stm32_mavlinkUart    { enum { WARN = 50, FATAL = 200 }; }

}  // namespace PingEntries

// ---------------------------------------------------------------------------
// Stm32 topology state
// Passed through the F' lifecycle functions (initComponents, startTasks, …).
// CdhCore::SubtopologyState is an empty struct; it is included here to
// satisfy the generated code that checks for it.
// ---------------------------------------------------------------------------
namespace Stm32 {

struct TopologyState {
    CdhCore::SubtopologyState cdhCore;  ///< required by CdhCore subtopology
};

namespace PingEntries = ::PingEntries;

}  // namespace Stm32

#endif  // STM32_TOPOLOGY_DEFS_HPP
