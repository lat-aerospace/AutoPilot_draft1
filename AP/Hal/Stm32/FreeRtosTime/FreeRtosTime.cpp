// ======================================================================
// \title  AP/Hal/Stm32/FreeRtosTime/FreeRtosTime.cpp
// \brief  FreeRTOS time source implementation
// ======================================================================
#include <AP/Hal/Stm32/FreeRtosTime/FreeRtosTime.hpp>

namespace Ap {

FreeRtosTime::FreeRtosTime(const char* const compName)
    : FreeRtosTimeComponentBase(compName) {}

// ---------------------------------------------------------------------------
// timeGetPort_handler – called by every F' component when it needs the time
// ---------------------------------------------------------------------------
void FreeRtosTime::timeGetPort_handler(FwIndexType /*portNum*/,
                                        Fw::Time& time) {
    const TickType_t ticks = xTaskGetTickCount();

    // Break tick count into whole seconds and sub-second microseconds.
    // configTICK_RATE_HZ ticks per second, each tick = 1/configTICK_RATE_HZ s.
    const U32 seconds      = static_cast<U32>(ticks / configTICK_RATE_HZ);
    const U32 sub_ticks    = static_cast<U32>(ticks % configTICK_RATE_HZ);
    const U32 microseconds = (sub_ticks * 1000000U) / configTICK_RATE_HZ;

    time.set(TimeBase::TB_WORKSTATION_TIME, 0, seconds, microseconds);
}

}  // namespace Ap
