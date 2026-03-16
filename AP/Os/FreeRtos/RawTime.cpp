// ======================================================================
// \title  AP/Os/FreeRtos/RawTime.cpp
// \brief  FreeRTOS implementation of Os::RawTimeInterface
// ======================================================================
#include <AP/Os/FreeRtos/RawTime.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Types/Serializable.hpp>

// DWT cycle counter register (ARM Cortex-M3/M4/M7)
#ifndef DWT_CYCCNT
#define DWT_CYCCNT (*((volatile uint32_t*)0xE0001004U))
#endif

namespace Os {
namespace FreeRtos {
namespace RawTime {

// ---------------------------------------------------------------------------
// now – capture current FreeRTOS tick and DWT cycle counter
// ---------------------------------------------------------------------------
RawTimeInterface::Status FreeRtosRawTime::now() {
    // Read tick count and cycle counter as close together as possible.
    // On Cortex-M these two reads are effectively atomic for our purposes.
    m_handle.m_ticks       = xTaskGetTickCount();
    m_handle.m_cycle_count = DWT_CYCCNT;
    return RawTimeInterface::Status::OP_OK;
}

// ---------------------------------------------------------------------------
// getTimeInterval – difference between (this) and (other) timestamps
//
// Result = other - this  (other is the later timestamp).
// Uses ticks for coarse resolution and cycle counts for sub-tick precision.
// ---------------------------------------------------------------------------
RawTimeInterface::Status FreeRtosRawTime::getTimeInterval(
    const Os::RawTime& other_time,
    Fw::TimeInterval&  interval) const
{
    const FreeRtosRawTimeHandle* other =
        reinterpret_cast<const FreeRtosRawTimeHandle*>(
            const_cast<Os::RawTime&>(other_time).getHandle());
    FW_ASSERT(other != nullptr);

    // Tick difference (handles 32-bit wraparound correctly via unsigned subtraction)
    const TickType_t tick_diff = other->m_ticks - m_handle.m_ticks;

    // Each tick = 1 / configTICK_RATE_HZ seconds = 1 000 000 / configTICK_RATE_HZ µs
    constexpr U32 US_PER_TICK = 1000000U / configTICK_RATE_HZ;
    U64 total_us = static_cast<U64>(tick_diff) * US_PER_TICK;

    // Add sub-tick precision from cycle counter
    const U32 cycle_diff = other->m_cycle_count - m_handle.m_cycle_count;
    // cycles_to_us = cycle_diff / (configCPU_CLOCK_HZ / 1 000 000)
    const U32 cycles_per_us = configCPU_CLOCK_HZ / 1000000U;
    if (cycles_per_us > 0) {
        total_us += static_cast<U64>(cycle_diff / cycles_per_us);
    }

    const U32 seconds = static_cast<U32>(total_us / 1000000ULL);
    const U32 us_rem  = static_cast<U32>(total_us % 1000000ULL);
    interval = Fw::TimeInterval(seconds, us_rem);

    return RawTimeInterface::Status::OP_OK;
}

// ---------------------------------------------------------------------------
// serializeTo – write ticks (U32) + cycle_count (U32)  [8 bytes]
// ---------------------------------------------------------------------------
Fw::SerializeStatus FreeRtosRawTime::serializeTo(Fw::SerialBufferBase& buffer,
                                                  Fw::Endianness /*mode*/) const {
    Fw::SerializeStatus status;
    status = buffer.serializeFrom(static_cast<U32>(m_handle.m_ticks));
    if (status != Fw::FW_SERIALIZE_OK) { return status; }
    status = buffer.serializeFrom(m_handle.m_cycle_count);
    return status;
}

// ---------------------------------------------------------------------------
// deserializeFrom – read ticks (U32) + cycle_count (U32)
// ---------------------------------------------------------------------------
Fw::SerializeStatus FreeRtosRawTime::deserializeFrom(Fw::SerialBufferBase& buffer,
                                                      Fw::Endianness /*mode*/) {
    U32 ticks       = 0;
    U32 cycle_count = 0;
    Fw::SerializeStatus status;
    status = buffer.deserializeTo(ticks);
    if (status != Fw::FW_SERIALIZE_OK) { return status; }
    status = buffer.deserializeTo(cycle_count);
    if (status == Fw::FW_SERIALIZE_OK) {
        m_handle.m_ticks       = static_cast<TickType_t>(ticks);
        m_handle.m_cycle_count = cycle_count;
    }
    return status;
}

// ---------------------------------------------------------------------------
// getHandle
// ---------------------------------------------------------------------------
RawTimeHandle* FreeRtosRawTime::getHandle() {
    return &m_handle;
}

}  // namespace RawTime
}  // namespace FreeRtos
}  // namespace Os
