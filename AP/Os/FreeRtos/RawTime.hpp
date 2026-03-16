// ======================================================================
// \title  AP/Os/FreeRtos/RawTime.hpp
// \brief  FreeRTOS implementation of Os::RawTimeInterface
//
// Time is stored as a (ticks, cycle_count) pair where:
//   ticks       = xTaskGetTickCount()  [1 tick = 1/configTICK_RATE_HZ s]
//   cycle_count = DWT->CYCCNT          [ARM Cortex-M cycle counter]
//
// This gives sub-millisecond resolution on ARM Cortex-M3/M4/M7 targets.
// DWT must be enabled before use (see HalInit.cpp).
//
// Serialization: two U32 values (ticks, cycle_count), 8 bytes total.
// Ensure FW_RAW_TIME_SERIALIZATION_MAX_SIZE >= 8 in config/FpConfig.h.
// ======================================================================
#ifndef AP_OS_FREERTOS_RAWTIME_HPP
#define AP_OS_FREERTOS_RAWTIME_HPP

#include <Os/RawTime.hpp>
#include <FreeRTOS.h>
#include <task.h>

namespace Os {
namespace FreeRtos {
namespace RawTime {

struct FreeRtosRawTimeHandle : public RawTimeHandle {
    TickType_t m_ticks       = 0;   ///< FreeRTOS system tick count
    U32        m_cycle_count = 0;   ///< DWT cycle counter snapshot
};

class FreeRtosRawTime final : public RawTimeInterface {
  public:
    FreeRtosRawTime()  = default;
    ~FreeRtosRawTime() override = default;

    RawTimeHandle* getHandle() override;

    //! Capture the current time into this object
    Status now() override;

    //! Calculate the time interval between \a other and this time
    Status getTimeInterval(const Os::RawTime& other,
                           Fw::TimeInterval& interval) const override;

    //! Serialize (ticks:U32, cycle_count:U32)
    Fw::SerializeStatus serializeTo(
        Fw::SerialBufferBase& buffer,
        Fw::Endianness mode = Fw::Endianness::BIG) const override;

    //! Deserialize (ticks:U32, cycle_count:U32)
    Fw::SerializeStatus deserializeFrom(
        Fw::SerialBufferBase& buffer,
        Fw::Endianness mode = Fw::Endianness::BIG) override;

  private:
    FreeRtosRawTimeHandle m_handle;
};

}  // namespace RawTime
}  // namespace FreeRtos
}  // namespace Os

#endif  // AP_OS_FREERTOS_RAWTIME_HPP
