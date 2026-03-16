// ======================================================================
// \title  AP/Hal/Stm32/FreeRtosTime/FreeRtosTime.hpp
// \brief  FreeRTOS time source for F' components on STM32 hardware
// ======================================================================
#ifndef AP_HAL_FREERTOS_TIME_HPP
#define AP_HAL_FREERTOS_TIME_HPP

#include <FreeRTOS.h>
#include <task.h>
#include <Fw/Time/Time.hpp>
#include <AP/Hal/Stm32/FreeRtosTime/FreeRtosTimeComponentAc.hpp>

namespace Ap {

class FreeRtosTime final : public FreeRtosTimeComponentBase {
  public:
    explicit FreeRtosTime(const char* const compName);
    ~FreeRtosTime() override = default;

  private:
    //! Handler for the timeGetPort — returns Fw::Time from FreeRTOS tick count
    void timeGetPort_handler(FwIndexType portNum, Fw::Time& time) override;
};

}  // namespace Ap

#endif  // AP_HAL_FREERTOS_TIME_HPP
