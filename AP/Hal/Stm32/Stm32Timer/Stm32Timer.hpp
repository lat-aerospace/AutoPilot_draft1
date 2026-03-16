// ======================================================================
// \title  AP/Hal/Stm32/Stm32Timer/Stm32Timer.hpp
// \brief  STM32 TIM6 hardware timer – 400 Hz base-tick for F' rate groups
//
// Architecture (mirrors Svc.LinuxTimer pattern):
//   1. configure(rate_hz) stores the desired tick rate.
//   2. startTimer() initialises TIM6, creates a high-priority FreeRTOS task.
//   3. The TIM6_DAC_IRQHandler gives a notification to the task via
//      vTaskNotifyGiveFromISR().
//   4. The task wakes, builds a Svc::TimerVal, and calls CycleOut_out().
//   5. CycleOut_out() calls RateGroupDriver.CycleIn synchronously, which
//      dispatches the rate-group message queue.
//   6. stopTimer() disables the ISR and signals the task to exit.
// ======================================================================
#ifndef AP_HAL_STM32_TIMER_HPP
#define AP_HAL_STM32_TIMER_HPP

#include <FreeRTOS.h>
#include <task.h>
#include <stm32h7xx_hal.h>
#include <AP/Hal/Stm32/Stm32Timer/Stm32TimerComponentAc.hpp>

namespace Ap {

class Stm32Timer final : public Stm32TimerComponentBase {
  public:
    explicit Stm32Timer(const char* const compName);
    ~Stm32Timer() override;

    //! Store desired tick rate (call during topology setup)
    void configure(U32 rate_hz = 400U);

    //! Configure TIM6 + create FreeRTOS driver task (call after startTasks())
    void startTimer();

    //! Disable TIM6 ISR and stop the driver task (call during teardown)
    void stopTimer();

    //! Called from TIM6_DAC_IRQHandler – delivers notification to driver task
    void notifyFromISR();

  private:
    static void timerTask(void* pvParameters);
    void        initTim6(U32 rate_hz);

    U32           m_rate_hz     = 400U;
    TaskHandle_t  m_task        = nullptr;
    volatile bool m_running     = false;
    U32           m_cycle_count = 0U;
};

}  // namespace Ap

#endif  // AP_HAL_STM32_TIMER_HPP
