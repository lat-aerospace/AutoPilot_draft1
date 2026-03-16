// ======================================================================
// \title  AP/Hal/Stm32/Stm32PwmOutput/Stm32PwmOutput.hpp
// \brief  STM32 PWM output – 5 channels across 3 timers for servos / ESCs
//
// PWM specification:
//   Frequency:   50 Hz (20 ms period)   [standard RC servo]
//   Pulse width: 1000 µs (min) to 2000 µs (max)
//   Neutral:     1500 µs
//
// Channel mapping:
//   CH0 (Aileron)  → TIM1_CH3 (PA10)
//   CH1 (Elevator) → TIM1_CH4 (PA11)
//   CH2 (Throttle) → TIM3_CH3 (PB0)
//   CH3 (Rudder)   → TIM8_CH1 (PC6)
//   CH4 (Spare)    → TIM8_CH2 (PC7)
//
// All timers: prescaler=199, period=19999 → 1 MHz tick, 50 Hz.
// CCR values map directly to µs.
// ======================================================================
#ifndef AP_HAL_STM32_PWM_OUTPUT_HPP
#define AP_HAL_STM32_PWM_OUTPUT_HPP

#include <stm32h7xx_hal.h>
#include <AP/Types/ApTypesSerializableAc.hpp>
#include <AP/Hal/Stm32/Stm32PwmOutput/Stm32PwmOutputComponentAc.hpp>

namespace Ap {

class Stm32PwmOutput final : public Stm32PwmOutputComponentBase {
  public:
    static constexpr uint32_t PWM_MIN_US  = 1000U;
    static constexpr uint32_t PWM_MAX_US  = 2000U;
    static constexpr uint32_t PWM_MID_US  = 1500U;
    static constexpr uint8_t  NUM_CHANNELS = 5U;

    explicit Stm32PwmOutput(const char* const compName);
    ~Stm32PwmOutput() override = default;

    //! Bind 3 TIM handles for all PWM channels
    void configure(TIM_HandleTypeDef* htim1,
                   TIM_HandleTypeDef* htim3,
                   TIM_HandleTypeDef* htim8);

    //! Start all PWM channels at neutral position
    void startPwm();

    //! Disarm all channels (set to PWM_MIN_US – failsafe)
    void disarm();

  private:
    void surfaceCmdIn_handler(FwIndexType portNum,
                              Ap::SurfaceCmd& cmd) override;

    //! Set a channel pulse width [µs], clamped to [PWM_MIN_US, PWM_MAX_US]
    void setChannel(uint8_t ch, uint32_t pulse_us);

    //! Map [-1..+1] to [1000..2000] µs
    static uint32_t normalizedToMicros(F32 normalized);

    //! Map [0..1] to [1000..2000] µs (for throttle)
    static uint32_t throttleToMicros(F32 throttle);

    TIM_HandleTypeDef* m_htim1    = nullptr;
    TIM_HandleTypeDef* m_htim3    = nullptr;
    TIM_HandleTypeDef* m_htim8    = nullptr;
    bool               m_started  = false;
};

}  // namespace Ap

#endif  // AP_HAL_STM32_PWM_OUTPUT_HPP
