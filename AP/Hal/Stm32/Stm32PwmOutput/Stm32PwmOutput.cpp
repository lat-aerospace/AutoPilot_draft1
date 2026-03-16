// ======================================================================
// \title  AP/Hal/Stm32/Stm32PwmOutput/Stm32PwmOutput.cpp
// \brief  STM32 PWM output – 5 channels across 3 timers
//
// Channel mapping:
//   CH0 (Aileron)  → TIM1_CH3 (PA10)
//   CH1 (Elevator) → TIM1_CH4 (PA11)
//   CH2 (Throttle) → TIM3_CH3 (PB0)
//   CH3 (Rudder)   → TIM8_CH1 (PC6)
//   CH4 (Spare)    → TIM8_CH2 (PC7)
// ======================================================================
#include <AP/Hal/Stm32/Stm32PwmOutput/Stm32PwmOutput.hpp>
#include <Fw/Types/Assert.hpp>
#include <algorithm>

namespace Ap {

Stm32PwmOutput::Stm32PwmOutput(const char* const compName)
    : Stm32PwmOutputComponentBase(compName) {}

// ---------------------------------------------------------------------------
// configure – bind 3 TIM handles
// ---------------------------------------------------------------------------
void Stm32PwmOutput::configure(TIM_HandleTypeDef* htim1,
                                TIM_HandleTypeDef* htim3,
                                TIM_HandleTypeDef* htim8) {
    m_htim1 = htim1;
    m_htim3 = htim3;
    m_htim8 = htim8;
}

// ---------------------------------------------------------------------------
// startPwm – start all 5 channels in PWM mode at neutral (1500 µs)
// ---------------------------------------------------------------------------
void Stm32PwmOutput::startPwm() {
    FW_ASSERT(m_htim1 != nullptr);
    FW_ASSERT(m_htim3 != nullptr);
    FW_ASSERT(m_htim8 != nullptr);

    // TIM1 CH3/CH4
    __HAL_TIM_SET_COMPARE(m_htim1, TIM_CHANNEL_3, PWM_MID_US);
    HAL_TIM_PWM_Start(m_htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(m_htim1, TIM_CHANNEL_4, PWM_MID_US);
    HAL_TIM_PWM_Start(m_htim1, TIM_CHANNEL_4);

    // TIM3 CH3
    __HAL_TIM_SET_COMPARE(m_htim3, TIM_CHANNEL_3, PWM_MID_US);
    HAL_TIM_PWM_Start(m_htim3, TIM_CHANNEL_3);

    // TIM8 CH1/CH2
    __HAL_TIM_SET_COMPARE(m_htim8, TIM_CHANNEL_1, PWM_MID_US);
    HAL_TIM_PWM_Start(m_htim8, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(m_htim8, TIM_CHANNEL_2, PWM_MID_US);
    HAL_TIM_PWM_Start(m_htim8, TIM_CHANNEL_2);

    m_started = true;
}

// ---------------------------------------------------------------------------
// disarm – set all channels to minimum (motor off / servo neutral-low)
// ---------------------------------------------------------------------------
void Stm32PwmOutput::disarm() {
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        setChannel(ch, PWM_MIN_US);
    }
}

// ---------------------------------------------------------------------------
// setChannel – set pulse width with clamping, dispatched to correct timer
// ---------------------------------------------------------------------------
void Stm32PwmOutput::setChannel(uint8_t ch, uint32_t pulse_us) {
    if (!m_started) { return; }

    TIM_HandleTypeDef* htim = nullptr;
    uint32_t tim_ch = 0;

    switch (ch) {
        case 0: htim = m_htim1; tim_ch = TIM_CHANNEL_3; break;  // Aileron
        case 1: htim = m_htim1; tim_ch = TIM_CHANNEL_4; break;  // Elevator
        case 2: htim = m_htim3; tim_ch = TIM_CHANNEL_3; break;  // Throttle
        case 3: htim = m_htim8; tim_ch = TIM_CHANNEL_1; break;  // Rudder
        case 4: htim = m_htim8; tim_ch = TIM_CHANNEL_2; break;  // Spare
        default: return;
    }

    if (htim == nullptr) { return; }

    const uint32_t clamped = std::max(PWM_MIN_US,
                             std::min(PWM_MAX_US, pulse_us));
    __HAL_TIM_SET_COMPARE(htim, tim_ch, clamped);
}

// ---------------------------------------------------------------------------
// Mapping helpers
// ---------------------------------------------------------------------------
uint32_t Stm32PwmOutput::normalizedToMicros(F32 normalized) {
    // [-1..+1] → [1000..2000] µs
    const F32 clamped = std::max(-1.0F, std::min(1.0F, normalized));
    return static_cast<uint32_t>(PWM_MID_US + clamped * 500.0F);
}

uint32_t Stm32PwmOutput::throttleToMicros(F32 throttle) {
    // [0..1] → [1000..2000] µs
    const F32 clamped = std::max(0.0F, std::min(1.0F, throttle));
    return static_cast<uint32_t>(PWM_MIN_US + clamped * (PWM_MAX_US - PWM_MIN_US));
}

// ---------------------------------------------------------------------------
// surfaceCmdIn_handler – convert SurfaceCmd to PWM pulses
// ---------------------------------------------------------------------------
void Stm32PwmOutput::surfaceCmdIn_handler(FwIndexType /*portNum*/,
                                           Ap::SurfaceCmd& cmd) {
    if (!m_started) { return; }

    // CH0 = Aileron
    setChannel(0, normalizedToMicros(static_cast<F32>(cmd.get_aileron())));
    // CH1 = Elevator
    setChannel(1, normalizedToMicros(static_cast<F32>(cmd.get_elevator())));
    // CH2 = Throttle
    setChannel(2, throttleToMicros(static_cast<F32>(cmd.get_throttle())));
    // CH3 = Rudder
    setChannel(3, normalizedToMicros(static_cast<F32>(cmd.get_rudder())));
}

}  // namespace Ap
