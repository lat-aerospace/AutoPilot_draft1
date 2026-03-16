// ======================================================================
// \title  AP/Hal/Stm32/Stm32Timer/Stm32Timer.cpp
// \brief  STM32 TIM6 → FreeRTOS task notification → F' CycleOut
// ======================================================================
#include <AP/Hal/Stm32/Stm32Timer/Stm32Timer.hpp>
#include <Fw/Types/Assert.hpp>
#include <Os/RawTime.hpp>

static TIM_HandleTypeDef       s_tim6_handle;
static volatile TaskHandle_t   s_task_handle = nullptr;

// ---------------------------------------------------------------------------
// TIM6 ISR – one line, everything else in the task
// ---------------------------------------------------------------------------
extern "C" void TIM6_DAC_IRQHandler(void) {
    BaseType_t higher_prio = pdFALSE;
    if (__HAL_TIM_GET_FLAG(&s_tim6_handle, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&s_tim6_handle, TIM_IT_UPDATE);
        if (s_task_handle != nullptr) {
            vTaskNotifyGiveFromISR(s_task_handle, &higher_prio);
        }
    }
    portYIELD_FROM_ISR(higher_prio);
}

namespace Ap {

Stm32Timer::Stm32Timer(const char* const compName)
    : Stm32TimerComponentBase(compName) {}

Stm32Timer::~Stm32Timer() { stopTimer(); }

void Stm32Timer::configure(U32 rate_hz) { m_rate_hz = rate_hz; }

// ---------------------------------------------------------------------------
// initTim6
// ---------------------------------------------------------------------------
void Stm32Timer::initTim6(U32 rate_hz) {
    __HAL_RCC_TIM6_CLK_ENABLE();

    // AP_Tim6_ApbClockHz is a weak symbol; override in your BSP for accuracy.
    extern uint32_t AP_Tim6_ApbClockHz __attribute__((weak));
    const uint32_t apb_clk = (&AP_Tim6_ApbClockHz != nullptr)
                                  ? AP_Tim6_ApbClockHz
                                  : 120000000U;

    const uint32_t psc    = (apb_clk / (65535U * rate_hz));
    const uint32_t period = (apb_clk / ((psc + 1U) * rate_hz)) - 1U;

    s_tim6_handle.Instance               = TIM6;
    s_tim6_handle.Init.Prescaler         = static_cast<uint16_t>(psc);
    s_tim6_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_tim6_handle.Init.Period            = static_cast<uint16_t>(period);
    s_tim6_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_tim6_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_StatusTypeDef r = HAL_TIM_Base_Init(&s_tim6_handle);
    FW_ASSERT(r == HAL_OK, static_cast<FwAssertArgType>(r));

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    r = HAL_TIM_Base_Start_IT(&s_tim6_handle);
    FW_ASSERT(r == HAL_OK, static_cast<FwAssertArgType>(r));
}

// ---------------------------------------------------------------------------
// FreeRTOS driver task
// ---------------------------------------------------------------------------
void Stm32Timer::timerTask(void* pvParameters) {
    auto* self    = static_cast<Stm32Timer*>(pvParameters);
    s_task_handle = xTaskGetCurrentTaskHandle();

    // Start TIM6 hardware AFTER s_task_handle is set and scheduler is running,
    // so ISR's vTaskNotifyGiveFromISR has a valid target.
    self->initTim6(self->m_rate_hz);

    while (self->m_running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Os::RawTime rawTime;
        rawTime.now();
        self->CycleOut_out(0, rawTime);
        self->m_cycle_count++;
    }

    s_task_handle = nullptr;
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// startTimer / stopTimer
// ---------------------------------------------------------------------------
void Stm32Timer::startTimer() {
    m_running = true;
    // TIM6 hardware init is deferred to timerTask() so the ISR has a
    // valid task handle for vTaskNotifyGiveFromISR.

    const BaseType_t r = xTaskCreate(
        timerTask,
        "TimerDrv",
        512U,
        this,
        static_cast<UBaseType_t>(configMAX_PRIORITIES - 1),
        &m_task);
    FW_ASSERT(r == pdPASS, static_cast<FwAssertArgType>(r));
}

void Stm32Timer::stopTimer() {
    m_running = false;
    HAL_TIM_Base_Stop_IT(&s_tim6_handle);
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    if (m_task != nullptr) {
        xTaskNotifyGive(m_task);  // unblock so it sees m_running == false
        m_task = nullptr;
    }
}

void Stm32Timer::notifyFromISR() {
    BaseType_t hp = pdFALSE;
    if (s_task_handle != nullptr) {
        vTaskNotifyGiveFromISR(s_task_handle, &hp);
    }
    portYIELD_FROM_ISR(hp);
}

}  // namespace Ap
