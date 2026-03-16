// ======================================================================
// \title  AP/Os/FreeRtos/Console.cpp
// \brief  FreeRTOS implementation of Os::ConsoleInterface
//
// Primary output: ITM Stimulus Port 0 (ARM SWO / SWV).
// Fallback:       Weak symbol AP_ConsolePutChar() that the BSP can override
//                 to redirect output to a UART or RTT buffer.
// ======================================================================
#include <AP/Os/FreeRtos/Console.hpp>
#include <FreeRTOS.h>
#include <Fw/FPrimeBasicTypes.hpp>

// ---------------------------------------------------------------------------
// ITM register definitions (ARM Cortex-M3/M4/M7)
// ---------------------------------------------------------------------------
#ifndef ITM_PORT0_U32
#define ITM_PORT0_U32  (*((volatile uint32_t*)0xE0000000U))
#define ITM_TER        (*((volatile uint32_t*)0xE0000E00U))
#define ITM_TCR        (*((volatile uint32_t*)0xE0000E80U))
#define ITM_TCR_ITMENA 1U
#endif

// ---------------------------------------------------------------------------
// Weak fallback – override in BSP to redirect to UART / RTT
// ---------------------------------------------------------------------------
extern "C" __attribute__((weak)) void AP_ConsolePutChar(char c) {
    // ITM SWO output path (non-blocking, skips if debugger not connected)
    if ((ITM_TCR & ITM_TCR_ITMENA) != 0U) {
        if ((ITM_TER & 1U) != 0U) {
            // Wait briefly for ITM FIFO but don't spin forever
            for (volatile uint32_t i = 0; i < 1000U; i++) {
                if (ITM_PORT0_U32 != 0U) {
                    ITM_PORT0_U32 = static_cast<uint8_t>(c);
                    return;
                }
            }
        }
    }
    // If ITM unavailable, silently drop (prevents hard faults in release build)
}

// ---------------------------------------------------------------------------
// FreeRtosConsole implementation
// ---------------------------------------------------------------------------
namespace Os {
namespace FreeRtos {
namespace Console {

void FreeRtosConsole::writeMessage(const CHAR* message, const FwSizeType size) {
    if (message == nullptr) { return; }
    for (FwSizeType i = 0; i < size; i++) {
        AP_ConsolePutChar(message[i]);
    }
}

ConsoleHandle* FreeRtosConsole::getHandle() {
    return &m_handle;
}

}  // namespace Console
}  // namespace FreeRtos
}  // namespace Os
