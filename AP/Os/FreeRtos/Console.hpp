// ======================================================================
// \title  AP/Os/FreeRtos/Console.hpp
// \brief  FreeRTOS implementation of Os::ConsoleInterface
//
// Debug output is sent via ITM Stimulus Port 0 (SWO / SWV) on ARM Cortex-M.
// This allows non-blocking text output when a debugger / SWO probe is
// attached and incurs zero overhead (FIFO full check only) when it is not.
//
// For production targets or when no ITM is available, the implementation
// falls back to writing to USART3 via the weak symbol
// AP_ConsolePutChar() which users may override.
// ======================================================================
#ifndef AP_OS_FREERTOS_CONSOLE_HPP
#define AP_OS_FREERTOS_CONSOLE_HPP

#include <Os/Console.hpp>

namespace Os {
namespace FreeRtos {
namespace Console {

struct FreeRtosConsoleHandle : public ConsoleHandle {};

class FreeRtosConsole final : public ConsoleInterface {
  public:
    FreeRtosConsole()  = default;
    ~FreeRtosConsole() override = default;

    FreeRtosConsole(const FreeRtosConsole&)             = default;
    FreeRtosConsole& operator=(const FreeRtosConsole&) = delete;

    //! Write a null-terminated string to the debug console
    void writeMessage(const CHAR* message, const FwSizeType size) override;

    ConsoleHandle* getHandle() override;

  private:
    FreeRtosConsoleHandle m_handle;
};

}  // namespace Console
}  // namespace FreeRtos
}  // namespace Os

#endif  // AP_OS_FREERTOS_CONSOLE_HPP
