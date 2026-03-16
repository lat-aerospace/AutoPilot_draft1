// ======================================================================
// \title  AP/Os/FreeRtos/DefaultConsole.cpp
// \brief  Registers FreeRtosConsole as the default Os::Console via linker.
// ======================================================================
#include <Os/Console.hpp>
#include <Os/Delegate.hpp>
#include <AP/Os/FreeRtos/Console.hpp>

namespace Os {

ConsoleInterface* ConsoleInterface::getDelegate(
    ConsoleHandleStorage&    aligned_new_memory,
    const ConsoleInterface*  to_copy)
{
    return Os::Delegate::makeDelegate<
        ConsoleInterface,
        Os::FreeRtos::Console::FreeRtosConsole>(aligned_new_memory, to_copy);
}

}  // namespace Os
