// ======================================================================
// \title  AP/Os/FreeRtos/DefaultTask.cpp
// \brief  Registers FreeRtosTask as the default Os::Task implementation
//         via the F' linker-delegation mechanism.
// ======================================================================
#include <Os/Delegate.hpp>
#include <Os/Task.hpp>
#include <AP/Os/FreeRtos/Task.hpp>

namespace Os {

TaskInterface* TaskInterface::getDelegate(TaskHandleStorage& aligned_new_memory) {
    return Os::Delegate::makeDelegate<
        TaskInterface,
        Os::FreeRtos::Task::FreeRtosTask>(aligned_new_memory);
}

}  // namespace Os
