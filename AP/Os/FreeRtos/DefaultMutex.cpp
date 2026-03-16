// ======================================================================
// \title  AP/Os/FreeRtos/DefaultMutex.cpp
// \brief  Registers FreeRtos Mutex and ConditionVariable as the default
//         Os::Mutex / Os::ConditionVariable implementation via linker.
// ======================================================================
#include <Os/Delegate.hpp>
#include <Os/Mutex.hpp>
#include <Os/Condition.hpp>
#include <AP/Os/FreeRtos/Mutex.hpp>
#include <AP/Os/FreeRtos/ConditionVariable.hpp>

namespace Os {

MutexInterface* MutexInterface::getDelegate(MutexHandleStorage& aligned_new_memory) {
    return Os::Delegate::makeDelegate<
        MutexInterface,
        Os::FreeRtos::Mutex::FreeRtosMutex>(aligned_new_memory);
}

ConditionVariableInterface* ConditionVariableInterface::getDelegate(
    ConditionVariableHandleStorage& aligned_new_memory) {
    return Os::Delegate::makeDelegate<
        ConditionVariableInterface,
        Os::FreeRtos::Mutex::FreeRtosConditionVariable,
        ConditionVariableHandleStorage>(aligned_new_memory);
}

}  // namespace Os
