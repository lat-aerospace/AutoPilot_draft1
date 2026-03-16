// ======================================================================
// \title  AP/Os/FreeRtos/ConditionVariable.hpp
// \brief  FreeRTOS implementation of Os::ConditionVariableInterface
//
// Condition variables are implemented using FreeRTOS task notifications:
//
//   pend(mutex):
//     1. Register the calling task handle in the waiter list (critical section).
//     2. Release the associated mutex.
//     3. Call ulTaskNotifyTake() – blocks until notified.
//     4. Re-acquire the mutex before returning.
//
//   notify():
//     Dequeue the first waiter and call xTaskNotifyGive().
//
//   notifyAll():
//     Dequeue and notify every waiting task atomically.
//
// Race safety: the waiter is registered BEFORE the mutex is released, so a
// concurrent notify() will find the waiter in the list and will give a
// notification that is "sticky" in FreeRTOS – ulTaskNotifyTake returns
// immediately if a notification is already pending, preventing lost wakeups.
// ======================================================================
#ifndef AP_OS_FREERTOS_CONDITION_VARIABLE_HPP
#define AP_OS_FREERTOS_CONDITION_VARIABLE_HPP

#include <Os/Condition.hpp>
#include <FreeRTOS.h>
#include <task.h>

namespace Os {
namespace FreeRtos {
namespace Mutex {

struct FreeRtosConditionVariableHandle : public ConditionVariableHandle {
    static constexpr FwSizeType MAX_WAITERS = 8;

    TaskHandle_t m_waiters[MAX_WAITERS];
    FwSizeType   m_count = 0;
};

class FreeRtosConditionVariable final : public ConditionVariableInterface {
  public:
    FreeRtosConditionVariable();
    ~FreeRtosConditionVariable() override = default;

    FreeRtosConditionVariable(const FreeRtosConditionVariable&)            = delete;
    ConditionVariableInterface& operator=(const ConditionVariableInterface&) override = delete;

    //! Block on the condition, atomically releasing \a mutex
    Status pend(Os::Mutex& mutex) override;

    //! Wake one waiting task
    void notify() override;

    //! Wake all waiting tasks
    void notifyAll() override;

    //! Return internal handle pointer
    ConditionVariableHandle* getHandle() override;

  private:
    FreeRtosConditionVariableHandle m_handle;
};

}  // namespace Mutex
}  // namespace FreeRtos
}  // namespace Os

#endif  // AP_OS_FREERTOS_CONDITION_VARIABLE_HPP
