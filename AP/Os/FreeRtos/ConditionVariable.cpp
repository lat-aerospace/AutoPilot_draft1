// ======================================================================
// \title  AP/Os/FreeRtos/ConditionVariable.cpp
// \brief  FreeRTOS implementation of Os::ConditionVariableInterface
// ======================================================================
#include <AP/Os/FreeRtos/ConditionVariable.hpp>
#include <Fw/Types/Assert.hpp>
#include <cstring>

namespace Os {
namespace FreeRtos {
namespace Mutex {

FreeRtosConditionVariable::FreeRtosConditionVariable()
    : ConditionVariableInterface(), m_handle() {
    (void)memset(m_handle.m_waiters, 0, sizeof(m_handle.m_waiters));
}

// ---------------------------------------------------------------------------
// pend – atomically release mutex and wait for notification
// ---------------------------------------------------------------------------
ConditionVariableInterface::Status FreeRtosConditionVariable::pend(Os::Mutex& mutex) {
    // Step 1: Register ourselves in the waiter list while in a critical section.
    // This prevents a race where notify() fires between registration and the
    // actual ulTaskNotifyTake() call.
    taskENTER_CRITICAL();
    FW_ASSERT(m_handle.m_count < FreeRtosConditionVariableHandle::MAX_WAITERS,
              static_cast<FwAssertArgType>(m_handle.m_count));
    m_handle.m_waiters[m_handle.m_count++] = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();

    // Step 2: Release the caller's mutex so other tasks can make progress.
    mutex.unLock();

    // Step 3: Block until notified.  FreeRTOS notifications are "sticky":
    // if notify() already ran, ulTaskNotifyTake returns immediately (no lost
    // wakeup), so this is correct even without holding the mutex here.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Step 4: Re-acquire the mutex before returning to the caller.
    mutex.lock();

    return ConditionVariableInterface::Status::OP_OK;
}

// ---------------------------------------------------------------------------
// notify – wake one waiting task
// ---------------------------------------------------------------------------
void FreeRtosConditionVariable::notify() {
    TaskHandle_t waiter = nullptr;

    taskENTER_CRITICAL();
    if (m_handle.m_count > 0) {
        waiter = m_handle.m_waiters[0];
        m_handle.m_count--;
        // Shift remaining waiters towards the front
        for (FwSizeType i = 0; i < m_handle.m_count; i++) {
            m_handle.m_waiters[i] = m_handle.m_waiters[i + 1];
        }
    }
    taskEXIT_CRITICAL();

    if (waiter != nullptr) {
        xTaskNotifyGive(waiter);
    }
}

// ---------------------------------------------------------------------------
// notifyAll – wake every waiting task
// ---------------------------------------------------------------------------
void FreeRtosConditionVariable::notifyAll() {
    // Snapshot the list inside a critical section, then notify outside it to
    // keep the critical section as short as possible.
    TaskHandle_t snapshot[FreeRtosConditionVariableHandle::MAX_WAITERS];
    FwSizeType count = 0;

    taskENTER_CRITICAL();
    count = m_handle.m_count;
    for (FwSizeType i = 0; i < count; i++) {
        snapshot[i] = m_handle.m_waiters[i];
    }
    m_handle.m_count = 0;
    taskEXIT_CRITICAL();

    for (FwSizeType i = 0; i < count; i++) {
        xTaskNotifyGive(snapshot[i]);
    }
}

// ---------------------------------------------------------------------------
// getHandle
// ---------------------------------------------------------------------------
ConditionVariableHandle* FreeRtosConditionVariable::getHandle() {
    return &m_handle;
}

}  // namespace Mutex
}  // namespace FreeRtos
}  // namespace Os
