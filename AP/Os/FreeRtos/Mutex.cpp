// ======================================================================
// \title  AP/Os/FreeRtos/Mutex.cpp
// \brief  FreeRTOS implementation of Os::MutexInterface
// ======================================================================
#include <AP/Os/FreeRtos/Mutex.hpp>
#include <Fw/Types/Assert.hpp>

namespace Os {
namespace FreeRtos {
namespace Mutex {

FreeRtosMutex::FreeRtosMutex() : MutexInterface(), m_handle() {
    // Static recursive mutex: no heap allocation, supports re-entrant locking.
    // FreeRTOS recursive mutexes provide priority inheritance.
    m_handle.m_mutex = xSemaphoreCreateRecursiveMutexStatic(&m_handle.m_mutex_buf);
    FW_ASSERT(m_handle.m_mutex != nullptr);
}

FreeRtosMutex::~FreeRtosMutex() {
    if (m_handle.m_mutex != nullptr) {
        vSemaphoreDelete(m_handle.m_mutex);
        m_handle.m_mutex = nullptr;
    }
}

MutexHandle* FreeRtosMutex::getHandle() {
    return &m_handle;
}

MutexInterface::Status FreeRtosMutex::take() {
    // Block indefinitely (portMAX_DELAY) – matches POSIX pthread_mutex_lock semantics
    const BaseType_t result = xSemaphoreTakeRecursive(m_handle.m_mutex, portMAX_DELAY);
    if (result != pdPASS) {
        return MutexInterface::Status::ERROR_OTHER;
    }
    return MutexInterface::Status::OP_OK;
}

MutexInterface::Status FreeRtosMutex::release() {
    const BaseType_t result = xSemaphoreGiveRecursive(m_handle.m_mutex);
    if (result != pdPASS) {
        return MutexInterface::Status::ERROR_OTHER;
    }
    return MutexInterface::Status::OP_OK;
}

}  // namespace Mutex
}  // namespace FreeRtos
}  // namespace Os
