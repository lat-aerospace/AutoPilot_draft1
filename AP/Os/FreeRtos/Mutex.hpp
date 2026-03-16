// ======================================================================
// \title  AP/Os/FreeRtos/Mutex.hpp
// \brief  FreeRTOS implementation of Os::MutexInterface
//
// Uses a statically-allocated FreeRTOS recursive mutex so that F' components
// that take the mutex more than once on the same task do not deadlock.
// Priority inheritance is inherent in FreeRTOS mutex semantics.
// ======================================================================
#ifndef AP_OS_FREERTOS_MUTEX_HPP
#define AP_OS_FREERTOS_MUTEX_HPP

#include <Os/Mutex.hpp>
#include <FreeRTOS.h>
#include <semphr.h>

namespace Os {
namespace FreeRtos {
namespace Mutex {

struct FreeRtosMutexHandle : public MutexHandle {
    SemaphoreHandle_t m_mutex      = nullptr;
    StaticSemaphore_t m_mutex_buf;          ///< static storage; no heap alloc
};

class FreeRtosMutex final : public MutexInterface {
  public:
    FreeRtosMutex();
    ~FreeRtosMutex() override;

    FreeRtosMutex(const FreeRtosMutex&)            = delete;
    FreeRtosMutex& operator=(const FreeRtosMutex&) = delete;

    MutexHandle* getHandle() override;
    Status take()    override;   ///< Blocking lock
    Status release() override;   ///< Unlock

  private:
    FreeRtosMutexHandle m_handle;
};

}  // namespace Mutex
}  // namespace FreeRtos
}  // namespace Os

#endif  // AP_OS_FREERTOS_MUTEX_HPP
