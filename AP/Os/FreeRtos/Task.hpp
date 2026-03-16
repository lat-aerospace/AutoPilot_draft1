// ======================================================================
// \title  AP/Os/FreeRtos/Task.hpp
// \brief  FreeRTOS implementation of Os::TaskInterface
//
// Maps F' task lifecycle to FreeRTOS tasks (xTaskCreate / vTaskDelete).
// Priority is translated: F' uses POSIX-style numerics (higher = higher),
// FreeRTOS uses 0-based (0 = idle).  The helper fprime_to_freertos_priority()
// performs a linear mapping onto [1 .. configMAX_PRIORITIES-1].
//
// Stack size: F' passes bytes; FreeRTOS configures depth in StackType_t words
// (4 bytes on ARM Cortex-M).  The helper converts automatically.
//
// join() is implemented via a binary semaphore that the task gives just before
// it exits – no dynamic allocation after startup.
// ======================================================================
#ifndef AP_OS_FREERTOS_TASK_HPP
#define AP_OS_FREERTOS_TASK_HPP

#include <Os/Task.hpp>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

namespace Os {
namespace FreeRtos {
namespace Task {

// ---------------------------------------------------------------------------
// Per-task bookkeeping stored in handle storage
// ---------------------------------------------------------------------------
struct FreeRtosTaskHandle : public TaskHandle {
    TaskHandle_t    m_task_handle  = nullptr;
    SemaphoreHandle_t m_join_sem   = nullptr;   ///< given when task exits
    StaticSemaphore_t m_join_sem_buf;           ///< static storage for semaphore
    bool            m_is_valid     = false;
};

// ---------------------------------------------------------------------------
// Context block passed through the FreeRTOS task entry wrapper
// ---------------------------------------------------------------------------
struct TaskStartupContext {
    Os::TaskInterface::taskRoutine routine;
    void*                           arg;
    FreeRtosTaskHandle*             handle;
};

// ---------------------------------------------------------------------------
// FreeRtosTask – concrete implementation of Os::TaskInterface
// ---------------------------------------------------------------------------
class FreeRtosTask final : public TaskInterface {
  public:
    FreeRtosTask()  = default;
    ~FreeRtosTask() override;

    FreeRtosTask(const FreeRtosTask&)            = delete;
    FreeRtosTask& operator=(const FreeRtosTask&) = delete;

    //! Called once the task has started (inside the new thread)
    void onStart() override;

    //! Create and start the FreeRTOS task
    Status start(const Arguments& arguments) override;

    //! Block until this task exits
    Status join() override;

    //! Suspend the FreeRTOS task
    void suspend(SuspensionType suspensionType) override;

    //! Resume a suspended FreeRTOS task
    void resume() override;

    //! Delay the calling task
    Status _delay(Fw::TimeInterval interval) override;

    //! Return a pointer to the handle (for framework inspection)
    TaskHandle* getHandle() override;

  private:
    //! Convert an F' priority number to a FreeRTOS priority (1-based)
    static UBaseType_t fprimePriorityToFreeRtos(FwTaskPriorityType fpriority);

    //! Entry wrapper called by FreeRTOS; runs the user routine then signals join
    static void taskEntry(void* pvParameters);

    FreeRtosTaskHandle     m_handle;
    TaskStartupContext     m_startup_ctx;   ///< must stay valid until task starts
};

}  // namespace Task
}  // namespace FreeRtos
}  // namespace Os

#endif  // AP_OS_FREERTOS_TASK_HPP
