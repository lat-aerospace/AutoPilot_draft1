// ======================================================================
// \title  AP/Os/FreeRtos/Task.cpp
// \brief  FreeRTOS implementation of Os::TaskInterface
// ======================================================================
#include <AP/Os/FreeRtos/Task.hpp>
#include <Fw/Types/Assert.hpp>
#include <cstring>

namespace Os {
namespace FreeRtos {
namespace Task {

// ---------------------------------------------------------------------------
// Priority mapping
//
//  F' SITL uses priorities 42-45 for rate groups, 40 for MAVLink gateway.
//  We map [0..49] -> [1..configMAX_PRIORITIES-1] linearly, clamped.
//  configMAX_PRIORITIES is set to 8 in FreeRTOSConfig.h (0=idle, 7=highest).
// ---------------------------------------------------------------------------
UBaseType_t FreeRtosTask::fprimePriorityToFreeRtos(FwTaskPriorityType fpriority) {
    // Map F' priority range [0..49] to FreeRTOS [1..configMAX_PRIORITIES-1]
    constexpr FwTaskPriorityType FP_MAX = 49;
    constexpr UBaseType_t RTOS_MIN = 1;
    constexpr UBaseType_t RTOS_MAX = configMAX_PRIORITIES - 1;

    if (fpriority <= 0) {
        return RTOS_MIN;
    }
    if (fpriority >= FP_MAX) {
        return RTOS_MAX;
    }
    const UBaseType_t mapped =
        RTOS_MIN + static_cast<UBaseType_t>((fpriority * (RTOS_MAX - RTOS_MIN)) / FP_MAX);
    return mapped;
}

// ---------------------------------------------------------------------------
// FreeRTOS task entry wrapper
// Runs the user routine, then signals the join semaphore and deletes itself.
// ---------------------------------------------------------------------------
void FreeRtosTask::taskEntry(void* pvParameters) {
    auto* ctx = static_cast<TaskStartupContext*>(pvParameters);
    FW_ASSERT(ctx != nullptr);
    FW_ASSERT(ctx->routine != nullptr);

    // Run the actual F' task body
    ctx->routine(ctx->arg);

    // Signal that the task has finished so join() can unblock
    xSemaphoreGive(ctx->handle->m_join_sem);

    // Self-delete – FreeRTOS requires tasks to not return from their function
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// onStart – called from inside the new task after it begins executing
// ---------------------------------------------------------------------------
void FreeRtosTask::onStart() {
    // Nothing extra required; FreeRTOS handles scheduling natively.
}

// ---------------------------------------------------------------------------
// start – create the FreeRTOS task
// ---------------------------------------------------------------------------
TaskInterface::Status FreeRtosTask::start(const Arguments& arguments) {
    FW_ASSERT(arguments.m_routine != nullptr);

    // Create join semaphore (binary, static storage)
    m_handle.m_join_sem = xSemaphoreCreateBinaryStatic(&m_handle.m_join_sem_buf);
    FW_ASSERT(m_handle.m_join_sem != nullptr);

    // Fill startup context (stored in member so it outlives this stack frame)
    m_startup_ctx.routine = arguments.m_routine;
    m_startup_ctx.arg     = arguments.m_routine_argument;
    m_startup_ctx.handle  = &m_handle;

    // Convert stack size: F' gives bytes, FreeRTOS wants StackType_t words
    FwSizeType stack_bytes = (arguments.m_stackSize > 0)
                                 ? static_cast<FwSizeType>(arguments.m_stackSize)
                                 : (configMINIMAL_STACK_SIZE * sizeof(StackType_t));
    const uint32_t stack_depth =
        static_cast<uint32_t>(stack_bytes / sizeof(StackType_t));

    const UBaseType_t prio = fprimePriorityToFreeRtos(
        static_cast<FwTaskPriorityType>(arguments.m_priority));

    // Truncate name to FreeRTOS limit
    char name_buf[configMAX_TASK_NAME_LEN];
    (void)strncpy(name_buf, arguments.m_name.toChar(), configMAX_TASK_NAME_LEN - 1);
    name_buf[configMAX_TASK_NAME_LEN - 1] = '\0';

    const BaseType_t result = xTaskCreate(
        FreeRtosTask::taskEntry,
        name_buf,
        static_cast<configSTACK_DEPTH_TYPE>(stack_depth),
        &m_startup_ctx,
        prio,
        &m_handle.m_task_handle);

    if (result != pdPASS) {
        return TaskInterface::Status::ERROR_RESOURCES;
    }

    m_handle.m_is_valid = true;
    return TaskInterface::Status::OP_OK;
}

// ---------------------------------------------------------------------------
// join – block until the task exits
// ---------------------------------------------------------------------------
TaskInterface::Status FreeRtosTask::join() {
    if (!m_handle.m_is_valid) {
        return TaskInterface::Status::INVALID_HANDLE;
    }
    // Block indefinitely waiting for the task to give the semaphore
    xSemaphoreTake(m_handle.m_join_sem, portMAX_DELAY);
    return TaskInterface::Status::OP_OK;
}

// ---------------------------------------------------------------------------
// suspend / resume
// ---------------------------------------------------------------------------
void FreeRtosTask::suspend(SuspensionType /*suspensionType*/) {
    if (m_handle.m_is_valid && m_handle.m_task_handle != nullptr) {
        vTaskSuspend(m_handle.m_task_handle);
    }
}

void FreeRtosTask::resume() {
    if (m_handle.m_is_valid && m_handle.m_task_handle != nullptr) {
        vTaskResume(m_handle.m_task_handle);
    }
}

// ---------------------------------------------------------------------------
// _delay – sleep the calling task
// ---------------------------------------------------------------------------
TaskInterface::Status FreeRtosTask::_delay(Fw::TimeInterval interval) {
    // Convert interval to FreeRTOS ticks
    // 1 tick = 1 / configTICK_RATE_HZ seconds
    const uint64_t total_us =
        static_cast<uint64_t>(interval.getSeconds()) * 1000000ULL +
        static_cast<uint64_t>(interval.getUSeconds());
    const TickType_t ticks =
        static_cast<TickType_t>((total_us * configTICK_RATE_HZ) / 1000000ULL);

    vTaskDelay(ticks > 0 ? ticks : 1);
    return TaskInterface::Status::OP_OK;
}

// ---------------------------------------------------------------------------
// getHandle
// ---------------------------------------------------------------------------
TaskHandle* FreeRtosTask::getHandle() {
    return &m_handle;
}

// ---------------------------------------------------------------------------
// Destructor – clean up semaphore if not already done
// ---------------------------------------------------------------------------
FreeRtosTask::~FreeRtosTask() {
    if (m_handle.m_join_sem != nullptr) {
        vSemaphoreDelete(m_handle.m_join_sem);
        m_handle.m_join_sem = nullptr;
    }
}

}  // namespace Task
}  // namespace FreeRtos
}  // namespace Os
