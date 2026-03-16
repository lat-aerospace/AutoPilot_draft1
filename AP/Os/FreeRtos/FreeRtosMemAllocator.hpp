// ======================================================================
// \title  AP/Os/FreeRtos/FreeRtosMemAllocator.hpp
// \brief  Fw::MemAllocator implementation backed by pvPortMalloc/vPortFree
//
// Used by the Generic PriorityQueue (and any other F' component that
// requests dynamic memory) so that all heap activity goes through the
// FreeRTOS heap manager (heap_4.c by default).
//
// Register during topology setup:
//   Fw::MemAllocatorRegistry::getInstance().registerAllocator(
//       Fw::MemoryAllocation::MemoryAllocatorType::OS_GENERIC_PRIORITY_QUEUE,
//       gFreeRtosAllocator);
// ======================================================================
#ifndef AP_OS_FREERTOS_MEM_ALLOCATOR_HPP
#define AP_OS_FREERTOS_MEM_ALLOCATOR_HPP

#include <Fw/Types/MemAllocator.hpp>
#include <FreeRTOS.h>
#include <cstddef>

namespace Ap {
namespace Os {

class FreeRtosMemAllocator final : public Fw::MemAllocator {
  public:
    FreeRtosMemAllocator()  = default;
    ~FreeRtosMemAllocator() override = default;

    //! Allocate \a size bytes from the FreeRTOS heap
    void* allocate(const FwEnumStoreType identifier,
                   FwSizeType&           size,
                   bool&                 recoverable,
                   FwSizeType            alignment = alignof(std::max_align_t)) override;

    //! Return memory to the FreeRTOS heap
    void deallocate(const FwEnumStoreType identifier,
                    void*                 ptr) override;
};

}  // namespace Os
}  // namespace Ap

#endif  // AP_OS_FREERTOS_MEM_ALLOCATOR_HPP
