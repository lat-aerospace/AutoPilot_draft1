// ======================================================================
// \title  AP/Os/FreeRtos/FreeRtosMemAllocator.cpp
// \brief  Fw::MemAllocator backed by pvPortMalloc/vPortFree
// ======================================================================
#include <AP/Os/FreeRtos/FreeRtosMemAllocator.hpp>
#include <Fw/Types/Assert.hpp>
#include <FreeRTOS.h>

namespace Ap {
namespace Os {

void* FreeRtosMemAllocator::allocate(const FwEnumStoreType /*identifier*/,
                                      FwSizeType&           size,
                                      bool&                 recoverable,
                                      FwSizeType            /*alignment*/) {
    recoverable = false;  // FreeRTOS heap is not recoverable (non-compacting)
    void* ptr = pvPortMalloc(static_cast<size_t>(size));
    FW_ASSERT(ptr != nullptr, static_cast<FwAssertArgType>(size));
    return ptr;
}

void FreeRtosMemAllocator::deallocate(const FwEnumStoreType /*identifier*/,
                                       void* memory) {
    vPortFree(memory);
}

}  // namespace Os
}  // namespace Ap
