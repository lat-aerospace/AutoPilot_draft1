// ======================================================================
// \title  AP/Os/FreeRtos/DefaultQueue.cpp
// \brief  Selects the Generic PriorityQueue as the default Os::Queue
//         implementation for FreeRTOS targets.
//
// The Generic PriorityQueue is a pure-C++ priority queue that uses
// Os::Mutex and Os::ConditionVariable internally.  Because those are now
// backed by FreeRTOS primitives (FreeRtosMutex, FreeRtosConditionVariable),
// the full queue behaviour is FreeRTOS-native without writing a duplicate
// queue implementation.
//
// Memory allocation: the PriorityQueue uses Fw::MemAllocator which must be
// configured to call pvPortMalloc in FreeRTOSMemAllocator.cpp (see that file).
// All allocation happens once during Os::Queue::create(), before the scheduler
// loop begins, so fragmentation is not a concern.
// ======================================================================
#include <Os/Delegate.hpp>
#include <Os/Generic/PriorityQueue.hpp>
#include <Os/Queue.hpp>

namespace Os {

QueueInterface* QueueInterface::getDelegate(QueueHandleStorage& aligned_new_memory) {
    return Os::Delegate::makeDelegate<
        QueueInterface,
        Os::Generic::PriorityQueue,
        QueueHandleStorage>(aligned_new_memory);
}

}  // namespace Os
