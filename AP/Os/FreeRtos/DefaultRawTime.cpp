// ======================================================================
// \title  AP/Os/FreeRtos/DefaultRawTime.cpp
// \brief  Registers FreeRtosRawTime as the default Os::RawTime via linker.
// ======================================================================
#include <Os/Delegate.hpp>
#include <Os/RawTime.hpp>
#include <AP/Os/FreeRtos/RawTime.hpp>

namespace Os {

RawTimeInterface* RawTimeInterface::getDelegate(
    RawTimeHandleStorage&    aligned_new_memory,
    const RawTimeInterface*  to_copy)
{
    return Os::Delegate::makeDelegate<
        RawTimeInterface,
        Os::FreeRtos::RawTime::FreeRtosRawTime,
        RawTimeHandleStorage>(aligned_new_memory, to_copy);
}

}  // namespace Os
