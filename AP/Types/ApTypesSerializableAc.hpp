// Umbrella header for all FPP-autocoded AP type headers.
// FPP generates one header per struct/enum, but HAL drivers include this
// single header for convenience.
#ifndef AP_TYPES_ALL_HPP
#define AP_TYPES_ALL_HPP

#include <AP/Types/Vec3SerializableAc.hpp>
#include <AP/Types/QuatSerializableAc.hpp>
#include <AP/Types/ImuDataSerializableAc.hpp>
#include <AP/Types/GpsDataSerializableAc.hpp>
#include <AP/Types/BaroDataSerializableAc.hpp>
#include <AP/Types/MagDataSerializableAc.hpp>
#include <AP/Types/AircraftStateSerializableAc.hpp>
#include <AP/Types/SurfaceCmdSerializableAc.hpp>
#include <AP/Types/RcChannelsSerializableAc.hpp>
#include <AP/Types/FlightModeEnumAc.hpp>
#include <AP/Types/GuidanceCmdSerializableAc.hpp>
#include <AP/Types/MissionWaypointSerializableAc.hpp>

#endif // AP_TYPES_ALL_HPP
