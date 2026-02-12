# Status of AutoPilot project

**Current state (Feb 2026):** Full closed-loop SITL running. 6-DOF physics → sim sensors → state estimator → controller → servo driver. MavlinkGateway sends telemetry to MissionPlanner (aircraft visible on map, attitude/position/HUD working). Autonomy component wired with FBWB stick mapping. Joystick input from MissionPlanner not yet verified — debugging MANUAL_CONTROL / RC_CHANNELS_OVERRIDE reception.

## Phase 1: Skeleton (see README)

- [x] Eigen 3.4.0 git submodule (`lib/eigen/`)
- [x] Eigen INTERFACE library in top-level CMakeLists
- [x] `AP/Math/` — ApMath.hpp (typedefs, constants), CoordTransforms.hpp (NED/LLA, DCM, quat)
- [x] `AP/Types/ApTypes.fpp` — Vec3, Quat, ImuData, GpsData, BaroData, MagData, AircraftState, SurfaceCmd, RcChannels, FlightMode, GuidanceCmd, MissionWaypoint
- [x] `AP/Ports/ApPorts.fpp` — ImuPort, GpsPort, BaroPort, MagPort, StatePort, SurfaceCmdPort, RcPort, ModePort, GuidanceCmdPort, MissionWaypointPort
- [x] `config/AcConstants.fpp` — RateGroupDriverRateGroupPorts = 4
- [x] `AP/Top/Sitl/` — SITL topology: LinuxTimer 400Hz, 4 rate groups (400/100/50/10Hz), CdhCore + ComCcsds subtopologies, TcpClient for GDS
- [x] `sitl` build preset in CMakePresets.json
- [x] Build passes (binary + GDS dictionary generated)
- [x] Verify: run binary, connect GDS, see rate group telemetry ticking

## Phase 2: Simulation — 6-DOF + Sim Sensors [COMPLETE]

Simplified model: inline force/moment math (~10 constants), no interface hierarchy.
Swap in real aero derivatives later without changing integrator or F Prime wiring.

### 2a: Pure C++ math (no F Prime dependency)

- [x] `RigidBody6DOF` — 13-state vector, forward Euler integrator, inline simple force/moment model, ~12 tunable constants in `SimpleAircraftParams`, build passes

### 2b: F Prime sim components

- [x] `SimDynamics.fpp/.hpp/.cpp` — queued, wraps RigidBody6DOF, outputs truth AircraftState via truthStateOut[4] fan-out
- [x] `SimServoDriver.fpp/.hpp/.cpp` — passive, forwards SurfaceCmd to SimDynamics
- [x] `SimImu.fpp/.hpp/.cpp` — queued 400Hz, gravity-in-body + noise → ImuData
- [x] `SimGps.fpp/.hpp/.cpp` — queued 10Hz, NED→LLA + noise → GpsData
- [x] `SimBaro.fpp/.hpp/.cpp` — queued 50Hz, ISA pressure model + noise → BaroData
- [x] `SimMag.fpp/.hpp/.cpp` — queued 100Hz, earth field rotated to body + noise → MagData

### 2c: Integration

- [x] Wire sim components into topology (instances.fpp + topology.fpp)
- [x] Build passes
- [x] Verify: surface cmd via GDS → aircraft moves in telemetry

## Phase 3: Flight Control + MissionPlanner Integration [IN PROGRESS]

Linear chain: MavlinkGateway → Autonomy → Controller → ServoDriver

### 3a: Foundation (types, ports, mavlink submodule) [COMPLETE]

- [x] Add `FlightMode` enum, `GuidanceCmd`, `MissionWaypoint` to ApTypes.fpp
- [x] Add `ModePort`, `GuidanceCmdPort`, `MissionWaypointPort` to ApPorts.fpp
- [x] Add `c_library_v2` git submodule under `lib/mavlink`
- [x] Add `mavlink` INTERFACE library to root CMakeLists.txt
- [x] Build passes

### 3b: StateEstimator (complementary filter) [COMPLETE]

- [x] Create `AP/Components/FlightControl/StateEstimator/` — FPP, HPP, CPP, CMakeLists
- [x] Complementary filter: gyro integration + accel (roll/pitch) + tilt-compensated mag (yaw), α=0.98
- [x] Position from GPS lat/lon → NED, altitude from baro, velocity from GPS
- [x] Wire sensors → StateEstimator, stateOut[0-2] fan-out (Controller, Autonomy, MavlinkGateway)
- [x] Build passes, binary runs stable
- [x] Verify estimated state tracks truth in GDS

### 3c: Controller (cascaded P controllers) [COMPLETE]

- [x] Create `AP/Components/FlightControl/Controller/` — FPP, HPP, CPP, CMakeLists
- [x] Heading → desired roll (Kp=0.05) → aileron (Kp=0.02), clamp ±30° bank
- [x] Altitude → desired pitch (Kp=0.1) → elevator (Kp=0.03), clamp ±15° pitch
- [x] Airspeed error → throttle (Kp=0.05, trim=0.5)
- [x] Default GuidanceCmd: 2000m MSL, 50 m/s, heading 0°
- [x] Wire: stateEstimator.stateOut[0] → controller.stateIn, controller.surfaceCmdOut → simServoDriver
- [x] Build passes, binary runs stable
- [x] Aircraft holds approximate altitude/heading with hardcoded GuidanceCmd

### 3d: MavlinkGateway (heartbeat + telemetry out) [COMPLETE]

- [x] Create `AP/Components/Mavlink/MavlinkGateway/` — active component, FPP, HPP, CPP, CMakeLists
- [x] POSIX UDP socket: send to `127.0.0.1:14550`, recv on `0.0.0.0:14540`
- [x] HEARTBEAT 1Hz (MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC)
- [x] Telemetry 10Hz: ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS
- [x] Wire: stateEstimator.stateOut[2] → mavlinkGateway.stateIn, schedIn on RG4
- [x] Build passes, binary runs stable, UDP packets verified
- [x] MissionPlanner sees aircraft on map with live attitude/position/HUD

### 3e: Autonomy + FBWB mode [IN PROGRESS]

- [x] Create `AP/Components/FlightControl/Autonomy/` — FPP, HPP, CPP, CMakeLists
- [x] FBWB stick mapping: pitch→alt rate (±5 m/s), roll→heading rate (±30°/s), throttle→airspeed (20-80 m/s)
- [x] MavlinkGateway recv: non-blocking UDP, parses MANUAL_CONTROL + RC_CHANNELS_OVERRIDE
- [x] MavlinkGateway sends COMMAND_ACK for COMMAND_LONG (arm/disarm)
- [x] Wire full chain: MavlinkGateway → Autonomy → Controller → ServoDriver
- [x] Build passes, binary runs stable
- [ ] **BLOCKED:** MissionPlanner joystick data not reaching SITL — `lastRecvMsgId` shows only HEARTBEAT (ID 0) and COMMAND_LONG (ID 76) from MissionPlanner. Joystick visible in MissionPlanner HUD but MANUAL_CONTROL / RC_CHANNELS_OVERRIDE not being sent. Likely needs further MissionPlanner configuration or protocol handshake.

### 3f: Auto mode [NOT STARTED]

- [ ] Add waypoint storage + path guidance to Autonomy
- [ ] Add MISSION protocol to MavlinkGateway
- [ ] Test: upload mission, switch to Auto, aircraft follows waypoints

## Known Issues

- **Magnetic declination**: estYaw reads ~345° instead of 0° due to magnetic declination at Albuquerque reference point (not true north). Correctable with declination offset.
- **P-only controller**: No integral term, so steady-state errors exist. Aircraft holds approximately but not perfectly.
- **No wind model**: Airspeed = groundspeed in state estimator. Fine for SITL, needs wind correction later.
- **MissionPlanner joystick**: Data not reaching SITL yet — see 3e BLOCKED note above.

## Future

See README for roadmap (more modes, full EKF, MAVLink params, hardware).
