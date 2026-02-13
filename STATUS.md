# Status of AutoPilot project

**Current state (Feb 13 2026):** Full closed-loop SITL running end-to-end. QGroundControl joystick (FBWB mode) controls the aircraft through the complete chain: QGC → MavlinkGateway → Autonomy → Controller → SimServoDriver → SimDynamics → SimSensors → StateEstimator → telemetry back to QGC. Aircraft visible on QGC map with live attitude, position, and HUD. Stick inputs confirmed via GDS telemetry (`rcRoll`, `rcPitch`, `rcThrottle`, `rcYaw` → `desHeading`, `desAlt`, `desAirspeed`).

## Phase 1: Skeleton [COMPLETE]

Types, ports, rate groups, GDS link. See README for details.

- Eigen 3.4.0, `AP/Math/`, `AP/Types/`, `AP/Ports/`
- SITL topology: LinuxTimer 400Hz, 4 rate groups (400/100/50/10Hz), CdhCore + ComCcsds, TcpClient for GDS
- Binary runs, GDS connects, rate group telemetry ticking

## Phase 2: Simulation — 6-DOF + Sim Sensors [COMPLETE]

- `RigidBody6DOF` — 13-state vector, forward Euler, inline force/moment model (~12 tunable constants)
- `SimDynamics` (queued 400Hz), `SimServoDriver` (passive), `SimImu` (400Hz), `SimGps` (10Hz), `SimBaro` (50Hz), `SimMag` (100Hz)
- All wired into topology, truth state verified in GDS

## Phase 3: Flight Control + GCS Integration [IN PROGRESS]

### 3a: Foundation [COMPLETE]

FlightMode/GuidanceCmd/MissionWaypoint types, MAVLink c_library_v2 submodule.

### 3b: StateEstimator [COMPLETE]

Complementary filter (α=0.98): gyro integration + accel (roll/pitch) + tilt-compensated mag (yaw). Position from GPS→NED, altitude from baro, velocity from GPS. Fan-out to Controller, Autonomy, MavlinkGateway.

### 3c: Controller [COMPLETE]

Cascaded P controllers: heading→roll→aileron, altitude→pitch→elevator, airspeed→throttle. Holds approximate altitude/heading/speed with hardcoded or commanded GuidanceCmd.

### 3d: MavlinkGateway [COMPLETE]

Active component with POSIX UDP. Heartbeat 1Hz (`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_GENERIC`). Telemetry 10Hz: ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS. Non-blocking recv parses MANUAL_CONTROL, RC_CHANNELS_OVERRIDE, COMMAND_LONG, PARAM_REQUEST_LIST/READ, HEARTBEAT. Minimal 3-param table for GCS handshake. Dynamic GCS address capture.

### 3e: Autonomy + FBWB mode [COMPLETE]

FBWB stick mapping in Autonomy (queued, 10Hz):

| RC Input | Range | Guidance Output | Mapping |
|----------|-------|-----------------|---------|
| `rcPitch` | -1 to +1 | `desAlt` | Integrates at ±5 m/s per full stick |
| `rcRoll` | -1 to +1 | `desHeading` | Integrates at ±30°/s per full stick |
| `rcThrottle` | 0 to 1 | `desAirspeed` | Linear map to 20-80 m/s |
| `rcYaw` | -1 to +1 | *(unused)* | — |

Full chain wired: MavlinkGateway.rcOut → Autonomy.rcIn → guidanceCmdOut → Controller → SimServoDriver.

**GCS setup:** QGroundControl over UDP 14550. MissionPlanner was tested but never sends joystick data (requires ArduPilot-specific RC calibration params). QGC sends MANUAL_CONTROL at 10-20Hz without special param requirements. Throttle axis couldn't be mapped to main stick in QGC — using SB stick on joystick as workaround.

**Telemetry channels** (MavlinkGateway): `rcRoll`, `rcPitch`, `rcThrottle`, `rcYaw`, `rcMsgCount`. Combined with Autonomy channels (`desAlt`, `desAirspeed`, `desHeading`) for full input→output visibility in GDS.

### 3f: Auto mode [NOT STARTED]

- [ ] Waypoint storage + path guidance in Autonomy
- [ ] MISSION protocol in MavlinkGateway
- [ ] Test: upload mission in QGC, switch to Auto, aircraft follows waypoints

## Known Issues

- **Magnetic declination**: estYaw reads ~345° instead of 0° (Albuquerque reference point). Correctable with declination offset.
- **P-only controller**: No integral term → steady-state errors. Aircraft holds approximately but not perfectly.
- **No wind model**: Airspeed = groundspeed. Fine for SITL.
- **QGC throttle axis**: Cannot map throttle to main joystick stick — using SB stick as workaround.

## Future

See README for roadmap (more modes, full EKF, MAVLink params, STM32H7 hardware).
