# AutoPilot — Fixed-Wing SITL Autopilot on F Prime

An end-to-end fixed-wing autopilot SITL environment built on NASA JPL's F Prime (F') framework. Connects to **QGroundControl (QGC)** via **MAVLink v2 over UDP** for joystick control (FBWB mode) and autonomous waypoint following (Auto mode). Inside F Prime: MAVLink gateway → autonomy → flight controller (TECS/L1/PID) → 6-DOF physics sim → simulated sensors → state estimation → telemetry back to QGC. Monitor in real-time via QGC and/or fprime-gds.

This is the foundation that will later grow into the full autopilot with hardware layers (STM32H7).

> **Current status:** See [`STATUS.md`](STATUS.md) for detailed phase-by-phase progress and known issues.

---

## Architecture Overview

```
┌─────────────────────┐     UDP MAVLink (14550/14540)    ┌──────────────────────────────────────┐
│  QGroundControl     │ <────────────────────────────>   │  F Prime SITL Deployment             │
│  (GCS)              │  joystick, missions, telemetry   │                                      │
│  - Joystick (FBWB)  │                                  │  MavlinkGateway (active, UDP)        │
│  - Mission (Auto)   │                                  │      │                               │
│  - Telemetry view   │                                  │      v                               │
└─────────────────────┘                                  │  Autonomy (mode manager + guidance)  │
                                                         │      │                               │
┌─────────────────────┐       TCP (localhost:5000)       │      v                               │
│  fprime-gds         │ <─────────────────────────────>  │  Controller (TECS + L1 + PIDs)       │
│  (F Prime debug)    │                                  │      │                               │
└─────────────────────┘                                  │      v                               │
                                                         │  SimServoDriver ──> SimDynamics      │
                                                         │                      │               │
                                                         │      ┌──────────────┘               │
                                                         │      v                               │
                                                         │  SimSensors ──> StateEstimator ──┐   │
                                                         │      ↑ (feedback to Autonomy,    │   │
                                                         │      │  Controller, MavlinkGW)   │   │
                                                         │      └───────────────────────────┘   │
                                                         └──────────────────────────────────────┘
```

### Aircraft model 

Aircraft: Cessna 172-Class Fixed-Wing                                                                                                                         
                  
  Think of a conventional high-wing, single-engine, propeller-driven airplane. This is the most well-documented airframe in flight dynamics literature, making  
  it ideal for a first sim.                                                                                                                                     

  How It Looks

          ┌───────────────────────────┐
          │                           │  ← High wing (rectangular, ~11m span)
          │                           │
          └───────────┬───────────────┘
                      │
                 ┌────┴────┐
                 │  FUSE-  │  ← Fuselage (~8m long)
      Propeller ●│  LAGE   │
                 │         │
                 └────┬────┘
                      │
                ┌─────┴─────┐
                │ Horizontal │  ← Horizontal tail (elevator)
                │    Tail    │
                └─────┬──────┘
                      │
                     ┌┴┐        ← Vertical tail (rudder)
                     │ │
                     └─┘

  Control Surfaces (Actuators)

  There are 4 actuator channels, each normalized to a range:
  ```
  ┌──────────┬────────────────────────────────────────────────┬─────────────────────────────────┬─────────────────────────┐
  │ Actuator │                    Location                    │          What It Does           │          Range          │
  ├──────────┼────────────────────────────────────────────────┼─────────────────────────────────┼─────────────────────────┤
  │ Aileron  │ Wing trailing edges (left+right, differential) │ Controls roll (bank left/right) │ -1.0 to +1.0 (~±25 deg) │
  ├──────────┼────────────────────────────────────────────────┼─────────────────────────────────┼─────────────────────────┤
  │ Elevator │ Horizontal tail trailing edge                  │ Controls pitch (nose up/down)   │ -1.0 to +1.0 (~±25 deg) │
  ├──────────┼────────────────────────────────────────────────┼─────────────────────────────────┼─────────────────────────┤
  │ Rudder   │ Vertical tail trailing edge                    │ Controls yaw (nose left/right)  │ -1.0 to +1.0 (~±25 deg) │
  ├──────────┼────────────────────────────────────────────────┼─────────────────────────────────┼─────────────────────────┤
  │ Throttle │ Engine/propeller                               │ Controls thrust (speed)         │ 0.0 to 1.0              │
  └──────────┴────────────────────────────────────────────────┴─────────────────────────────────┴─────────────────────────┘
  ```
  WASD mapping: A/D → aileron, W/S → elevator, Q/E → rudder, key up/down → throttle.

---

## Components

### Custom FPP Types (`AP/Types/ApTypes.fpp`)

```
Vec3           { x: F64, y: F64, z: F64 }
Quat           { w: F64, x: F64, y: F64, z: F64 }
ImuData        { accel_mps2: Vec3, gyro_dps: Vec3, time_s: F64 }
GpsData        { lat_deg: F64, lon_deg: F64, alt_msl_m: F64, vel_ned_mps: Vec3, time_s: F64 }
BaroData       { pressure_pa: F64, altitude_m: F64, time_s: F64 }
MagData        { field_gauss: Vec3, time_s: F64 }
AircraftState  { position_ned: Vec3, velocity_ned: Vec3, attitude_quat: Quat,
                 euler_deg: Vec3, angular_rate_dps: Vec3, airspeed_ms: F64, time_s: F64 }
SurfaceCmd     { aileron: F64, elevator: F64, rudder: F64, throttle: F64 }
RcChannels     { roll: F32, pitch: F32, yaw: F32, throttle: F32 }
FlightMode     enum { FBWB, AUTO }
GuidanceCmd    { desired_alt_m: F64, desired_airspeed_ms: F64, desired_heading_deg: F64 }
MissionWaypoint { seq: U16, lat_deg: F64, lon_deg: F64, alt_msl_m: F32, speed_ms: F32 }
```

### Custom FPP Ports (`AP/Ports/ApPorts.fpp`)

```
ImuPort(ref data: ImuData)
GpsPort(ref data: GpsData)
BaroPort(ref data: BaroData)
MagPort(ref data: MagData)
StatePort(ref state: AircraftState)
SurfaceCmdPort(ref cmd: SurfaceCmd)
RcPort(ref rc: RcChannels)
ModePort(mode: FlightMode)
GuidanceCmdPort(ref cmd: GuidanceCmd)
MissionWaypointPort(ref wp: MissionWaypoint)
```

### Component Table

| # | Component | Type | Rate | Purpose |
|---|-----------|------|------|---------|
| 1 | **MavlinkGateway** | active | recv thread + 10Hz | UDP socket, MAVLink v2 encode/decode, heartbeat (1Hz), telemetry to QGC, forwards RC/mode/mission downstream |
| 2 | **Autonomy** | queued | 10Hz | Mode state machine (FBWB/Auto). FBWB: sticks → GuidanceCmd. Auto: waypoints + state → GuidanceCmd. Always outputs `GuidanceCmd` |
| 3 | **Controller** | queued | 100Hz | Full flight controller: TECS (alt+speed → pitch+throttle), L1 (heading → roll), rate PIDs → `SurfaceCmd` |
| 4 | **StateEstimator** | queued | 100Hz | Complementary filter (upgradeable to EKF): IMU/GPS/Baro/Mag → estimated `AircraftState` |
| 5 | **SimServoDriver** | passive | on-call | Receives `SurfaceCmd`, forwards to `SimDynamics` |
| 6 | **SimDynamics** | queued | 400Hz | 6-DOF rigid body sim (forward Euler). Reads surface commands, integrates EOM, outputs truth state |
| 7 | **SimImu** | queued | 400Hz | Reads truth state, adds noise, outputs `ImuData` |
| 8 | **SimGps** | queued | 10Hz | Reads truth state, adds noise, outputs `GpsData` |
| 9 | **SimBaro** | queued | 50Hz | Reads truth state, adds noise, outputs `BaroData` |
| 10 | **SimMag** | queued | 100Hz | Reads truth state, adds noise, outputs `MagData` |

**Linear data chain**: MavlinkGateway → Autonomy → Controller → SimServoDriver → SimDynamics → SimSensors → StateEstimator → (feedback to Autonomy, Controller, MavlinkGateway)

Queued components receive cross-rate-group data on **async input ports** (thread-safe message queue). On `schedIn`, they call `doDispatch()` to drain queued messages, then run their algorithm. This avoids race conditions between rate group threads.

### Framework Instances (from F Prime `Svc/` and `Drv/`)

- `Svc.LinuxTimer` — 400Hz base tick (2.5ms)
- `Svc.RateGroupDriver` — divides into rate groups
- `Svc.ActiveRateGroup` x4 — RG1-RG4
- `Svc.PosixTime` — timestamps
- CdhCore subtopology — command dispatch, events, health
- ComCcsds subtopology — GDS communication framing
- `Drv.TcpClient` — GDS link on port 5000 (runs in parallel with QGC)

---

## Rate Group Design

**Base tick**: 400Hz (2.5ms via `LinuxTimer`)

| Rate Group | Freq | Divisor | Members (execution order) |
|------------|------|---------|---------------------------|
| RG1 | 400Hz | 1 | SimDynamics, SimImu |
| RG2 | 100Hz | 4 | SimMag, CDH services, StateEstimator, Controller |
| RG3 | 50Hz | 8 | SimBaro, SystemResources |
| RG4 | 10Hz | 40 | SimGps, Health, BufferManager, Autonomy, MavlinkGateway |

---

## Closed-Loop Data Flow

```
RG1 (400Hz):
  SimDynamics.schedIn → doDispatch() drains queued SurfaceCmd
                      → forward Euler integrate 6-DOF by dt=2.5ms
                      → outputs truth state to all sim sensors

  SimImu.schedIn      → doDispatch() drains queued truth state
                      → adds noise → outputs ImuData

RG2 (100Hz):
  SimMag.schedIn          → truth state + noise → MagData
  StateEstimator.schedIn  → drains IMU/GPS/Baro/Mag queues
                          → complementary filter → outputs estimated AircraftState
                          → fan-out to Controller, Autonomy, MavlinkGateway
  Controller.schedIn      → drains GuidanceCmd + AircraftState
                          → TECS (alt+speed → pitch+throttle)
                          → L1 (heading → roll)
                          → rate PIDs → SurfaceCmd → SimServoDriver

RG3 (50Hz):
  SimBaro.schedIn  → truth state + noise → BaroData

RG4 (10Hz):
  SimGps.schedIn          → truth state + noise → GpsData
  Autonomy.schedIn        → drains RC/mode/waypoint/state queues
                          → FBWB: sticks → GuidanceCmd
                          → Auto: waypoints + state → GuidanceCmd
  MavlinkGateway.schedIn  → packs HEARTBEAT + ATTITUDE + GPS + VFR_HUD
                          → sends to QGC via UDP
```

**Linear chain**: MavlinkGateway → Autonomy → Controller → ServoDriver → SimDynamics
**Loop closure**: SimDynamics (RG1) reads `SurfaceCmd` from Controller (RG2) in the previous cycle. One-cycle latency is realistic.

---

## SimDynamics 6-DOF Engine

Pure C++ class (no F Prime dependency in the math):

| Class | Purpose |
|-------|---------|
| `RigidBody6DOF` | 13-state vector [pos_NED(3), vel_body(3), quat(4), omega_body(3)]. Forward Euler integration. Inline force/moment model with ~12 tunable constants in `SimpleAircraftParams`. |

The `SimDynamics` F Prime component wraps this class, calling `RigidBody6DOF::step(dt)` on each `schedIn`. Swap in real aero derivatives later without changing the F Prime wiring.

---

## Sensor Noise Models

| Sensor | Noise Model |
|--------|-------------|
| IMU accel | White noise (0.5 mg/√Hz) + constant bias (±20 mg) |
| IMU gyro | White noise (0.01 deg/s/√Hz) + constant bias (±1 deg/s) |
| GPS pos | White noise (sigma = 2.5m per axis) + 200ms latency buffer |
| GPS vel | White noise (sigma = 0.1 m/s) |
| Baro alt | White noise (sigma = 0.5m) + slow drift |
| Mag | White noise (sigma = 5 mGauss) + hard iron offset |

---

## State Estimator

Starts as a **complementary filter** (upgradeable to 15-state EKF later):
- **Gyro integration** (100Hz): Attitude propagation from IMU angular rates
- **GPS correction** (10Hz): Position + velocity
- **Baro correction** (50Hz): Altitude
- **Mag correction** (100Hz): Heading

Uses Eigen for math. Outputs estimated `AircraftState` fanned out to Controller, Autonomy, and MavlinkGateway.

---

## Controller

Single component containing the full flight control system:

- **TECS** (Total Energy Control System): altitude error + airspeed error → pitch command + throttle command
- **L1 Navigation**: heading error → roll command (bank angle for coordinated turn)
- **Rate PIDs**: desired roll/pitch/yaw rates → aileron/elevator/rudder surface deflections

**Input**: `GuidanceCmd { desired_alt_m, desired_airspeed_ms, desired_heading_deg }` + estimated `AircraftState`
**Output**: `SurfaceCmd { aileron, elevator, rudder, throttle }`

Controller does not know about flight modes — it always receives the same `GuidanceCmd` format from Autonomy.

---

## Autonomy

Mode state machine + guidance logic in a single component. Always in the data path.

**Modes:**
- **FBWB**: Maps QGC joystick sticks to `GuidanceCmd` (pitch → desired altitude rate, roll → desired heading rate, throttle → desired airspeed)
- **Auto**: Computes `GuidanceCmd` from waypoint list + estimated state (bearing to next waypoint → desired heading, waypoint alt → desired alt, waypoint speed → desired airspeed)

Receives mode change requests from MavlinkGateway. Broadcasts current mode back to MavlinkGateway (for HEARTBEAT).

---

## MAVLink / QGC Integration

**MavlinkGateway** is an active component with its own thread for UDP recv.

- **Protocol**: MAVLink v2, `common` message set
- **UDP**: Send to QGC on `127.0.0.1:14550`, receive on `0.0.0.0:14540`
- **Heartbeat**: 1Hz (sysid=1, compid=1, type=FIXED_WING)
- **Telemetry out** (10Hz): ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS
- **Commands in**: MANUAL_CONTROL (joystick), SET_MODE, MISSION_ITEM_INT
- **Library**: `mavlink/c_library_v2` (header-only, git submodule under `lib/mavlink`)

Compatible with **MissionPlanner** and **QGroundControl** — both speak MAVLink v2 on UDP 14550. fprime-gds (TCP/CCSDS on port 5000) runs in parallel.

---

## Directory Structure

```
AP/
├── CMakeLists.txt
├── Types/
│   ├── CMakeLists.txt
│   └── ApTypes.fpp
├── Ports/
│   ├── CMakeLists.txt
│   └── ApPorts.fpp
├── Math/
│   ├── CMakeLists.txt
│   ├── ApMath.hpp                          # Eigen typedefs, scalar aliases
│   └── CoordTransforms.hpp                 # NED<->LLA, DCM from euler
├── Components/
│   ├── CMakeLists.txt
│   ├── Mavlink/
│   │   └── MavlinkGateway/
│   │       ├── CMakeLists.txt
│   │       ├── MavlinkGateway.fpp, .hpp, .cpp
│   ├── FlightControl/
│   │   ├── Autonomy/
│   │   │   ├── CMakeLists.txt
│   │   │   ├── Autonomy.fpp, .hpp, .cpp
│   │   ├── Controller/
│   │   │   ├── CMakeLists.txt
│   │   │   ├── Controller.fpp, .hpp, .cpp
│   │   └── StateEstimator/
│   │       ├── CMakeLists.txt
│   │       ├── StateEstimator.fpp, .hpp, .cpp
│   └── Sim/
│       ├── SimDynamics/
│       │   ├── CMakeLists.txt
│       │   ├── SimDynamics.fpp, .hpp, .cpp
│       │   └── RigidBody6DOF.hpp/cpp
│       ├── SimServoDriver/
│       │   ├── CMakeLists.txt
│       │   ├── SimServoDriver.fpp, .hpp, .cpp
│       ├── SimImu/
│       │   ├── SimImu.fpp, .hpp, .cpp
│       ├── SimGps/
│       │   ├── SimGps.fpp, .hpp, .cpp
│       ├── SimBaro/
│       │   ├── SimBaro.fpp, .hpp, .cpp
│       └── SimMag/
│           ├── SimMag.fpp, .hpp, .cpp
├── Top/
│   └── Sitl/
│       ├── CMakeLists.txt
│       ├── Main.cpp
│       ├── instances.fpp
│       ├── topology.fpp
│       ├── SitlTopology.cpp
│       └── SitlTopologyDefs.hpp
lib/
├── eigen/                                  # git submodule (Eigen 3.4, header-only)
├── fprime/                                 # git submodule (F Prime v4.1.1)
└── mavlink/                                # git submodule (c_library_v2, header-only)
```

---

## Build System Changes

### `CMakeLists.txt` (top-level) — add Eigen

```cmake
add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE "${CMAKE_CURRENT_LIST_DIR}/lib/eigen")
```

### `CMakePresets.json` — add SITL preset

```json
{
  "name": "sitl",
  "displayName": "SITL Deployment",
  "inherits": "fprime-debug",
  "binaryDir": "${sourceDir}/build-sitl",
  "cacheVariables": {
    "FPRIME_DEPLOYMENT": "${sourceDir}/AP/Top/Sitl"
  }
}
```

### Component CMakeLists.txt pattern

```cmake
set(SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/Component.fpp"
    "${CMAKE_CURRENT_LIST_DIR}/Component.cpp"
)
set(MOD_DEPS Ap_Types Ap_Ports Ap_Math)
register_fprime_module()
target_link_libraries(${MODULE_NAME} PUBLIC eigen)
```

---

## Implementation Phases

### Phase 1: Skeleton — Types, Ports, Rate Groups, GDS Link
1. Add Eigen git submodule, create Math library
2. Create `AP/Types/ApTypes.fpp` with all data types
3. Create `AP/Ports/ApPorts.fpp` with all custom ports
4. Create `AP/Top/Sitl/` deployment: LinuxTimer → RateGroupDriver → 4 RGs + CdhCore + ComCcsds
5. **Verify**: Build, run, connect GDS, see rate group telemetry ticking

### Phase 2: Simulation — 6-DOF + Sim Sensors
1. Implement RigidBody6DOF (pure C++ math, forward Euler, ~12 tunable constants)
2. Implement SimDynamics, SimServoDriver, SimImu, SimGps, SimBaro, SimMag
3. Wire into topology
4. **Verify**: Aircraft state telemetry shows realistic motion in GDS

### Phase 3: Flight Control + QGC Integration
1. **3a — Foundation**: Add FlightMode/GuidanceCmd/MissionWaypoint types, MAVLink submodule
2. **3b — StateEstimator**: Complementary filter, wire sensors → estimator → state fan-out
3. **3c — Controller**: TECS + L1 + PIDs, wire estimator → controller → servo driver
4. **3d — MavlinkGateway (telemetry out)**: POSIX UDP, heartbeat + telemetry → QGC sees aircraft
5. **3e — Autonomy + FBWB**: Mode manager, stick mapping, QGC joystick → aircraft flies
6. **3f — Auto mode**: Waypoint storage, path guidance, mission protocol
7. **Verify**: QGC joystick in FBWB, upload mission in Auto, fprime-gds works in parallel

---

## Key F Prime Patterns to Follow

| Pattern | Reference File |
|---------|---------------|
| Component registration | `lib/fprime/Svc/ActiveRateGroup/CMakeLists.txt` |
| Topology wiring | `lib/fprime/Ref/Top/topology.fpp` |
| Instance declarations | `lib/fprime/Ref/Top/instances.fpp` |
| Deployment Main.cpp | `lib/fprime/Ref/Main.cpp` |
| Deployment lifecycle | `lib/fprime/Ref/Top/RefTopology.cpp` |
| Rate group config | `lib/fprime/Ref/Top/RefTopology.cpp` (line 26, DividerSet) |
| UDP driver | `lib/fprime/Drv/Udp/Udp.fpp` |
| Signal gen (passive + sched) | `lib/fprime/Ref/SignalGen/SignalGen.fpp` |

---

## Verification (End-to-End)

1. Terminal 1: `fprime-util build && ./build-fprime-automatic-native/bin/AP_Top -a 0.0.0.0 -p 5000`
2. Terminal 2 (optional): `fprime-gds --dictionary build-artifacts/Linux/AP_Top/dict/SitlTopologyDictionary.json`
3. Launch QGroundControl — should see heartbeat, aircraft on map, attitude indicator
4. Connect joystick in QGC → FBWB mode: aircraft responds to stick inputs
5. Upload 3-waypoint mission → switch to Auto: aircraft follows waypoints
6. Compare estimated state vs truth state channels in GDS

---

## Dev Notes / Gotchas

- **FPP reserved words**: `throttle`, `state`, `send`, `recv`, `health` need backtick escape in FPP: `$throttle`, `$state`, etc.
- **FPP autocoded accessors**: Field `position_ned` generates `get_position_ned()` / `set_position_ned()` — underscores preserved, `get_`/`set_` prefix added.
- **Output port fan-out**: F Prime output ports are **1-to-1**. To send the same data to N destinations, declare an array: `output port truthStateOut: [4] Ap.StatePort`, then loop over indices in the `.cpp`. Each `[i]` connects to one input port in the topology.
- **Header-only CMake modules**: `register_fprime_module()` requires at least one `.cpp` source. For header-only libraries (e.g. `AP/Math/`), use plain CMake `add_library(Ap_Math INTERFACE)` instead.
- **Config overrides replace entire file**: When overriding `AcConstants.fpp`, you must copy ALL constants from the default file — the override replaces, not merges.
- **Queued component pattern**: Use `sync input port schedIn` + `async input port dataIn`. In `schedIn_handler`, call `this->dispatchCurrentMessages()` to drain **all** queued async messages before processing. Using `doDispatch()` only processes one message — if the sender is faster than the receiver, the queue will overflow and assert.
- **Queue sizing for rate mismatch**: If component A sends at 400Hz to component B's async port, and B drains at 10Hz, the queue needs at least 400/10 = 40 depth. Always use `dispatchCurrentMessages()` and size queues generously (50+).
- **Virtual env for dev**: Use the venv: `source ../venvs/fprime311/bin/activate` - as it contains Python3.11.x as 3.12 was not supported by fprime.

---

## Future Roadmap

- **More flight modes**: LOITER, RTL, LAND
- **Full EKF**: Upgrade StateEstimator from complementary filter to 15-state EKF
- **MAVLink parameter protocol**: Expose F Prime params to QGC for live tuning
- **Hardware**: STM32H7 deployment with FreeRTOS, real sensor drivers (SPI/I2C/UART), PWM servo output
