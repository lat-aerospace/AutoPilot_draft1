# AutoPilot — Fixed-Wing SITL Autopilot on F Prime

An end-to-end autopilot software environment built on NASA JPL's F Prime (F') framework. A Python script captures WASD keyboard input as RC stick commands, sends them over UDP to the flight software. Inside F Prime: RC input → control laws → actuator commands → 6-DOF physics sim → simulated sensors → EKF state estimation → telemetry to GDS. Monitor aircraft kinematics in real-time via fprime-gds.

This is the foundation that will later grow into the full autopilot with hardware layers (STM32H7).

---

## Architecture Overview

```
┌─────────────────────┐       UDP (localhost:5010)       ┌──────────────────────────────────────┐
│  Python RC Script   │ ──────────────────────────────>  │  F Prime SITL Deployment             │
│  (WASD keyboard)    │                                  │                                      │
│  ~40 lines          │                                  │  Drv.Udp ──> RcInput                 │
└─────────────────────┘                                  │      │                               │
                                                         │      v                               │
                                                         │  RcInput ──────> AttitudeController  │
                                                         │                      │               │
┌─────────────────────┐       TCP (localhost:5000)       │                      v               │
│  fprime-gds         │ <─────────────────────────────>  │  SimServoDriver ──> SimDynamics      │
│  (telemetry viewer) │                                  │                      │               │
└─────────────────────┘                                  │      ┌──────────────┘               │
                                                         │      v                               │
                                                         │  SimSensors ──> StateEstimator       │
                                                         │                      │               │
                                                         │              telemetry to GDS        │
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
```

### Component Table

| # | Component | Type | Rate | Purpose |
|---|-----------|------|------|---------|
| 1 | **RcInput** | passive | 50Hz | Receives UDP bytes from `Drv.Udp`, deserializes into `RcChannels`, outputs on `RcPort` |
| 2 | **AttitudeController** | queued | 100Hz | PID inner/outer loops (roll/pitch/yaw). RC sticks → desired attitude angles, outputs `SurfaceCmd` |
| 3 | **SimServoDriver** | passive | 100Hz | Receives `SurfaceCmd`, stores it for `SimDynamics` to read |
| 4 | **SimDynamics** | queued | 400Hz | 6-DOF rigid body sim (RK4). Reads surface commands, integrates EOM, outputs truth state |
| 5 | **SimImu** | queued | 400Hz | Reads truth state, adds noise/bias, outputs `ImuData` |
| 6 | **SimGps** | queued | 10Hz | Reads truth state, adds noise + latency, outputs `GpsData` |
| 7 | **SimBaro** | queued | 50Hz | Reads truth state, adds noise, outputs `BaroData` |
| 8 | **SimMag** | queued | 50Hz | Reads truth state, adds distortion, outputs `MagData` |
| 9 | **StateEstimator** | queued | 100Hz | EKF fusing IMU/GPS/Baro/Mag → outputs estimated `AircraftState` |

Queued components receive cross-rate-group data on **async input ports** (thread-safe message queue). On `schedIn`, they call `doDispatch()` to drain queued messages, then run their algorithm. This avoids race conditions between rate group threads.

### Framework Instances (from F Prime `Svc/` and `Drv/`)

- `Svc.LinuxTimer` — 400Hz base tick (2.5ms)
- `Svc.RateGroupDriver` — divides into rate groups
- `Svc.ActiveRateGroup` x4 — RG1-RG4
- `Drv.Udp` — receives RC input on port 5010
- `Svc.PosixTime` — timestamps
- CdhCore subtopology — command dispatch, events, health
- ComCcsds subtopology — GDS communication framing
- `Drv.TcpClient` — GDS link on port 5000

---

## Rate Group Design

**Base tick**: 400Hz (2.5ms via `LinuxTimer`)

| Rate Group | Freq | Divisor | Members (execution order) |
|------------|------|---------|---------------------------|
| RG1 | 400Hz | 1 | SimDynamics, SimImu |
| RG2 | 100Hz | 4 | StateEstimator, AttitudeController, SimServoDriver |
| RG3 | 50Hz | 8 | RcInput, SimBaro, SimMag |
| RG4 | 10Hz | 40 | SimGps, health, telemetry downlink |

---

## Closed-Loop Data Flow

Each tick, components execute in order within their rate group:

```
RG1 (400Hz):
  SimDynamics.schedIn → doDispatch() drains queued SurfaceCmd
                      → integrates 6-DOF by dt=2.5ms → outputs truth state
  SimImu.schedIn      → doDispatch() drains queued truth state
                      → adds noise → outputs ImuData

RG2 (100Hz):
  StateEstimator.schedIn      → doDispatch() drains queued ImuData/GpsData/BaroData/MagData
                               → runs EKF predict+update → outputs AircraftState
  AttitudeController.schedIn  → doDispatch() drains queued RcChannels + AircraftState
                               → RC sticks map to desired attitude angles
                               → PID compute → outputs SurfaceCmd
  SimServoDriver.schedIn      → reads SurfaceCmd, stores for SimDynamics

RG3 (50Hz):
  RcInput.schedIn     → checks Drv.Udp for new data → outputs RcChannels
  SimBaro.schedIn     → doDispatch() drains queued truth state → outputs BaroData
  SimMag.schedIn      → doDispatch() drains queued truth state → outputs MagData

RG4 (10Hz):
  SimGps.schedIn      → doDispatch() drains queued truth state → outputs GpsData
```

**Loop closure**: SimDynamics (RG1) reads `SurfaceCmd` written by SimServoDriver (RG2) in the previous cycle. One-cycle latency is realistic and acceptable.

---

## SimDynamics 6-DOF Engine

Pure C++ classes (no F Prime dependency in the math):

| Class | Purpose |
|-------|---------|
| `RigidBody6DOF` | 13-state vector [pos_NED(3), vel_body(3), quat(4), omega_body(3)]. RK4 integration. |
| `AeroModel` | Forces & moments from alpha, beta, airspeed, rates, surfaces. Cessna 172-class stability derivatives. |
| `PropModel` | Thrust = f(throttle, airspeed). Simple first-order. |
| `AtmosphereModel` | ISA standard atmosphere: density, pressure, temperature vs altitude. |

The `SimDynamics` F Prime component wraps these classes, calling `RigidBody6DOF::step(dt)` on each `schedIn`.

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

## State Estimator (EKF)

**15-state vector**: position_NED(3), velocity_NED(3), attitude_euler(3), gyro_bias(3), accel_bias(3)

- **Predict** (100Hz): Propagate with IMU (strapdown integration)
- **GPS update** (10Hz): Position + velocity measurement
- **Baro update** (50Hz): Altitude measurement
- **Mag update** (50Hz): Heading measurement

Uses Eigen for matrix math.

---

## Attitude Controller

**Outer loop** (angle): PID on roll_error, pitch_error → rate commands
**Inner loop** (rate): PID on roll_rate_error, pitch_rate_error, yaw_rate_error → surface deflections

RC stick maps to desired attitude angles (roll: +/-45 deg, pitch: +/-20 deg).
Controller computes PID to track commanded angles. Throttle passes through directly.

---

## Python RC Script (`tools/rc_input.py`)

~40 lines. Uses `curses` + `socket`:

```
W → pitch stick forward (nose down)     S → pitch stick back (nose up)
A → roll stick left                      D → roll stick right
Q → yaw left                             E → yaw right
Shift+W / Shift+S → throttle up/down
```

Sends a 16-byte UDP packet (4x float32: roll, pitch, yaw, throttle) to `localhost:5010` at ~50Hz.

---

## Directory Structure

```
AP/
├── CMakeLists.txt                          # extends existing: adds Types, Ports, Math, Components, Top/Sitl
├── Types/
│   ├── CMakeLists.txt
│   └── ApTypes.fpp
├── Ports/
│   ├── CMakeLists.txt
│   └── ApPorts.fpp
├── Math/
│   ├── CMakeLists.txt
│   ├── ApMath.hpp                          # Eigen typedefs, scalar aliases
│   └── CoordTransforms.hpp/cpp             # NED<->LLA, DCM from euler
├── Components/
│   ├── CMakeLists.txt                      # includes all subdirs
│   ├── RcInput/
│   │   ├── CMakeLists.txt
│   │   ├── RcInput.fpp
│   │   ├── RcInput.hpp
│   │   └── RcInput.cpp
│   ├── AttitudeController/
│   │   ├── CMakeLists.txt
│   │   ├── AttitudeController.fpp
│   │   ├── AttitudeController.hpp
│   │   ├── AttitudeController.cpp
│   │   ├── PID.hpp
│   │   └── PID.cpp
│   ├── StateEstimator/
│   │   ├── CMakeLists.txt
│   │   ├── StateEstimator.fpp
│   │   ├── StateEstimator.hpp
│   │   └── StateEstimator.cpp
│   └── Sim/
│       ├── SimDynamics/
│       │   ├── CMakeLists.txt
│       │   ├── SimDynamics.fpp
│       │   ├── SimDynamics.hpp
│       │   ├── SimDynamics.cpp
│       │   ├── RigidBody6DOF.hpp/cpp
│       │   ├── AeroModel.hpp/cpp
│       │   ├── PropModel.hpp/cpp
│       │   └── AtmosphereModel.hpp/cpp
│       ├── SimImu/
│       │   ├── CMakeLists.txt
│       │   ├── SimImu.fpp, .hpp, .cpp
│       ├── SimGps/
│       │   ├── CMakeLists.txt
│       │   ├── SimGps.fpp, .hpp, .cpp
│       ├── SimBaro/
│       │   ├── CMakeLists.txt
│       │   ├── SimBaro.fpp, .hpp, .cpp
│       ├── SimMag/
│       │   ├── CMakeLists.txt
│       │   ├── SimMag.fpp, .hpp, .cpp
│       └── SimServoDriver/
│           ├── CMakeLists.txt
│           ├── SimServoDriver.fpp, .hpp, .cpp
├── Top/
│   └── Sitl/
│       ├── CMakeLists.txt
│       ├── Main.cpp
│       ├── instances.fpp
│       ├── topology.fpp
│       ├── SitlTopology.cpp
│       └── SitlTopologyDefs.hpp
tools/
└── rc_input.py                             # Python keyboard RC script
```

### Top-level additions

- `lib/eigen/` — git submodule (Eigen 3.4, header-only)
- `CMakePresets.json` — add `sitl` preset

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
1. Add Eigen git submodule
2. Create `AP/Types/ApTypes.fpp` with all data types
3. Create `AP/Ports/ApPorts.fpp` with all custom ports
4. Create `AP/Math/` with Eigen wrappers and coordinate transforms
5. Create `AP/Top/Sitl/` deployment: wire LinuxTimer → RateGroupDriver → 4 ActiveRateGroups + CdhCore + ComCcsds subtopologies
6. Add `sitl` build preset
7. **Verify**: Build, run, connect GDS, see rate group telemetry ticking

### Phase 2: Simulation — 6-DOF + Sim Sensors
1. Implement `SimDynamics` with RigidBody6DOF, AeroModel, PropModel, AtmosphereModel
2. Implement SimImu, SimGps, SimBaro, SimMag, SimServoDriver
3. Wire into topology, send constant surface deflection via GDS command on SimServoDriver
4. **Verify**: Aircraft state telemetry shows realistic motion (pitch up when elevator applied, etc.)

### Phase 3: RC Input
1. Implement `RcInput` component (deserializes UDP → RcChannels)
2. Write `tools/rc_input.py`
3. Wire Drv.Udp + RcInput → AttitudeController
4. **Verify**: Press WASD keys → aircraft responds in real-time in GDS telemetry

### Phase 4: State Estimation
1. Implement `StateEstimator` (15-state EKF using Eigen)
2. Wire sensor outputs → StateEstimator → telemetry
3. **Verify**: Estimated state tracks truth state (compare in GDS)

### Phase 5: Attitude Control (Closed Loop)
1. Implement PID utility class
2. Implement `AttitudeController`
3. Wire: StateEstimator + RcInput → AttitudeController → SimServoDriver
4. **Verify**: WASD commands desired attitudes. Aircraft holds stable. Release keys → returns to level flight.

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

## Notes

Keep in mind to use the venv: `source ../venvs/fprime311/bin/activate`

---

## Verification (End-to-End)

1. Terminal 1: `fprime-util build sitl && fprime-util gds sitl`
2. Terminal 2: `python3 tools/rc_input.py`
3. Open GDS dashboard in browser
4. Press W/A/S/D — observe attitude changes in GDS telemetry plots
5. Release keys — aircraft returns to level flight
6. Compare estimated state vs truth state channels — EKF tracks accurately

---

## Future Roadmap

- **Guidance**: L1 lateral navigation, TECS airspeed/altitude control
- **Autonomy**: Waypoint mission manager, mode state machine (AUTO, LOITER, RTL, LAND)
- **Hardware**: STM32H7 deployment with FreeRTOS, real sensor drivers (SPI/I2C/UART), PWM servo output
- **Ground Station**: Evolve Python RC script into a full mission planner GUI
