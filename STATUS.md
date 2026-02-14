# Status of AutoPilot project

**Current state (Feb 14 2026):** Basic stable flight achieved. Aircraft holds altitude, heading, and airspeed in closed-loop SITL without diverging. QGC joystick (FBWB) controls the aircraft end-to-end. Now furnishing — upgrading sim fidelity, state estimation, and control.

---

## Part A: Project Setup [COMPLETE]

Building the skeleton, wiring components, getting the loop running.

### A1: Skeleton [COMPLETE]

- Eigen 3.4.0, `AP/Math/`, `AP/Types/`, `AP/Ports/`
- SITL topology: LinuxTimer 400Hz, 4 rate groups (400/100/50/10Hz), CdhCore + ComCcsds, TcpClient for GDS
- Binary runs, GDS connects, rate group telemetry ticking

### A2: Simulation — 6-DOF + Sim Sensors [COMPLETE]

- `RigidBody6DOF` — 13-state vector, RK4 integrator, inline force/moment model
- `SimDynamics` (queued 400Hz), `SimServoDriver` (passive), `SimImu` (400Hz), `SimGps` (10Hz), `SimBaro` (50Hz), `SimMag` (100Hz)
- All wired into topology, truth state verified in GDS

### A3: Flight Control + GCS Integration [COMPLETE]

**A3a — Foundation:** FlightMode/GuidanceCmd/MissionWaypoint types, MAVLink c_library_v2 submodule.

**A3b — StateEstimator:** Complementary filter (α=0.98), proper Euler kinematic gyro propagation + accel/mag fusion. Sensor availability guard. Position from GPS→NED, altitude from baro.

**A3c — Controller:** Cascaded P controllers: heading→roll→aileron, altitude→pitch→elevator, airspeed→throttle.

**A3d — MavlinkGateway:** POSIX UDP. Heartbeat 1Hz (`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_GENERIC`). Telemetry 10Hz: ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS. Parses MANUAL_CONTROL, RC_CHANNELS_OVERRIDE, COMMAND_LONG, PARAM_REQUEST_LIST/READ. Minimal param table for GCS handshake.

**A3e — Autonomy + FBWB:**

| RC Input | Range | Guidance Output | Mapping |
|----------|-------|-----------------|---------|
| `rcPitch` | -1 to +1 | `desAlt` | Integrates at ±5 m/s per full stick |
| `rcRoll` | -1 to +1 | `desHeading` | Integrates at ±30°/s per full stick |
| `rcThrottle` | 0 to 1 | `desAirspeed` | Linear map to 20-80 m/s |
| `rcYaw` | -1 to +1 | *(unused)* | — |

**GCS:** QGC over UDP 14550. Throttle on SB stick (main stick mapping workaround).

### A4: Stability Tuning [COMPLETE]

Fixes to achieve basic stable flight:
- Lift trim: liftCoeff 0.55 → 4.09 (mass×g/V² at 50 m/s)
- RK4 integrator replacing forward Euler
- Static stability: pitch stiffness (Cma = 5000 Nm/rad), yaw weathercock (Cnβ = 2000 Nm/rad)
- Sensor init guard: StateEstimator waits for all 4 sensors before first output
- Euler kinematics: proper body-rate → Euler-rate conversion
- Controller gains tuned (HDG=0.08, ROLL=0.005, ALT=0.02, PITCH=0.01, SPD=0.01)
- Initial conditions: throttle 0.5 at start, default altitude = 2600m MSL

---

## Part B: Furnishing [IN PROGRESS]

Upgrading each block from "works" to "realistic/robust". Each phase is independent — tick items as we go.

### B1: Simulation Upgrade

- [x] Stability-derivative aero model (25+ coefficients, Cessna 172 params, post-stall rolloff)
- [x] ISA atmosphere model (density, temperature, pressure vs altitude)
- [x] Propeller thrust model (density + airspeed effects)
- [x] Wind model (steady wind NED + Dryden turbulence, disabled by default)
- [ ] Ground contact model (landing gear, ground reaction)
- [x] Sensor error models:
  - [x] IMU: bias random walk + gyro scale error + white noise
  - [x] GPS: 300ms delay buffer + position/velocity noise
  - [x] Baro: slow bias drift (random walk) + white noise
  - [x] Mag: hard-iron offset + soft-iron distortion + white noise

### B2: State Estimator Upgrade

- [ ] Extended Kalman Filter (EKF) replacing complementary filter
- [ ] 15-state EKF: attitude(4) + velocity(3) + position(3) + gyro bias(3) + accel bias(3)
- [ ] Proper measurement models for GPS, baro, mag
- [ ] Gyro bias estimation and removal
- [ ] Magnetic declination correction
- [ ] GPS-denied fallback (IMU-only coast)

### B3: Controller Upgrade

- [ ] PID loops replacing P-only (integral for steady-state, derivative for damping)
- [ ] TECS (Total Energy Control System) for coordinated altitude + speed
- [ ] L1 navigation controller for path following
- [ ] Gain scheduling with airspeed
- [ ] Rate-limited setpoints (smooth transitions)
- [ ] Auto-trim (trim elevator/throttle for level flight)

### B4: Auto Mode + Mission

- [ ] Waypoint storage + path guidance in Autonomy
- [ ] MISSION protocol in MavlinkGateway (upload/download waypoints from QGC)
- [ ] Loiter mode (orbit a point)
- [ ] RTL (return to launch)
- [ ] Test: upload mission in QGC, switch to Auto, aircraft follows waypoints

---

## Known Issues

- **Magnetic declination**: estYaw reads ~345° instead of 0° (Albuquerque reference point)
- **P-only controller**: No integral term → steady-state errors in altitude/heading
- **Wind disabled by default**: WindModel exists but needs controller tuning before enabling
- **QGC throttle axis**: Cannot map throttle to main joystick stick — using SB stick
