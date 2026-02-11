# Status of AutoPilot project

## Phase 1: Skeleton (see README)

- [x] Eigen 3.4.0 git submodule (`lib/eigen/`)
- [x] Eigen INTERFACE library in top-level CMakeLists
- [x] `AP/Math/` — ApMath.hpp (typedefs, constants), CoordTransforms.hpp (NED/LLA, DCM, quat)
- [x] `AP/Types/ApTypes.fpp` — Vec3, Quat, ImuData, GpsData, BaroData, MagData, AircraftState, SurfaceCmd, RcChannels
- [x] `AP/Ports/ApPorts.fpp` — ImuPort, GpsPort, BaroPort, MagPort, StatePort, SurfaceCmdPort, RcPort
- [x] `config/AcConstants.fpp` — RateGroupDriverRateGroupPorts = 4
- [x] `AP/Top/Sitl/` — SITL topology: LinuxTimer 400Hz, 4 rate groups (400/100/50/10Hz), CdhCore + ComCcsds subtopologies, TcpClient for GDS
- [x] `sitl` build preset in CMakePresets.json
- [x] Build passes (binary + GDS dictionary generated)
- [x] Verify: run binary, connect GDS, see rate group telemetry ticking

## Phase 2-5: Not started

See README for full implementation phases.
