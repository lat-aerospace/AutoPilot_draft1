module Sitl {

  # ----------------------------------------------------------------------
  # Base ID Convention: 0xDSSCCxxx
  #   D   = Deployment digit (1)
  #   SS  = Subtopology digits (00 for main topology)
  #   CC  = Component digits
  #   xxx = Reserved for internal items (events, commands, telemetry)
  #
  # CdhCore subtopology uses   0x01000000
  # ComCcsds subtopology uses  0x02000000
  # Main topology uses         0x10000000+
  # ----------------------------------------------------------------------

  # ----------------------------------------------------------------------
  # Defaults
  # ----------------------------------------------------------------------

  module Default {
    constant QUEUE_SIZE = 10
    constant STACK_SIZE = 64 * 1024
  }

  # ----------------------------------------------------------------------
  # Active component instances
  # ----------------------------------------------------------------------

  # Rate groups — higher priority number = higher scheduling priority
  # RG1 runs physics + IMU at 400Hz, needs highest priority

  instance rateGroup1Comp: Svc.ActiveRateGroup base id 0x10001000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 45

  instance rateGroup2Comp: Svc.ActiveRateGroup base id 0x10002000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 44

  instance rateGroup3Comp: Svc.ActiveRateGroup base id 0x10003000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 43

  instance rateGroup4Comp: Svc.ActiveRateGroup base id 0x10004000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 42

  # MAVLink gateway (active — own thread for future recv loop)
  instance mavlinkGateway: Ap.MavlinkGateway base id 0x10040000 \
    queue size 20 \
    stack size Default.STACK_SIZE \
    priority 40

  # ----------------------------------------------------------------------
  # Sim component instances (queued — need queue size but no thread)
  # ----------------------------------------------------------------------

  # All sim sensors receive truth state at 400Hz from SimDynamics via async ports.
  # Queue depth must be >= (400 / sensor_rate) to avoid overflow between drain cycles.
  # Add margin for startup timing.

  instance simDynamics: Ap.SimDynamics base id 0x10030000 \
    queue size Default.QUEUE_SIZE

  instance simImu: Ap.SimImu base id 0x10031000 \
    queue size 50

  instance simGps: Ap.SimGps base id 0x10032000 \
    queue size 50

  instance simBaro: Ap.SimBaro base id 0x10033000 \
    queue size 50

  instance simMag: Ap.SimMag base id 0x10034000 \
    queue size 50

  # ----------------------------------------------------------------------
  # Flight control component instances (queued)
  # ----------------------------------------------------------------------

  instance stateEstimator: Ap.StateEstimator base id 0x10043000 \
    queue size 50

  instance autonomy: Ap.Autonomy base id 0x10041000 \
    queue size 20

  # Controller split into 3 components (one per control loop rate)
  # navController:      10Hz  (RG4) — TECS + L1 → DesiredAttitude
  # attitudeController: 50Hz  (RG3) — attitude error → DesiredRates
  # rateController:    100Hz  (RG2) — rate PID → SurfaceCmd

  instance navController: Ap.NavController base id 0x10042000 \
    queue size 20

  instance attitudeController: Ap.AttitudeController base id 0x10044000 \
    queue size 20

  instance rateController: Ap.RateController base id 0x10045000 \
    queue size 20

  instance flightLogger: Ap.FlightLogger base id 0x10046000 \
    queue size 20

  # ----------------------------------------------------------------------
  # Passive component instances
  # ----------------------------------------------------------------------

  instance simServoDriver: Ap.SimServoDriver base id 0x10035000

  instance linuxTimer: Svc.LinuxTimer base id 0x10020000

  instance rateGroupDriverComp: Svc.RateGroupDriver base id 0x10021000

  instance posixTime: Svc.PosixTime base id 0x10022000

  instance comDriver: Drv.TcpClient base id 0x10023000

  instance systemResources: Svc.SystemResources base id 0x10024000

}
