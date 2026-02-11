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

  # ----------------------------------------------------------------------
  # Passive component instances
  # ----------------------------------------------------------------------

  instance linuxTimer: Svc.LinuxTimer base id 0x10020000

  instance rateGroupDriverComp: Svc.RateGroupDriver base id 0x10021000

  instance posixTime: Svc.PosixTime base id 0x10022000

  instance comDriver: Drv.TcpClient base id 0x10023000

  instance systemResources: Svc.SystemResources base id 0x10024000

}
