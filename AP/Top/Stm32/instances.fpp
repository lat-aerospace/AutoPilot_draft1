module Stm32 {

  # ----------------------------------------------------------------------
  # Base ID Convention:
  #   0x20xxxxxx  = STM32 hardware deployment
  #   0x2001xxxx  = Rate groups + timing
  #   0x2002xxxx  = HAL sensor drivers
  #   0x2003xxxx  = HAL actuators
  #   0x2004xxxx  = Communications
  #   0x2005xxxx  = Flight control
  # ----------------------------------------------------------------------

  # ----------------------------------------------------------------------
  # Memory defaults for embedded targets
  #
  # IMPORTANT: Stack sizes are in bytes.  The FreeRTOS task abstraction
  # in AP/Os/FreeRtos/Task.cpp converts bytes → StackType_t words.
  # Smaller stacks than SITL (64 KB → 4-8 KB) to fit within on-chip SRAM.
  # ----------------------------------------------------------------------

  module Default {
    constant QUEUE_SIZE  = 10
    constant STACK_SIZE  = 4 * 1024     # 4 KB default
    constant CTRL_STACK  = 8 * 1024     # 8 KB for Eigen-heavy components
    constant COMM_STACK  = 8 * 1024     # 8 KB for MAVLink / protocol tasks
  }

  # ----------------------------------------------------------------------
  # Rate group instances
  # Priority mapping:  F'45→RTOS7, F'44→RTOS6, F'43→RTOS5, F'42→RTOS4
  # ----------------------------------------------------------------------

  instance rateGroup1Comp: Svc.ActiveRateGroup base id 0x20011000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 45

  instance rateGroup2Comp: Svc.ActiveRateGroup base id 0x20012000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.CTRL_STACK \
    priority 44

  instance rateGroup3Comp: Svc.ActiveRateGroup base id 0x20013000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 43

  instance rateGroup4Comp: Svc.ActiveRateGroup base id 0x20014000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 42

  # ----------------------------------------------------------------------
  # HAL timing / OS services
  # stm32Timer is passive – it creates its own internal FreeRTOS task
  # via startTimer() rather than using F' active component machinery.
  # freeRtosTime is passive – time queries are handled synchronously.
  # ----------------------------------------------------------------------

  instance stm32Timer: Ap.Stm32Timer base id 0x20015000

  instance rateGroupDriverComp: Svc.RateGroupDriver base id 0x20016000

  instance freeRtosTime: Ap.FreeRtosTime base id 0x20017000

  # ----------------------------------------------------------------------
  # HAL sensor drivers (queued – scheduled by rate groups)
  # stm32Imu handles both IMU (accel+gyro) and magnetometer (AK09916
  # via ICM-20948's internal I2C master). No separate mag component.
  # ----------------------------------------------------------------------

  instance stm32Imu: Ap.Stm32SpiImu base id 0x20020000

  instance stm32Baro: Ap.Stm32I2cBaro base id 0x20021000

  instance stm32Gps: Ap.Stm32GpsUart base id 0x20023000

  # ----------------------------------------------------------------------
  # HAL actuator
  # ----------------------------------------------------------------------

  instance stm32Pwm: Ap.Stm32PwmOutput base id 0x20030000

  # ----------------------------------------------------------------------
  # Communications
  # ----------------------------------------------------------------------

  instance mavlinkUart: Ap.Stm32Uart base id 0x20040000 \
    queue size 50 \
    stack size Default.COMM_STACK \
    priority 40

  # ----------------------------------------------------------------------
  # Flight control (queued – driven by rate groups)
  # ----------------------------------------------------------------------

  instance stateEstimator: Ap.StateEstimator base id 0x20050000 \
    queue size 50

  instance autonomy: Ap.Autonomy base id 0x20051000 \
    queue size 20

  instance controller: Ap.Controller base id 0x20052000 \
    queue size 20

}
