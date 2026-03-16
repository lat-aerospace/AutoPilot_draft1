module Stm32 {

  # ----------------------------------------------------------------------
  # Rate group port index names
  # Base tick = 400 Hz (from Stm32Timer → TIM6 ISR → FreeRTOS task)
  # RG1 = 400 Hz (÷1)   IMU
  # RG2 = 100 Hz (÷4)   StateEstimator, Controller
  # RG3 =  50 Hz (÷8)   Barometer
  # RG4 =  10 Hz (÷40)  Autonomy, MAVLink telemetry, GPS, health
  # ----------------------------------------------------------------------

  enum Ports_RateGroups {
    rateGroup1    # 400 Hz
    rateGroup2    # 100 Hz
    rateGroup3    #  50 Hz
    rateGroup4    #  10 Hz
  }

  topology Stm32 {

    # ------------------------------------------------------------------
    # Subtopology imports
    # CdhCore provides: cmdDisp, events, tlmSend, textLogger, health,
    #                   version, fatalAdapter, fatalHandler
    # ComCcsds is NOT used: the bare-metal deployment uses Stm32Uart
    # (MAVLink UART) for all GCS communication; there is no GDS TCP link.
    # ------------------------------------------------------------------

    import CdhCore.Subtopology

    # ------------------------------------------------------------------
    # Component instances declared in instances.fpp
    # ------------------------------------------------------------------

    instance stm32Timer
    instance rateGroupDriverComp
    instance rateGroup1Comp
    instance rateGroup2Comp
    instance rateGroup3Comp
    instance rateGroup4Comp
    instance freeRtosTime

    # HAL sensors
    instance stm32Imu
    instance stm32Baro
    instance stm32Gps

    # HAL actuator
    instance stm32Pwm

    # Flight control
    instance stateEstimator
    instance autonomy
    instance controller

    # Communications
    instance mavlinkUart

    # ------------------------------------------------------------------
    # Pattern graph specifiers
    # ------------------------------------------------------------------

    command   connections instance CdhCore.cmdDisp
    event     connections instance CdhCore.events
    telemetry connections instance CdhCore.tlmSend
    text event connections instance CdhCore.textLogger
    health    connections instance CdhCore.$health
    time      connections instance freeRtosTime

    # ------------------------------------------------------------------
    # Rate group connections
    # ------------------------------------------------------------------

    connections RateGroups {

      # STM32 TIM6 hardware timer drives rate group driver (400 Hz base)
      stm32Timer.CycleOut -> rateGroupDriverComp.CycleIn

      # --- RG1: 400 Hz – IMU ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup1] -> rateGroup1Comp.CycleIn
      rateGroup1Comp.RateGroupMemberOut[0] -> stm32Imu.schedIn

      # --- RG2: 100 Hz – state estimation, control, CDH bookkeeping ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup2] -> rateGroup2Comp.CycleIn
      rateGroup2Comp.RateGroupMemberOut[0] -> stateEstimator.schedIn
      rateGroup2Comp.RateGroupMemberOut[1] -> controller.schedIn
      rateGroup2Comp.RateGroupMemberOut[2] -> CdhCore.tlmSend.Run
      rateGroup2Comp.RateGroupMemberOut[3] -> CdhCore.cmdDisp.run

      # --- RG3: 50 Hz – barometer ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup3] -> rateGroup3Comp.CycleIn
      rateGroup3Comp.RateGroupMemberOut[0] -> stm32Baro.schedIn

      # --- RG4: 10 Hz – autonomy, MAVLink telemetry, GPS parser, health ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup4] -> rateGroup4Comp.CycleIn
      rateGroup4Comp.RateGroupMemberOut[0] -> autonomy.schedIn
      rateGroup4Comp.RateGroupMemberOut[1] -> mavlinkUart.schedIn
      rateGroup4Comp.RateGroupMemberOut[2] -> CdhCore.$health.Run
      rateGroup4Comp.RateGroupMemberOut[3] -> stm32Gps.schedIn

    }

    # ------------------------------------------------------------------
    # LAT_Autopilot: closed-loop flight data flow
    #
    # Hardware data path:
    #   Sensors (IMU/Baro/Mag/GPS) → StateEstimator → Controller → PWM
    #   MavlinkUart (RC/GCS)       → Autonomy → Controller
    #
    # Note: Mag data comes from stm32Imu (ICM-20948's internal I2C master
    #       reads AK09916), not from a separate mag component.
    # ------------------------------------------------------------------

    connections LAT_Autopilot {

      # All sensors → StateEstimator
      stm32Imu.imuOut   -> stateEstimator.imuIn
      stm32Imu.magOut   -> stateEstimator.magIn
      stm32Baro.baroOut -> stateEstimator.baroIn
      stm32Gps.gpsOut   -> stateEstimator.gpsIn

      # StateEstimator → consumers
      # [0] = Controller, [1] = Autonomy, [2] = MavlinkUart
      stateEstimator.stateOut[0] -> controller.stateIn
      stateEstimator.stateOut[1] -> autonomy.stateIn
      stateEstimator.stateOut[2] -> mavlinkUart.stateIn

      # MavlinkUart → Autonomy (RC sticks / GCS commands)
      mavlinkUart.rcOut -> autonomy.rcIn

      # Autonomy → Controller → PWM actuators
      autonomy.guidanceCmdOut  -> controller.guidanceCmdIn
      controller.surfaceCmdOut -> stm32Pwm.surfaceCmdIn

      # Autonomy → MavlinkUart (flight mode for heartbeat)
      autonomy.modeOut[0] -> mavlinkUart.modeIn

      # MavlinkUart → Autonomy (mission waypoints + mode commands)
      mavlinkUart.missionWaypointOut -> autonomy.missionWaypointIn
      mavlinkUart.modeCommandOut -> autonomy.modeCommandIn

    }

  }

}
