module Sitl {

  # ----------------------------------------------------------------------
  # Symbolic constants for rate group port indices
  # ----------------------------------------------------------------------

  enum Ports_RateGroups {
    rateGroup1    # 400Hz
    rateGroup2    # 100Hz
    rateGroup3    #  50Hz
    rateGroup4    #  10Hz
  }

  topology Sitl {

    # ----------------------------------------------------------------------
    # Subtopology imports
    # ----------------------------------------------------------------------

    import CdhCore.Subtopology
    import ComCcsds.Subtopology

    # ----------------------------------------------------------------------
    # Instances used in the topology
    # ----------------------------------------------------------------------

    instance linuxTimer
    instance rateGroupDriverComp
    instance rateGroup1Comp
    instance rateGroup2Comp
    instance rateGroup3Comp
    instance rateGroup4Comp
    instance posixTime
    instance comDriver
    instance systemResources

    # Sim components
    instance simDynamics
    instance simServoDriver
    instance simImu
    instance simGps
    instance simBaro
    instance simMag

    # Flight control components
    instance stateEstimator
    instance autonomy
    instance navController
    instance attitudeController
    instance rateController
    instance flightLogger

    # MAVLink
    instance mavlinkGateway

    # ----------------------------------------------------------------------
    # Pattern graph specifiers
    # ----------------------------------------------------------------------

    command connections instance CdhCore.cmdDisp

    event connections instance CdhCore.events

    telemetry connections instance CdhCore.tlmSend

    text event connections instance CdhCore.textLogger

    health connections instance CdhCore.$health

    time connections instance posixTime

    # ----------------------------------------------------------------------
    # Rate group connections
    # ----------------------------------------------------------------------

    connections RateGroups {

      # Linux timer drives the rate group driver
      linuxTimer.CycleOut -> rateGroupDriverComp.CycleIn

      # --- RG1: 400Hz (physics + IMU) ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup1] -> rateGroup1Comp.CycleIn
      rateGroup1Comp.RateGroupMemberOut[0] -> simDynamics.schedIn
      rateGroup1Comp.RateGroupMemberOut[1] -> simImu.schedIn

      # --- RG2: 100Hz (mag, state estimator, rate controller, CDH services) ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup2] -> rateGroup2Comp.CycleIn
      rateGroup2Comp.RateGroupMemberOut[0] -> simMag.schedIn
      rateGroup2Comp.RateGroupMemberOut[1] -> stateEstimator.schedIn
      rateGroup2Comp.RateGroupMemberOut[2] -> rateController.schedIn
      rateGroup2Comp.RateGroupMemberOut[3] -> CdhCore.tlmSend.Run
      rateGroup2Comp.RateGroupMemberOut[4] -> CdhCore.cmdDisp.run
      rateGroup2Comp.RateGroupMemberOut[5] -> ComCcsds.comQueue.run
      rateGroup2Comp.RateGroupMemberOut[6] -> ComCcsds.aggregator.timeout

      # --- RG3: 50Hz (baro, attitude controller, system resources) ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup3] -> rateGroup3Comp.CycleIn
      rateGroup3Comp.RateGroupMemberOut[0] -> simBaro.schedIn
      rateGroup3Comp.RateGroupMemberOut[1] -> attitudeController.schedIn
      rateGroup3Comp.RateGroupMemberOut[2] -> systemResources.run

      # --- RG4: 10Hz (GPS, autonomy, nav controller, MAVLink, health) ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup4] -> rateGroup4Comp.CycleIn
      rateGroup4Comp.RateGroupMemberOut[0] -> simGps.schedIn
      rateGroup4Comp.RateGroupMemberOut[1] -> autonomy.schedIn
      rateGroup4Comp.RateGroupMemberOut[2] -> navController.schedIn
      rateGroup4Comp.RateGroupMemberOut[3] -> mavlinkGateway.schedIn
      rateGroup4Comp.RateGroupMemberOut[4] -> CdhCore.$health.Run
      rateGroup4Comp.RateGroupMemberOut[5] -> ComCcsds.commsBufferManager.schedIn
      rateGroup4Comp.RateGroupMemberOut[6] -> flightLogger.schedIn
    }

    # ----------------------------------------------------------------------
    # Data flow: closed-loop autopilot chain
    # SimServoDriver → Dynamics → Sensors → StateEstimator
    #   → NavController → AttitudeController → RateController → SimServoDriver
    # ----------------------------------------------------------------------

    connections LAT_Autopilot {

      # Servo → Dynamics → Sensors
      simServoDriver.surfaceCmdOut -> simDynamics.surfaceCmdIn
      simDynamics.truthStateOut[0] -> simImu.truthStateIn
      simDynamics.truthStateOut[1] -> simGps.truthStateIn
      simDynamics.truthStateOut[2] -> simBaro.truthStateIn
      simDynamics.truthStateOut[3] -> simMag.truthStateIn

      # Sensors → StateEstimator
      simImu.imuOut   -> stateEstimator.imuIn
      simGps.gpsOut   -> stateEstimator.gpsIn
      simBaro.baroOut -> stateEstimator.baroIn
      simMag.magOut   -> stateEstimator.magIn

      # StateEstimator fan-out to all consumers
      stateEstimator.stateOut[0] -> navController.stateIn
      stateEstimator.stateOut[1] -> attitudeController.stateIn
      stateEstimator.stateOut[2] -> rateController.stateIn
      stateEstimator.stateOut[3] -> autonomy.stateIn
      stateEstimator.stateOut[4] -> mavlinkGateway.stateIn
      stateEstimator.stateOut[5] -> flightLogger.stateIn

      # MavlinkGateway → Autonomy (RC sticks)
      mavlinkGateway.rcOut -> autonomy.rcIn

      # Autonomy → NavController (guidance command) + FlightLogger
      autonomy.guidanceCmdOut[0] -> navController.guidanceCmdIn
      autonomy.guidanceCmdOut[1] -> flightLogger.guidanceCmdIn

      # Controller cascade: Nav → Attitude → Rate → ServoDriver
      navController.desiredAttOut        -> attitudeController.desiredAttIn
      attitudeController.desiredRatesOut -> rateController.desiredRatesIn
      rateController.surfaceCmdOut       -> simServoDriver.surfaceCmdIn

      # Autonomy → MavlinkGateway (mode for heartbeat)
      autonomy.modeOut[0] -> mavlinkGateway.modeIn
    }

    # ----------------------------------------------------------------------
    # Communications: comDriver <-> ComCcsds.comStub
    # ----------------------------------------------------------------------

    connections Communications {
      # Buffer allocation
      comDriver.allocate   -> ComCcsds.commsBufferManager.bufferGetCallee
      comDriver.deallocate -> ComCcsds.commsBufferManager.bufferSendIn

      # Uplink: comDriver -> comStub
      comDriver.$recv                      -> ComCcsds.comStub.drvReceiveIn
      ComCcsds.comStub.drvReceiveReturnOut -> comDriver.recvReturnIn

      # Downlink: comStub -> comDriver
      ComCcsds.comStub.drvSendOut -> comDriver.$send
      comDriver.ready             -> ComCcsds.comStub.drvConnected
    }

    # ----------------------------------------------------------------------
    # Cross-subtopology: CdhCore <-> ComCcsds
    # ----------------------------------------------------------------------

    connections ComCcsds_CdhCore {
      # Events and telemetry to comQueue for downlink
      CdhCore.events.PktSend   -> ComCcsds.comQueue.comPacketQueueIn[ComCcsds.Ports_ComPacketQueue.EVENTS]
      CdhCore.tlmSend.PktSend  -> ComCcsds.comQueue.comPacketQueueIn[ComCcsds.Ports_ComPacketQueue.TELEMETRY]

      # Router <-> CmdDispatcher (uplinked commands)
      ComCcsds.fprimeRouter.commandOut -> CdhCore.cmdDisp.seqCmdBuff
      CdhCore.cmdDisp.seqCmdStatus    -> ComCcsds.fprimeRouter.cmdResponseIn
    }

  }

}
