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

      # --- RG1: 400Hz (physics sim, IMU — members added in Phase 2) ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup1] -> rateGroup1Comp.CycleIn

      # --- RG2: 100Hz (estimator, controller — members added in Phase 2+) ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup2] -> rateGroup2Comp.CycleIn
      rateGroup2Comp.RateGroupMemberOut[0] -> CdhCore.tlmSend.Run
      rateGroup2Comp.RateGroupMemberOut[1] -> CdhCore.cmdDisp.run
      rateGroup2Comp.RateGroupMemberOut[2] -> ComCcsds.comQueue.run
      rateGroup2Comp.RateGroupMemberOut[3] -> ComCcsds.aggregator.timeout

      # --- RG3: 50Hz (RC input, baro, mag — members added in Phase 3) ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup3] -> rateGroup3Comp.CycleIn
      rateGroup3Comp.RateGroupMemberOut[0] -> systemResources.run

      # --- RG4: 10Hz (GPS, health, telemetry downlink) ---
      rateGroupDriverComp.CycleOut[Ports_RateGroups.rateGroup4] -> rateGroup4Comp.CycleIn
      rateGroup4Comp.RateGroupMemberOut[0] -> CdhCore.$health.Run
      rateGroup4Comp.RateGroupMemberOut[1] -> ComCcsds.commsBufferManager.schedIn

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
