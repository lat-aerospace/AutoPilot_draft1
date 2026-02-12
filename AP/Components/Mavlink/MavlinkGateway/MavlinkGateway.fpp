module Ap {

    @ MAVLink gateway — UDP bridge to MissionPlanner / QGC.
    @ Sends heartbeat + telemetry; receives RC and mission commands (future).
    active component MavlinkGateway {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (10Hz from RG4) — sends telemetry
        async input port schedIn: Svc.Sched

        @ Estimated aircraft state (from StateEstimator)
        async input port stateIn: Ap.StatePort

        @ RC channel output (decoded MANUAL_CONTROL → Autonomy)
        output port rcOut: Ap.RcPort

        @ Current mode input (from Autonomy, for telemetry)
        async input port modeIn: Ap.ModePort

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Number of MAVLink messages sent
        telemetry mavMsgsSent: U32

        @ Number of MAVLink messages received
        telemetry mavMsgsRecvd: U32

        @ Last received MAVLink message ID (for debug)
        telemetry lastRecvMsgId: U32

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Socket opened successfully
        event MavlinkSocketOpened() \
            severity activity high \
            format "MAVLink UDP socket opened"

        @ Socket open failed
        event MavlinkSocketFailed(errno: I32) \
            severity warning high \
            format "MAVLink UDP socket failed to open, errno={}"

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
