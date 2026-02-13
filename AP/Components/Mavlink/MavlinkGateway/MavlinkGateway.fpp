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

        @ RC roll input (-1 to +1). Maps to desHeading via integration at 30 deg/s.
        telemetry rcRoll: F32

        @ RC pitch input (-1 to +1). Maps to desAlt via integration at 5 m/s.
        telemetry rcPitch: F32

        @ RC throttle input (0 to 1). Maps to desAirspeed linearly (20-80 m/s).
        telemetry rcThrottle: F32

        @ RC yaw input (-1 to +1). Currently unused.
        telemetry rcYaw: F32

        @ Number of RC/joystick messages received
        telemetry rcMsgCount: U32

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

        @ Param download requested by GCS
        event ParamRequestReceived(count: U32) \
            severity activity high \
            format "GCS requested param download (sent {} params)"

        @ Received a MAVLink message (for debug, logs msg ID)
        event MavMsgReceived(msgId: U32) \
            severity activity low \
            format "MAVLink msg received: ID={}"

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
