module Ap {

    @ Autonomy — flight mode manager and guidance command generator.
    @ FBWB: maps RC sticks to GuidanceCmd.
    @ Auto: waypoints + state → GuidanceCmd (future).
    queued component Autonomy {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (10Hz from RG4)
        sync input port schedIn: Svc.Sched

        @ RC stick inputs (async — from MavlinkGateway)
        async input port rcIn: Ap.RcPort

        @ Estimated aircraft state (async — from StateEstimator)
        async input port stateIn: Ap.StatePort

        @ Guidance command output (to Controller)
        output port guidanceCmdOut: Ap.GuidanceCmdPort

        @ Current mode broadcast [0]=MavlinkGateway
        output port modeOut: [1] Ap.ModePort

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Current flight mode
        telemetry currentMode: Ap.FlightMode

        @ Desired altitude (m MSL)
        telemetry desAlt: F64

        @ Desired airspeed (m/s)
        telemetry desAirspeed: F64

        @ Desired heading (deg)
        telemetry desHeading: F64

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
