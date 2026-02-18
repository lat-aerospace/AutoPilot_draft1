module Ap {

    @ Autonomy — flight mode manager and guidance command generator.
    @ FBWB: maps RC sticks to integrated GuidanceCmd.
    @ AUTO: follows mission waypoints via AP_Mission.
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

        @ Guidance command output [0]=NavController [1]=FlightLogger
        output port guidanceCmdOut: [2] Ap.GuidanceCmdPort

        @ Current mode broadcast [0]=MavlinkGateway
        output port modeOut: [1] Ap.ModePort

        # ----------------------------------------------------------------------
        # Live-tuning commands (mission management + mode control)
        # ----------------------------------------------------------------------

        @ Add a waypoint to the mission
        sync command AddWaypoint(lat: F64, lon: F64, alt: F32, speed: F32)

        @ Clear all waypoints and return to FBWB mode
        sync command ClearMission()

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

        @ Waypoint count in mission
        telemetry wpCount: U16

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        @ Port for receiving commands
        command recv port CmdDisp

        @ Port for sending command registration requests
        command reg port CmdReg

        @ Port for sending command responses
        command resp port CmdStatus

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
