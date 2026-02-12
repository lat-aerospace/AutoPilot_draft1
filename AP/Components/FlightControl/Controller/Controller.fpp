module Ap {

    @ Simple proportional flight controller.
    @ Takes GuidanceCmd + estimated AircraftState, outputs SurfaceCmd.
    queued component Controller {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (100Hz from RG2)
        sync input port schedIn: Svc.Sched

        @ Guidance command input (async — from Autonomy)
        async input port guidanceCmdIn: Ap.GuidanceCmdPort

        @ Estimated aircraft state input (async — from StateEstimator)
        async input port stateIn: Ap.StatePort

        @ Surface command output (to SimServoDriver)
        output port surfaceCmdOut: Ap.SurfaceCmdPort

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Commanded aileron
        telemetry cmdAileron: F64

        @ Commanded elevator
        telemetry cmdElevator: F64

        @ Commanded throttle
        telemetry cmdThrottle: F64

        @ Heading error (deg)
        telemetry headingErr: F64

        @ Altitude error (m)
        telemetry altErr: F64

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
