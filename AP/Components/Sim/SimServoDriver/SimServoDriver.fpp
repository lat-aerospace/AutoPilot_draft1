module Ap {

    @ Passive servo driver for simulation.
    @ Receives SurfaceCmd and forwards to SimDynamics.
    passive component SimServoDriver {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Surface command input (from autopilot or GDS)
        sync input port surfaceCmdIn: Ap.SurfaceCmdPort

        @ Surface command output (to SimDynamics)
        output port surfaceCmdOut: Ap.SurfaceCmdPort

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Aileron command (normalized)
        telemetry aileron: F64

        @ Elevator command (normalized)
        telemetry elevator: F64

        @ Rudder command (normalized)
        telemetry rudder: F64

        @ Throttle command (0-1)
        telemetry $throttle: F64

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Emitted on first command received
        event FirstCmdReceived() \
            severity activity high \
            format "SimServoDriver: first surface command received"

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
