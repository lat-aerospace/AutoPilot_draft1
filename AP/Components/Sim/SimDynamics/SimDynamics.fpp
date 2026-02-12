module Ap {

    @ 6-DOF rigid body simulation. Wraps RigidBody6DOF pure-math class.
    @ Called at 400Hz by RG1. Drains queued SurfaceCmd, steps physics, outputs truth state.
    queued component SimDynamics {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (400Hz)
        sync input port schedIn: Svc.Sched

        @ Surface command input (async — queued from SimServoDriver)
        async input port surfaceCmdIn: Ap.SurfaceCmdPort

        @ Truth aircraft state output (to sim sensors — one per sensor)
        output port truthStateOut: [4] Ap.StatePort

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Position North (m)
        telemetry posN: F64

        @ Position East (m)
        telemetry posE: F64

        @ Position Down (m)
        telemetry posD: F64

        @ Roll angle (deg)
        telemetry roll: F64

        @ Pitch angle (deg)
        telemetry pitch: F64

        @ Yaw angle (deg)
        telemetry yaw: F64

        @ True airspeed (m/s)
        telemetry airspeed: F64

        @ Simulation time (s)
        telemetry simTime: F64

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Emitted once at startup
        event SimStarted() \
            severity activity high \
            format "SimDynamics initialized — 6-DOF running"

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
