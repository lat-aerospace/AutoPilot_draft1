module Ap {

    @ Simulated IMU sensor.
    @ Receives truth state, adds noise, outputs ImuData at 400Hz.
    queued component SimImu {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (400Hz)
        sync input port schedIn: Svc.Sched

        @ Truth aircraft state input (async — from SimDynamics)
        async input port truthStateIn: Ap.StatePort

        @ Noisy IMU data output (to state estimator)
        output port imuOut: Ap.ImuPort

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
