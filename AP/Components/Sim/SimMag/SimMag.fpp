module Ap {

    @ Simulated magnetometer sensor.
    @ Receives truth state, adds noise, outputs MagData at 100Hz.
    queued component SimMag {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (100Hz — RG2)
        sync input port schedIn: Svc.Sched

        @ Truth aircraft state input (async — from SimDynamics)
        async input port truthStateIn: Ap.StatePort

        @ Noisy magnetometer data output
        output port magOut: Ap.MagPort

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
