module Ap {

    @ Simulated GPS sensor.
    @ Receives truth state, adds noise, outputs GpsData at 10Hz.
    queued component SimGps {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (10Hz — RG4)
        sync input port schedIn: Svc.Sched

        @ Truth aircraft state input (async — from SimDynamics)
        async input port truthStateIn: Ap.StatePort

        @ Noisy GPS data output
        output port gpsOut: Ap.GpsPort

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
