module Ap {

    @ Simulated barometric pressure sensor.
    @ Receives truth state, adds noise, outputs BaroData at 50Hz.
    queued component SimBaro {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (50Hz — RG3)
        sync input port schedIn: Svc.Sched

        @ Truth aircraft state input (async — from SimDynamics)
        async input port truthStateIn: Ap.StatePort

        @ Noisy baro data output
        output port baroOut: Ap.BaroPort

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
