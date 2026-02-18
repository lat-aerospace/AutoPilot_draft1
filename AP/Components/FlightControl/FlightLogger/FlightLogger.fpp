module Ap {

    @ FlightLogger — logs AircraftState and GuidanceCmd to a CSV file at 10Hz.
    @ Output: logs/flight_log.csv — importable into Python/matplotlib for plotting.
    queued component FlightLogger {

        @ Schedule input (10Hz — RG4): flushes latest state to CSV
        sync input port schedIn: Svc.Sched

        @ Aircraft state (async — from StateEstimator[5])
        async input port stateIn: Ap.StatePort

        @ Guidance command (async — from Autonomy[1])
        async input port guidanceCmdIn: Ap.GuidanceCmdPort

    }

}
