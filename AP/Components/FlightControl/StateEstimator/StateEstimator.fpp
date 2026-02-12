module Ap {

    @ Complementary filter state estimator.
    @ Fuses IMU, GPS, baro, and mag into estimated AircraftState.
    queued component StateEstimator {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Rate group schedule input (100Hz from RG2)
        sync input port schedIn: Svc.Sched

        @ Sensor inputs (async — from sim sensors)
        async input port imuIn: Ap.ImuPort
        async input port gpsIn: Ap.GpsPort
        async input port baroIn: Ap.BaroPort
        async input port magIn: Ap.MagPort

        @ Estimated state output fan-out:
        @ [0] = Controller, [1] = Autonomy, [2] = MavlinkGateway
        output port stateOut: [3] Ap.StatePort

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Estimated roll (deg)
        telemetry estRoll: F64

        @ Estimated pitch (deg)
        telemetry estPitch: F64

        @ Estimated yaw (deg)
        telemetry estYaw: F64

        @ Estimated altitude MSL (m)
        telemetry estAlt: F64

        @ Estimated airspeed (m/s)
        telemetry estAirspeed: F64

        # ----------------------------------------------------------------------
        # Standard AC Ports
        # ----------------------------------------------------------------------

        time get port timeCaller

        import Fw.Event

        import Fw.Channel

    }

}
