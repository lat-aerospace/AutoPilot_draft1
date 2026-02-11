module Ap {

    @ Port for IMU sensor data
    port ImuPort(ref data: ImuData)

    @ Port for GPS sensor data
    port GpsPort(ref data: GpsData)

    @ Port for barometric altimeter data
    port BaroPort(ref data: BaroData)

    @ Port for magnetometer data
    port MagPort(ref data: MagData)

    @ Port for full aircraft state (truth or estimated)
    port StatePort(ref $state: AircraftState)

    @ Port for control surface commands
    port SurfaceCmdPort(ref cmd: SurfaceCmd)

    @ Port for RC receiver channels
    port RcPort(ref rc: RcChannels)

}
