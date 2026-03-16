@ STM32 SPI IMU driver component.
@ Targets ICM-20948 with integrated AK09916 magnetometer.
@ Reads accel + gyro at 400 Hz via SPI + DMA.
@ Reads magnetometer at 100 Hz via ICM-20948 internal I2C master.
@ Replaces Ap.SimImu in the hardware topology.
module Ap {

  @ SPI-based IMU driver.  Produces ImuData at 400 Hz, MagData at 100 Hz.
  passive component Stm32SpiImu {

    # ------------------------------------------------------------------
    # Scheduling – driven by RG1 at 400 Hz
    # ------------------------------------------------------------------
    sync input port schedIn: Svc.Sched

    # ------------------------------------------------------------------
    # Output – sensor data
    # ------------------------------------------------------------------
    @ Raw IMU measurement (accel + gyro + timestamp)
    output port imuOut: Ap.ImuPort

    @ Magnetometer measurement from AK09916 (body-frame, Gauss)
    output port magOut: Ap.MagPort

    # ------------------------------------------------------------------
    # Telemetry, events, time
    # ------------------------------------------------------------------
    command recv port cmdIn
    command reg  port cmdRegOut
    command resp port cmdResponseOut

    event      port logOut
    text event port logTextOut
    time get   port timeGetOut
    telemetry  port tlmOut

  }

}
