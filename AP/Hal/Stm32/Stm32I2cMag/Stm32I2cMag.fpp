@ IST8310 3-axis I2C magnetometer driver.
@ Communicates with an IST8310 (or HMC5883L-pin-compatible device)
@ via I2C at up to 400 kHz (fast mode).
@ Scheduled at 50 Hz by RG3.  Each even cycle triggers a single
@ measurement; each odd cycle reads the result and emits MagData.
@ Replaces Ap.SimMag in the hardware topology.
module Ap {

  @ IST8310 I2C magnetometer driver. Outputs MagData in body frame.
  passive component Stm32I2cMag {

    # ------------------------------------------------------------------
    # Scheduling – polled via RG3 at 50 Hz
    # ------------------------------------------------------------------
    sync input port schedIn: Svc.Sched

    # ------------------------------------------------------------------
    # Output – magnetometer measurement (body-frame, Gauss)
    # ------------------------------------------------------------------
    output port magOut: Ap.MagPort

    # ------------------------------------------------------------------
    # CDH
    # ------------------------------------------------------------------
    command recv port cmdIn
    command reg  port cmdRegOut
    command resp port cmdResponseOut

    event      port logOut
    text event port logTextOut
    time get   port timeGetOut
    telemetry  port tlmOut

    # ------------------------------------------------------------------
    # Telemetry channels
    # ------------------------------------------------------------------
    @ Magnetic field X (body) [Gauss]
    telemetry magX: F32

    @ Magnetic field Y (body) [Gauss]
    telemetry magY: F32

    @ Magnetic field Z (body) [Gauss]
    telemetry magZ: F32

  }

}
