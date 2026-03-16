@ STM32 I2C barometer driver.
@ Targets MS5611-01BA03 via I2C1 (7-bit address 0x77).
@ Produces BaroData at 50 Hz (scheduled via RG3).
@ Replaces Ap.SimBaro in the hardware topology.
module Ap {

  passive component Stm32I2cBaro {

    sync input port schedIn: Svc.Sched

    output port baroOut: Ap.BaroPort

    command recv port cmdIn
    command reg  port cmdRegOut
    command resp port cmdResponseOut

    event      port logOut
    text event port logTextOut
    time get   port timeGetOut
    telemetry  port tlmOut

  }

}
