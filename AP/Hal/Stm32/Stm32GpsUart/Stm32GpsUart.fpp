@ STM32 UART GPS driver.
@ Parses UBLOX UBX-NAV-PVT messages from a UBLOX M8N/M9N module connected
@ to USART1 (or any USART configured at 115200 baud).
@ Outputs GpsData at up to 10 Hz (rate configured in UBLOX by UBX-CFG-RATE).
@ Replaces Ap.SimGps in the hardware topology.
module Ap {

  @ UBLOX GPS UART driver. Parses UBX-NAV-PVT messages.
  passive component Stm32GpsUart {

    # ------------------------------------------------------------------
    # Scheduling – polled via RG4 at 10 Hz to drain DMA receive buffer
    # ------------------------------------------------------------------
    sync input port schedIn: Svc.Sched

    # ------------------------------------------------------------------
    # Output – GPS measurement (position + velocity + fix quality)
    # ------------------------------------------------------------------
    output port gpsOut: Ap.GpsPort

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

  }

}
