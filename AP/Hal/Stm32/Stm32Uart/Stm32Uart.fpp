@ STM32 UART driver component.
@ Generic UART with DMA for MAVLink telemetry (USART2) and UBLOX GPS (USART1).
@ Each sensor uses its own instance with a different USART peripheral.
@ Replaces Ap.MavlinkGateway (UDP) for the hardware telemetry link.
module Ap {

  @ UART driver providing MAVLink telemetry downlink and RC override uplink.
  active component Stm32Uart {

    # ------------------------------------------------------------------
    # Scheduling
    # ------------------------------------------------------------------
    async input port schedIn: Svc.Sched

    # ------------------------------------------------------------------
    # Ports shared with MavlinkGateway – same interface for topology reuse
    # ------------------------------------------------------------------

    @ Current estimated aircraft state (for telemetry)
    async input port stateIn: Ap.StatePort

    @ RC channel override from GCS / joystick
    output port rcOut: Ap.RcPort

    @ Flight mode for heartbeat
    async input port modeIn: Ap.ModePort

    @ Mission waypoint output (to Autonomy)
    output port missionWaypointOut: Ap.MissionWaypointPort

    @ Mode command output (to Autonomy, for SET_MODE)
    output port modeCommandOut: Ap.ModePort

    # ------------------------------------------------------------------
    # Commands / events / telemetry
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
