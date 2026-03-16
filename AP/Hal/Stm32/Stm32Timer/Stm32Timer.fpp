@ STM32 hardware timer component.
@ Generates the 400 Hz base cycle used by the F' RateGroupDriver.
@ Replaces Svc.LinuxTimer for bare-metal STM32 targets.
@
@ Implementation uses TIM6 (basic timer) configured at the requested rate.
@ The TIM6 ISR signals a private FreeRTOS task via vTaskNotifyGiveFromISR().
@ That task wakes up, reads a TimerVal, and emits CycleOut, which drives
@ the RateGroupDriver exactly like LinuxTimer does on Linux.
@
@ This component is PASSIVE: it creates its own FreeRTOS task internally
@ (via startTimer()), not through the F' active-component threading model.
module Ap {

  @ Hardware timer that drives the F' rate-group scheduler.
  @ Drop-in replacement for Svc.LinuxTimer in the STM32 topology.
  passive component Stm32Timer {

    # ------------------------------------------------------------------
    # Output port – drives RateGroupDriver.CycleIn at the configured rate
    # ------------------------------------------------------------------
    output port CycleOut: Svc.Cycle

    # ------------------------------------------------------------------
    # CDH (events, telemetry, time)
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
