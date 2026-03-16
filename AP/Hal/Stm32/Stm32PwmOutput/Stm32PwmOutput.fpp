@ STM32 PWM output driver for servo and ESC control.
@ Uses TIM1 (or TIM8) in PWM mode: 8 channels at 50 Hz (20 ms period).
@ Pulse width range 1000–2000 µs (standard RC PWM).
@ Replaces Ap.SimServoDriver in the hardware topology.
module Ap {

  @ PWM output component. Converts SurfaceCmd to RC-PWM pulses.
  passive component Stm32PwmOutput {

    # ------------------------------------------------------------------
    # Input – surface deflection commands from Controller
    # ------------------------------------------------------------------
    @ Surface commands: aileron, elevator, rudder, throttle (+ 4 aux)
    sync input port surfaceCmdIn: Ap.SurfaceCmdPort

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
