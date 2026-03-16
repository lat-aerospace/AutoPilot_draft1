@ FreeRTOS-based time source component.
@ Provides Fw::Time to all F' components on the STM32 hardware target.
@ Replaces Svc.PosixTime (which requires POSIX clock_gettime).
@
@ Time base:
@   seconds      = xTaskGetTickCount() / configTICK_RATE_HZ
@   microseconds = (xTaskGetTickCount() % configTICK_RATE_HZ) * 1000000
@                  / configTICK_RATE_HZ
@
@ This component is PASSIVE and exposes a single time-server port.
@ It is wired to every component via:
@   time connections instance freeRtosTime
@ in topology.fpp.
module Ap {

  @ FreeRTOS time-source component.
  passive component FreeRtosTime {

    @ Port to retrieve time (matches Svc.Time interface)
    sync input port timeGetPort: Fw.Time

  }

}
