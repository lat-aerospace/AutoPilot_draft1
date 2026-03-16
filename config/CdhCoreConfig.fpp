module CdhCoreConfig {
    # Base ID for the CdhCore Subtopology
    constant BASE_ID = 0x01000000

    module QueueSizes {
        constant cmdDisp     = 10
        constant events      = 10
        constant tlmSend     = 10
        constant $health     = 25
    }

    # Reduced stack sizes for embedded target (STM32H745, ~640 KB total SRAM).
    # Default F' uses 64 KB per task which is for workstation deployments.
    module StackSizes {
        constant cmdDisp     = 8 * 1024
        constant events      = 8 * 1024
        constant tlmSend     = 8 * 1024
    }

    module Priorities {
        constant cmdDisp     = 35
        constant $health     = 24
        constant events      = 23
        constant tlmSend     = 22
    }
}
