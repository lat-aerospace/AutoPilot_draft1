# ======================================================================
# PlatformCfg.fpp — Project overrides for OS handle storage sizes
#
# FreeRTOS implementations embed StaticSemaphore_t (72 bytes on ARM)
# in handles, which exceeds the F' defaults (40 for Task, 72 for Mutex).
# ======================================================================

@ Maximum size of a handle for Os::Task (FreeRTOS: ~120 bytes)
constant FW_TASK_HANDLE_MAX_SIZE = 160

@ Maximum size of a handle for Os::Mutex (FreeRTOS: ~96 bytes)
constant FW_MUTEX_HANDLE_MAX_SIZE = 128

@ Maximum size of a handle for Os::ConditionVariable
constant FW_CONDITION_VARIABLE_HANDLE_MAX_SIZE = 64

@ Maximum size of a handle for Os::RawTime
constant FW_RAW_TIME_HANDLE_MAX_SIZE = 56

@ Maximum size of a handle for Os::Console
constant FW_CONSOLE_HANDLE_MAX_SIZE = 24

@ Maximum size of a handle for Os::File
constant FW_FILE_HANDLE_MAX_SIZE = 16

@ Maximum size of a handle for Os::Queue (PriorityQueue embeds Mutex + 2 CondVars)
constant FW_QUEUE_HANDLE_MAX_SIZE = 640

@ Maximum size of a handle for Os::Directory
constant FW_DIRECTORY_HANDLE_MAX_SIZE = 16

@ Maximum size of a handle for Os::FileSystem
constant FW_FILESYSTEM_HANDLE_MAX_SIZE = 16

@ Maximum size of a handle for Os::Cpu
constant FW_CPU_HANDLE_MAX_SIZE = 16

@ Maximum size of a handle for Os::Memory
constant FW_MEMORY_HANDLE_MAX_SIZE = 16

@ Alignment of handle storage
constant FW_HANDLE_ALIGNMENT = 8

@ Maximum allowed serialization size for Os::RawTime objects
constant FW_RAW_TIME_SERIALIZATION_MAX_SIZE = 8

@ Chunk size for working with files in the OSAL layer
constant FW_FILE_CHUNK_SIZE = 512
