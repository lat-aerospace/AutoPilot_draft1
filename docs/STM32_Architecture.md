# STM32 Autopilot - Complete System Architecture

This document explains the entire STM32 hardware deployment of the F Prime
autopilot, from power-on to a flying airplane. It is written so that someone
who has never seen F Prime or FreeRTOS can follow along.

---

## Table of Contents

1. [The Big Picture](#1-the-big-picture)
2. [What Happens When You Apply Power](#2-what-happens-when-you-apply-power)
3. [The Three Software Layers](#3-the-three-software-layers)
4. [F Prime in Plain English](#4-f-prime-in-plain-english)
5. [FreeRTOS in Plain English](#5-freertos-in-plain-english)
6. [How F Prime and FreeRTOS Work Together](#6-how-f-prime-and-freertos-work-together)
7. [The Boot Sequence Step by Step](#7-the-boot-sequence-step-by-step)
8. [The 400 Hz Heartbeat - How the Main Loop Works](#8-the-400-hz-heartbeat---how-the-main-loop-works)
9. [Every Hardware Driver Explained](#9-every-hardware-driver-explained)
10. [The OS Abstraction Layer](#10-the-os-abstraction-layer)
11. [Memory Layout on the Chip](#11-memory-layout-on-the-chip)
12. [The Build System](#12-the-build-system)
13. [Complete File Reference](#13-complete-file-reference)
14. [Glossary](#14-glossary)

---

## 1. The Big Picture

This autopilot reads sensors, estimates the aircraft's state, computes
control commands, and drives servo motors -- 400 times per second. It also
talks to a ground station (Mission Planner) over a radio link using the
MAVLink protocol.

Here is the physical picture:

```
                        CUSTOM PCB (STM32H745ZIT6)
                   +-----------------------------------------+
                   |                                         |
  ICM-42688 IMU --[SPI1]--> |                             |--[TIM1 CH1]--> Aileron Servo
  MS5611 Baro  ---[I2C1]--> |        Cortex-M7 @ 480 MHz  |--[TIM1 CH2]--> Elevator Servo
  IST8310 Mag  ---[I2C2]--> |        512 KB SRAM           |--[TIM1 CH3]--> Throttle ESC
  UBLOX GPS  ---[USART1]--> |        1 MB Flash            |--[TIM1 CH4]--> Rudder Servo
  Radio/GCS  ---[USART2]--> |                             |
                   |                                         |
                   +-----------------------------------------+
```

And here is the software picture:

```
  +------------------------------------------------------+
  |              F PRIME COMPONENTS                       |
  |                                                       |
  |  Sensors:       StateEstimator  Controller  Autonomy  |
  |  Stm32SpiImu    (Kalman filter) (PID loops) (FBWB/    |
  |  Stm32I2cBaro                                AUTO)    |
  |  Stm32I2cMag        Stm32Uart (MAVLink)              |
  |  Stm32GpsUart       Stm32PwmOutput (servos)          |
  +------------------------------------------------------+
  |              OS ABSTRACTION (OSAL)                    |
  |  Task -> xTaskCreate    Mutex -> xSemaphoreTake       |
  |  RawTime -> DWT+Tick    Console -> ITM SWO            |
  +------------------------------------------------------+
  |              FreeRTOS KERNEL                          |
  |  Task scheduler, heap_4 memory, tick timer            |
  +------------------------------------------------------+
  |              STM32 HAL + BSP                          |
  |  Clock tree, GPIO pins, DMA, SPI, I2C, UART, TIM     |
  +------------------------------------------------------+
  |              HARDWARE (Cortex-M7 CPU)                 |
  +------------------------------------------------------+
```

---

## 2. What Happens When You Apply Power

Think of it like starting a car. There is a strict sequence:

1. **CPU wakes up** - The Cortex-M7 core starts executing from Flash memory
2. **Basic hardware setup** - Clock speed set to 480 MHz, caches turned on
3. **Peripherals configured** - SPI, I2C, UART, and timer registers programmed
4. **Operating system starts** - FreeRTOS takes control of the CPU
5. **Autopilot runs** - Sensors read, state estimated, controls computed, servos driven

The CPU never stops after step 5. It runs this loop indefinitely until power
is removed. There is no "shutdown" -- the system is designed to run forever.

---

## 3. The Three Software Layers

### Layer 1: BSP (Board Support Package) - "Talk to the chip"

**Files:** `bsp.c`, `bsp.h`, `stm32h7xx_hal_msp.c`

This is the lowest layer. It knows the exact chip (STM32H745), the exact
clock crystal frequency (8 MHz), and which physical pins connect to which
peripherals. If you change the PCB, only this layer changes.

Example: "SPI1 uses pins PA5 (clock), PA6 (data in), PA7 (data out),
running at 3.75 MHz."

### Layer 2: OSAL (OS Abstraction Layer) - "Threads and timing"

**Files:** `AP/Os/FreeRtos/Task.cpp`, `Mutex.cpp`, `RawTime.cpp`, `Console.cpp`

F Prime does not call FreeRTOS directly. Instead, it calls generic functions
like `Os::Task::start()` and `Os::Mutex::take()`. The OSAL translates these
into FreeRTOS calls (`xTaskCreate`, `xSemaphoreTakeRecursive`).

Why? Because the same F Prime code runs on Linux (SITL) using pthreads, or
on FreeRTOS on STM32, without changing a single line of application code.

### Layer 3: F Prime Components - "The actual autopilot"

**Files:** Everything in `AP/Hal/Stm32/`, `AP/Components/`

These are modular blocks that each do one job. A sensor driver reads one
sensor. A controller runs PID loops. A communication driver sends MAVLink
packets. They are connected by "ports" -- typed data connections defined
in `.fpp` files. The F Prime framework handles threading, message queues,
and scheduling automatically.

---

## 4. F Prime in Plain English

### What is a Component?

A component is a self-contained block of code that does one specific job.
Think of it like a LEGO brick -- it has defined connection points (ports)
and snaps together with other bricks.

Example: `Stm32SpiImu` is a component. Its job: "Read the IMU sensor over
SPI and output acceleration and rotation data."

It has:
- One **input port** (`schedIn`) -- a trigger that says "do your job now"
- One **output port** (`imuOut`) -- where the sensor data goes

The component does NOT know who triggers it or who receives the data. It
just responds to triggers and sends data to whoever is connected.

### What is a Port?

A port is a typed connection between two components. Think of it like an
electrical plug -- the shape of the plug determines what kind of data flows
through it.

The data types are defined in `AP/Types/ApTypes.fpp`:

```
ImuData     = { accel (x,y,z in m/s^2),  gyro (x,y,z in deg/s),  timestamp }
GpsData     = { latitude, longitude, altitude, velocity (N,E,D),   timestamp }
BaroData    = { pressure (Pa), altitude (m),                       timestamp }
MagData     = { magnetic field (x,y,z in Gauss),                   timestamp }

AircraftState = { position, velocity, attitude, euler angles, angular rates,
                  airspeed, timestamp }

SurfaceCmd  = { aileron (-1 to +1), elevator, rudder, throttle (0 to 1) }
RcChannels  = { roll, pitch, yaw, throttle }
GuidanceCmd = { desired altitude, airspeed, heading }
```

The port types are defined in `AP/Ports/ApPorts.fpp`:

```
ImuPort         carries  ImuData         (IMU -> StateEstimator)
GpsPort         carries  GpsData         (GPS -> StateEstimator)
BaroPort        carries  BaroData        (Baro -> StateEstimator)
MagPort         carries  MagData         (Mag -> StateEstimator)
StatePort       carries  AircraftState   (StateEstimator -> Controller, Autonomy, MAVLink)
SurfaceCmdPort  carries  SurfaceCmd      (Controller -> PWM output)
RcPort          carries  RcChannels      (MAVLink -> Autonomy)
GuidanceCmdPort carries  GuidanceCmd     (Autonomy -> Controller)
ModePort        carries  FlightMode      (Autonomy <-> MAVLink)
```

### What is a Topology?

A topology is a wiring diagram. It says "connect component A's output to
component B's input." This is defined in `topology.fpp`:

```
stm32Imu.imuOut          ->  stateEstimator.imuIn
stateEstimator.stateOut  ->  controller.stateIn
controller.surfaceCmdOut ->  stm32Pwm.surfaceCmdIn
```

This creates a direct chain: IMU -> StateEstimator -> Controller -> Servos.

### The Three Component Types

| Type | Has a message queue? | Has its own thread? | When does the handler run? |
|------|---------------------|--------------------|-----------------------------|
| **Passive** | No | No | Immediately, in whoever called it |
| **Queued** | Yes | No | Borrows the calling thread to drain the queue |
| **Active** | Yes | Yes | In its own dedicated FreeRTOS task |

**Passive example: `Stm32PwmOutput`**
When the Controller sends a SurfaceCmd, the PWM handler runs immediately
in the Controller's thread. Writing a PWM register takes nanoseconds, so
there is no need for a separate thread.

**Queued example: `Stm32SpiImu`**
The rate group sends a schedule message to the IMU's queue. The rate group
thread then drains the queue and runs the IMU handler. This allows the rate
group to control timing precisely.

**Active example: `Stm32Uart` (MAVLink)**
Has its own FreeRTOS task that sits in a loop waiting for messages. It
handles incoming MAVLink packets asynchronously from the sensor loops.

### What is FPP?

FPP (F Prime Prime) is a domain-specific language for defining components,
ports, and topologies. The FPP compiler reads `.fpp` files and generates
C++ code (the `*Ac.hpp` and `*Ac.cpp` files). You never write this generated
code by hand.

For example, `Stm32SpiImu.fpp` says:
```
queued component Stm32SpiImu {
    async input port schedIn: Svc.Sched
    output port imuOut: Ap.ImuPort
}
```

The FPP compiler generates `Stm32SpiImuComponentAc.hpp` containing:
- A base class with a pure virtual `schedIn_handler()` you must implement
- Port connection methods (`set_imuOut_OutputPort`, `get_schedIn_InputPort`)
- A message queue that serializes `schedIn` arguments, enqueues them, and
  later deserializes and calls your handler

You write `Stm32SpiImu.cpp` which extends the base class and implements
`schedIn_handler()` -- the actual SPI read logic.

---

## 5. FreeRTOS in Plain English

FreeRTOS is a real-time operating system that runs on the bare metal CPU.
Its job is to share one CPU among many tasks, giving each task the illusion
that it has the processor to itself.

### Tasks

A task is like a thread. It has its own stack (private memory for local
variables) and runs independently. FreeRTOS switches between tasks very
fast (microseconds), so they appear to run simultaneously.

This autopilot creates approximately 10 tasks:

| Task | Priority | What it does |
|------|----------|-------------|
| TimerDrv | 7 (highest) | Wakes up at 400 Hz from TIM6, kicks rate groups |
| rateGroup1 | 7 | Dispatches 400 Hz work (IMU) |
| rateGroup2 | 7 | Dispatches 100 Hz work (state estimator, controller) |
| rateGroup3 | 7 | Dispatches 50 Hz work (baro, mag) |
| rateGroup4 | 7 | Dispatches 10 Hz work (GPS, autonomy, MAVLink) |
| cmdDisp | 6 | Handles ground station commands |
| mavlinkUart | 6 | MAVLink TX/RX on USART2 |
| events | 4 | Logs events |
| tlmSend | 4 | Sends telemetry packets |
| IDLE | 0 (lowest) | Runs when nothing else needs the CPU |

Higher-priority tasks always preempt lower-priority ones. The TimerDrv task
runs first because timing accuracy is critical. The IDLE task only runs
when all other tasks are blocked waiting for something.

### Message Queues

Tasks communicate through queues. A queue is a FIFO buffer. One task puts
a message in; another task takes a message out. If the queue is empty, the
receiving task sleeps (blocks) until a message arrives. This is how F Prime
components exchange data without shared global variables.

### Semaphores and Mutexes

A semaphore is a signaling mechanism. The IMU driver uses one to wait for
a DMA transfer to complete:

```
IMU task: start DMA, then xSemaphoreTake(sem) -> blocks
DMA ISR:  transfer done, xSemaphoreGiveFromISR(sem) -> unblocks IMU task
```

A mutex is a lock. If two tasks need to read/write the same data, they
lock a mutex first so they don't corrupt each other.

### The Heap

FreeRTOS manages a block of memory (128 KB in this project) called the heap.
Task stacks and queue buffers are allocated from this heap. The heap manager
(`heap_4.c`) uses a best-fit algorithm with coalescing (merging adjacent
free blocks) to minimize fragmentation.

---

## 6. How F Prime and FreeRTOS Work Together

F Prime does not call FreeRTOS directly. There is a translation layer
called the OSAL (OS Abstraction Layer). Here is how each F Prime concept
maps to FreeRTOS:

```
F Prime concept         OSAL translation            FreeRTOS call
---------------------   -----------------------     ----------------------
Os::Task::start()       FreeRtosTask::start()       xTaskCreate()
Os::Task::join()        FreeRtosTask::join()        xSemaphoreTake(join_sem)
Os::Task::delay()       FreeRtosTask::_delay()      vTaskDelay()
Os::Mutex::take()       FreeRtosMutex::take()       xSemaphoreTakeRecursive()
Os::Mutex::release()    FreeRtosMutex::release()    xSemaphoreGiveRecursive()
Os::RawTime::now()      FreeRtosRawTime::now()      xTaskGetTickCount() + DWT
Os::Console::write()    FreeRtosConsole::write()    ITM_SendChar() (SWO debug)
MemAllocator::alloc()   FreeRtosMemAllocator        pvPortMalloc()
```

### The Delegate Pattern

F Prime uses a pattern called "delegates" to avoid virtual function
overhead. Each OS object (Task, Mutex, etc.) contains a small fixed-size
byte array. When you create an `Os::Task`, the system constructs a
`FreeRtosTask` object directly inside that byte array (using "placement
new"). This avoids heap allocation for OS objects and avoids the cost of
virtual function dispatch.

The `DefaultTask.cpp` file (one per platform) provides a factory function
`getDelegate()` that tells F Prime which concrete class to construct. On
STM32 it constructs `FreeRtosTask`; on Linux it constructs `PosixTask`.

### Priority Mapping

F Prime uses priorities from 0 to 49 (higher = higher priority).
FreeRTOS uses priorities from 0 to 7 (configMAX_PRIORITIES = 8).

The OSAL maps linearly:

```
F' priority 0-9   -> FreeRTOS 1
F' priority 10-19 -> FreeRTOS 2-3
F' priority 20-29 -> FreeRTOS 3-4
F' priority 30-39 -> FreeRTOS 5-6
F' priority 40-49 -> FreeRTOS 6-7
```

### Stack Size Conversion

F Prime specifies stack sizes in bytes (e.g., 4096 bytes = 4 KB).
FreeRTOS specifies stack depth in `StackType_t` words (4 bytes on ARM).
The OSAL divides by 4 automatically.

---

## 7. The Boot Sequence Step by Step

### File: `startup_stm32h745xx.s` (Assembly, runs first)

When power is applied, the Cortex-M7 CPU does this:
1. Loads the initial stack pointer from address 0x08000000
2. Loads the Reset_Handler address from 0x08000004
3. Jumps to Reset_Handler

Reset_Handler (in the assembly file) does:
1. Copies the `.data` section from Flash to RAM (initialized global variables)
2. Zeros the `.bss` section in RAM (uninitialized global variables)
3. Calls `main()` in C

### File: `Main.cpp` (C++, the real entry point)

```
main()
  |
  |  PHASE 1: Hardware initialization (no OS yet, bare metal)
  |
  +-> CPU_CACHE_Enable()          Turn on instruction + data caches
  |                                (makes code run ~5x faster)
  |
  +-> HAL_Init()                  Initialize STM32 HAL library
  |                                - Sets up SysTick for HAL_Delay()
  |                                - Configures NVIC priority grouping
  |                                - Calls HAL_MspInit() for SYSCFG clock
  |
  +-> SystemClock_Config()        Set CPU to 480 MHz via PLL
  |                                HSE 8 MHz -> PLL -> 480 MHz SYSCLK
  |                                AHB = 240 MHz, APB1/APB2 = 120 MHz
  |                                Timer clocks = 240 MHz (x2 multiplier)
  |
  +-> MX_GPIO_Init()              Configure IMU chip-select pin (output, HIGH)
  |
  +-> MX_DMA_Init()               Enable DMA1/DMA2 clocks, set IRQ priorities
  |
  +-> MX_SPI1_Init()              Configure SPI1 for IMU (3.75 MHz, Mode 3)
  |                                Triggers HAL_SPI_MspInit() which maps GPIO
  |                                pins to SPI1 and configures DMA
  |
  +-> MX_I2C1_Init()              Configure I2C1 for barometer (400 kHz)
  +-> MX_I2C2_Init()              Configure I2C2 for magnetometer (400 kHz)
  +-> MX_USART1_UART_Init()       Configure USART1 for GPS (115200 baud)
  +-> MX_USART2_UART_Init()       Configure USART2 for MAVLink (921600 baud)
  +-> MX_TIM1_Init()              Configure TIM1 for servo PWM (50 Hz, 4 ch)
  |
  |  (TIM6 is NOT initialized here -- Stm32Timer does it later)
  |
  +-> enableDwtCycleCounter()     Turn on CPU cycle counter (480 MHz resolution)
  |                                Used by RawTime for sub-microsecond timestamps
  |
  |  PHASE 2: Operating system initialization
  |
  +-> Os::init()                  Register FreeRTOS implementations with F Prime
  |                                Now Os::Task::start() -> xTaskCreate()
  |                                Now Os::Mutex::take() -> xSemaphoreTakeRecursive()
  |                                etc.
  |
  |  PHASE 3: Build the F Prime component graph
  |
  +-> Fill HardwareBindings       Collect all HAL handle pointers into one struct:
  |     hw.hspi1 = &hspi1           SPI handle for IMU
  |     hw.hi2c1 = &hi2c1           I2C handle for barometer
  |     hw.hi2c2 = &hi2c2           I2C handle for magnetometer
  |     hw.huart1 = &huart1         UART handle for GPS
  |     hw.huart2 = &huart2         UART handle for MAVLink
  |     hw.htim1 = &htim1           Timer handle for PWM
  |     hw.imu_cs_port = GPIOB      IMU chip-select GPIO port
  |     hw.imu_cs_pin = PIN_6       IMU chip-select GPIO pin
  |
  +-> setupTopology(hw)           This is where F Prime comes alive:
  |     |
  |     +-> initComponents()      Construct all component objects in memory
  |     +-> setBaseIds()          Assign telemetry/command ID numbers
  |     +-> connectComponents()   Wire all port connections (the topology graph)
  |     +-> regCommands()         Register command handlers with command dispatcher
  |     +-> configComponents()    Framework-level configuration
  |     |
  |     +-> configureTopology()   Hardware-specific configuration:
  |     |     |
  |     |     +-> Register FreeRTOS heap allocator with F Prime
  |     |     +-> Configure rate group divisors (1, 4, 8, 40)
  |     |     +-> stm32Imu.configure(hspi1, GPIOB, PIN_6)
  |     |     +-> stm32Imu.initSensor()    Write ICM-42688 registers
  |     |     +-> stm32Baro.configure(hi2c1)
  |     |     +-> stm32Baro.initSensor()   Read MS5611 calibration data
  |     |     +-> stm32Mag.configure(hi2c2)
  |     |     +-> stm32Mag.initSensor()    Verify IST8310 device ID
  |     |     +-> stm32Gps.configure(huart1) + startRx()
  |     |     +-> mavlinkUart.configure(huart2) + startRx()
  |     |     +-> stm32Pwm.configure(htim1) + startPwm()
  |     |
  |     +-> startTasks()          Create FreeRTOS tasks for all active/queued
  |                                components (rate groups, cmdDisp, events,
  |                                tlmSend, mavlinkUart). Tasks exist but are
  |                                NOT running yet -- scheduler hasn't started.
  |
  +-> startRateGroups()           Configure TIM6 for 400 Hz interrupts.
  |                                Create the TimerDrv FreeRTOS task at max
  |                                priority. TIM6 is now armed.
  |
  |  PHASE 4: FreeRTOS takes control
  |
  +-> vTaskStartScheduler()       *** POINT OF NO RETURN ***
  |                                FreeRTOS creates the idle task, starts
  |                                SysTick at 1 kHz, and performs the first
  |                                context switch. The highest-priority ready
  |                                task begins executing. This function NEVER
  |                                returns under normal operation.
  |
  +-> (unreachable)               If we get here, the heap was too small.
```

---

## 8. The 400 Hz Heartbeat - How the Main Loop Works

There is no `while(1)` loop in this system. Instead, everything is driven
by a hardware timer interrupt:

### The Trigger Chain

```
STEP 1: Hardware interrupt
    TIM6 counter reaches its period (every 2.5 ms = 400 Hz)
    -> CPU jumps to TIM6_DAC_IRQHandler() [Stm32Timer.cpp line 15]
    -> Clears the interrupt flag
    -> Sends a notification to the TimerDrv task
    -> Returns from interrupt (takes ~1 microsecond total)

STEP 2: TimerDrv task wakes up
    timerTask() was blocked on ulTaskNotifyTake() [Stm32Timer.cpp line 76]
    -> Captures a timestamp
    -> Calls CycleOut_out(0, rawTime)
    -> This is a DIRECT function call to RateGroupDriver (no queue)

STEP 3: RateGroupDriver distributes the tick
    RateGroupDriver.CycleIn_handler() runs (still in TimerDrv task context)
    -> Increments an internal counter
    -> If counter % 1 == 0:  fire RateGroup1 (400 Hz)
    -> If counter % 4 == 0:  fire RateGroup2 (100 Hz)
    -> If counter % 8 == 0:  fire RateGroup3 (50 Hz)
    -> If counter % 40 == 0: fire RateGroup4 (10 Hz)

STEP 4: Rate group dispatches work
    ActiveRateGroup.CycleIn_handler() runs
    -> Puts a message on its own queue
    -> Its own task wakes up
    -> Iterates over all connected member ports:
       rateGroup1Comp.RateGroupMemberOut[0] -> stm32Imu.schedIn
    -> This sends an async message to stm32Imu's queue
    -> The rate group task then drains the IMU's queue, running its handler

STEP 5: Component does its job (example: IMU)
    Stm32SpiImu.schedIn_handler() runs
    -> Kicks off a DMA SPI transfer (non-blocking)
    -> Waits for DMA completion (semaphore, ~100 microseconds)
    -> Parses raw bytes into acceleration and rotation
    -> Calls imuOut_out(0, imuData)
    -> This sends the data to StateEstimator.imuIn (async message)
```

### The Complete Data Flow

Every cycle, this is what happens from sensor to servo:

```
  400 Hz (every 2.5 ms):
    TIM6 ISR -> TimerDrv -> RateGroupDriver -> RateGroup1
      -> Stm32SpiImu reads accelerometer + gyroscope
         -> ImuData goes to StateEstimator

  100 Hz (every 10 ms):
    RateGroupDriver -> RateGroup2
      -> StateEstimator runs sensor fusion (Kalman filter)
         -> AircraftState goes to Controller, Autonomy, MAVLink
      -> Controller runs PID loops
         -> SurfaceCmd goes to Stm32PwmOutput
         -> PWM registers updated -> servos move

  50 Hz (every 20 ms):
    RateGroupDriver -> RateGroup3
      -> Stm32I2cBaro reads pressure + computes altitude
         -> BaroData goes to StateEstimator
      -> Stm32I2cMag reads magnetic field
         -> MagData goes to StateEstimator

  10 Hz (every 100 ms):
    RateGroupDriver -> RateGroup4
      -> Stm32GpsUart parses GPS position + velocity
         -> GpsData goes to StateEstimator
      -> Autonomy computes guidance commands (heading, altitude, speed)
         -> GuidanceCmd goes to Controller
      -> Stm32Uart sends MAVLink telemetry to ground station
         -> Receives RC stick commands and mission waypoints
```

### Why This Design?

Different sensors need different update rates:
- **IMU at 400 Hz**: Gyroscopes drift if you read them too slowly
- **State estimation at 100 Hz**: The Kalman filter needs frequent updates
  for accurate attitude, but not as fast as raw IMU
- **Barometer/Magnetometer at 50 Hz**: These sensors are slow (50+ ms
  measurement time), reading faster would just get stale data
- **GPS at 10 Hz**: UBLOX modules output at 5-10 Hz maximum
- **MAVLink at 10 Hz**: Radio bandwidth is limited, 10 Hz is enough for
  telemetry

---

## 9. Every Hardware Driver Explained

### Stm32Timer - The System Heartbeat

**What it does:** Generates the 400 Hz base tick that drives everything else.

**How it works:**
1. Configures TIM6 (a basic hardware timer) to generate an interrupt at 400 Hz
2. Creates a high-priority FreeRTOS task called "TimerDrv"
3. Every 2.5 ms, TIM6 fires an interrupt
4. The ISR sends a notification to TimerDrv (one line of code)
5. TimerDrv wakes up and calls CycleOut -> RateGroupDriver

**Why a separate task instead of doing everything in the ISR?**
ISRs (interrupt service routines) must be extremely short. They run with
other interrupts disabled, so any delay causes jitter. By sending a
notification and returning immediately, the ISR takes ~1 microsecond.
The actual work happens in a normal task where it's safe to take longer.

**Key calculation:**
```
APB1 timer clock = 240 MHz
Desired rate = 400 Hz
Prescaler and period are computed so that:
  240,000,000 / ((prescaler+1) * (period+1)) = 400
```

### Stm32SpiImu - IMU Sensor (ICM-42688-P)

**What it does:** Reads 3-axis accelerometer and 3-axis gyroscope at 400 Hz.

**The sensor:** ICM-42688-P is an inertial measurement unit. It has:
- Accelerometer: measures linear acceleration in m/s^2 (gravity, motion)
- Gyroscope: measures rotational velocity in degrees/s (roll, pitch, yaw rates)

**Communication protocol:** SPI (Serial Peripheral Interface)
- 4 wires: Clock (SCK), Data Out (MOSI), Data In (MISO), Chip Select (CS)
- Master (STM32) generates clock, slave (IMU) sends data
- Mode 3: clock idle high, data sampled on rising edge
- Speed: 3.75 MHz
- CS is a GPIO pin manually controlled (LOW = selected, HIGH = deselected)

**How one measurement cycle works:**
```
1. schedIn_handler called (400 Hz trigger)
2. Set TX buffer: [0x80 | 0x1F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                   ^read flag  ^register address (ACCEL_DATA_X1)
                   The 12 zero bytes clock out data from the sensor
3. Pull CS pin LOW (select the sensor)
4. Start DMA transfer (HAL_SPI_TransmitReceive_DMA)
   - DMA hardware handles byte-by-byte SPI transfer
   - CPU is free to do other things while DMA runs
5. Wait for DMA completion (xSemaphoreTake, blocks ~100 us)
6. DMA done -> ISR fires -> gives semaphore -> task unblocks
7. Pull CS pin HIGH (deselect the sensor)
8. Parse RX buffer: bytes [1..12] contain:
     [AX_H][AX_L][AY_H][AY_L][AZ_H][AZ_L][GX_H][GX_L][GY_H][GY_L][GZ_H][GZ_L]
   Each pair is a 16-bit signed integer (big-endian)
9. Convert to engineering units:
     accel = raw * (9.81 / 2048)    m/s^2  (at +/-16g full scale)
     gyro  = raw * (pi/180 / 16.4)  rad/s  (at +/-2000 deg/s full scale)
10. Build ImuData struct, call imuOut_out() -> data goes to StateEstimator
```

**Sensor initialization (done once at boot):**
- Soft reset (write 0x01 to register 0x11)
- Wait 10 ms for reset
- Power on accelerometer + gyroscope in low-noise mode
- Set full-scale ranges: accelerometer +/-16g, gyroscope +/-2000 deg/s
- Set output data rate: 1 kHz (hardware decimates to match read rate)

### Stm32I2cBaro - Barometer (MS5611-01BA03)

**What it does:** Measures atmospheric pressure and computes altitude at 50 Hz.

**The sensor:** MS5611 is a high-precision barometric pressure sensor.
By measuring air pressure, we can calculate altitude (pressure decreases
with altitude at a known rate).

**Communication protocol:** I2C (Inter-Integrated Circuit)
- 2 wires: Clock (SCL), Data (SDA) - bidirectional
- 7-bit address: 0x77
- Speed: 400 kHz (Fast Mode)

**Why is it slower than SPI?** I2C is simpler (fewer wires) but slower.
The barometer doesn't need high speed -- one measurement takes 8-10 ms
internally, so reading faster than 50 Hz would be pointless.

**Measurement cycle (50 Hz, alternating):**
```
Even cycles: Start a pressure measurement
  -> Send command 0x48 (D1 conversion, OSR=4096)
  -> Wait for next cycle (~20 ms for conversion to complete)

Odd cycles: Read the result, start temperature measurement
  -> Send command 0x00 (read ADC)
  -> Read 3 bytes = 24-bit raw pressure
  -> Send command 0x58 (D2 conversion, OSR=4096)
  -> Wait for next cycle

Even cycles: Read temperature, compute altitude
  -> Read 3 bytes = 24-bit raw temperature
  -> Apply second-order compensation polynomial using PROM calibration data
  -> Compute true pressure in Pascals
  -> Compute altitude using the hypsometric formula:
       altitude = 44330 * (1 - (pressure / 101325) ^ 0.1903)
  -> Build BaroData, call baroOut_out()
```

**Calibration:** At boot, the driver reads 8 calibration coefficients from
the sensor's PROM (factory-programmed memory). These coefficients correct
for manufacturing variations and are used in every measurement.

### Stm32I2cMag - Magnetometer (IST8310)

**What it does:** Measures the Earth's magnetic field at 50 Hz. Used by the
state estimator to determine heading (compass direction).

**Communication protocol:** I2C at 400 kHz, address 0x0E

**Measurement cycle (alternating trigger/read):**
```
Even cycles: Trigger a measurement
  -> Write 0x01 to CNTL1 register (single measurement mode)

Odd cycles: Read the result
  -> Check STAT1 register for DRDY (data ready) bit
  -> Burst-read 6 bytes: [XL][XH][YL][YH][ZL][ZH]
  -> Convert to Gauss: raw * 0.003 (3 mGauss per LSB)
  -> Build MagData, call magOut_out()
```

### Stm32GpsUart - GPS Receiver (UBLOX)

**What it does:** Parses GPS position and velocity data at 10 Hz.

**Communication protocol:** UART at 115200 baud with DMA circular receive.

**How DMA circular receive works:**
The UART peripheral continuously receives bytes into a circular buffer
(ring buffer) in RAM via DMA, without any CPU involvement. Every 100 ms,
the schedIn_handler checks how many new bytes have arrived and parses them.

**Parsing UBX-NAV-PVT messages:**
UBLOX GPS modules send binary UBX protocol frames (not NMEA text).
Each NAV-PVT message contains position, velocity, and time in 92 bytes:

```
[0xB5][0x62]          - UBX sync bytes
[0x01][0x07]          - Class=NAV, ID=PVT
[0x5C][0x00]          - Length = 92 bytes
[... 92 bytes ...]    - Payload
[CK_A][CK_B]          - Fletcher-8 checksum

Payload fields (byte offsets):
  Offset 4:  year, month, day, hour, min, sec (UTC time)
  Offset 20: fixType (0=no fix, 3=3D fix)
  Offset 21: flags (bit 0 = gnssFixOK)
  Offset 24: lon (degrees * 1e-7, signed 32-bit)
  Offset 28: lat (degrees * 1e-7, signed 32-bit)
  Offset 36: hMSL (altitude above mean sea level, mm)
  Offset 48: velN (north velocity, mm/s)
  Offset 52: velE (east velocity, mm/s)
  Offset 56: velD (down velocity, mm/s)
```

The parser scans the ring buffer byte by byte looking for the 0xB5 0x62
sync pattern, validates the checksum, and extracts the fields.

### Stm32Uart - MAVLink Telemetry

**What it does:** Sends aircraft state to the ground station and receives
RC stick commands, mission waypoints, and mode changes.

**Communication protocol:** UART at 921600 baud (fast!) with DMA TX and
circular DMA RX on USART2. Connected to a radio modem that bridges to
the ground station running Mission Planner.

**This is an Active component** -- it has its own FreeRTOS task because
MAVLink communication is asynchronous and must not block the flight
control loops.

**What it sends (downlink, 10 Hz):**
- HEARTBEAT: system status, flight mode, armed state
- ATTITUDE: roll, pitch, yaw angles and rates
- GLOBAL_POSITION_INT: GPS position (lat, lon, alt)
- VFR_HUD: airspeed, groundspeed, heading, altitude, climb rate, throttle

**What it receives (uplink):**
- MANUAL_CONTROL: joystick/RC stick positions (pitch, roll, yaw, throttle)
  -> Converted to RcChannels, sent to Autonomy
- MISSION_ITEM_INT: waypoint uploads
  -> Converted to MissionWaypoint, sent to Autonomy
- SET_MODE: flight mode changes (FBWB, AUTO)
  -> Sent to Autonomy

**RX handling:**
Uses UART idle-line detection. When the UART sees a gap in the byte
stream (transmitter paused), it triggers a callback. The driver then
checks the DMA head pointer to find new bytes and feeds them to the
MAVLink parser (`mavlink_parse_char`).

### Stm32PwmOutput - Servo/ESC Output

**What it does:** Converts control commands to PWM signals that physically
move the airplane's control surfaces and motor.

**This is a Passive component** -- the surfaceCmdIn_handler runs in the
Controller's thread context. Writing a timer register takes nanoseconds,
so there is no need for queuing or a separate task.

**PWM basics:**
RC servos and ESCs expect a 50 Hz pulse signal (20 ms period).
The pulse width encodes the commanded position:
```
1000 us = full left / motor off
1500 us = center / motor half
2000 us = full right / motor full
```

**Channel mapping:**
```
TIM1 CH1 -> Aileron   (roll control)
TIM1 CH2 -> Elevator  (pitch control)
TIM1 CH3 -> Throttle  (engine power)
TIM1 CH4 -> Rudder    (yaw control)
```

**Conversion:**
The SurfaceCmd has normalized values (-1 to +1 for surfaces, 0 to 1 for
throttle). The driver maps these to microseconds:
```
aileron/elevator/rudder: [-1..+1] -> [1000..2000] us
  pulse = 1500 + value * 500

throttle: [0..1] -> [1000..2000] us
  pulse = 1000 + value * 1000
```

The pulse value is written to the TIM1 compare register using
`__HAL_TIM_SET_COMPARE()`. The timer hardware generates the pulse
automatically in hardware -- no CPU involvement until the next update.

### FreeRtosTime - System Clock for F Prime

**What it does:** Provides timestamps to every F Prime component.

**This is a Passive component** -- any component that needs the time
calls `timeGetOut` which invokes `FreeRtosTime::timeGetPort_handler()`
synchronously.

**How it works:**
```
ticks = xTaskGetTickCount()          // FreeRTOS tick count (1 kHz)
seconds = ticks / 1000
microseconds = (ticks % 1000) * 1000
time.set(TB_WORKSTATION_TIME, 0, seconds, microseconds)
```

Resolution is 1 ms (limited by the FreeRTOS tick rate). For sub-ms
timing, use `Os::RawTime` which adds the DWT cycle counter.

---

## 10. The OS Abstraction Layer

### Task (`AP/Os/FreeRtos/Task.cpp`)

When F Prime creates an active component, it calls `Os::Task::start()`.
The OSAL:

1. Creates a binary semaphore for join() synchronization
2. Converts the stack size from bytes to FreeRTOS words (divide by 4)
3. Maps the F Prime priority to a FreeRTOS priority
4. Calls `xTaskCreate()` to create the FreeRTOS task

The FreeRTOS entry wrapper runs the F Prime task routine. When it finishes,
it gives the join semaphore (so `join()` can unblock) and self-deletes.

### Mutex (`AP/Os/FreeRtos/Mutex.cpp`)

Uses FreeRTOS recursive mutexes. "Recursive" means the same task can lock
the mutex multiple times without deadlocking (useful for nested F Prime
component calls). The mutex also provides **priority inheritance** -- if a
low-priority task holds a lock that a high-priority task needs, FreeRTOS
temporarily boosts the low-priority task to prevent priority inversion.

All mutex storage is static (no heap allocation). The `StaticSemaphore_t`
buffer is embedded directly in the mutex object.

### RawTime (`AP/Os/FreeRtos/RawTime.cpp`)

Captures two hardware values for high-precision timing:
- `xTaskGetTickCount()` -- 1 ms resolution (from SysTick)
- `DWT->CYCCNT` -- ~2 ns resolution (480 MHz CPU cycle counter)

The DWT (Data Watchpoint and Trace) unit is an ARM debug feature that
includes a free-running cycle counter. It wraps around every ~9 seconds
at 480 MHz, but combined with the tick count, it gives precise intervals.

### Console (`AP/Os/FreeRtos/Console.cpp`)

Debug output goes to the ARM ITM (Instrumentation Trace Macrocell) SWO
(Serial Wire Output) port. This is a hardware debug trace channel on
the SWD connector. When a debugger (ST-Link, J-Link) is connected, text
appears in the trace viewer. When no debugger is connected, the output is
silently discarded with near-zero overhead.

`AP_ConsolePutChar()` is a weak symbol. You can override it in your BSP
to redirect debug output to a UART if you prefer.

### MemAllocator (`AP/Os/FreeRtos/FreeRtosMemAllocator.cpp`)

F Prime components with message queues need dynamic memory. This allocator
wraps `pvPortMalloc()` and `vPortFree()` from FreeRTOS heap_4. It is
registered during topology setup so that F Prime's Generic PriorityQueue
uses the FreeRTOS heap instead of the system `malloc()` (which may not
exist on a bare-metal target).

---

## 11. Memory Layout on the Chip

The STM32H745 has several memory regions with different properties:

```
Address         Region     Size    Speed           Used For
-----------     ------     -----   -----------     ----------------------------
0x0800_0000     Flash      1 MB    Fast (cached)   Code + constants (read-only)
0x2000_0000     DTCM       128 KB  Zero-wait       Reserved for future use
0x2400_0000     AXI SRAM   512 KB  Fast (cached)   All runtime data
0x3000_0000     SRAM1      128 KB  Bus speed       Available for DMA buffers
0x3002_0000     SRAM2      128 KB  Bus speed       Available for DMA buffers
0x3004_0000     SRAM3      32 KB   Bus speed       Available
0x3800_0000     SRAM4      64 KB   Low power       Available
```

### What goes where (linker script: `stm32h745_flash.ld`)

**Flash (208 KB used of 1024 KB = 20%):**
- `.isr_vector`: Interrupt vector table (128 entries x 4 bytes)
- `.text`: All compiled code (F Prime, FreeRTOS, HAL, drivers)
- `.rodata`: Constants, string literals
- `.init_array`: C++ static constructor table

**AXI SRAM (275 KB used of 512 KB = 54%):**
- `.data`: Initialized global variables (copied from Flash at startup)
- `.bss`: Zero-initialized globals, including:
  - FreeRTOS heap (`ucHeap[128KB]`) - for task stacks and queue buffers
  - TlmChan double-buffer hash table (~59 KB)
  - All F Prime component instance objects
  - All BSP peripheral handles (hspi1, hi2c1, huart1, etc.)
- `._user_heap_stack`: Newlib C library heap (8 KB) + boot stack (4 KB)

### Why AXI SRAM and not DTCM?

DTCM (Data Tightly Coupled Memory) has zero wait states -- the absolute
fastest memory. But it is only 128 KB and can only be accessed by the M7
core (not DMA). Since DMA is critical for SPI, UART, and I2C transfers,
the main data goes in AXI SRAM which is accessible by both CPU and DMA.

DTCM could be used in the future for performance-critical data like the
Kalman filter state or PID controller variables.

---

## 12. The Build System

### How it compiles

```
cmake --preset stm32h7                    # Configure
cmake --build build-stm32h7              # Build
arm-none-eabi-size build-stm32h7/bin/... # Check sizes
```

The `stm32h7` preset uses `cmake/stm32h7_toolchain.cmake` which:
1. Sets the cross-compiler to `arm-none-eabi-gcc`
2. Adds CPU flags: `-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard`
3. Defines: `-DSTM32H7 -DSTM32H745xx -DCORE_CM7 -DUSE_HAL_DRIVER`
4. Sets the linker script: `stm32h745_flash.ld`
5. Uses `nano.specs` (small C library) and `nosys.specs` (no OS syscalls)
6. Creates header-only INTERFACE targets for `freertos_kernel` and `stm32_hal`

The root `CMakeLists.txt` creates STATIC libraries for FreeRTOS and STM32 HAL
sources (compiled once, linked to all consumers).

### FPP autocoding pipeline

When you build, CMake runs the FPP compiler in this sequence:
```
*.fpp files → fpp-to-cpp → *ComponentAc.hpp / *ComponentAc.cpp
*.fpp files → fpp-to-dict → dictionary files (for ground tools)
topology.fpp + instances.fpp → fpp-to-cpp → StmTopologyAc.cpp
                                              (all connect/init/start code)
```

---

## 13. Complete File Reference

```
AP/Top/Stm32/
  Main.cpp                  Entry point (boot sequence)
  Stm32Topology.cpp/hpp     Hardware binding + configure + setup/teardown
  instances.fpp             Component instances (IDs, queue sizes, stack sizes)
  topology.fpp              Port connections (the wiring diagram)
  bsp.c / bsp.h             Clock config + peripheral init (chip-specific)
  stm32h7xx_hal_msp.c       Pin muxing (which GPIO pin -> which peripheral)
  stm32h7xx_it.c            Fault handlers (HardFault, NMI, etc.)
  startup_stm32h745xx.s     Vector table + Reset_Handler (assembly)
  stm32h745_flash.ld        Linker script (memory regions)
  FreeRTOSConfig.h          FreeRTOS kernel settings

AP/Hal/Stm32/
  Stm32Timer/               TIM6 -> 400 Hz ISR -> FreeRTOS task -> CycleOut
  Stm32SpiImu/              SPI1 + DMA -> ICM-42688 -> ImuData
  Stm32I2cBaro/             I2C1 -> MS5611 -> BaroData
  Stm32I2cMag/              I2C2 -> IST8310 -> MagData
  Stm32GpsUart/             USART1 + DMA circular -> UBLOX UBX -> GpsData
  Stm32Uart/                USART2 + DMA -> MAVLink encode/decode
  Stm32PwmOutput/           TIM1 CH1-4 -> PWM 1000-2000 us -> servos
  FreeRtosTime/             xTaskGetTickCount -> Fw::Time for timestamps

AP/Os/FreeRtos/
  Task.cpp/hpp              Os::Task -> xTaskCreate / vTaskDelete
  Mutex.cpp/hpp             Os::Mutex -> recursive semaphore
  RawTime.cpp/hpp           Os::RawTime -> tick count + DWT cycle counter
  Console.cpp/hpp           Os::Console -> ITM SWO debug trace
  FreeRtosMemAllocator      Fw::MemAllocator -> pvPortMalloc / vPortFree

AP/Types/ApTypes.fpp        Data types (ImuData, GpsData, AircraftState, etc.)
AP/Ports/ApPorts.fpp        Port definitions (ImuPort, StatePort, etc.)

AP/Components/FlightControl/
  StateEstimator/           Sensor fusion (Kalman filter)
  Controller/               PID control loops
  Autonomy/                 FBWB + AUTO mode logic

cmake/
  stm32h7_toolchain.cmake  Cross-compiler config + include paths

lib/
  freertos/                 FreeRTOS V10.6.1 kernel source
  stm32h7_hal/              STM32CubeH7 HAL drivers
  eigen/                    Linear algebra library (header-only)
  mavlink/                  MAVLink v2 C headers (header-only)
```

---

## 14. Glossary

| Term | Meaning |
|------|---------|
| **AHB** | Advanced High-performance Bus. Internal bus connecting CPU to peripherals at high speed. |
| **APB** | Advanced Peripheral Bus. Slower bus for peripherals (SPI, I2C, UART, timers). |
| **BSP** | Board Support Package. Code specific to one particular circuit board (pin mapping, clock config). |
| **CDH** | Command and Data Handling. The F Prime subsystem for commands, events, and telemetry. |
| **CS** | Chip Select. A GPIO pin that selects which SPI device to talk to (active LOW). |
| **DMA** | Direct Memory Access. Hardware that moves data between peripherals and memory without CPU help. |
| **DWT** | Data Watchpoint and Trace. ARM debug unit with a CPU cycle counter. |
| **ESC** | Electronic Speed Controller. Drives a brushless motor; controlled by PWM like a servo. |
| **FBWB** | Fly-By-Wire-B. A flight mode where stick inputs command heading and altitude changes. |
| **FPP** | F Prime Prime. The interface definition language that generates C++ component base classes. |
| **GPIO** | General Purpose Input/Output. A digital pin that can be set HIGH or LOW by software. |
| **HAL** | Hardware Abstraction Layer. ST's library for accessing STM32 peripherals via function calls. |
| **I2C** | Inter-Integrated Circuit. A 2-wire serial bus (SCL + SDA) for low-speed peripherals. |
| **ISR** | Interrupt Service Routine. A function that runs immediately when hardware signals an event. |
| **ITM** | Instrumentation Trace Macrocell. ARM hardware for sending debug text over SWD. |
| **MAVLink** | Micro Air Vehicle Link. A standard protocol for drone communication. |
| **MSP** | MCU Support Package. STM32 HAL callbacks that configure GPIO pins for each peripheral. |
| **NED** | North-East-Down. A coordinate frame where X=North, Y=East, Z=Down. |
| **NVIC** | Nested Vectored Interrupt Controller. ARM hardware that manages interrupt priorities. |
| **OSAL** | OS Abstraction Layer. Translates F Prime OS calls to FreeRTOS (or POSIX on Linux). |
| **PID** | Proportional-Integral-Derivative. A feedback control algorithm. |
| **PLL** | Phase-Locked Loop. Hardware that multiplies a reference clock to produce a faster clock. |
| **PWM** | Pulse Width Modulation. A signal where the pulse width encodes a value (e.g., servo position). |
| **RTOS** | Real-Time Operating System. An OS that guarantees tasks run within strict time deadlines. |
| **SITL** | Software In The Loop. Running the autopilot on a PC with simulated sensors (no hardware). |
| **SPI** | Serial Peripheral Interface. A 4-wire high-speed serial bus (SCK, MOSI, MISO, CS). |
| **SWD** | Serial Wire Debug. A 2-wire debug interface for ARM Cortex-M (used by ST-Link / J-Link). |
| **SWO** | Serial Wire Output. A single-wire trace output for debug text (part of SWD). |
| **UART** | Universal Asynchronous Receiver-Transmitter. Serial port (TX + RX wires). |
| **UBX** | UBLOX Binary Protocol. A compact binary format for GPS data (alternative to NMEA text). |
| **VOS** | Voltage Output Scaling. STM32 power mode that determines maximum clock speed. |
