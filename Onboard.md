# AutoPilot SITL — Developer Onboarding Guide

This guide walks you through setting up the AutoPilot SITL project from scratch on a fresh Windows PC. Follow every step in order.

---

## Table of Contents

1. [Prerequisites Overview](#1-prerequisites-overview)
2. [Install WSL2 (Windows Subsystem for Linux)](#2-install-wsl2-windows-subsystem-for-linux)
3. [Install System Packages](#3-install-system-packages)
4. [Install Python 3.11](#4-install-python-311)
5. [Install Java (JDK 17+)](#5-install-java-jdk-17)
6. [Clone the Repository](#6-clone-the-repository)
7. [Set Up the Python Virtual Environment](#7-set-up-the-python-virtual-environment)
8. [Build the Project](#8-build-the-project)
9. [Run the SITL Binary](#9-run-the-sitl-binary)
10. [Connect the F Prime GDS (Ground Data System)](#10-connect-the-f-prime-gds-ground-data-system)
11. [Connect QGroundControl / Mission Planner](#11-connect-qgroundcontrol--mission-planner)
12. [Project Structure Overview](#12-project-structure-overview)
13. [Development Workflow](#13-development-workflow)
14. [Common Build Errors & Fixes](#14-common-build-errors--fixes)

---

## 1. Prerequisites Overview

| Tool | Version | Why |
|------|---------|-----|
| WSL2 + Ubuntu 22.04+ | Latest | F Prime builds and runs on Linux |
| Python | 3.11.x | Required by F Prime v4.1.1 tooling |
| Java JDK | 17+ | FPP compiler is a JVM-based tool |
| Git | 2.x | Cloning the repo and submodules |
| CMake | 3.24.2+ | Build system (installed via pip into venv) |
| Ninja | Latest | Build backend (installed via pip into venv) |
| GCC / G++ | 11+ | C/C++ compiler |

---

## 2. Install WSL2 (Windows Subsystem for Linux)

F Prime is a Linux-native framework. The entire build and run cycle happens inside WSL.

### 2.1 Enable WSL

Open **PowerShell as Administrator** and run:

```powershell
wsl --install
```

This installs WSL2 with Ubuntu by default. If you already have WSL1, upgrade:

```powershell
wsl --set-default-version 2
```

### 2.2 Set Up Ubuntu

After restart, the Ubuntu terminal will launch automatically. Create your UNIX username and password when prompted.

### 2.3 Update the System

Inside the Ubuntu terminal:

```bash
sudo apt update && sudo apt upgrade -y
```

### 2.4 Verify

```bash
wsl --version        # (from PowerShell) — should show WSL version 2.x
lsb_release -a       # (from Ubuntu) — should show Ubuntu 22.04 or 24.04
```

> **All remaining steps are done inside the WSL Ubuntu terminal.**

---

## 3. Install System Packages

Install the essential build tools and libraries:

```bash
sudo apt install -y \
  build-essential \
  gcc \
  g++ \
  git \
  curl \
  wget \
  software-properties-common \
  pkg-config \
  libssl-dev \
  libffi-dev \
  zlib1g-dev \
  libbz2-dev \
  libreadline-dev \
  libsqlite3-dev \
  libncurses5-dev \
  libncursesw5-dev \
  xz-utils \
  tk-dev \
  liblzma-dev
```

Verify GCC version:

```bash
gcc --version    # Should be 11.x or higher
```

---

## 4. Install Python 3.11

F Prime v4.1.1 requires Python 3.11. Ubuntu 22.04 ships with 3.10, so install 3.11 from the deadsnakes PPA:

```bash
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update
sudo apt install -y python3.11 python3.11-venv python3.11-dev
```

Verify:

```bash
python3.11 --version    # Should print Python 3.11.x
```

> **Do NOT overwrite the system `python3` symlink.** We will use `python3.11` explicitly to create the virtual environment.

---

## 5. Install Java (JDK 17+)

The FPP (F Prime Prime) compiler runs on the JVM:

```bash
sudo apt install -y openjdk-17-jdk
```

Verify:

```bash
java -version    # Should show openjdk 17.x.x
```

---

## 6. Clone the Repository

### 6.1 Choose a Working Directory

```bash
mkdir -p ~/projects
cd ~/projects
```

### 6.2 Clone with Submodules

The project depends on three git submodules (`lib/fprime`, `lib/eigen`, `lib/mavlink`). Clone everything in one go:

```bash
git clone --recurse-submodules <REPO_URL> AutoPilot_draft1
cd AutoPilot_draft1
```

If you already cloned without `--recurse-submodules`, initialize them manually:

```bash
git submodule update --init --recursive
```

### 6.3 Verify Submodules

```bash
ls lib/fprime/    # Should contain cmake/, Fw/, Svc/, Drv/, etc.
ls lib/eigen/     # Should contain Eigen/ directory
ls lib/mavlink/   # Should contain common/, ardupilotmega/, etc.
```

All three must be populated. If any is empty, re-run `git submodule update --init --recursive`.

---

## 7. Set Up the Python Virtual Environment

### 7.1 Create the Virtual Environment

From the project root (`AutoPilot_draft1/`):

```bash
python3.11 -m venv fprime-venv
```

### 7.2 Activate the Virtual Environment

```bash
source fprime-venv/bin/activate
```

Your shell prompt should now show `(fprime-venv)`.

> **You must activate this venv every time you open a new terminal to work on the project.**

### 7.3 Upgrade pip

```bash
pip install --upgrade pip setuptools wheel
```

### 7.4 Install F Prime Tools and Dependencies

```bash
pip install -r requirements.txt
```

This installs all F Prime tools including:
- `fprime-util` — the F Prime build orchestrator
- `fprime-gds` — the Ground Data System web UI
- `fpp` — the FPP compiler (v3.1.0)
- `cmake` and `ninja` — build system tools
- All other Python dependencies

### 7.5 Verify the Installation

```bash
fprime-util --help      # Should print usage info
fpp --version           # Should print FPP 3.1.0
cmake --version         # Should print 3.24.2 or higher
ninja --version         # Should print a version number
```

---

## 8. Build the Project

### 8.1 Generate Build Files

From the project root, with the venv activated:

```bash
fprime-util generate
```

This runs CMake configuration using the default preset (`fprime`) and creates the `build-fprime-automatic-native/` directory.

### 8.2 Build

```bash
fprime-util build
```

This compiles all FPP models, autocoders, components, and links the final binary. The first build takes several minutes. Subsequent builds are incremental and much faster.

### 8.3 Verify the Binary

```bash
ls build-fprime-automatic-native/bin/Linux/AP_Top
```

The file should exist. If it does, the build succeeded.

---

## 9. Run the SITL Binary

### 9.1 Launch the Autopilot

```bash
./build-fprime-automatic-native/bin/Linux/AP_Top -a 0.0.0.0 -p 5000
```

| Flag | Meaning |
|------|---------|
| `-a 0.0.0.0` | Listen on all network interfaces (for GDS connection) |
| `-p 5000` | TCP port for GDS communication |

You should see startup logs and the simulation running at 400 Hz. The terminal will stay busy — the sim is now live.

### 9.2 Stop

Press `Ctrl+C` to gracefully shut down.

---

## 10. Connect the F Prime GDS (Ground Data System)

The GDS is a web-based UI for telemetry, commands, and events. Open a **second terminal**, activate the venv, and run:

```bash
cd ~/projects/AutoPilot_draft1
source fprime-venv/bin/activate

fprime-gds --dictionary build-artifacts/Linux/AP_Top/dict/SitlTopologyDictionary.json -n
```

The `-n` flag tells GDS not to launch its own instance of the binary (since we already started it manually).

### 10.1 Open the GDS in a Browser

On your **Windows host**, open a browser and navigate to:

```
http://localhost:5000
```

You should see the F Prime GDS dashboard with:
- **Channels** tab — live telemetry values (position, attitude, airspeed, etc.)
- **Events** tab — system events and logs
- **Commands** tab — send commands to the running system

---

## 11. Connect QGroundControl / Mission Planner

The MavlinkGateway component sends MAVLink telemetry on UDP port **14550** and listens for commands on UDP port **14540**.

### 11.1 Install QGroundControl (Windows)

1. Download QGroundControl from https://qgroundcontrol.com
2. Install and launch it on your **Windows host**

### 11.2 Configure the UDP Link

QGroundControl should auto-discover the SITL vehicle via UDP broadcast on port 14550. If it doesn't:

1. Go to **Application Settings** > **Comm Links**
2. Add a new **UDP** link
3. Set the **Listening Port** to `14550`
4. Click **Connect**

### 11.3 Verify Connection

Once connected, you should see:
- The aircraft on the map (starting at Albuquerque, NM reference point)
- Live attitude indicator
- HUD with airspeed, altitude, heading

> **Note:** QGroundControl/Mission Planner joystick RC input to the SITL is a known work-in-progress (see STATUS.md for details).

---

## 12. Project Structure Overview

```
AutoPilot_draft1/
├── AP/                                 # All project source code
│   ├── Types/ApTypes.fpp               # Data structures (Vec3, ImuData, AircraftState, etc.)
│   ├── Ports/ApPorts.fpp               # Port definitions (ImuPort, GpsPort, StatePort, etc.)
│   ├── Math/                           # Header-only math library (Eigen-based)
│   │   ├── ApMath.hpp                  # Vec3d, Quatd, Mat3d typedefs + constants
│   │   └── CoordTransforms.hpp         # DCM, quaternion, NED↔LLA transforms
│   ├── Components/
│   │   ├── Sim/                        # Simulation components
│   │   │   ├── SimDynamics/            # 6-DOF rigid body physics (400 Hz)
│   │   │   ├── SimImu/                 # Simulated IMU with noise (400 Hz)
│   │   │   ├── SimGps/                 # Simulated GPS with latency (10 Hz)
│   │   │   ├── SimBaro/               # Simulated barometer (50 Hz)
│   │   │   ├── SimMag/                # Simulated magnetometer (100 Hz)
│   │   │   └── SimServoDriver/        # Actuator passthrough
│   │   ├── FlightControl/
│   │   │   ├── StateEstimator/        # Complementary filter (100 Hz)
│   │   │   ├── Controller/            # Cascaded P-controller (100 Hz)
│   │   │   └── Autonomy/             # FBWB/AUTO mode manager (10 Hz)
│   │   └── Mavlink/
│   │       └── MavlinkGateway/        # MAVLink UDP bridge (10 Hz)
│   └── Top/
│       └── Sitl/                       # Deployment topology
│           ├── instances.fpp           # Component instance declarations
│           ├── topology.fpp            # Wiring (port connections)
│           ├── SitlTopology.cpp        # Topology setup/teardown code
│           └── Main.cpp                # Entry point
├── lib/                                # External dependencies (git submodules)
│   ├── fprime/                         # NASA F Prime framework (v4.1.1)
│   ├── eigen/                          # Eigen 3.4.0 (linear algebra)
│   └── mavlink/                        # MAVLink v2 C headers
├── config/
│   └── AcConstants.fpp                 # Framework constant overrides
├── fprime-venv/                        # Python virtual environment (not committed)
├── build-fprime-automatic-native/      # Build output (not committed)
├── CMakeLists.txt                      # Root CMake configuration
├── CMakePresets.json                   # Build presets (release, debug, ut, sitl)
├── settings.ini                        # F Prime project settings
├── requirements.txt                    # Python dependencies
├── README.md                           # Architecture documentation
└── STATUS.md                           # Development phase tracking
```

### Data Flow (Closed Loop)

```
            Controller → SimServoDriver → SimDynamics (6-DOF physics)
               ↑                              |
               |                    truth state fans out to:
               |                    SimImu / SimGps / SimBaro / SimMag
               |                              |
               |                    sensor data flows to:
               |                       StateEstimator
               |                        /    |    \
               |                Controller Autonomy MavlinkGateway
               |                    ↑        |
               +--------------------+  GuidanceCmd
```

### Rate Groups

| Group | Frequency | Components |
|-------|-----------|------------|
| RG1 | 400 Hz | SimDynamics, SimImu |
| RG2 | 100 Hz | SimMag, StateEstimator, Controller |
| RG3 | 50 Hz | SimBaro, SystemResources |
| RG4 | 10 Hz | SimGps, Autonomy, MavlinkGateway |

---

## 13. Development Workflow

### 13.1 Editing Components

Each component lives in its own directory under `AP/Components/` with:
- `<Name>.fpp` — FPP model (ports, commands, telemetry, parameters)
- `<Name>.cpp` / `<Name>.hpp` — Implementation (handler logic)
- `CMakeLists.txt` — Build registration

**Typical workflow:**
1. Edit the `.fpp` file to add/change ports, telemetry, commands
2. Edit the `.cpp` file to implement handler logic
3. Rebuild: `fprime-util build`
4. Re-run the binary and connect GDS

### 13.2 Rebuild After Changes

```bash
source fprime-venv/bin/activate
fprime-util build
```

Incremental builds only recompile changed files. If you modify `.fpp` files, the autocoder regenerates the corresponding C++ base classes automatically.

### 13.3 Full Clean Rebuild (if needed)

```bash
fprime-util purge          # Deletes the build directory
fprime-util generate       # Re-runs CMake configuration
fprime-util build          # Full rebuild
```

### 13.4 Using Build Presets

The project defines several CMake presets in `CMakePresets.json`:

| Preset | Purpose | Build Dir |
|--------|---------|-----------|
| `fprime` (default) | Release build | `build-fprime-automatic-native/` |
| `fprime-debug` | Debug build with symbols | `build-fprime-automatic-native/` |
| `fprime-ut` | Unit test build | `build-fprime-automatic-native-ut/` |
| `sitl` | SITL deployment only | `build-sitl/` |

---

## 14. Common Build Errors & Fixes

### "fpp: command not found"
**Cause:** Virtual environment not activated.
**Fix:** Run `source fprime-venv/bin/activate` before building.

### "java: command not found" or FPP compiler errors
**Cause:** Java JDK not installed.
**Fix:** `sudo apt install -y openjdk-17-jdk`

### Submodule directories are empty
**Cause:** Submodules weren't initialized during clone.
**Fix:** `git submodule update --init --recursive`

### CMake version too old
**Cause:** Using system CMake instead of the one in the venv.
**Fix:** Make sure the venv is activated. Run `which cmake` — it should point to `fprime-venv/bin/cmake`.

### "Assert code 8" crash at runtime
**Cause:** A message queue overflowed. A component is receiving data faster than it can process.
**Fix:** Increase the `queueSize` in `instances.fpp` for the affected component. Rule of thumb: queue depth >= (sender_rate / receiver_rate). For 400 Hz sender to 10 Hz receiver, use at least 50.

### Build succeeds but binary not found
**Expected location:** `build-fprime-automatic-native/bin/Linux/AP_Top`
**Fix:** Make sure you ran `fprime-util build` from the project root directory.

---

## Quick Start Cheat Sheet

```bash
# --- One-time setup ---
sudo apt install build-essential git openjdk-17-jdk python3.11 python3.11-venv python3.11-dev
git clone --recurse-submodules <REPO_URL> AutoPilot_draft1
cd AutoPilot_draft1
python3.11 -m venv fprime-venv
source fprime-venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
fprime-util generate
fprime-util build

# --- Every session ---
cd ~/projects/AutoPilot_draft1
source fprime-venv/bin/activate

# Terminal 1: Run the SITL
./build-fprime-automatic-native/bin/Linux/AP_Top -a 0.0.0.0 -p 5000

# Terminal 2: Run the GDS
fprime-gds --dictionary build-artifacts/Linux/AP_Top/dict/SitlTopologyDictionary.json -n

# Browser: http://localhost:5000
```
