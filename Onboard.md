# AutoPilot SITL — Onboarding

## Steps

1. **Linux environment** — WSL2 + Ubuntu 22.04 (on Windows)
2. **System packages** — gcc, g++, git, cmake, ninja, pkg-config
3. **Python 3.11** — required by F Prime tooling (3.12 not supported)
4. **Java JDK 17+** — FPP compiler runs on JVM
5. **VS Code + extensions** — Remote-WSL, C/C++, FPP syntax
6. **Clone repo** — with `--recurse-submodules` (fprime, eigen, mavlink)
7. **Python venv** — install F Prime tools (`fprime-util`, `fprime-gds`, `fpp`)
8. **Build** — `fprime-util generate && fprime-util build`
9. **Run + verify** — binary, GDS, QGroundControl connection

---

## 1. Linux environment

WSL2 with Ubuntu. All development happens inside WSL.

```bash
# PowerShell (Admin)
wsl --install                    # installs WSL2 + Ubuntu by default
# Restart PC, Ubuntu terminal opens — create username/password

# Inside Ubuntu
sudo apt update && sudo apt upgrade -y
```

**Validate:**
```bash
wsl --version          # (PowerShell) — WSL version 2.x
lsb_release -a         # (Ubuntu) — Ubuntu 22.04 or 24.04
```

---

## 2. System packages

```bash
sudo apt install -y \
  build-essential gcc g++ git curl wget \
  pkg-config libssl-dev libffi-dev zlib1g-dev \
  libbz2-dev libreadline-dev libsqlite3-dev \
  libncurses5-dev libncursesw5-dev xz-utils \
  tk-dev liblzma-dev
```

**Validate:**
```bash
gcc --version          # 11.x+
g++ --version          # 11.x+
git --version          # 2.x+
```

---

## 3. Python 3.11

F Prime v4.1.1 requires 3.11. Ubuntu 22.04 ships 3.10, so use deadsnakes PPA.

```bash
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update
sudo apt install -y python3.11 python3.11-venv python3.11-dev
```

Do NOT overwrite the system `python3` symlink.

**Validate:**
```bash
python3.11 --version   # Python 3.11.x
```

---

## 4. Java JDK 17+

FPP compiler is JVM-based.

```bash
sudo apt install -y openjdk-17-jdk
```

**Validate:**
```bash
java -version          # openjdk 17.x.x
```

---

## 5. VS Code + extensions

Install VS Code on Windows, then install these extensions:

- **WSL** (ms-vscode-remote.remote-wsl) — open WSL folders in VS Code
- **C/C++** (ms-vscode.cpptools) — IntelliSense, debugging
- **FPP** (search "FPP" in marketplace) — syntax highlighting for .fpp files

```bash
# From Ubuntu terminal, open project in VS Code:
code .
```

**Validate:** VS Code opens with WSL indicator in bottom-left corner (`WSL: Ubuntu`).

---

## 6. Clone repo

Three git submodules: `lib/fprime` (F Prime v4.1.1), `lib/eigen` (Eigen 3.4.0), `lib/mavlink` (c_library_v2).

```bash
mkdir -p ~/projects && cd ~/projects
git clone --recurse-submodules https://github.com/lat-aerospace/AutoPilot_draft1.git AutoPilot_draft1
cd AutoPilot_draft1
```

If already cloned without submodules:
```bash
git submodule update --init --recursive
```

**Validate:**
```bash
ls lib/fprime/Fw/       # F Prime framework files exist
ls lib/eigen/Eigen/     # Eigen headers exist
ls lib/mavlink/common/  # MAVLink headers exist
```

---

## 7. Python venv + F Prime bootstrap

```bash
pip install fprime-bootstrap          # installs fprime-util CLI
cd ~/projects/AutoPilot_draft1
python3.11 -m venv fprime-venv
source fprime-venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt       # installs pinned F Prime tools + deps
```

`requirements.txt` pulls from `lib/fprime/requirements.txt` which includes:
- `fprime-tools==4.1.0` — provides `fprime-util` (build orchestrator)
- `fprime-gds==4.1.0` — Ground Data System web UI
- `fprime-fpp==3.1.0` — FPP compiler
- `cmake==3.26.0`, `ninja==1.11.1.4` — build system

No separate `pip install fprime-tools` bootstrap needed — it's all in the requirements file.

**Validate:**
```bash
fprime-util --help     # prints usage
fpp-check --version    # FPP 3.1.0
cmake --version        # 3.26.0
which cmake            # should point to fprime-venv/bin/cmake
```

---

## 8. Build

```bash
source fprime-venv/bin/activate
fprime-util generate       # CMake configuration → creates build-fprime-automatic-native/
fprime-util build          # compile everything (first build takes several minutes)
```

**Validate:**
```bash
ls build-fprime-automatic-native/bin/Linux/AP_Top    # binary exists
```

---

## 9. Run + verify

### 9a. Run the SITL binary

```bash
# Terminal 1
./build-fprime-automatic-native/bin/Linux/AP_Top -a 0.0.0.0 -p 5000
```

**Validate:** startup logs appear, no crash. Ctrl+C to stop.

### 9b. Connect F Prime GDS

```bash
# Terminal 2 (venv activated)
fprime-gds --dictionary build-artifacts/Linux/AP_Top/dict/SitlTopologyDictionary.json -n
```

Open browser at `http://localhost:5000`. Channels tab shows live telemetry ticking.

**Validate:** telemetry values (estRoll, estPitch, estAlt, etc.) are updating.

### 9c. Connect QGroundControl

Install QGC on Windows from https://qgroundcontrol.com. Launch it — auto-connects via UDP 14550.

**Validate:** aircraft appears on map at Albuquerque, NM. Attitude indicator and HUD show live data.
