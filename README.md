# Train Collision Avoidance System (TCAS)

A modern, production-grade C++23 real-time train collision-avoidance system (TCAS / Kavach architecture), designed and implemented as a cohesive 12-module safety-critical pipeline.

---

## Architectural Overview

The system features a multi-threaded, real-time periodic execution pipeline:
- **Physics Loop (20 ms / 50 Hz)**: Continuous numerical kinematics integration, gradient compensation, boundary progression, and atomic snapshot publishing under strict mutex synchronization.
- **Safety Pipeline (100 ms / 10 Hz)**: Multi-horizon trajectory prediction, spatial-temporal conflict detection, dynamic risk scoring, priority arbitration, resource reservation, and automated command dispatch.
- **Communication Loop (100 ms / 10 Hz)**: V2V and V2I wireless telemetry exchange, propagation latency simulation, probabilistic and deterministic packet loss injection, and mail delivery.
- **HMI & Telemetry Loop (200 ms / 5 Hz)**: Non-blocking ANSI live dashboard rendering, real-time telemetry CSV logging, conflict event logging, and safety metrics computation.
- **Input Dispatch Thread**: Dedicated user command loop queuing operations to a thread-safe `UserCommandQueue` without blocking the real-time simulation or causing terminal interleaving.

```
                  ┌──────────────────────┐
                  │    User Terminal     │
                  └──────────┬───────────┘
                             │ Keyboard input
                             ▼
                  ┌──────────────────────┐
                  │     Input Thread     │
                  └──────────┬───────────┘
                             │ Thread-safe post
                             ▼
                  ┌──────────────────────┐
                  │  UserCommandQueue    │
                  └──────────┬───────────┘
                             │ Drained synchronously
                             ▼
┌──────────────────────────────────────────────────────────────┐
│                     ThreadOrchestrator                       │
│                                                              │
│  Physics Thread (20 ms) ───► WorldState ───► Safety (100 ms) │
│           ▲                                         │        │
│           │            CommandQueue                 │        │
│           └─────────────────────────────────────────┘        │
│                                                              │
│  Comm Thread (100 ms) ──────► HMI Display Thread (200 ms)    │
└──────────────────────────────────────────────────────────────┘
```

---

## Implemented Modules (1–12)

1. **Railway Infrastructure**: Graph topology with directed tracks, stations, junctions, platforms, speed limits, and gradients.
2. **Train Management**: Fleet registry, polymorphic train dynamics (`ExpressTrain`, `PassengerTrain`, `FreightTrain`), and state lifecycle.
3. **Simulation & Clock**: High-resolution discrete simulation timer, deterministic time advancing, and synchronized ticks.
4. **Physics & Kinematics**: Continuous numerical kinematics, slope/gradient acceleration adjustment, braking distance calculation, and emergency stop curves.
5. **Route Navigation**: Dijkstra shortest-path route planner across graph networks with node-to-node connectivity and validation.
6. **Sensor & State Estimation**: Odometer noise modeling, GPS localization simulation, and 1D Kalman Filter state estimation.
7. **Wireless Communication**: V2V / V2I communication channel simulator modeling transmission latency, configurable packet loss rate, and entity mailboxes.
8. **Predictive Position Engine**: Multi-horizon trajectory extrapolation incorporating route transitions, speed restrictions, and uncertainty propagation.
9. **Conflict Detection & Spatial Reservation**: Same-track (Head-On and Rear-End) and convergence (Junction and Platform) conflict detection with space-time reservation grids.
10. **Safety & Resolution Engine**: Quantitative Risk Engine, Priority Arbitration Engine, Conflict Priority Queue, and automated command generator (`ReduceSpeed`, `HoldAtSignal`, `EmergencyBrake`).
11. **Safety Pipeline & Multi-Threaded Orchestration**: Complete safety lifecycle pipeline integrating Modules 8–10, lock-free/mutex-protected queues, and thread orchestrator.
12. **HMI, Telemetry & Evaluation**: Real-time ANSI Control Center display, automated telemetry/conflict loggers, and safety performance metric tracking.

---

## Pre-Configured Test Scenarios

The system includes pre-configured scenarios selectable via interactive menu or command line:
- **Junction Conflict**: Express and Freight converging on Alpha Junction. Freight yields or slows down based on priority arbitration.
- **Rear-End Conflict**: Express train closing rapidly on a slower Passenger train along the same track corridor.
- **Head-On Conflict**: Express train (+25 m/s) and Freight train (-15 m/s) converging head-on on Track 101, triggering immediate critical emergency braking.
- **Platform Conflict**: Two passenger trains converging on Platform A simultaneously via separate approach tracks.
- **Multiple Simultaneous Conflicts**: 3-train multi-hazard scenario stressing the Conflict Priority Queue.
- **Sensor Failure Injection**: Degradation of sensor readings, triggering uncertainty growth and conservative safety buffers.
- **Communication Failure Injection**: Wireless degradation / blackout test verifying autonomous fail-safe behavior.

---

## Building and Running

### Linux (RHEL / Rocky / CentOS / Ubuntu / Debian)
Prerequisites: GCC 13+ / GCC Toolset 14 or Clang 16+, CMake 3.20+, Make.

```bash
# 1. Configure and Build
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j$(nproc)

# 2. Run Test Suite (GoogleTest)
ctest --test-dir build --output-on-failure

# 3. Launch Interactive TCAS Control Center
./build/tcas
```

### Windows (Visual Studio / MinGW)
Prerequisites: MSVC 2022 / MinGW-w64 with C++23 support, CMake 3.20+.

```powershell
# 1. Configure and Build
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --config RelWithDebInfo --parallel

# 2. Run Test Suite
ctest -C RelWithDebInfo --test-dir build --output-on-failure

# 3. Launch TCAS Control Center
.\build\RelWithDebInfo\tcas.exe
```

---

## Static Analysis (Cppcheck)

Run Cppcheck with C++23 standards compliance:

```bash
# Linux
make check

# Or directly with cppcheck
cppcheck --enable=warning,style,performance,portability \
         --suppress=missingIncludeSystem \
         --suppress=unusedFunction \
         --std=c++23 \
         -I include src tests
```
