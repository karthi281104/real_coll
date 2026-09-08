# Train Collision Avoidance System (TCAS)

[![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)](https://en.cppreference.com/w/cpp/23)
[![Build & Test](https://img.shields.io/badge/GoogleTest-299%20Passed-brightgreen.svg)](https://github.com/google/googletest)
[![Coding Standard](https://img.shields.io/badge/Code%20Style-Google%20C%2B%2B-orange.svg)](https://google.github.io/styleguide/cppguide.html)
[![License](https://img.shields.io/badge/License-Proprietary-lightgrey.svg)]()

A high-performance, real-time Train Collision Avoidance System (TCAS) with multi-horizon predictive trajectory projection, dynamic resource reservation, multi-factor risk assessment, priority-based conflict resolution, and automated traffic resumption.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Key Capabilities](#key-capabilities)
3. [Architecture & Documentation Index](#architecture--documentation-index)
4. [Build & Installation](#build--installation)
   - [Windows (MSVC / MinGW)](#windows-build--run)
   - [Linux (GCC / Clang)](#linux-build--run)
5. [Running Automated Tests (GoogleTest)](#running-automated-tests-googletest)
   - [Running with CTest](#running-with-ctest)
   - [Running the Test Binary Directly](#running-the-test-binary-directly)
   - [Test Filtering & Advanced Flags](#test-filtering--advanced-flags)
6. [Interactive Application & Live Radar](#interactive-application--live-radar)
7. [Coding Standards (Google C++ Style Guide)](#coding-standards)
8. [Project Layout](#project-layout)

---

## 1. System Overview

Traditional train safety systems are fundamentally reactive: they apply brakes only when a train crosses a physical red signal or breaches a fixed block boundary. 

**TCAS is an autonomous predictive safety platform.** It projects where every train on the railway will be $5, 10, 20, 30, 60, 90,$ and $120$ seconds into the future. By analyzing these multi-train space-time trajectories against directed track topology, switch junctions, track gradients, and station platforms, TCAS detects potential collisions minutes in advance. It arbitrates operational right-of-way, issues proportional speed reduction or signal holding commands, and **automatically resumes** held traffic as soon as the line clears.

---

## 2. Key Capabilities

- **Predictive Horizon Trajectory Modeling**: Projects 7 discrete future horizons ($5\text{s}$ to $120\text{s}$) across multi-segment routes with track speed limits and gradient physics.
- **Four-Class Conflict Detection**: Identifies and classifies Head-On, Rear-End, Junction Convergence, and Platform Contention hazards.
- **Space-Time Resource Reservation**: Locks critical track switches and junction nodes over temporal intervals $[t_{\text{start}}, t_{\text{end}}]$, preventing conflicting access.
- **Quantitative Risk Scoring**: Computes a multi-factor risk score ($0\text{–}100$) evaluating Time-to-Collision (TTC), relative speed, braking feasibility, train mass, and sensor/communication confidence.
- **Operational Priority Arbitration**: Express passenger trains take priority over commuter trains, which take priority over heavy freight ($300 > 200 > 100$).
- **Intelligent Auto-Resumption**: Trains held at a red signal automatically restart and accelerate back to their scheduled dispatch speed once the conflicting train clears the junction.
- **Fail-Safe Sticky Emergency Brakes**: Emergency-braked trains remain permanently interlocked until authorized human operator release.
- **1D Kalman Filter Sensor Fusion**: Filters tachometer noise, models wheel slip drift ($1\%$), and integrates Eurobalise transponder anchors.
- **Wireless Communication Simulation**: Models realistic V2V and V2I radio networks (LTE-R / GSM-R) with latency, distance attenuation, and packet drop.
- **Multi-Threaded Real-Time Cadence**: 4 decoupled concurrent loops (Physics $20\text{ms}$, Safety $100\text{ms}$, Comms $100\text{ms}$, HMI $200\text{ms}$).
- **Interactive ANSI Console Dashboard & Black-Box Logger**: Real-time fleet tracking radar, interactive menu, and CSV telemetry persistence.

---

## 3. Architecture & Documentation Index

The project is thoroughly documented across 13 dedicated module specifications and a comprehensive end-to-end master document:

| Module | Name & Domain | Key Components | Documentation |
|:---:|---|---|:---:|
| **01** | **Railway Infrastructure** | Directed graphs, Nodes, Tracks, Junctions, Platforms, BFS/DFS reachability | [Module 01 README](docs/module-01-railway-infrastructure/README.md) |
| **02** | **Train Entity Management** | Train polymorphism (Express, Passenger, Freight), TrainManager, fleet lifecycle | [Module 02 README](docs/module-02-train-management/README.md) |
| **03** | **Simulation & Clock** | Deterministic SimClock, quantum tick progression, periodic SimulationTimers | [Module 03 README](docs/module-03-simulation-clock/README.md) |
| **04** | **Physics & Braking Dynamics** | KinematicsEngine, gradient corrections, service/emergency stopping distance | [Module 04 README](docs/module-04-physics-braking/README.md) |
| **05** | **Route Planning & Navigation** | RouteNavigator (Dijkstra shortest path over directed graph), RouteResult | [Module 05 README](docs/module-05-route-navigation/README.md) |
| **06** | **Sensor Modeling & Estimation** | 1D Kalman Filter, wheel slip drift, Balise transponder anchor, outlier gating | [Module 06 README](docs/module-06-sensor-state-estimation/README.md) |
| **07** | **Wireless Communication** | CommunicationChannel, V2V/V2I messaging, latency delay, packet loss, mailboxes | [Module 07 README](docs/module-07-communication-simulation/README.md) |
| **08** | **Predictive Position Engine** | PredictionEngine, 7-horizon trajectories (5-120s), route boundary clamping | [Module 08 README](docs/module-08-predictive-position-engine/README.md) |
| **09** | **Conflict Detection & Reservation**| ConflictDetector (Head-On, Rear-End, Junction, Platform), ResourceReservationManager | [Module 09 README](docs/module-09-conflict-detection/README.md) |
| **10** | **Risk, Priority & Resolution** | RiskEngine (0-100), PriorityEngine, ConflictPriorityQueue, ResolutionEngine | [Module 10 README](docs/module-10-risk-priority-resolution/README.md) |
| **11** | **Scenario Simulation & Catalog** | 10 Catalog Routes (R-01..R-10), 8 Test Scenarios (Junction, Head-On, Failures) | [Module 11 README](docs/module-11-scenario-simulation/README.md) |
| **12** | **Thread Orchestrator & Safety Pipeline**| 4 concurrent loops (20/100/100/200ms), SafetyPipeline, CommandQueue, Auto-Resume | [Module 12 README](docs/module-12-thread-orchestrator/README.md) |
| **13** | **HMI, Telemetry & Application** | HmiDisplay (ANSI), TelemetryLogger (CSV), PerformanceMetrics, TcasApplication | [Module 13 README](docs/module-13-hmi-telemetry-application/README.md) |
| **ALL**| **Complete System Flow & Edge Cases**| Master guide: end-to-end architecture, multi-threaded flow, 4 demo flows, 10 edge cases | [FULL SYSTEM GUIDE](docs/FULL_SYSTEM.md) |

---

## 4. Build & Installation

### Prerequisites
- **CMake**: Version 3.20 or newer
- **C++ Compiler**: A compiler with full **C++23** support:
  - Microsoft Visual C++ (MSVC) 2022 v19.36 or higher
  - GCC 13.0 or higher
  - Clang 16.0 or higher
- **Threads**: POSIX threads or Win32 threads (automatically discovered by CMake)
- **GoogleTest**: Retrieved automatically via CMake `FetchContent` (Internet connection required for first build).

---

### Windows Build & Run

#### Option A: Visual Studio 2022 (MSVC)
Open Developer PowerShell or Command Prompt for Visual Studio 2022:

```powershell
# 1. Configure the project with RelWithDebInfo or Release
cmake -S . -B build -G "Visual Studio 17 2022" -A x64

# 2. Build all targets (tcas executable and tcas_tests) in parallel
cmake --build build --config RelWithDebInfo --parallel

# 3. Launch the interactive TCAS application
.\build\RelWithDebInfo\tcas.exe
```

#### Option B: MinGW / Ninja on Windows
```powershell
# 1. Configure with Ninja or MinGW Makefiles
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 2. Build
cmake --build build --parallel

# 3. Run
.\build\tcas.exe
```

---

### Linux Build & Run

Open a standard Linux shell (Ubuntu, Debian, Fedora, Arch, etc.):

```bash
# 1. Install prerequisites (e.g., on Ubuntu / Debian)
sudo apt update && sudo apt install -y build-essential cmake git

# 2. Configure the build
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 3. Compile the system (using all available CPU cores)
cmake --build build --parallel $(nproc)

# 4. Launch the interactive TCAS application
./build/tcas
```

---

## 5. Running Automated Tests (GoogleTest)

The TCAS test suite contains **299 automated unit, integration, and safety tests**.

### Running with CTest

#### On Windows:
```powershell
ctest --test-dir build -C RelWithDebInfo --output-on-failure
```

#### On Linux:
```bash
ctest --test-dir build --output-on-failure
```

*Expected output:*
```text
100% tests passed, 0 tests failed out of 299
Total Test time (real) = 0.45 sec
```

---

### Running the Test Binary Directly

Running the compiled test binary directly provides rich GoogleTest colored output:

#### Windows:
```powershell
.\build\RelWithDebInfo\tcas_tests.exe
```

#### Linux:
```bash
./build/tcas_tests
```

---

### Test Filtering & Advanced Flags

GoogleTest supports powerful command-line switches to run specific test subsets:

```bash
# Run only Module 10 Safety & Resolution tests:
./build/tcas_tests --gtest_filter="ResolutionEngineTest.*:RiskEngineTest.*"

# Run only Conflict Detection tests:
./build/tcas_tests --gtest_filter="ConflictDetectorTest.*"

# Run end-to-end multi-module integration tests:
./build/tcas_tests --gtest_filter="*IntegrationTest.*"

# Run thread orchestrator tests:
./build/tcas_tests --gtest_filter="ThreadOrchestratorTest.*"

# Run tests and export an XML report for CI/CD:
./build/tcas_tests --gtest_output="xml:test_results.xml"

# Repeat all tests 5 times to verify zero flakiness / thread race conditions:
./build/tcas_tests --gtest_repeat=5 --gtest_break_on_failure
```

---

## 6. Interactive Application & Live Radar

When you run `./build/tcas` or `tcas.exe`, the interactive console launches:

```text
================================================================================
          TCAS - TRAIN COLLISION AVOIDANCE SYSTEM (v0.1.0)
================================================================================
 [1] Fleet Management Menu     (Dispatch catalog/custom routes, list fleet)
 [2] Safety & Interlocking     (View active conflicts, reservations, commands)
 [3] Fault Injection Console   (Toggle sensor degradation, radio blackout)
 [4] Simulation Control Menu   (Start, pause, resume, reset simulation)
 [5] Live Radar View           (Real-time full-screen terminal tracking)
 [Q] Quit Application
================================================================================
Enter selection: 
```

### Trying the Junction Conflict Demo:
1. Select **`[1] Fleet Management`** $\to$ **`[1] Dispatch Catalog Route`**.
2. Dispatch Route **`7`** (Approach Yard Freight on Track 105).
3. Dispatch Route **`8`** (Alpha Converging Passenger on Track 101).
4. Select **`[5] Live Radar View`**.
5. Watch the real-time simulation:
   - The system detects the junction conflict at $t=25\text{s}$.
   - The Freight train receives `HoldAtSignal` and smoothly comes to a stop.
   - The Passenger train receives right-of-way and clears Alpha Junction.
   - The Freight train **automatically resumes**, accelerates to $20\text{ m/s}$, and continues its journey!

---

## 7. Coding Standards

This project strictly adheres to the **[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)** and modern C++23 best practices:

### 1. Naming Conventions
- **Types / Classes / Structs / Enums**: `PascalCase` (e.g., `ThreadOrchestrator`, `KinematicsEngine`, `RiskAssessment`, `SafetyCommand`).
- **Functions / Methods**: `camelCase` (e.g., `calculateBrakingFeasibility()`, `predictStandardHorizon()`, `updateOdometry()`).
- **Class Member Variables**: Private members end with a trailing underscore (e.g., `network_`, `trainManager_`, `reservations_`).
- **Constants**: Prefixed with `k` followed by `PascalCase` (e.g., `kGravity`, `kDefaultReactionTime`, `kMinReduceSpeedFloor`).
- **Enums & Enumerators**: Scoped `enum class` with `PascalCase` enumerators (e.g., `ConflictType::HeadOn`, `RiskLevel::Critical`).

### 2. Memory & Resource Safety (RAII)
- **Zero Raw Owning Pointers**: Dynamic memory is managed exclusively using smart pointers (`std::unique_ptr` and `std::shared_ptr`).
- **Rule of Zero / Five**: Resource-owning classes define explicit or defaulted copy/move operations (`= delete` for non-copyable singletons/orchestrators).
- **Thread Safety**: Strict scoped locking idioms (`std::lock_guard`, `std::unique_lock`, `std::shared_lock`).

### 3. Type Safety & Modern C++23 Idioms
- **Attributes**: Liberal use of `[[nodiscard]]` on all pure queries and calculation routines.
- **Const Correctness**: All non-modifying methods are marked `const`; parameters passed by `const &` or value for primitives.
- **Noexcept Specification**: Mathematical and query routines guaranteed not to throw are marked `noexcept`.
- **Variant Types**: Message payloads are strongly typed using `std::variant` rather than void pointers or raw byte buffers.
- **Header Guards**: All headers use `#pragma once`.

### 4. Compiler Warnings
- Compiled with clean warning flags: `/W4 /permissive-` on MSVC, and `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion` on GCC/Clang. Zero compiler warnings allowed.

---

## 8. Project Layout

```text
real_coll/
├── CMakeLists.txt             # Root CMake build configuration
├── README.md                  # Project overview, build/run guides, standards
├── data/                      # Network topologies, train configurations
├── docs/                      # Architectural specifications & module docs
│   ├── FULL_SYSTEM.md         # Master all-in-one system flow & edge cases guide
│   ├── module-01-.../         # Module 01: Railway Infrastructure
│   ├── module-02-.../         # Module 02: Train Management
│   ├── module-03-.../         # Module 03: Simulation & Clock
│   ├── module-04-.../         # Module 04: Physics & Braking
│   ├── module-05-.../         # Module 05: Route & Navigation
│   ├── module-06-.../         # Module 06: Sensor & State Estimation
│   ├── module-07-.../         # Module 07: Communication Simulation
│   ├── module-08-.../         # Module 08: Predictive Position Engine
│   ├── module-09-.../         # Module 09: Conflict Detection & Reservation
│   ├── module-10-.../         # Module 10: Risk, Priority & Resolution
│   ├── module-11-.../         # Module 11: Scenario Simulation & Route Catalog
│   ├── module-12-.../         # Module 12: Thread Orchestrator & Safety Pipeline
│   └── module-13-.../         # Module 13: HMI, Telemetry & Interactive Application
├── include/                   # Public C++ header files
│   ├── application/           # Interactive TcasApplication CLI
│   ├── common/                # Common types, physical units, constants
│   ├── communication/         # Messages and V2V/V2I channel simulation
│   ├── conflict/              # Conflict detectors, zones, resource reservation
│   ├── hmi/                   # HMI display, telemetry logger, performance metrics
│   ├── infrastructure/        # Nodes, tracks, stations, junctions, platforms
│   ├── navigation/            # Dijkstra route navigator and route results
│   ├── orchestrator/          # Multi-threaded orchestrator, safety pipeline, queues
│   ├── physics/               # KinematicsEngine physics and braking equations
│   ├── prediction/            # Predictive trajectory projection engine
│   ├── safety/                # Risk, priority arbitration, and resolution engines
│   ├── scenario/              # Route catalog and scenario manager
│   ├── sensor/                # Odometer simulation and 1D Kalman filter
│   └── train/                 # Train entity hierarchy and fleet manager
├── logs/                      # Telemetry CSV outputs generated at runtime
├── scripts/                   # Automated build and test scripts
├── src/                       # Production C++ implementation files
└── tests/                     # GoogleTest test suites (299 tests)
    ├── communication/
    ├── conflict/
    ├── hmi/
    ├── infrastructure/
    ├── integration/           # Cross-module seam tests
    ├── navigation/
    ├── orchestrator/
    ├── physics/
    ├── prediction/
    ├── safety/
    ├── scenario/
    ├── sensor/
    ├── simulation/
    └── train/
```
