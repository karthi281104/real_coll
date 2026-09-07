# TCAS Integrated System Architecture & Operational Guide

## 1. Executive Summary & Mission Statement

The **TCAS (Train Collision Avoidance System)** is a mission-critical, hard real-time train protection and collision prevention system engineered in modern **C++23**. 

Designed to meet the stringent functional and temporal demands of contemporary railway networks (such as European ETCS Level 2/3 and Indian Kavach), TCAS operates as an automated guardian layer. It autonomously forecasts train positions along assigned routes, identifies spatial-temporal convergence across shared infrastructure (head-on, rear-end, junction, and platform conflicts), assesses multi-factor risk, grants temporal resource leases, and issues deterministic braking commands—all while guaranteeing absolute fail-safe precedence over manual or operational priorities.

---

## 2. End-to-End System Workflow

```
[ Physical World / Sensors ]
       │  (Position, Velocity, Acceleration, Odometer Pulses)
       ▼
[ Module 6: State Estimator & Sensor Fusion ]
       │  (Fused State + Position Uncertainty Envelope ±σ)
       ▼
[ Module 12: Physics Thread (50 Hz / 20 ms) ] ───▶ [ Module 4: Kinematics Engine ]
       │  (Updates Real-Time Kinematics & Boundaries)
       ▼
[ WorldState Snapshot (std::shared_mutex) ]
       │
       ▼
[ Module 12: Safety Thread (10 Hz / 100 ms) ] ───▶ [ SafetyPipeline ]
       │
       ├──▶ [ Module 8: Trajectory Prediction ]
       │        Projects FutureStates across {5s, 10s, 20s, 30s, 60s} horizons
       │        Propagates track gradients, speed limits, and expanding uncertainty ±σ(t)
       │
       ├──▶ [ Module 9: Conflict Detection & Resource Reservation ]
       │        Analyzes all train pairs via continuous analytical interval solver
       │        Detects Head-On, Rear-End, Junction, and Platform conflicts
       │        Requests exclusive temporal leases on Junctions & Platforms
       │
       └──▶ [ Module 10: Risk Assessment, Priority & Resolution ]
                Stage 1: Computes multi-factor risk score (0 - 100)
                Stage 2: Sorts by urgency in max-heap ConflictPriorityQueue
                Stage 3: Verifies physical stopping feasibility (d_avail >= d_brake + d_margin)
                Stage 4: Emits SafetyCommands (NoAction, ReduceSpeed, HoldAtSignal, EmergencyBrake)
       │
       ▼
[ CommandQueue (Thread-Safe FIFO) ]
       │
       ▼
[ Module 12: Physics Thread ]
       │  Consumes SafetyCommands, enforces safety limits, actuates train deceleration
       ▼
[ Module 12: HMI Thread (5 Hz / 200 ms) ]
       ├──▶ [ HmiDisplay ]: Renders live in-place VT100 ANSI Control Center
       ├──▶ [ TelemetryLogger ]: Streams telemetry.csv, conflicts.csv, events.csv
       └──▶ [ PerformanceMetrics ]: Tracks latency, cycle deadlines, and safety counts
```

---

## 3. The 12-Module Layered Architecture

The codebase is organized into 12 cleanly decoupled modules following a strict unidirectional dependency hierarchy:

```text
┌────────────────────────────────────────────────────────────────────────┐
│               LAYER 4: APPLICATION & ORCHESTRATION                     │
│  Module 11: Scenario Simulation     │  Module 12: Orchestrator & HMI   │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────▼────────────────────────────────────┐
│                    LAYER 3: TCAS SAFETY PIPELINE                       │
│  Module 8: Trajectory Prediction    │  Module 9: Conflict Detection    │
│  Module 10: Risk Assessment, Priority Engine & Resolution Engine       │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────▼────────────────────────────────────┐
│                    LAYER 2: KINEMATICS & COMMUNICATION                 │
│  Module 4: Physics & Braking        │  Module 5: Route Navigation      │
│  Module 6: Sensor & State Estimation│  Module 7: Communication Sim     │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────▼────────────────────────────────────┐
│                    LAYER 1: SYSTEM FOUNDATION                          │
│  Module 1: Railway Infrastructure   │  Module 2: Train Management      │
│  Module 3: Simulation Clock         │                                  │
└────────────────────────────────────────────────────────────────────────┘
```

### Module Summary Directory

| Module | Purpose | Key Classes / Files |
|---|---|---|
| **Module 1: Infrastructure** | Topological network graph; nodes (stations, junctions, platforms) and directed tracks (length, gradient, speed limit). | `Node`, `Track`, `RailwayNetwork` |
| **Module 2: Train Management** | Train entity modeling, physical parameters (mass, max speed, service/emergency braking), and fleet management. | `Train`, `ExpressTrain`, `PassengerTrain`, `FreightTrain`, `TrainManager` |
| **Module 3: Simulation Clock** | Deterministic discrete time management, fixed timesteps ($\Delta t = 20\text{ ms}$), and cyclic timers. | `SimClock`, `SimulationConfig`, `SimulationTimer` |
| **Module 4: Physics & Braking** | Numerical integration of kinematics, gradient gravity adjustments, and stopping distance formulas. | `KinematicsEngine` |
| **Module 5: Route Navigation** | Dijkstra shortest-path route planning, route validation, and total distance/travel time calculations. | `RouteNavigator`, `RouteResult` |
| **Module 6: Sensor Estimation** | Wheel-odometer simulation, sensor noise modeling, and fused position/velocity estimation with uncertainty. | `Odometer`, `StateEstimator` |
| **Module 7: Communication** | V2V/V2I wireless channel simulation, transmission latency, physical range limits, and packet loss drop rates. | `CommunicationChannel`, `Message` |
| **Module 8: Trajectory Prediction** | Multi-horizon kinematic lookahead projection across connected tracks with dynamic uncertainty expansion. | `PredictionEngine`, `FutureState` |
| **Module 9: Conflict Detection** | Analytical trajectory solver for same-track conflicts, converging junction events, and resource reservation leasing. | `ConflictDetector`, `ResourceReservationManager`, `ConflictZone` |
| **Module 10: Risk & Resolution** | 7-factor risk scoring ($0-100$), operational priority hierarchy, conflict queue, and safety command resolution. | `RiskEngine`, `PriorityEngine`, `ConflictPriorityQueue`, `ResolutionEngine` |
| **Module 11: Scenarios** | 8 standardized collision, degradation, and infrastructure benchmark scenarios for testing and demonstration. | `ScenarioManager`, `ScenarioResult` |
| **Module 12: Orchestrator & HMI** | Multi-threaded real-time executive, thread-safe command dispatching, CSV telemetry logging, and live ANSI HMI. | `ThreadOrchestrator`, `TcasApplication`, `HmiDisplay`, `TelemetryLogger` |

---

## 4. Multi-Threaded Real-Time Execution Model

### 4.1 Scheduling & Period Deadlines

The TCAS runtime orchestrator manages 4 background worker threads plus an event-driven user input thread:

```text
Time (ms)   0    20   40   60   80   100  120  140  160  180  200
Physics   : [T1] [T2] [T3] [T4] [T5] [T6] [T7] [T8] [T9] [TA] [TB]  (Every 20ms)
Safety    : [   S1   ]               [   S2   ]               [   S3   ] (Every 100ms)
Comm      : [   C1   ]               [   C2   ]               [   C3   ] (Every 100ms)
HMI       : [             H1             ]                    [   H2   ] (Every 200ms)
```

1. **Physics Thread ($50\text{ Hz} / 20\text{ ms}$)**:
   - Drains operator commands from `userCommandQueue_`.
   - Drains safety commands from `commandQueue_`.
   - Evaluates active speed limits: $v_{limit} = \min(v_{operator}, v_{safety}, v_{track}, v_{max})$.
   - Performs kinematic numerical integration: $p(t+\Delta t), v(t+\Delta t)$.
   - Handles track boundary progression and transitions.
   - Atomically updates the authoritative `WorldState` snapshot.

2. **Safety Thread ($10\text{ Hz} / 100\text{ ms}$)**:
   - Takes a read-only snapshot of `WorldState`.
   - Projects trajectories across all horizons (Module 8).
   - Detects all pairwise conflicts (Module 9).
   - Computes risk scores and sorts conflicts in `ConflictPriorityQueue` (Module 10).
   - Acquires/renews junction and platform resource reservations (Module 9).
   - Determines yielding trains and issues `SafetyCommand`s (Module 10).
   - Pushes safety commands to the thread-safe `commandQueue_`.

3. **Communication Thread ($10\text{ Hz} / 100\text{ ms}$)**:
   - Advances wireless channel simulation time.
   - Delivers in-flight telemetry and emergency broadcast packets.
   - Measures packet delivery and loss rates; flags communication degradation if drop rate exceeds $25\%$.

4. **HMI Dashboard Thread ($5\text{ Hz} / 200\text{ ms}$)**:
   - Takes a read-only `WorldState` snapshot.
   - Updates `PerformanceMetrics` (execution times, deadline misses, conflict counts).
   - Appends high-frequency telemetry streams to `telemetry.csv`, `conflicts.csv`, and `events.csv`.
   - Renders the live VT100 ANSI Control Center dashboard to `std::cout`.

5. **User Input Thread (Event-Driven)**:
   - Non-blocking line reader on `std::cin`.
   - Converts operator menu actions into thread-safe `UserCommand` structs.
   - Dispatches commands to `userCommandQueue_` for safe processing by the Physics Thread.

### 4.2 Concurrency & Thread-Safety Guarantees

- **Single Simulation Owner**: Only the Physics Thread modifies `Train` objects. The input thread and safety thread never directly touch train state.
- **Read-Copy-Update Synchronization**: `WorldState` is guarded by `std::shared_mutex worldMutex_`. Writer threads hold short-lived unique write locks (`std::unique_lock`) only during data copy. Reader threads acquire shared read locks (`std::shared_lock`).
- **Single-Writer Terminal Architecture**: Only the HMI Thread writes to standard output (`std::cout`). Operator action acknowledgments and error diagnostics are routed through `worldState_.operatorMessage` and displayed on the status banner during the next frame, completely eliminating terminal interleaving and garbled text.

---

## 5. Core Mathematical Formulations

### 5.1 Emergency Stopping Distance
The physical emergency stopping distance on a track with gradient $\theta = \tan(\alpha)$ is:
$$d_{stopping} = \frac{v^2}{2 \cdot (a_{emergency} + g \cdot \theta)}$$
where $g = 9.81\text{ m/s}^2$. Uphill grades ($\theta > 0$) reduce required stopping distance; downhill grades ($\theta < 0$) extend it.

### 5.2 Dynamic Position Uncertainty
Sensor measurement uncertainty expands linearly over lookahead time $t$:
$$\sigma(t) = \sigma_0 + 0.5 \cdot t$$
- **Nominal Sensor**: $\sigma_0 = 1.0\text{ m} \implies \sigma(60\text{s}) = 31.0\text{ m}$
- **Degraded Sensor**: $\sigma_0 = 15.0\text{ m} \implies \sigma(60\text{s}) = 45.0\text{ m}$

### 5.3 Analytical Conflict Quadratic Solver
For trains $A$ and $B$ on a shared track segment, relative displacement is $\Delta p(t) = \Delta p_0 + \dot{p}t$ and dynamic protected cushion is $m(t) = d_{min} + \sigma_A(t) + \sigma_B(t) = m_0 + \dot{m}t$. Conflict occurs when $\Delta p(t)^2 \le m(t)^2$, solved via the quadratic equation:
$$q_a t^2 + q_b t + q_c \le 0$$
$$\text{where } q_a = \dot{p}^2 - \dot{m}^2, \quad q_b = 2(\Delta p_0 \dot{p} - m_0 \dot{m}), \quad q_c = \Delta p_0^2 - m_0^2$$
The roots of the discriminant $\mathcal{D} = q_b^2 - 4 q_a q_c$ yield the exact boundary timestamps `firstConflictTime` and `lastConflictTime`.

### 5.4 Composite Risk Scoring Formula
$$\text{Risk Score} = S_{TTC} + S_{\Delta v} + S_{braking} + S_{type} + S_{mass} + S_{sensor} + S_{comm} \in [0.0, 100.0]$$
- **Severity Levels**: Low ($\le 30$), Medium ($31-60$), High ($61-80$), Critical ($> 80$).

### 5.5 Braking Feasibility Precedence
A train is evaluated for braking feasibility before an intervention command is issued:
$$\text{Feasible} = (d_{available} \ge d_{stopping} + \text{Safety Margin})$$
If $\text{Feasible} = \text{false}$, operational priority is immediately revoked, and an **Emergency Brake** command is issued to prevent collision.

---

## 6. Fault Injection & Degraded Modes

TCAS features built-in fault injection mechanisms for real-time stress testing:

1. **Sensor Fault Injection (`postCommand(InjectSensorFailure)`)**:
   - Degrades odometer precision; expands $\sigma_0$ from $1.0\text{ m}$ to $15.0\text{ m}$.
   - Safety pipeline widens protected separation envelopes.
   - Status banner updates to `SENSORS : DEGRADED` / `SYSTEM STATUS : DEGRADED`.
2. **Communication Fault Injection (`postCommand(SetCommLossRate)`)**:
   - Simulates realistic RF packet loss ($30\%$ or $70\%$).
   - Communication thread flags channel degradation when packet loss exceeds $25\%$.
   - Risk engine increases risk score by $+10\text{ pts}$ to enforce conservative spacing.
3. **Emergency Stop Override**:
   - Operator can command immediate train halt (`HoldTrain`) or resume (`ResumeTrain`).
   - Safety pipeline retains override authority if a physical collision risk emerges.

---

## 7. Interactive HMI Menu & Command Reference

The TCAS Control Center provides an interactive CLI interface with the following commands:

```text
==============================================================
OPERATOR ACTIONS:
  [1] Start      [2] Pause      [3] Resume     [4] Add Train   [5] Remove Train
  [6] Set Speed  [7] Chg Route  [8] Hold Train [9] Resume Train
 [10] Fault Sens [11] Recv Sens [12] Fault Comm [13] Recv Comm
 [14] Conflicts  [15] Reserv    [16] Telemetry  [17] Metrics   [18] Reset   [19] Shutdown

DEMO SCENARIOS:
 [24] Junction Conflict    [25] Rear-End Conflict   [26] Head-On Conflict
 [27] Platform Conflict    [28] Multiple Conflicts  [29] Sensor Failure
 [30] Comm Failure         [31] Unsafe Stopping Distance
Shortcuts: P=Pause, R=Resume, F=Sensor, C=Comm, S <id> <v>=Speed, H <id>=Hold, Q=Quit
==============================================================
```

---

## 8. Build, Test & Verification Guide

### 8.1 Linux Build & Test (GCC / Clang)

```bash
# 1. Clone or pull latest branch
git checkout finalupdated
git pull origin finalupdated

# 2. Configure build with CMake (C++23)
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 3. Build in parallel
cmake --build build -j$(nproc)

# 4. Execute all 293 unit & integration tests
ctest --test-dir build --output-on-failure

# 5. Run static analysis (Cppcheck C++23)
make check

# 6. Launch the live real-time Control Center
./build/tcas
```

### 8.2 Windows Build & Test (MSVC / PowerShell)

```powershell
# 1. Configure build
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 2. Build binaries
cmake --build build --config RelWithDebInfo

# 3. Execute all 293 tests
ctest -C RelWithDebInfo --test-dir build --output-on-failure

# 4. Launch live Control Center
.\build\RelWithDebInfo\tcas.exe
```

---

## 9. Conclusion

The TCAS project provides a complete, mathematically sound, and rigorously verified train collision avoidance architecture. By combining deterministic multi-threaded orchestration, continuous trajectory interpolation, multi-factor risk assessment, temporal resource reservation, and live single-writer telemetry, TCAS demonstrates the highest standard of railway safety automation in modern C++23.
