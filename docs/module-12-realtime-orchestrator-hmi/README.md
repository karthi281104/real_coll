# Module 12 — Real-Time Orchestrator, HMI & Telemetry

## 1. Architectural Role

Module 12 is the central runtime engine of the TCAS application. It coordinates all physical, predictive, communicative, and human-facing subsystems across a **multi-threaded, real-time operating architecture**.

It enforces strict deterministic scheduling, real-time period deadlines, thread-safe state synchronization, high-frequency telemetry logging, and a live, single-writer Human-Machine Interface (HMI).

```
                            User Input Thread
                        (Non-blocking stdin parser)
                                     │
                                     ▼ (Thread-safe)
                              userCommandQueue_
                                     │
                                     ▼
                      ┌──────────────────────────────┐
                      │      ThreadOrchestrator      │
                      └──────────────┬───────────────┘
                                     │
         ┌───────────────────────────┼───────────────────────────┐
         │                           │                           │
         ▼ (50 Hz / 20ms)            ▼ (10 Hz / 100ms)           ▼ (10 Hz / 100ms)
    Physics Thread             Safety Thread               Comm Thread
 ├── Drains User Commands   ├── Snapshot WorldState     ├── Advance Comm Channel
 ├── Drains Safety Commands ├── Run SafetyPipeline      └── Update Drop/Loss Stats
 ├── Update Kinematics      │   (Modules 8, 9, 10)               │
 └── Update WorldState      └── Push Safety Commands             │
         │                           │                           │
         └───────────────────────────┼───────────────────────────┘
                                     │
                                     ▼
                             std::shared_mutex
                                (WorldState)
                                     │
                                     ▼ (5 Hz / 200ms)
                                 HMI Thread
                         ├── TelemetryLogger (CSV)
                         ├── PerformanceMetrics
                         └── HmiDisplay (ANSI Live Terminal)
```

---

## 2. Multi-Threaded Real-Time Architecture

The TCAS runtime orchestrator manages 4 dedicated background worker threads plus an interactive input thread:

| Thread | Frequency | Period | Thread Function | Responsibilities |
|---|---|---|---|---|
| **Physics Thread** | $50\text{ Hz}$ | $20\text{ ms}$ | `physicsLoop()` | Physical state owner. Drains user commands, drains safety commands, performs numerical integration of kinematics, manages track boundary progression, and updates the authoritative `WorldState`. |
| **Safety Thread** | $10\text{ Hz}$ | $100\text{ ms}$ | `safetyLoop()` | Executes the `SafetyPipeline` (trajectory prediction, conflict detection, risk scoring, priority queueing, resource reservation, and resolution). Dispatches `SafetyCommand`s to `commandQueue_`. |
| **Comm Thread** | $10\text{ Hz}$ | $100\text{ ms}$ | `communicationLoop()` | Simulates V2V and V2I wireless transmission, delays, packet loss drop rates, and link quality metrics. |
| **HMI Thread** | $5\text{ Hz}$ | $200\text{ ms}$ | `hmiLoop()` | Reads read-only `WorldState` snapshot. Renders the flicker-free ANSI terminal dashboard to `std::cout`, updates `PerformanceMetrics`, and appends high-frequency telemetry logs to CSV. |
| **Input Thread** | Event-driven | Asynchronous | `inputLoop()` | Listens for operator keyboard input on `std::cin` and converts inputs into thread-safe `UserCommand` structs posted to `userCommandQueue_`. |

---

## 3. Thread Synchronization & Single-Owner Model

To prevent data races and lock contention without degrading hard real-time deadlines:

1. **Single Simulation Owner**: Only the **Physics Thread** is permitted to mutate physical train objects (`TrainManager`, position, velocity, acceleration). The input thread never mutates train objects directly.
2. **Read-Copy-Update with `std::shared_mutex`**: `worldState_` is protected by `worldMutex_`. The physics and safety loops acquire unique write locks (`std::unique_lock`) only during instantaneous atomic snapshot updates. The HMI and external monitoring threads acquire shared read locks (`std::shared_lock`).
3. **Thread-Safe Queues**:
   - `userCommandQueue_`: Operator commands (speed changes, route changes, dynamic train additions, fault injections) are enqueued by the input thread and consumed by the physics thread.
   - `commandQueue_`: Safety commands (`EmergencyBrake`, `ReduceSpeed`, `HoldAtSignal`) emitted by the safety pipeline are consumed by the physics thread.
4. **Single-Writer Terminal Principle**: Only the **HMI thread** writes to `std::cout`. User command confirmations and error diagnostics are posted to `worldState_.operatorMessage` and displayed on the status banner during the next frame, eliminating screen tearing.

---

## 4. HMI Display Engine

Implemented in [`src/hmi/HmiDisplay.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/src/hmi/HmiDisplay.cpp).

The HMI employs standard VT100 ANSI escape codes (`\033[H\033[J` - cursor home and erase below) to create a flicker-free in-place control center:

```text
==============================================================
                    TCAS CONTROL CENTER
==============================================================

SYSTEM STATUS : RUNNING
SAFETY STATUS : CONFLICT ACTIVE
SIMULATION    : 1.40 s
COMMUNICATION : OK
SENSORS       : OK

--------------------------------------------------------------
THREADS
Physics : 70 | Safety : 14 | Comm : 14 | HMI : 7
--------------------------------------------------------------
TRAIN STATUS
--------------------------------------------------------------
ID       TYPE       TRACK       POSITION    SPEED     STATE
       1    Express        101    1828.00 m     20.00 m/s  RUNNING
       3    Freight        105    1810.50 m      7.50 m/s  BRAKING

--------------------------------------------------------------
ACTIVE CONFLICTS
--------------------------------------------------------------
Type: JUNCTION  Node: 2  Trains: 1 <-> 3  TTC: 8.60 s  MinSep: 0.00 m

--------------------------------------------------------------
SAFETY DECISIONS
--------------------------------------------------------------
Priority Train #1  Yielding Train #3  Risk: 52.00  Command: REDUCE SPEED

--------------------------------------------------------------
JUNCTION RESERVATIONS
--------------------------------------------------------------
Node 2 : owner Train #1  state=1

--------------------------------------------------------------
ACTION:
Train #3 -> REDUCE SPEED -> 7.50 m/s

OPERATOR MSG  : [OK] Simulation started. Real-time safety pipeline active.
==============================================================
```

---

## 5. Telemetry Logging & Performance Metrics

### 5.1 TelemetryLogger

Implemented in [`src/hmi/TelemetryLogger.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/src/hmi/TelemetryLogger.cpp).

Writes structured real-time event streams to the `logs/` directory:
- **`telemetry.csv`**: Time, TrainId, TrackId, Position, Velocity, Acceleration, State.
- **`conflicts.csv`**: Time, ConflictType, TrainA, TrainB, NodeId, TrackId, TTC, MinSeparation.
- **`events.csv`**: System lifecycle transitions, fault injections, manual speed overrides, and emergency braking actions.

### 5.2 PerformanceMetrics

Tracks runtime operational statistics:
- Physics and safety loop actual execution times vs period deadlines.
- Number of active conflict observations and emergency braking interventions.
- Wireless communication packets transmitted, delivered, dropped, and packet drop rate percentage.

---

## 6. Verification & Testing

Implemented in:
- [`tests/orchestrator/ThreadOrchestratorTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/orchestrator/ThreadOrchestratorTest.cpp)
- [`tests/orchestrator/CommandQueueTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/orchestrator/CommandQueueTest.cpp)
- [`tests/hmi/HmiDisplayTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/hmi/HmiDisplayTest.cpp)
- [`tests/hmi/TelemetryLoggerTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/hmi/TelemetryLoggerTest.cpp)
- [`tests/hmi/PerformanceMetricsTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/hmi/PerformanceMetricsTest.cpp)
