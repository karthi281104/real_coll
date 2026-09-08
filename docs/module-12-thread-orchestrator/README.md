# Module 12 — Multi-Threaded Real-Time Orchestrator & Safety Pipeline

## 1. Module Overview

Module 12 is the real-time, multi-threaded operational backbone and safety execution pipeline of the Train Collision Avoidance System (TCAS).

In a production railway operations center, physical train movement, safety computations, wireless communications, and graphical operator displays run at vastly different cadences:
- Train physics must update rapidly and smoothly ($20\text{ ms} = 50\text{ Hz}$).
- Heavy trajectory prediction, conflict detection, and mathematical risk evaluations run periodically ($50\text{–}100\text{ ms} = 10\text{–}20\text{ Hz}$).
- Wireless radio packet transmission and mailbox processing step at network intervals ($100\text{ ms} = 10\text{ Hz}$).
- Human-Machine Interfaces (HMI) and telemetry logging update at display refresh rates ($200\text{ ms} = 5\text{ Hz}$).

Module 12 brings these decoupled subsystems together into a synchronized, thread-safe, lock-free/read-write locked runtime environment. It implements the end-to-end `SafetyPipeline` (chaining Modules 8 through 10) and features the **automated train resumption engine** that safely restarts held trains once higher-priority trains have cleared the line.

---

## 2. Objectives

1. **Four Concurrent Real-Time Threads**: Coordinate Physics ($20\text{ms}$), Safety ($100\text{ms}$), Comms ($100\text{ms}$), and HMI ($200\text{ms}$) with zero jitter accumulation.
2. **Lock-Free Command Passing**: Route safety commands from the Safety thread to the Physics thread using a thread-safe `CommandQueue`.
3. **End-to-End Safety Pipeline Integration**: Encapsulate Modules 8, 9, and 10 into a cohesive, stateless `SafetyStep` callable.
4. **State Machine Management**: Enforce strict train operational states (`Idle`, `Running`, `Slowing`, `Braking`, `Stopped`, `EmergencyBrake`, `Completed`).
5. **Intelligent Auto-Resumption**: Automatically resume trains held at signals when priority traffic clears, restoring their catalog dispatch speed without human intervention.
6. **Sticky Emergency Brakes**: Ensure emergency-braked trains remain permanently locked until explicitly cleared by an authorized operator.
7. **Thread-Safe World Snapshotting**: Provide immutable `WorldState` snapshots to external UIs via `std::shared_mutex` without blocking the physics loop.

---

## 3. Architectural Role & Multi-Threaded Model

```text
                           ThreadOrchestrator
                                   │
      ┌────────────────┬───────────┴───────────┬────────────────┐
      ▼                ▼                       ▼                ▼
[Physics Thread] [Safety Thread]       [Comms Thread]     [HMI Thread]
   (20 ms)          (100 ms)              (100 ms)           (200 ms)
      │                │                       │                │
KinematicsEngine  SafetyPipeline:        CommsChannel:      HmiDisplay /
  - updatePos     - PredictionEngine(8)   - Latency delay    TelemetryLogger:
  - updateVel     - ConflictDetector(9)   - Range cut-off    - Render console
  - Boundary check- RiskEngine(10)        - Packet drop      - Log CSV events
  - Clamping      - PriorityEngine(10)    - Mailbox delivery - Latency stats
                  - PriorityQueue(10)
                  - ReservationMgr(9)
                  - ResolutionEngine(10)
      ▲                │
      │   CommandQueue │
      └────────────────┘
```

---

## 4. Implementation Details

### 4.1 The Four Real-Time Loops

| Thread Loop | Cadence | Execution Responsibilities |
|---|---|---|
| **`physicsLoop()`** | $20\text{ ms}$ ($50\text{ Hz}$) | Drains user commands; drains safety commands from `CommandQueue`; runs `KinematicsEngine::updatePosition/Velocity`; executes track boundary transitions; updates train states (`Slowing` $\to$ `Running`, `Braking` $\to$ `Stopped`); increments simulation time $\Delta t = 0.020\text{ s}$. |
| **`safetyLoop()`** | $100\text{ ms}$ ($10\text{ Hz}$) | Takes immutable `WorldState` snapshot; executes `SafetyPipeline`; generates `SafetyCycleResult`; pushes corrective `SafetyCommand`s to `CommandQueue`; executes **Auto-Resume** evaluation for held trains. |
| **`communicationLoop()`** | $100\text{ ms}$ ($10\text{ Hz}$) | Steps `CommunicationChannel`; routes in-flight messages past their delivery latency into recipient mailboxes; tracks packet delivery rate and radio health. |
| **`hmiLoop()`** | $200\text{ ms}$ ($5\text{ Hz}$) | Extracts `WorldState` snapshot; formats ANSI terminal display; writes telemetry log entries to disk; records jitter and latency metrics. |

### 4.2 The Safety Pipeline Chain
The `SafetyPipeline` executes the following deterministic 7-stage chain during every safety tick:
1. **`PredictionEngine` (Mod 8)**: Projects 7-horizon trajectory profiles for all active trains.
2. **`ConflictDetector` (Mod 9)**: Identifies `HeadOn`, `RearEnd`, `Junction`, and `Platform` hazards.
3. **`RiskEngine` (Mod 10)**: Computes risk score $S \in [0, 100]$ and classifies severity (`Low` to `Critical`).
4. **`PriorityEngine` (Mod 10)**: Evaluates service class (Express $>$ Passenger $>$ Freight) and distances.
5. **`ConflictPriorityQueue` (Mod 10)**: Orders multiple active conflicts so the highest risk is arbitrated first.
6. **`ResourceReservationManager` (Mod 9)**: Locks track and junction resources for the priority train; rejects non-priority reservations.
7. **`ResolutionEngine` (Mod 10)**: Computes `SafetyCommand`s (`NoAction`, `ReduceSpeed`, `HoldAtSignal`, `EmergencyBrake`).

---

## 5. Automatic Resumption Logic (How Held Trains Move Again)

A common problem in naive collision avoidance systems is that once a train is stopped to let another train pass, it gets stuck forever. TCAS implements a sophisticated, race-condition-free **Auto-Resume Engine**:

```text
[Priority Train 1 (Express) Approaches Junction with Non-Priority Train 2 (Freight)]
                                    │
                                    ▼
       1. SafetyPipeline issues HoldAtSignal to Train 2 (Freight)
          - Train 2 state -> Braking -> velocity reaches 0 -> state -> Stopped
          - Train 2 recorded in trainsHeldBySafety_
                                    │
                                    ▼
       2. Train 1 (Express) traverses and clears the Junction
          - Reservation on Junction expires / releases
          - PredictionEngine shows no further spatial overlap
          - ConflictDetector reports activeConflicts is now EMPTY
                                    │
                                    ▼
       3. Safety Loop evaluates trainsHeldBySafety_
          - Is Train 2 in emergencyBrakeSet_? NO (HoldAtSignal, not EmergencyBrake)
          - Is Train 2 involved in any active conflicts this cycle? NO.
          - BUG-4 Fix: Has Train 2 physically come to a full stop?
            (state == Stopped or Slowing, velocity <= 0.1 m/s) -> YES!
                                    │
                                    ▼
       4. Auto-Resume Triggered!
          - Restore catalog speed: safetySpeedLimits_[tid] = dispatchSpeeds_[tid]
            (BUG-1 Fix: uses original catalog speed, e.g. 20 m/s, not hardcoded 15 m/s)
          - train->setAcceleration(0.5) (service acceleration)
          - train->setState(TrainState::Running)
          - Remove tid from trainsHeldBySafety_
                                    │
                                    ▼
       5. Train 2 safely accelerates and continues its journey!
```

### Sticky Emergency Brake vs Auto-Resume
- **`HoldAtSignal` / `ReduceSpeed`**: Automatic recovery. Once the conflict clears and the train is stopped, the safety loop resumes it automatically.
- **`EmergencyBrake`**: **STICKY**. Stored in `emergencyBrakeSet_`. Emergency braking indicates a critical safety breach (e.g. potential head-on crash or sensor failure). The safety loop **will never auto-resume an emergency-braked train**. It requires explicit operator intervention via CLI (`SetSpeed` or `ResumeTrain` command) to clear the fault.

---

## 6. Public API Reference

```cpp
namespace tcas::orchestrator {

struct OrchestratorConfig {
    std::chrono::milliseconds physicsPeriod{ 20 };
    std::chrono::milliseconds safetyPeriod{ 100 };
    std::chrono::milliseconds communicationPeriod{ 100 };
    std::chrono::milliseconds hmiPeriod{ 200 };
    bool printHmi{ false };
    std::string telemetryDirectory{ "logs" };
};

class ThreadOrchestrator {
public:
    ThreadOrchestrator(
        const infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager,
        communication::CommunicationChannel& communicationChannel,
        std::vector<TrainId> trainIds,
        OrchestratorConfig config = {},
        SafetyStep safetyStep = {}
    );
    ~ThreadOrchestrator();

    void start();
    void stop();
    void pause();
    void resume();

    [[nodiscard]] bool isRunning() const noexcept;
    [[nodiscard]] bool isPaused() const noexcept;
    [[nodiscard]] WorldState snapshot() const;

    void postCommand(UserCommand command);
    void setSensorFault(TrainId trainId, bool fault);
    void setCommFault(bool fault);
    void addTrain(TrainId trainId);
    void removeTrain(TrainId trainId);
    void setTrainRoute(TrainId trainId, TrackId startTrackId, navigation::RouteResult route);
};

class SafetyPipeline {
public:
    SafetyPipeline(
        const infrastructure::RailwayNetwork& network,
        const train::TrainManager& trainManager,
        std::vector<TrainRoute> routes
    );
    void setRoutes(std::vector<TrainRoute> routes);
    void addOrUpdateRoute(TrainRoute route);
    void removeRoute(TrainId trainId);
    SafetyStep makeStep();
};

} // namespace tcas::orchestrator
```

---

## 7. Edge Cases & Bug Fixes Incorporated

1. **BUG-1 (Dispatch Speed Preservation)**: Previously, resuming a held train restored a hardcoded $15\text{ m/s}$, ignoring whether the train was originally scheduled for $35\text{ m/s}$ (Express) or $12\text{ m/s}$ (Freight). Fixed via `dispatchSpeeds_` tracking map.
2. **BUG-3 (State Overwrite Fix)**: Fixed race condition where setting `TrainState::Stopped` was prematurely overwritten by route completion logic before the train had arrived at the destination node.
3. **BUG-4 (Braking Resume Race Condition)**: Previously, if a conflict cleared while a non-priority train was still in `TrainState::Braking` (slowing down from $30\text{ m/s}$), the safety loop cleared the brake limit immediately. This caused the train to surge forward before ever stopping. Fixed by requiring `state == TrainState::Stopped` before allowing auto-resume.
4. **LOGIC-2 (Deadlock Junction Breaker)**: When two trains are held at the same junction, neither would move because each saw the other as an obstacle. The safety loop detects when both trains are stopped at the junction and grants release to the priority train to break deadlock.
5. **LOGIC-4 (Sticky Emergency Brake)**: Prevents dangerous automated restart after emergency stop.
6. **LOGIC-7 (Communication Degradation Hysteresis)**: Under wireless blackout, trains are capped to a fail-safe speed of $10\text{ m/s}$ until radio contact is stably restored.

---

## 8. Automated Test Verification

Validated through 4 comprehensive test suites:
- **`ThreadOrchestratorTest.cpp`**: Multi-threaded start/pause/resume/stop lifecycle, cycle increment assertions, command queue processing, and snapshot consistency.
- **`SafetyPipelineTest.cpp`**: End-to-end evaluation of multi-train scenarios producing valid decisions and reservations.
- **`CommandQueueTest.cpp`**: Concurrent producer-consumer thread safety of safety command delivery.
- **`Module8To12IntegrationTest.cpp`**: Complete system integration across all 12 modules running concurrently.

All 299 project tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Imagine a busy railway control tower staffed by 4 specialist operators:

1. **The Train Engineer (Physics Loop, 50 times a second)**: Sits in the cab of every train, moving the throttle, adjusting the brakes, and calculating exact position along the track.
2. **The Safety Officer (Safety Pipeline, 10 times a second)**: Stares at the radar screen, runs collision predictions into the future, and orders trains to slow down or stop at red signals.
3. **The Radio Operator (Communications Loop, 10 times a second)**: Handles wireless radio messages between trains and the dispatch center, managing radio lag and static.
4. **The Station Master (HMI Display, 5 times a second)**: Updates the giant electronic departures and arrivals board for human supervisors.

**Module 12 is the "Conductor & Brain" that runs all 4 operators simultaneously.**

Most importantly, it solves the **"Who Goes First?"** dilemma:
When a VIP high-speed train and a heavy freight train approach the same track switch:
- Module 12 politely orders the freight train: *"Please stop at the red signal and wait."*
- The freight train smoothly brakes to a complete stop.
- The VIP train zooms through the switch safely.
- As soon as the VIP train is clear, Module 12 automatically turns the signal green and tells the freight train: *"The track is clear! You may now accelerate back to your normal cruising speed."*

All of this happens completely automatically in real time without requiring a human to flip switches!
