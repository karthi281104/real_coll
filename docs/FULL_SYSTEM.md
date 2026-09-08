# TCAS Complete System Architecture, End-to-End Flow & Edge Cases

## 1. Executive Summary

The **Real-Time Train Collision Avoidance System (TCAS)** is an autonomous, mission-critical safety system built in modern C++23. It provides deterministic, predictive protection against train collisions across complex railway networks.

Unlike traditional reactive railway cab-signaling systems that only alert drivers when a signal is passed at danger (SPAD), TCAS continuously projects multi-train space-time trajectories up to 120 seconds into the future. It autonomously identifies potential collisions, determines priority right-of-way, issues proportional speed-reduction and signal-holding commands, and automatically resumes held traffic once the conflict zone clears.

This document serves as the **master reference**, detailing how all 13 modules interact in real time, walking through concrete operational demo flows, and analyzing critical edge cases and fail-safe mechanisms.

---

## 2. Complete Module Decomposition

The TCAS architecture is decomposed into 13 modular subsystems across the physical, logical, safety, and application layers:

```text
┌──────────────────────────────────────────────────────────────────────────────────┐
│                            APPLICATION & HMI LAYER                               │
│  Module 13: HmiDisplay, TelemetryLogger, PerformanceMetrics, TcasApplication     │
└────────────────────────────────────────┬─────────────────────────────────────────┘
                                         │
┌────────────────────────────────────────▼─────────────────────────────────────────┐
│                          REAL-TIME ORCHESTRATION LAYER                           │
│  Module 12: ThreadOrchestrator, SafetyPipeline, CommandQueue, WorldState         │
└───────┬────────────────────────────────┬────────────────────────────────┬────────┘
        │ (20ms)                         │ (100ms)                        │ (100ms)
┌───────▼──────────────┐  ┌──────────────▼──────────────┐  ┌──────────────▼────────┐
│    PHYSICS LAYER     │  │        SAFETY LAYER         │  │  COMMUNICATION LAYER  │
│ Module 4:            │  │ Module 8: PredictionEngine  │  │ Module 7:             │
│   KinematicsEngine   │  │ Module 9: ConflictDetector, │  │   CommunicationChannel│
│ Module 3:            │  │   ResourceReservationManager│  │   (V2V/V2I, Latency,  │
│   SimClock, Timer    │  │ Module 10: RiskEngine,      │  │    Packet Loss)       │
│                      │  │   PriorityEngine, Queue,    │  │                       │
│                      │  │   ResolutionEngine          │  │                       │
└───────┬──────────────┘  └──────────────┬──────────────┘  └──────────────┬────────┘
        │                                │                                │
┌───────▼────────────────────────────────▼────────────────────────────────▼────────┐
│                        CORE DOMAIN & INFRASTRUCTURE LAYER                        │
│ Module 1: RailwayNetwork, Node, Track, Junction, Platform, Station               │
│ Module 2: Train, ExpressTrain, PassengerTrain, FreightTrain, TrainManager        │
│ Module 5: RouteNavigator (Dijkstra Shortest Path Routing)                        │
│ Module 6: StateEstimator (1D Kalman Filter), Odometer (Sensor Drift & Noise)     │
│ Module 11: ScenarioManager, RouteCatalog (10 Predefined Routes, 8 Test Scenarios)│
└──────────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. The Multi-Threaded Real-Time Cadence

TCAS runs four asynchronous, concurrent thread loops coordinated by the `ThreadOrchestrator`. Subsystems are decoupled so that computationally heavy predictive mathematics never block high-frequency physical train movement.

```text
================================================================================
Thread               Period    Frequency   Core Tasks
================================================================================
1. Physics Loop      20 ms     50 Hz       - Drains user commands
                                           - Drains safety commands from queue
                                           - Integrates kinematics (x, v, a)
                                           - Advances track boundary transitions
                                           - Updates train states (Braking->Stopped)
                                           - Publishes WorldState snapshot

2. Safety Loop       100 ms    10 Hz       - Snapshots world state
                                           - Runs 7-stage SafetyPipeline:
                                             Prediction -> Conflict -> Risk ->
                                             Priority -> Reservation -> Resolution
                                           - Dispatches SafetyCommands to queue
                                           - Evaluates Auto-Resume for held trains

3. Comms Loop        100 ms    10 Hz       - Steps wireless channel
                                           - Delivers ready in-flight messages
                                           - Computes live delivery rate
                                           - Enforces communication degradation cap

4. HMI Loop          200 ms    5 Hz        - Extracts WorldState snapshot
                                           - Renders ANSI terminal dashboard
                                           - Flushes CSV telemetry to disk
                                           - Updates KPIs (collisions, near misses)
================================================================================
```

### Thread Synchronization Architecture
- **State Read/Write Locking**: A central `mutable std::shared_mutex worldMutex_` protects the live `WorldState`. The HMI and Safety threads acquire shared read locks (`std::shared_lock`), while the Physics thread acquires exclusive write locks (`std::unique_lock`) only during state integration.
- **Lock-Free Command Delivery**: Corrective actions generated by the Safety thread are enqueued onto a concurrent `CommandQueue` and consumed by the Physics thread on its subsequent 20ms tick.
- **Graceful Shutdown**: Managed via `std::condition_variable shutdownCondition_` and atomic boolean flags (`running_`, `paused_`).

---

## 4. End-to-End System Flow: From Sensing to Resumption

Every simulation cycle moves through 10 deterministic steps:

```text
  ┌────────────────────────────────────────────────────────────────────────┐
  │ 1. INFRASTRUCTURE & FLEET DISPATCH                                     │
  │    Train dispatched via RouteCatalog or Dijkstra RouteNavigator.       │
  │    Assigned RouteResult [Track 101 -> 103 -> 107] and dispatch speed.  │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 2. PHYSICAL KINEMATICS (20ms Tick)                                     │
  │    KinematicsEngine updates x(t) and v(t) accounting for track slope.  │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 3. SENSOR SIMULATION & STATE ESTIMATION                                │
  │    Odometer adds 1% wheel slip drift. Kalman Filter filters noise,     │
  │    fuses absolute track balises, and outputs position uncertainty.     │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 4. WIRELESS COMMUNICATION EXCHANGE (100ms)                             │
  │    Trains broadcast telemetry via CommunicationChannel. Latency and    │
  │    range cut-offs (10 km) simulated; messages placed in mailboxes.     │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 5. MULTI-HORIZON TRAJECTORY PROJECTION (Module 8)                      │
  │    PredictionEngine projects future coordinates at 5, 10, 20, 30,      │
  │    60, 90, 120 seconds ahead across track boundaries.                  │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 6. CONFLICT DETECTION & RESOURCE LOCKING (Module 9)                    │
  │    ConflictDetector identifies HeadOn, RearEnd, Junction, or Platform  │
  │    hazards. ResourceReservationManager checks space-time locks.        │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 7. QUANTITATIVE RISK SCORING & PRIORITY ARBITRATION (Module 10)        │
  │    RiskEngine scores hazard (0-100). PriorityEngine grants right-of-   │
  │    way: Express (300) > Passenger (200) > Freight (100).               │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 8. SAFETY COMMAND GENERATION (ResolutionEngine)                        │
  │    Braking feasibility evaluated. Commands issued:                     │
  │    - Priority Train: ReduceSpeed (cautionary) or NoAction              │
  │    - Non-Priority Train: HoldAtSignal or ReduceSpeed                   │
  │    - Imminent / Infeasible Hazard: EmergencyBrake                      │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 9. BRAKE ACTUATION & SPEED FLOOR (Physics Thread)                      │
  │    Non-priority train begins service braking (a = -a_brake).           │
  │    Transitions: Running -> Slowing -> Braking -> Stopped (at v = 0).   │
  │    ReduceSpeed floor (< 3.0 m/s) cleanly escalates to HoldAtSignal.    │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
  ┌───────────────────────────────────▼────────────────────────────────────┐
  │ 10. AUTOMATIC RESUMPTION ENGINE (The Resolution Flow)                  │
  │     Priority train clears conflict zone -> active conflict disappears. │
  │     Safety loop confirms non-priority train is fully Stopped.          │
  │     Speed restored to catalog dispatch speed (BUG-1 / BUG-4 fix).      │
  │     Train sets acceleration = +0.5 m/s^2, returns to Running state!    │
  └────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Concrete Operational Demo Flows

### Demo Flow A: Junction Convergence & Auto-Resumption (The Core Benchmark)

**Setup**: Catalog Route R-08 (Passenger Train 1, $v = 25\text{ m/s}$, departing Central Station toward North Terminal) and Route R-07 (Freight Train 2, $v = 20\text{ m/s}$, departing Freight Approach toward Freight Yard) converge on **Alpha Junction (Node 2)**.

```text
Track 101 (Passenger Train 1: 25 m/s) ──────┐
                                            ▼
                                   [Alpha Junction: Node 2] ──► Track 103
                                            ▲
Track 105 (Freight Train 2: 20 m/s) ────────┘
```

1. **$t = 0.0\text{ s}$ — Initial Dispatch**: Both trains are running at cruising speed. Trajectory projection shows both trains will reach Alpha Junction simultaneously at approximately $t = 25.0\text{ s}$.
2. **$t = 5.0\text{ s}$ — Conflict Detection**: `ConflictDetector` detects `ConflictType::Junction` at Node 2 with arrival time difference $|\Delta t| < 3.0\text{ s}$.
3. **$t = 5.1\text{ s}$ — Risk & Priority Arbitration**:
   - `RiskEngine` calculates risk score $S = 64.5$ (`RiskLevel::High`).
   - `PriorityEngine` evaluates priority: Passenger Train 1 (weight 200) vs Freight Train 2 (weight 100). Train 1 is granted priority; Train 2 must yield.
4. **$t = 5.2\text{ s}$ — Safety Command Execution**:
   - Train 1 receives `ReduceSpeed` (cautionary reduction through junction).
   - Train 2 receives `HoldAtSignal` (target speed $0.0\text{ m/s}$).
   - `ResourceReservationManager` reserves Alpha Junction for Train 1 for time window $[22.0\text{ s}, 28.0\text{ s}]$.
5. **$t = 5.3\text{ s} \to 18.0\text{ s}$ — Braking & Stopping**:
   - Train 2 enters `TrainState::Braking` and applies service brake ($a = -0.5\text{ m/s}^2$).
   - Train 2 speed decelerates from $20\text{ m/s}$ down to $0.0\text{ m/s}$.
   - Upon reaching $v \le 0.1\text{ m/s}$, the Physics thread sets $v = 0.0, a = 0.0$ and transitions Train 2 to `TrainState::Stopped`.
   - Train 2 sits safely held behind Alpha Junction.
6. **$t = 24.0\text{ s} \to 28.0\text{ s}$ — Junction Traversal**:
   - Train 1 crosses through Alpha Junction without obstruction.
   - At $t = 28.0\text{ s}$, Train 1 exits the junction clearance zone ($>25\text{ m}$ onto Track 103).
7. **$t = 28.1\text{ s}$ — Auto-Resumption**:
   - Train 1's junction reservation expires and is cleared by `clearExpired()`.
   - `ConflictDetector` scans trajectories: no conflicts remain!
   - The Safety loop evaluates `trainsHeldBySafety_`: Train 2 has no active conflicts and is in `TrainState::Stopped`.
   - **Auto-Resume triggered!** Train 2's speed limit is restored to its catalog dispatch speed ($20.0\text{ m/s}$).
   - Train 2 state transitions to `TrainState::Running` with service acceleration $a = +0.5\text{ m/s}^2$.
8. **$t = 35.0\text{ s}$**: Both trains have successfully traversed their routes. Zero near-misses, zero emergency stops, zero human interventions!

---

### Demo Flow B: Head-On Conflict on Single Line

**Setup**: Catalog Route R-03 (Express Train 1, Eastbound on Track 101 at $30\text{ m/s}$) and Route R-04 (Passenger Train 2, Westbound opposing run on single-track corridor at $30\text{ m/s}$).

1. **$t = 0.0\text{ s}$**: Trains approach each other head-on with a closing velocity of $60\text{ m/s}$ ($216\text{ km/h}$).
2. **$t = 0.1\text{ s}$**: `ConflictDetector` detects `ConflictType::HeadOn`.
3. **$t = 0.1\text{ s}$**: `RiskEngine` calculates risk score $S = 88.5$ (`RiskLevel::Critical`) due to head-on category weight ($+15$), high relative velocity ($+20$), and rapidly collapsing TTC.
4. **$t = 0.2\text{ s}$**: `ResolutionEngine` enforces **Rule 1 (Absolute Safety Precedence)**: both trains receive `EmergencyBrake`.
5. **$t = 0.22\text{ s}$**: Physics thread applies maximum emergency braking deceleration ($a = -1.2\text{ m/s}^2$ for Express, $a = -1.0\text{ m/s}^2$ for Passenger).
6. **$t = 0.25\text{ s}$**: Both trains are placed in `emergencyBrakeSet_`. Trains come to a full stop with $>150\text{ m}$ of empty buffer space between them.
7. **Post-Stop State**: Emergency brake is **STICKY**. The system refuses to auto-resume either train until an operator enters the interactive menu, investigates the opposing dispatch error, and issues a manual override command.

---

### Demo Flow C: High-Speed Rear-End Overtaking

**Setup**: Catalog Route R-05 (Heavy Freight Train 1, $v = 15\text{ m/s}$) dispatched ahead on Track 101. Catalog Route R-06 (Express Train 2, $v = 35\text{ m/s}$) dispatched behind it on the same track.

1. **Closing Rate**: Express closes in on Freight at $\Delta v = 20\text{ m/s}$ ($72\text{ km/h}$).
2. **Prediction Horizon**: `PredictionEngine` projects Express catching Freight within $30\text{ s}$.
3. **Classification**: `ConflictDetector` classifies as `ConflictType::RearEnd`.
4. **Resolution**: Risk is assessed as `Medium` ($S = 48.0$). `ResolutionEngine` issues `ReduceSpeed` to the trailing Express Train, halving its target speed to $17.5\text{ m/s}$.
5. **Headway Stabilization**: Express smoothly decelerates using service braking to $17.5\text{ m/s}$, matching the forward train's speed profile and stabilizing safe separation headway without stopping passenger service.

---

### Demo Flow D: Wireless Radio Blackout & Fail-Safe Speed Cap

**Setup**: Operator injects communication failure (`commFault = true`) simulating antenna mast damage.

1. **Channel Drop**: Packet delivery rate drops to $0\%$.
2. **Hysteresis Trigger**: Comms thread flags `commChannelDegraded_ = true`. System status transitions from `RUNNING` to `DEGRADED`.
3. **Fail-Safe Speed Cap**: All active trains on the network automatically have their maximum permitted speed clamped to $10.0\text{ m/s}$ ($36\text{ km/h}$ sight-distance running speed).
4. **Fault Recovery**: When the fault is cleared and packet delivery rate stably exceeds $85\%$, the speed cap is removed and trains return to cruising speed.

---

## 6. Comprehensive Edge Cases & Fail-Safe Mechanisms

| # | Edge Case / Hazard | Root Cause | Architectural Mitigation & Resolution |
|---|---|---|---|
| **1** | **Unsafe Stopping** (Overspeed approaching red signal) | Train travelling too fast for available remaining track length. | `ResolutionEngine::calculateBrakingFeasibility` compares $d_{\text{avail}}$ against $d_{\text{req}} + d_{\text{margin}}$. If service braking is infeasible, escalates instantly to `EmergencyBrake`. |
| **2** | **Asymptotic Speed Creep** (BUG-2 Fix) | Repeatedly halving speed ($v / 2$) causes a train to crawl at $0.05\text{ m/s}$ indefinitely without stopping. | Enforced `kMinReduceSpeedFloor = 3.0 m/s`. If halved speed falls below $3.0\text{ m/s}$, the command automatically escalates to `HoldAtSignal` ($0.0\text{ m/s}$), ensuring a crisp, full stop. |
| **3** | **Braking Resume Race Condition** (BUG-4 Fix) | Safety loop resumes a held train while it is still decelerating (`state == Braking`). Clearing limits causes premature throttle surge. | Safety loop enforces `state == Stopped` check before allowing auto-resume. Train must have physically halted before release. |
| **4** | **Dispatch Speed Obliteration** (BUG-1 Fix) | Resumed train was hardcoded to $15\text{ m/s}$, forgetting its original catalog speed ($35\text{ m/s}$ Express or $12\text{ m/s}$ Freight). | Implemented `dispatchSpeeds_` tracking map in `ThreadOrchestrator`. Auto-resume restores the train's specific scheduled catalog velocity. |
| **5** | **Stationary Train Deadlock** (LOGIC-2 Fix) | Two stopped trains at a junction see each other as static obstacles, refusing to ever move. | `ThreadOrchestrator` deadlock breaker detects when both trains are stationary at a junction and grants clearance to the priority train. |
| **6** | **Downhill Runaway Guard** | Steep track downgrade ($-5\%$) creates gravitational acceleration opposing train brakes. | `KinematicsEngine::effectiveDeceleration` enforces `kMinDeceleration = 0.01 m/s^2` floor, preventing zero or negative braking calculations. |
| **7** | **Sensor Noise & Balise Outliers** | Mechanical wheel slip causes odometry drift; electrical spikes cause measurement anomalies. | 1D Kalman Filter performs $3.5\sigma$ innovation gating to reject spikes; track balises collapse variance and reset accumulated drift to zero. |
| **8** | **Sticky Emergency Brake** (LOGIC-4 Fix) | An emergency-braked train auto-resuming once an obstacle moves out of range. | `emergencyBrakeSet_` creates a sticky safety interlock: emergency-braked trains can **only** be resumed by explicit human operator override. |
| **9** | **Route Boundary Clamp** | Train reaching the physical terminus of its route. | Trajectory projection and kinematics clamp position to the end of the final track with $v = 0$, transitioning state to `TrainState::Completed`. |
| **10** | **Non-Finite Value Injection** | Corrupt telemetry sending NaN or infinite values. | Math validation wrappers clamp non-finite risk scores to $100.0$ (`Critical`), forcing fail-safe shutdown. |

---

## 7. Master Summary in Layman's Terms

If you explain TCAS to a passenger sitting on the train, here is what is happening under their feet:

> *"Imagine you are riding an ultra-modern train traveling at 160 kilometers per hour. Up in the sky, an invisible supercomputer is watching your train and every other train on the network.*
>
> *Every single second, this computer gazes two minutes into the future. It draws an invisible digital movie of where your train will be, taking into account how heavy the train is, whether you are climbing a mountain or rolling downhill, and how fast your brakes can stop.*
>
> *Around a curve, five kilometers ahead, another train is approaching the same track switch. Before either train driver can even see each other, the computer spots the conflict. It calculates that your train is an Express passenger service with hundreds of commuters, while the other train is a slow freight train carrying coal.*
>
> *The computer immediately turns the signal red for the freight train, which smoothly glides to a stop and waits patiently behind the switch. Your train receives a cautionary advisory to ease off the throttle slightly. You zoom smoothly through the crossing with complete peace of mind.*
>
> *The very instant the rear car of your train clears the switch, the computer turns the signal back to green for the freight train, which automatically accelerates back up to speed and resumes its delivery.*
>
> *And if anything unexpected happens—like a sudden obstacle or a communications failure—the system fail-safes immediately, locking the emergency brakes to keep every passenger safe."*
