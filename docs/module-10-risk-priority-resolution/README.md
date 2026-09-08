# Module 10 — Risk Assessment, Priority Arbitration & Predictive Resolution

## 1. Module Overview

Module 10 is the core safety arbitration and collision resolution brain of the Train Collision Avoidance System (TCAS).

When Module 9 detects that two or more trains are on a collision course, Module 10 steps in to answer three fundamental questions:
1. **How severe is the danger?** (`RiskEngine`) Calculates a multi-factor risk score ($0.0$ to $100.0$) and categorizes it into `Low`, `Medium`, `High`, or `Critical`.
2. **Which train has the operational right-of-way?** (`PriorityEngine`) Arbitrates right-of-way between conflicting trains based on service classification (Express vs Passenger vs Freight), timetable adherence, and distance to the conflict zone.
3. **What corrective command must be issued to prevent disaster?** (`ResolutionEngine`) Issues safety commands (`NoAction`, `ReduceSpeed`, `HoldAtSignal`, `EmergencyBrake`) while enforcing the absolute rule: **Safety overrides priority at all times.**

---

## 2. Objectives

1. **Multi-Factor Quantitative Risk Assessment**: Compute a unified risk score ($0-100$) combining Time-To-Collision (TTC), relative speed, braking feasibility, collision type, train tonnage, and sensor/communication confidence.
2. **Deterministic Priority Arbitration**: Fairly and deterministically allocate right-of-way to the higher-priority train while providing robust tie-breaking for equal-class trains.
3. **Multi-Conflict Priority Ordering**: Maintain a priority queue that sorts simultaneous conflicts so that the highest-risk threats are resolved first.
4. **Feasibility-Guarded Braking Decisions**: Calculate whether service braking is physically capable of stopping the train before the danger zone; if infeasible, escalate immediately to emergency braking.
5. **Speed Reduction Floor & Escalation**: Ensure speed reduction commands do not lead to an asymptotic crawl below $3.0\text{ m/s}$, cleanly transitioning to a signal stop.

---

## 3. Architectural Role & System Seams

```text
               Detected Conflicts (Module 9)
                             │
                             ▼
      ┌──────────────────────────────────────────────┐
      │                  RiskEngine                  │
      │  (TTC, Relative Vel, Braking, Mass, Health)  │
      └──────────────────────┬───────────────────────┘
                             │
              RiskAssessment (Score & Level)
                             │
              ┌──────────────┴──────────────┐
              ▼                             ▼
       PriorityEngine            ConflictPriorityQueue
   (Service class, Timetable)     (Sorts conflicts by risk)
              │                             │
              └──────────────┬──────────────┘
                             │
                             ▼
                     ResolutionEngine
       ├── Safety Precedence Check (Critical -> EmergencyBrake)
       ├── Braking Feasibility Check (Available vs Required)
       └── SafetyCommand Generation
              ├── NoAction
              ├── ReduceSpeed (Halve speed, min 3.0 m/s floor)
              ├── HoldAtSignal (Target speed 0 m/s)
              └── EmergencyBrake (Immediate maximum brake)
                             │
                             ▼
         [ThreadOrchestrator / SafetyPipeline]
```

---

## 4. Implementation Details

### 4.1 Risk Assessment Formulation (`RiskEngine`)
The risk score $S \in [0, 100]$ is computed as the sum of 7 physical and operational components:
$$S = \text{clamp}\left(R_{\text{TTC}} + R_{\text{rel\_vel}} + R_{\text{braking}} + R_{\text{type}} + R_{\text{mass}} + R_{\text{sensor}} + R_{\text{comm}}, \; 0.0, \; 100.0\right)$$

1. **Time-To-Collision ($R_{\text{TTC}}$)**:
   - $\text{TTC} \le 5\text{ s} \implies 35$ points
   - $\text{TTC} \le 15\text{ s} \implies 25$ points
   - $\text{TTC} \le 30\text{ s} \implies 15$ points
   - $\text{TTC} > 30\text{ s} \implies 5$ points
2. **Relative Velocity ($R_{\text{rel\_vel}}$)**: Up to $20$ points proportional to convergence speed.
3. **Braking Distance Deficit ($R_{\text{braking}}$)**: Up to $20$ points if available distance is less than required stopping distance + safety margin.
4. **Conflict Type ($R_{\text{type}}$)**:
   - `HeadOn`: $15$ points (maximum catastrophic potential)
   - `RearEnd`: $10$ points
   - `Junction`: $8$ points
   - `Platform`: $5$ points
5. **Mass / Momentum ($R_{\text{mass}}$)**: Up to $5$ points (heavy freight requires higher caution).
6. **Sensor Health ($R_{\text{sensor}}$)**: Up to $2.5$ points if sensor degraded or high uncertainty.
7. **Communication Health ($R_{\text{comm}}$)**: Up to $2.5$ points if packet loss is detected.

#### Risk Classification Thresholds
- **$0.0 \le S \le 30.0$**: `RiskLevel::Low`
- **$30.0 < S \le 60.0$**: `RiskLevel::Medium`
- **$60.0 < S \le 80.0$**: `RiskLevel::High`
- **$S > 80.0$**: `RiskLevel::Critical`

### 4.2 Priority Arbitration (`PriorityEngine`)
Determines which train should proceed and which train should yield:
- **Base Service Class Weight**:
  - `Express Train`: Weight = $300$ (high-speed passenger express)
  - `Passenger Train`: Weight = $200$ (standard commuter service)
  - `Freight Train`: Weight = $100$ (cargo / logistics)
- **Kinematic Distance Modifier**: Trains closer to the junction boundary receive incremental priority to prevent halting a train that is already entering the interlocking.
- **Deterministic Tiebreaker**: If two trains have identical scores, a deterministic tiebreaker (e.g. scheduled arrival time or lower Train ID) resolves the tie without race conditions.

### 4.3 Resolution Strategy (`ResolutionEngine`)
The engine enforces hierarchical safety rules:

1. **Rule 1: Absolute Safety Precedence**
   If risk is `Critical` OR `!brakingFeasible`, **both** trains receive `EmergencyBrake`. No operational priority can override an imminent crash.
2. **Rule 2: Feasibility Gate**
   $$\text{Feasible} \iff d_{\text{available}} \ge d_{\text{required\_braking}} + \max(0.0, d_{\text{safety\_margin}})$$
   If the train physically cannot stop in time under normal service braking, it immediately triggers `EmergencyBrake`.
3. **Rule 3: Non-Priority Train Actions**
   - `RiskLevel::High` $\implies$ `HoldAtSignal` (target speed = $0.0\text{ m/s}$).
   - `RiskLevel::Medium` $\implies$ `ReduceSpeed` (halves current speed).
   - `RiskLevel::Low` $\implies$ `NoAction`.
4. **Rule 4: Speed Floor & Escalation (BUG-2 Fix)**
   When issuing `ReduceSpeed`, if halving the train's speed produces a target speed below $3.0\text{ m/s}$ ($10.8\text{ km/h}$), the command automatically escalates to `HoldAtSignal` ($0.0\text{ m/s}$). This prevents trains from asymptotically crawling indefinitely toward a red signal.
5. **Rule 5: Priority Train Cautionary Action**
   Even when granted priority, if risk is `High` or `Medium`, the priority train receives `ReduceSpeed` to exercise caution while traversing the conflict zone.

---

## 5. Public API Reference

```cpp
namespace tcas::safety {

enum class RiskLevel { Low, Medium, High, Critical };

struct RiskAssessment {
    double score{ 0.0 };
    RiskLevel level{ RiskLevel::Low };
    TimeSeconds timeToCollision{ 0.0 };
    DistanceMeters brakingDistance{ 0.0 };
    DistanceMeters safetyMargin{ 0.0 };
    [[nodiscard]] bool isActionRequired() const noexcept;
    [[nodiscard]] bool isCritical() const noexcept;
};

struct SafetyCommand {
    SafetyCommandType type{ SafetyCommandType::NoAction };
    TrainId trainId{ 0 };
    SpeedMetersPerSecond targetSpeed{ 0.0 };
    TimeSeconds issuedAt{ 0.0 };
    double riskScore{ 0.0 };
    [[nodiscard]] bool isEmergency() const noexcept;
};

class RiskEngine {
public:
    [[nodiscard]] RiskAssessment assess(const RiskInput& input) const noexcept;
    [[nodiscard]] static RiskLevel classify(double score) noexcept;
};

class PriorityEngine {
public:
    [[nodiscard]] PriorityDecision arbitrate(
        const train::Train& trainA,
        const train::Train& trainB,
        const conflict::Conflict& conflict
    ) const noexcept;
};

class ResolutionEngine {
public:
    [[nodiscard]] SafetyCommand resolve(const ResolutionInput& input) const noexcept;
};

} // namespace tcas::safety
```

---

## 6. Execution Flow

```text
Active Conflict detected between Train 1 (Express) and Train 2 (Freight)
                                │
                                ▼
         1. RiskEngine::assess(Train 1) & RiskEngine::assess(Train 2)
            Compute multi-factor score S -> determine RiskLevel
                                │
                                ▼
         2. PriorityEngine::arbitrate(Train 1, Train 2, conflict)
            Express (weight 300) > Freight (weight 100)
            Train 1: priorityGranted = true
            Train 2: priorityGranted = false
                                │
                                ▼
         3. ResolutionEngine::resolve(Input for each train)
            Train 2 (Non-Priority, Risk High):
              -> Feasibility OK? YES.
              -> Issue SafetyCommand: HoldAtSignal (speed = 0.0 m/s)
            Train 1 (Priority, Risk High):
              -> Issue SafetyCommand: ReduceSpeed (cautionary transit)
                                │
                                ▼
         4. Commands dispatched to Train Actuators / Orchestrator
```

---

## 7. Edge Cases Handled

1. **Unsafe Stopping (Insufficient Distance)**: If a speeding train is too close to a junction to stop under service braking, `calculateBrakingFeasibility` returns `false`, forcing an immediate `EmergencyBrake`.
2. **Asymptotic Crawl Prevention**: Halving velocity repeatedly could make a train creep at $0.05\text{ m/s}$ forever. The $3.0\text{ m/s}$ floor ensures a clean stop.
3. **Deadlock in Identical Train Classes**: Two passenger trains reaching a junction at the identical second are resolved by the deterministic tiebreaker rather than freezing the system.
4. **NaN and Infinite Value Protection**: If sensor calculations produce non-finite values, risk score clamps to $100.0$ (`Critical`), triggering fail-safe emergency stopping.

---

## 8. Automated Test Verification

Validated through 5 test suites under `tests/safety/`:

- **`RiskEngineTest.cpp`**: Tests TTC scales, relative speed risk, braking deficit risk, risk classification boundaries ($30, 60, 80$), and sensor degradation weighting.
- **`PriorityEngineTest.cpp`**: Tests Express vs Passenger vs Freight hierarchy, distance weighting, and tiebreaking.
- **`ConflictPriorityQueueTest.cpp`**: Tests priority queue ordering of multiple concurrent hazards.
- **`ResolutionEngineTest.cpp`**: Tests braking feasibility gating, priority/non-priority command mapping, speed floor escalation, and emergency brake overrides.
- **`SafetyIntegrationTest.cpp`**: Full end-to-end integration test from detected conflict through risk, priority, and resolution execution.

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Imagine a traffic cop standing at an intersection when two cars are speeding toward the same corner:

The traffic cop has to make split-second decisions:
1. **The Danger Meter (`RiskEngine`)**: How dangerous is this? If they are 2 miles apart traveling slowly, the risk is low. If an 80-ton freight train is 100 metres away from a passenger train, the risk meter hits 100% (Red Alert!).
2. **Who Gets Right-of-Way? (`PriorityEngine`)**: If an ambulance with sirens on meets a regular delivery truck, the ambulance goes first. On the railway, high-speed passenger Express trains get right-of-way over cargo freight trains.
3. **The Commands (`ResolutionEngine`)**:
   - The delivery truck (Freight Train) is ordered: *"Stop at the red signal and wait your turn!"* (`HoldAtSignal`)
   - The ambulance (Express Train) is told: *"You have right-of-way, but slow down a little through the crossing just to be safe!"* (`ReduceSpeed`)
   - If anyone is going so fast that their regular brakes cannot stop them in time, the cop hits the giant red panic button: *"SLAM ON THE EMERGENCY BRAKES RIGHT NOW!"* (`EmergencyBrake`)

Safety always comes first: no train, no matter how VIP, is allowed to run into danger!
