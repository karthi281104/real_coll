# Module 11 — Operational Scenarios & Simulation Testing

## 1. Architectural Role

Module 11 provides the standardized benchmarking and live demonstration suite for the TCAS project. Managed by the `ScenarioManager`, it initializes reproducible network topologies, provisions train types, injects initial kinematic states, assigns routes, and induces specific hazardous edge conditions.

These scenarios serve dual purposes:
1. **Automated Regression Testing**: Rigorously validated via Google Test integration suites to verify that the safety pipeline produces the mathematically correct safety interventions.
2. **Interactive Live Demonstration**: Executable directly from the TCAS Control Center HMI for live evaluation by railway safety operators.

---

## 2. Base Network Topology

Constructed in [`src/scenario/ScenarioManager.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/src/scenario/ScenarioManager.cpp):

```text
 [1] Central Station ──T101 (2000m, 35m/s)──> [2] Alpha Jct ──T102 (1500m)──> [3] Beta Jct ──T103 (2500m)──> [4] North Term
                                                     │                                │
                                                    T104 (3000m)                     T108 (600m)
                                                     │                                │
                                                     ▼                                ▼
                                               [5] South Harbor ──T107 (500m)──> [8] Platform A
                                                     ▲
                                                    T105 (2000m)
                                                     │
                                               [6] Freight Approach ──T106 (1800m)──> [7] Freight Yard
```

---

## 3. The 8 Standardized Operational Scenarios

### Scenario 1: Junction Conflict
- **Configuration**:
  - Express Train #1 on Track 101 ($20\text{ m/s}$) approaching Alpha Junction (Node 2).
  - Freight Train #3 on Track 105 ($15\text{ m/s}$) approaching Alpha Junction (Node 2).
- **Hazard**: Both trains converge on Node 2 simultaneously ($TTC \approx 10\text{ s}$).
- **Expected Outcome**: Express granted priority lease on Node 2. Freight commanded to `REDUCE SPEED` ($7.5\text{ m/s}$), delaying its arrival by $>16\text{ seconds}$ and safely averting the collision.

### Scenario 2: Rear-End Conflict
- **Configuration**:
  - Passenger Train #1 on Track 101 at $1200\text{ m}$ ($10\text{ m/s}$, slow).
  - Express Train #2 on Track 101 at $600\text{ m}$ ($25\text{ m/s}$, fast).
- **Hazard**: Express closing from behind at relative speed of $15\text{ m/s}$.
- **Expected Outcome**: Module 9 analytical quadratic solver detects spatial cushion violation within lookahead window. Express commanded to `REDUCE SPEED` / `BRAKE`.

### Scenario 3: Critical Closing Conflict
- **Configuration**:
  - Express Train #1 at $100\text{ m}$ traveling at $35\text{ m/s}$.
  - Freight Train #3 at $700\text{ m}$ traveling at $5\text{ m/s}$.
- **Hazard**: Extreme relative closing rate of $30\text{ m/s}$ ($108\text{ km/h}$) on a shared block.
- **Expected Outcome**: Time-to-collision drops below physical stopping threshold. Module 10 classifies risk as **Critical**, issuing immediate `EMERGENCY BRAKE`.

### Scenario 4: Platform Conflict
- **Configuration**:
  - Express Train #1 traveling via Track 102 & Track 108 into Platform A (Node 8).
  - Passenger Train #2 traveling via Track 104 & Track 107 into Platform A (Node 8).
- **Hazard**: Simultaneous occupancy of station platform track.
- **Expected Outcome**: Station resource reservation granted to Train #1; Train #2 commanded to `HOLD AT SIGNAL` until the platform is cleared.

### Scenario 5: Multiple Simultaneous Conflicts
- **Configuration**:
  - Express Train #1 on Track 101.
  - Passenger Train #2 on Track 101 (Rear-End conflict with Train #1).
  - Freight Train #3 on Track 105 (Junction conflict at Node 2).
- **Hazard**: Compound multi-train hazard requiring simultaneous prioritization and multi-resource management.
- **Expected Outcome**: Max-heap `ConflictPriorityQueue` sorts conflicts by urgency. Express maintains priority; Passenger and Freight receive coordinated deceleration commands.

### Scenario 6: Sensor Failure Degradation
- **Configuration**: Standard junction approach with simulated odometer failure.
- **Fault Injection**: Train #1's position measurement uncertainty expands from $1.0\text{ m}$ to $15.0\text{ m}$.
- **Expected Outcome**: System transitions to **DEGRADED** status. Safety pipeline widens protected dynamic boundaries. Proactive speed reduction triggered earlier to compensate for measurement noise.

### Scenario 7: Communication Failure
- **Configuration**: Standard operating conditions with simulated wireless channel blackout ($70\%$ packet loss).
- **Fault Injection**: V2V telemetry loss.
- **Expected Outcome**: WorldState flags `COMMUNICATION : DEGRADED`. Risk Engine increases risk score by $+10\text{ pts}$ due to reduced communication confidence, enforcing conservative safety margins.

### Scenario 8: Unsafe Stopping Distance
- **Configuration**: Train placed with insufficient stopping distance to an occupied node.
- **Hazard**: Available track length is less than the calculated emergency stopping distance ($d_{avail} < d_{brake} + d_{margin}$).
- **Expected Outcome**: `calculateBrakingFeasibility()` returns `false`. Operational priority is completely overridden, and immediate `EMERGENCY BRAKE` is commanded.

---

## 4. ScenarioManager API

Defined in [`include/scenario/ScenarioManager.hpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/include/scenario/ScenarioManager.hpp):

```cpp
enum class ScenarioType {
    JunctionConflict,
    RearEndConflict,
    HeadOnConflict,
    PlatformConflict,
    MultipleConflicts,
    SensorFailure,
    CommunicationFailure,
    UnsafeStoppingDistance
};

class ScenarioManager {
public:
    ScenarioManager(infrastructure::RailwayNetwork& network, train::TrainManager& trainManager);
    ScenarioResult load(ScenarioType type);
    // Individual loaders: loadJunctionConflict(), loadRearEndConflict(), etc.
};
```

---

## 5. Verification & Testing

Implemented in:
- [`tests/integration/Module8To12IntegrationTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/integration/Module8To12IntegrationTest.cpp)
- Verified automated execution of all 8 scenarios without crash or memory corruption.
