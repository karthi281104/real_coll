# Module 10 — Risk Assessment, Priority Engine & Resolution

## 1. Architectural Role

Module 10 is the central decision-making executive of the TCAS safety pipeline. Once Module 9 identifies conflicts, Module 10 assesses the risk of each conflict, determines train precedence, sorts conflicts by urgency, and issues executable safety commands.

```
                  Active Conflicts (Module 9)
                              │
                              ▼
        ┌───────────────────────────────────────────┐
        │            RiskEngine (Stage 1)           │
        │ Calculates composite risk score (0 - 100) │
        │ Low / Medium / High / Critical            │
        └─────────────────────┬─────────────────────┘
                              │
                              ▼
        ┌───────────────────────────────────────────┐
        │       PriorityEngine & Queue (Stage 2)    │
        │ Express > Passenger > Freight             │
        │ Max-heap sorted by Risk Score & TTC       │
        └─────────────────────┬─────────────────────┘
                              │
                              ▼
        ┌───────────────────────────────────────────┐
        │         ResolutionEngine (Stage 3)        │
        │ Braking Feasibility & Command Generation  │
        │ ├── EmergencyBrake (Immediate halt)       │
        │ ├── HoldAtSignal   (Stop before node)     │
        │ ├── ReduceSpeed    (Target speed 50%)     │
        │ └── NoAction       (Safe operation)       │
        └─────────────────────┬─────────────────────┘
                              │
                              ▼
                      SafetyCommandQueue
```

---

## 2. Stage 1: Risk Assessment Engine

Defined in [`include/safety/RiskEngine.hpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/include/safety/RiskEngine.hpp).

The Risk Engine scores hazards on an absolute scale from $0.0$ to $100.0$ based on 7 physical and operational variables:
$$\text{Score} = S_{TTC} + S_{\Delta v} + S_{braking} + S_{type} + S_{mass} + S_{sensor} + S_{comm}$$

### Risk Factor Weighting

1. **Time-To-Collision ($S_{TTC}$)**:
   - $TTC \le 2.0\text{ s} \implies 35\text{ pts}$
   - $TTC \le 5.0\text{ s} \implies 30\text{ pts}$
   - $TTC \le 10.0\text{ s} \implies 20\text{ pts}$
   - $TTC \le 20.0\text{ s} \implies 10\text{ pts}$
   - $TTC \le 30.0\text{ s} \implies 5\text{ pts}$
2. **Relative Velocity ($S_{\Delta v}$)**: Up to $20\text{ pts}$ based on closing rate $\Delta v$.
3. **Braking Distance Margin ($S_{braking}$)**: Up to $20\text{ pts}$ if available distance is less than the required stopping distance plus safety margin.
4. **Conflict Type Severity ($S_{type}$)**:
   - Head-On: $40\text{ pts}$ (catastrophic hazard)
   - Junction: $25\text{ pts}$ (derailment / side-impact hazard)
   - Platform: $20\text{ pts}$ (station impact hazard)
   - Rear-End: $20\text{ pts}$ (following impact hazard)
5. **Train Mass Inertia ($S_{mass}$)**: Heavy freight trains carry high kinetic energy ($2 - 5\text{ pts}$).
6. **Sensor Confidence ($S_{sensor}$)**: Odometer drift or sensor failure adds $10\text{ pts}$.
7. **Communication Confidence ($S_{comm}$)**: Packet loss or comm blackout adds $10\text{ pts}$.

### Classification Thresholds
- **Low**: $0.0 \le \text{Score} \le 30.0$ (Normal monitoring)
- **Medium**: $30.0 < \text{Score} \le 60.0$ (Speed reduction advised)
- **High**: $60.0 < \text{Score} \le 80.0$ (Signal hold / proactive stop)
- **Critical**: $\text{Score} > 80.0$ (Emergency braking mandatory)

---

## 3. Stage 2: Operational Priority Engine

Defined in [`include/safety/PriorityEngine.hpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/include/safety/PriorityEngine.hpp).

When two trains encounter a converging route or junction, operational rules dictate which train yields:

$$\text{Express Train (Class 3)} > \text{Passenger Train (Class 2)} > \text{Freight Train (Class 1)}$$

### Tie-Breaking Rules
If both trains possess the same operational tier:
1. The train traveling at the higher velocity takes precedence.
2. If velocities are identical, deterministic tie-breaking is enforced by comparing `TrainId` ($ID_A < ID_B$).

---

## 4. Stage 3: Resolution Engine & Safety Precedence

Defined in [`include/safety/ResolutionEngine.hpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/include/safety/ResolutionEngine.hpp).

### Core Architectural Axiom
> **"Safety has absolute precedence over operational priority. A priority train will NOT be allowed to proceed if stopping distances are physically infeasible or critical risk is detected."**

### Command Resolution Logic

```cpp
if (input.risk.isCritical() || !input.brakingFeasible || !calculateBrakingFeasibility(input))
{
    return emergencyBrake(input);
}

if (!input.priorityGranted) // Yielding Train
{
    switch (input.risk.level)
    {
    case RiskLevel::Critical: return emergencyBrake(input);
    case RiskLevel::High:     return holdAtSignal(input);
    case RiskLevel::Medium:   return reduceSpeed(input);  // Speed reduced to 50%
    case RiskLevel::Low:      return noAction(input);
    }
}
else // Priority Train
{
    switch (input.risk.level)
    {
    case RiskLevel::Critical: return emergencyBrake(input);
    case RiskLevel::High:     return reduceSpeed(input);
    case RiskLevel::Medium:   return reduceSpeed(input);
    case RiskLevel::Low:      return noAction(input);
    }
}
```

### Safety Command Types

| Command Type | Target Velocity | Train State | Physical Action |
|---|---|---|---|
| `NoAction` | Unchanged | Unchanged | Maintain speed; no safety intervention required. |
| `ReduceSpeed` | $\frac{1}{2} v_{current}$ | `TrainState::Braking` | Smooth deceleration under service braking limit. |
| `HoldAtSignal` | $0.0\text{ m/s}$ | `TrainState::Braking` | Controlled service stop prior to junction entrance. |
| `EmergencyBrake` | $0.0\text{ m/s}$ | `TrainState::EmergencyBrake` | Maximum deceleration ($1.4\text{ m/s}^2$); override all limits. |

---

## 5. Verification & Testing

Implemented in:
- [`tests/safety/RiskEngineTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/safety/RiskEngineTest.cpp)
- [`tests/safety/PriorityEngineTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/safety/PriorityEngineTest.cpp)
- [`tests/safety/ConflictPriorityQueueTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/safety/ConflictPriorityQueueTest.cpp)
- [`tests/safety/ResolutionEngineTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/safety/ResolutionEngineTest.cpp)
- [`tests/safety/SafetyIntegrationTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/safety/SafetyIntegrationTest.cpp)
