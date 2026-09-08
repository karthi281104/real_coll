# Module 9 — Conflict Detection & Resource Reservation

## 1. Module Overview

Module 9 is the predictive conflict detection and spatial-temporal resource reservation engine of the Train Collision Avoidance System (TCAS).

While Module 8 projects where trains will be at future timestamps, Module 9 analyzes those multi-train trajectories to identify impending collisions, dangerous convergences, and headway violations before they can materialize in the physical world.

The module detects four distinct classes of operational railway conflicts:
1. **Head-On Conflicts**: Two trains traveling toward each other on the same track section.
2. **Rear-End Conflicts**: A faster train closing in on a slower leading train on the same track.
3. **Junction Conflicts**: Two trains predicted to arrive simultaneously at a shared switch or crossover junction.
4. **Platform Conflicts**: Two trains scheduled or routed to occupy the same station platform.

Additionally, Module 9 implements a dynamic **Resource Reservation Manager** that enforces space-time interlocking, allowing the safety system to lock critical track segments and junctions for one train while denying access to others.

---

## 2. Objectives

1. **Multi-Trajectory Spatial-Temporal Correlation**: Compare predicted trajectories of all train pairs across common tracks and intersection points.
2. **Continuous Interpolation**: Interpolate between discrete prediction horizon samples to accurately pinpoint minimum separation distance and precise conflict start/end times.
3. **Uncertainty-Aware Safety Margins**: Dynamically expand separation thresholds by factoring in sensor covariance ($\sigma_A + \sigma_B$).
4. **Four-Way Conflict Classification**: Explicitly classify detected hazards into `HeadOn`, `RearEnd`, `Junction`, or `Platform` types.
5. **Space-Time Resource Reservation**: Provide locking, validation, and auto-expiration of critical railway resources over temporal intervals $[t_{\text{start}}, t_{\text{end}}]$.

---

## 3. Architectural Role & System Seams

```text
       std::vector<FutureState> Trajectories (Module 8)
                             │
                             ▼
              ┌─────────────────────────────┐
              │      ConflictDetector       │
              └──────────────┬──────────────┘
                             │
       ┌─────────────────────┴─────────────────────┐
       ▼                                           ▼
std::vector<Conflict>                     ResourceReservationManager
├── Type (HeadOn, RearEnd, ...)           ├── request(train, zone, t0, t1)
├── Train A & Train B                     ├── release(train, zone)
├── TrackId / NodeId                      ├── isReserved(zone, t0, t1)
├── firstConflictTime, lastConflictTime   └── clearExpired(currentTime)
└── minimumSeparation                               │
       │                                           │
       └─────────────────────┬─────────────────────┘
                             │
                             ▼
            [Module 10: Risk & Resolution Engine]
```

---

## 4. Implementation Details

### 4.1 Same-Track Conflict Detection (Head-On vs Rear-End)
When two trains occupy the same track segment during overlapping time windows:
1. **Linear Time Interpolation**: For any intermediate time $t \in [t_0, t_1]$:
   $$x(t) = x_0 + \frac{t - t_0}{t_1 - t_0}(x_1 - x_0)$$
   $$\sigma(t) = \sigma_0 + \frac{t - t_0}{t_1 - t_0}(\sigma_1 - \sigma_0)$$
2. **Separation Distance**:
   $$d_{\text{sep}}(t) = |x_A(t) - x_B(t)|$$
3. **Detection Threshold**: A conflict is declared if:
   $$d_{\text{sep}}(t) < d_{\text{threshold}} + \sigma_A(t) + \sigma_B(t)$$
   where default $d_{\text{threshold}} = 50.0\text{ m}$.
4. **Classification**:
   - If trains are traveling in opposite directions along the track, classified as `ConflictType::HeadOn`.
   - If traveling in the same direction, classified as `ConflictType::RearEnd`.

### 4.2 Resource Conflict Detection (Junction & Platform)
Junctions and platforms are shared physical resources modeled as graph nodes:
1. **Clearance Boundary ($25\text{ m}$)**: A train is considered to occupy a junction node if its position on an incoming/outgoing track is within $25.0\text{ m}$ of that node.
2. **Time Window Overlap**: If Train A arrives at node $N$ at time $t_A$ and Train B arrives at $t_B$:
   $$|t_A - t_B| < \Delta t_{\text{separation}} + \Delta t_{\text{clearance}}$$
   - Default: $\Delta t_{\text{separation}} = 5.0\text{ s}, \; \Delta t_{\text{clearance}} = 2.0\text{ s}$.
   - If node is a `Junction`: generates `ConflictType::Junction`.
   - If node is a `Platform`: generates `ConflictType::Platform`.

### 4.3 Space-Time Resource Reservation
The `ResourceReservationManager` enforces mutual exclusion:
- A reservation locks a `ConflictZone` (Track or Node) for interval $[t_{\text{start}}, t_{\text{end}}]$.
- `request(trainId, zone, t0, t1)` succeeds only if no other train holds an overlapping active reservation on the same zone:
  $$\text{Overlap} \iff \max(t_{\text{start}, 1}, t_{\text{start}, 2}) < \min(t_{\text{end}, 1}, t_{\text{end}, 2})$$
- `clearExpired(currentTime)` automatically purges historical reservations once the train has safely passed, preventing stale locks.

---

## 5. Public API Reference

```cpp
namespace tcas::conflict {

enum class ConflictType {
    RearEnd,
    HeadOn,
    Junction,
    Platform
};

struct Conflict {
    TrainId trainA{ 0 };
    TrainId trainB{ 0 };
    ConflictType type{ ConflictType::RearEnd };
    TrackId trackId{ 0 };
    NodeId resourceNodeId{ 0 };
    TimeSeconds firstConflictTime{ 0.0 };
    TimeSeconds lastConflictTime{ 0.0 };
    DistanceMeters minimumSeparation{ 0.0 };

    [[nodiscard]] bool involves(TrainId trainId) const noexcept;
};

struct ConflictDetectionConfig {
    DistanceMeters minimumTrackSeparation{ 50.0 };
    TimeSeconds resourceTimeSeparation{ 5.0 };
    TimeSeconds resourceClearanceTime{ 2.0 };
};

class ConflictDetector {
public:
    explicit ConflictDetector(ConflictDetectionConfig config = {});

    [[nodiscard]] std::vector<Conflict> detect(
        TrainId trainA,
        const std::vector<prediction::FutureState>& trajectoryA,
        TrainId trainB,
        const std::vector<prediction::FutureState>& trajectoryB,
        const infrastructure::RailwayNetwork& network
    ) const;
};

class ResourceReservationManager {
public:
    [[nodiscard]] bool request(TrainId trainId, const ConflictZone& zone,
                               TimeSeconds startTime, TimeSeconds endTime);
    [[nodiscard]] bool release(TrainId trainId, const ConflictZone& zone);
    void clearExpired(TimeSeconds currentTime);
    [[nodiscard]] bool isReserved(const ConflictZone& zone, TimeSeconds startTime,
                                 TimeSeconds endTime, TrainId requestingTrain) const;
    [[nodiscard]] const std::vector<ResourceReservation>& reservations() const noexcept;
};

} // namespace tcas::conflict
```

---

## 6. Execution Flow

```text
[Safety Cycle: Evaluate All Train Pairs (A, B)]
                     │
                     ▼
      ConflictDetector::detect(A, trajA, B, trajB, network)
                     │
    ┌────────────────┴────────────────────────┐
    │ 1. Scan shared tracks in trajectories   │
    │    - Interpolate positions              │
    │    - Compute minimum separation         │
    │    - If sep < 50m + sigmaA + sigmaB:    │
    │        Declare HeadOn or RearEnd        │
    │                                         │
    │ 2. Scan shared Junction / Platform nodes│
    │    - Extract arrival timestamps         │
    │    - If |tA - tB| < 5.0s:               │
    │        Declare Junction or Platform     │
    └────────────────┬────────────────────────┘
                     │
                     ▼
       Output std::vector<Conflict>
                     │
                     ▼
       Passed to RiskEngine & PriorityEngine
```

---

## 7. Edge Cases Handled

1. **Junction Ghosting / Early Trigger Suppression**: A train $500\text{ m}$ away on track $T_1$ does not hold the source junction. The $25\text{ m}$ node boundary gate ensures node events only trigger when the train is physically within the junction's clearance envelope.
2. **Dynamic Uncertainty Inflation**: If a train's sensor degrades and uncertainty balloons to $\pm 100\text{ m}$, the collision threshold expands accordingly, triggering earlier safety alerts.
3. **Expired Reservation Cleanup**: `clearExpired(currentTime)` ensures that trains that have passed through a junction do not permanently block subsequent trains.
4. **Re-entrant Reservations**: A train requesting an extension on its own held reservation is permitted without self-deadlock.

---

## 8. Automated Test Verification

Validated through `tests/conflict/ConflictDetectorTest.cpp` and `tests/integration/Module9IntegrationTest.cpp`:

- Head-on collision detection between converging trains
- Rear-end catching scenarios on long tracks
- Dual-train junction arrival within $5\text{ s}$ window
- Single-platform dual arrival contention
- Safe parallel track trains (zero false positive conflicts)
- Reservation locking, conflict rejection, and chronological expiration

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Imagine air traffic control radar screens at a busy airport:

Air traffic controllers don't wait for airplanes to touch before sounding the alarm. They watch their screens to see if two flight paths will intersect at the same altitude at the same minute.

**Module 9 is the "Radar Alert & Traffic Light Reservation System" for the railway.**

It takes the future paths predicted by Module 8 and compares them:
1. **Head-On Warning**: *"Warning! Train 1 and Train 2 are rushing toward each other on the exact same single track!"*
2. **Rear-End Warning**: *"Warning! Express Train 1 is driving at 160 km/h and will rear-end slow Freight Train 2 in 40 seconds!"*
3. **Junction Convergence**: *"Both trains will reach Alpha Switch at the exact same moment!"*
4. **The Electronic Reservation Pad**: When a train gets permission to cross a junction, Module 9 places a digital "RESERVED" lock on that track switch for the next 30 seconds. If any other train tries to cross, the system screams: *"Access Denied — Switch is occupied!"*
