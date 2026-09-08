# Module 8 — Predictive Position Engine

## 1. Module Overview

Module 8 is the future trajectory and multi-horizon position projection engine of the Train Collision Avoidance System (TCAS).

Collision avoidance cannot rely merely on a train's instantaneous position. Because a high-speed passenger or heavy freight train requires significant distance and time to decelerate, safety systems must "look ahead" into the future. By projecting where every train will be $5, 10, 20, 30, 60, 90,$ and $120$ seconds from now, the system can detect impending conflicts minutes before they occur, allowing smooth, energy-efficient deceleration rather than emergency braking.

Module 8 translates current kinematic states, planned route itineraries, track geometry, track speed limits, and track gradients into high-fidelity future space-time trajectory profiles.

---

## 2. Objectives

1. **Multi-Horizon Trajectory Projection**: Project train position, velocity, acceleration, and uncertainty at discrete future time horizons ($5\text{s}$ to $120\text{s}$).
2. **Multi-Track Route Traversal**: Accurately propagate movement across multiple contiguous track segments, resetting local track offsets at each boundary.
3. **Speed Limit & Gradient Enforcement**: Dynamically adjust acceleration and cap velocities according to individual track speed limits and track slopes along the route.
4. **Route Termination Handling**: Clamp predictions when a train reaches the end of its planned route itinerary, ensuring it comes to rest without overshooting.
5. **Uncertainty Growth Modeling**: Model the expansion of sensor uncertainty over time ($0.5\text{ m/s}$ growth rate).
6. **Stationary Obstacle Modeling**: Accurately project stopped or safety-held trains as static obstacles at their current physical position across all horizons.

---

## 3. Architectural Role & System Seams

```text
 Train Entity + Route (Module 5) + RailwayNetwork (Module 1)
                            │
                            ▼
              ┌───────────────────────────┐
              │     PredictionEngine      │
              │   (KinematicsEngine +     │
              │    Boundary Calculations) │
              └─────────────┬─────────────┘
                            │
                            ▼
              std::vector<FutureState>
     (Predicted states at t=5s, 10s, 20s, 30s, 60s, 90s, 120s)
                            │
                            ▼
               [Module 9: ConflictDetector]
      (Scans trajectories for spatial & temporal overlap)
```

---

## 4. Implementation Details

### 4.1 Prediction Horizons
The engine defines standard prediction horizons:
$$\mathcal{T} = \{5.0\text{ s}, 10.0\text{ s}, 20.0\text{ s}, 30.0\text{ s}, 60.0\text{ s}, 90.0\text{ s}, 120.0\text{ s}\}$$
Arbitrary custom horizons can also be evaluated via `predict()`.

### 4.2 Track Boundary Crossing Calculation
When a train moves along track $T_i$, the remaining distance to the track exit boundary is:
$$d_{\text{boundary}} = L_i - x_{\text{current}}$$
The time $t_{\text{boundary}}$ required to reach this boundary is computed by solving:
$$v_0 \cdot t + \frac{1}{2} \cdot a \cdot t^2 = d_{\text{boundary}}$$
1. If $a = 0$: $t_{\text{boundary}} = \frac{d_{\text{boundary}}}{v_0}$.
2. If $a \ne 0$: The quadratic discriminant $\Delta = v_0^2 + 2ad_{\text{boundary}}$ is computed:
   - If $\Delta < 0$: The train stops before reaching the boundary ($t_{\text{boundary}} = \infty$).
   - Otherwise: The smallest positive root determines $t_{\text{boundary}}$.

If the requested horizon $t_{\text{target}} < t_{\text{boundary}}$, the train remains on track $T_i$. If $t_{\text{target}} \ge t_{\text{boundary}}$, the train transitions to track $T_{i+1}$, resets its position offset to $0.0$, updates its speed limit to track $T_{i+1}$'s limit, and continues integrating the remaining time $t_{\text{target}} - t_{\text{boundary}}$.

### 4.3 Uncertainty Growth
Initial sensor uncertainty $\sigma_0$ expands over future projection time to reflect open-loop error accumulation:
$$\sigma(t) = \sigma_0 + k_{\text{growth}} \cdot t$$
- Default growth rate: $k_{\text{growth}} = 0.5\text{ m/s}$.

### 4.4 Stationary & Held Train Handling (LOGIC-2 Fix)
If a train is stopped or held by safety commands ($v = 0, a \le 0$):
- It is predicted as a static obstacle remaining on its current track and position for all future timestamps.
- This ensures downstream conflict detectors recognize it as a physical obstruction on the line while allowing clearance algorithms to evaluate when opposing trains have safely passed.

---

## 5. Public API Reference

```cpp
namespace tcas::prediction {

struct FutureState {
    TimeSeconds timestamp{ 0.0 };
    TrackId trackId{ 0 };
    DistanceMeters position{ 0.0 };
    SpeedMetersPerSecond velocity{ 0.0 };
    AccelerationMetersPerSecondSquared acceleration{ 0.0 };
    DistanceMeters uncertainty{ 0.0 };
};

class PredictionEngine {
public:
    PredictionEngine() = default;

    [[nodiscard]] static std::vector<FutureState> predictStandardHorizon(
        const train::Train& train,
        const infrastructure::RailwayNetwork& network,
        const navigation::RouteResult& route,
        TrackId currentTrackId,
        DistanceMeters initialUncertainty = 0.0
    );

    [[nodiscard]] static std::vector<FutureState> predict(
        const train::Train& train,
        const infrastructure::RailwayNetwork& network,
        const navigation::RouteResult& route,
        TrackId currentTrackId,
        const std::vector<TimeSeconds>& horizons,
        DistanceMeters initialUncertainty = 0.0
    );
};

} // namespace tcas::prediction
```

---

## 6. Execution Flow

```text
Safety Loop triggers Trajectory Prediction for Train A
                         │
                         ▼
        PredictionEngine::predictStandardHorizon()
                         │
        For each horizon H in [5, 10, 20, 30, 60, 90, 120]:
                         │
        ┌────────────────┴────────────────────────┐
        │ 1. Check distance to track exit         │
        │ 2. Compute timeToBoundary               │
        │ 3. If H < timeToBoundary:               │
        │      Compute (x, v, a) on current track │
        │    Else:                                │
        │      Advance to next track in route,    │
        │      apply new speed limit & gradient,  │
        │      recurse with remaining time        │
        │ 4. Compute uncertainty = sigma0 + 0.5*H │
        └────────────────┬────────────────────────┘
                         │
                         ▼
        Return std::vector<FutureState> (7 points)
```

---

## 7. Edge Cases Handled

1. **Current Track Not in Route**: If `currentTrackId` does not exist in `route.tracks`, throws `std::invalid_argument` with clear error diagnostics.
2. **Disconnected Route**: `validateRouteConnectivity` checks that every track's destination node matches the next track's source node.
3. **Route End Clamping**: If projected motion exceeds the total length of the final track, the train is clamped to the final coordinate with $v = 0$.
4. **Negative or Non-Finite Horizons**: Rejects negative, infinite, or NaN horizon values.
5. **Deceleration to Zero Before Boundary**: Correctly handles cases where a decelerating train stops completely before reaching the end of the track.

---

## 8. Automated Test Verification

Validated through `tests/prediction/PredictionEngineTest.cpp` and `tests/integration/Module8To12IntegrationTest.cpp`:

- Single-track constant velocity predictions
- Multi-track transition and distance accumulation
- Speed limit enforcement and deceleration
- Gradient acceleration and braking adjustments
- Route termination boundary clamping
- Standard horizon generation and sorting

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Imagine you are watching a game of chess or curling:

A master chess player doesn't just look at where the pieces are right now; they think: *"If I move here, where will my opponent be in 3 moves? Where will they be in 5 moves?"*

**Module 8 is the "Crystal Ball" of the Train Collision Avoidance System.**

Instead of just checking where the train is standing this very second, Module 8 looks ahead into the future along the train's planned track route. It asks:
- *"Where will this train be in 5 seconds?"*
- *"Where will it be in 30 seconds?"*
- *"Where will it be in 2 minutes?"*

It considers the train's speed, whether it is going uphill or downhill, and what the upcoming speed limits are. 

By drawing this future time-lapse path, it allows the safety computer to spot two trains heading toward the same junction long before they can even see each other's headlights!
