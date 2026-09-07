# Module 8 — Trajectory Prediction

## 1. Architectural Role

Module 8 is the predictive foundation of the TCAS (Train Collision Avoidance System) safety pipeline. It answers the fundamental question: **"Given a train's current kinematic state, route topology, and sensor uncertainty, where will the train physically be at future points in time?"**

By projecting train positions forward across a discrete set of time horizons (typically 5s, 10s, 20s, 30s, and 60s), downstream modules (Module 9 Conflict Detection and Module 10 Risk Assessment) can identify spatial-temporal convergence and execute evasive actions long before physical braking limits are breached.

```
Train State (Pos, Vel, Accel, Uncertainty)
                  +
Route Topology (Connected Tracks, Gradients, Speed Limits)
                  │
                  ▼
   ┌───────────────────────────────┐
   │       PredictionEngine        │
   │  ├── Kinematic Motion Model   │
   │  ├── Track Boundary Crossing  │
   │  ├── Gradient Gravity Accel   │
   │  ├── Track Speed Limits       │
   │  └── Uncertainty Propagation  │
   └──────────────┬────────────────┘
                  │
                  ▼
   std::vector<FutureState>
   ├── Horizon:  5.0s -> Track A, Pos X1, Vel V1, Uncert U1
   ├── Horizon: 10.0s -> Track A, Pos X2, Vel V2, Uncert U2
   ├── Horizon: 20.0s -> Track B, Pos X3, Vel V3, Uncert U3
   ├── Horizon: 30.0s -> Track B, Pos X4, Vel V4, Uncert U4
   └── Horizon: 60.0s -> Track C, Pos X5, Vel V5, Uncert U5
```

---

## 2. Public API Specification

### 2.1 `FutureState` Value Object

Defined in [`include/prediction/FutureState.hpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/include/prediction/FutureState.hpp):

```cpp
struct FutureState {
    TimeSeconds timestamp;        // Future lookahead time from current moment (s)
    TrackId trackId;              // Track ID on which train resides at this horizon
    DistanceMeters position;      // Position along that track (m, 0.0 to track.length())
    SpeedMetersPerSecond velocity;// Velocity at this horizon (m/s)
    AccelerationMetersPerSecondSquared acceleration; // Acceleration (m/s^2)
    DistanceMeters uncertainty;   // Dynamic position uncertainty envelope (±m)
};
```

### 2.2 `PredictionEngine`

Defined in [`include/prediction/PredictionEngine.hpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/include/prediction/PredictionEngine.hpp):

| Static Method | Description |
|---|---|
| `predictStandardHorizon(train, network, route, currentTrackId, initialUncertainty)` | Generates future states for the standard horizon set: `{ 5.0, 10.0, 20.0, 30.0, 60.0 }` seconds. |
| `predict(train, network, route, currentTrackId, horizons, initialUncertainty)` | Generates future states for an arbitrary, user-specified vector of chronological horizons. |
| `validateRouteConnectivity(network, route)` | Validates that route track segments exist and are strictly contiguous (`prev.destination == next.source`). |
| `calculateTimeToBoundary(distance, velocity, acceleration)` | Analytically solves quadratic/linear time required to reach the end of the current track block. |

---

## 3. Mathematical & Kinematic Model

### 3.1 Kinematic Propagation within a Track Block

For a time increment $\Delta t$ within a track of length $L$:
- **Position**: $p(t + \Delta t) = p(t) + v(t)\Delta t + \frac{1}{2} a \Delta t^2$
- **Velocity**: $v(t + \Delta t) = \min(v_{max}, \max(0.0, v(t) + a \Delta t))$

If the train is decelerating ($a < 0$), the time to stop is $t_{stop} = -\frac{v}{a}$. If $t_{stop} \le \Delta t$, the train comes to a complete rest:
$$p_{final} = p(t) + v(t)t_{stop} + \frac{1}{2}a t_{stop}^2 = p(t) - \frac{v^2}{2a}$$
$$v_{final} = 0.0, \quad a_{final} = 0.0$$

### 3.2 Track Gradient Effect on Deceleration

Track gradients alter the effective deceleration capacity of a train:
$$a_{effective} = a_{service} + g \cdot \theta$$
where:
- $g = 9.81 \text{ m/s}^2$ (gravitational acceleration)
- $\theta = \tan(\alpha)$ is the track gradient ratio (positive for uphill grade, negative for downhill slope).
- An uphill gradient ($\theta > 0$) aids braking, increasing deceleration.
- A downhill gradient ($\theta < 0$) counteracts braking, reducing stopping capability.

### 3.3 Multi-Track Boundary Transitions

When a train reaches the boundary of track $i$ ($p \ge L_i$), the remaining lookahead time $\Delta t_{rem}$ is carried over into track $i+1$:
1. Solve for time to boundary $t_{boundary}$ using `calculateTimeToBoundary()`:
   $$\Delta d = L_i - p_i$$
   $$t_{boundary} = \begin{cases} \frac{\Delta d}{v}, & \text{if } |a| < 10^{-9} \\ \frac{-v + \sqrt{v^2 + 2a\Delta d}}{a}, & \text{if } a \ne 0 \text{ and } v^2 + 2a\Delta d \ge 0 \end{cases}$$
2. Update kinematics at boundary:
   $$v_{boundary} = \min(v_i + a \cdot t_{boundary}, v_{limit, i+1})$$
3. Subtract $t_{boundary}$ from remaining lookahead: $\Delta t_{rem} \leftarrow \Delta t_{rem} - t_{boundary}$.
4. Transition state to Track $i+1$ with initial position $p_{i+1} = 0.0$.
5. Repeat iteratively until $\Delta t_{rem} = 0.0$ or the end of the assigned route is reached.

### 3.4 Dynamic Uncertainty Expansion

Sensor measurements possess intrinsic physical inaccuracy (GPS drift, wheel-slip odometer noise). This uncertainty grows monotonically over the lookahead horizon:
$$\sigma(t) = \sigma_0 + k_{growth} \cdot t$$
- $\sigma_0$: Initial measurement uncertainty at $t = 0$ (Nominal: $1.0\text{ m}$; Degraded Sensor: $15.0\text{ m}$).
- $k_{growth} = 0.5\text{ m/s}$ (linear dispersion coefficient).
- At horizon $t = 60\text{ s}$, nominal uncertainty expands to $1.0 + 0.5(60) = 31.0\text{ m}$.
- Under degraded sensor failure, uncertainty expands to $15.0 + 0.5(60) = 45.0\text{ m}$, forcing larger safety protective margins in collision detection.

---

## 4. Error Handling and Defensive Constraints

1. **Non-Negative Invariant**: Velocities and positions must satisfy $v \ge 0.0, p \ge 0.0$. Throws `std::invalid_argument` if violated.
2. **Disconnected Routes**: Traverses route track segments; if track $i$'s destination node does not match track $i+1$'s source node, throws `std::invalid_argument("Route contains disconnected tracks")`.
3. **Route Containment**: Current track must reside in the route; otherwise throws `std::invalid_argument("Current track is not present in the route")`.
4. **Finite Horizons**: All horizon timestamps must be finite, non-negative, and strictly ordered.

---

## 5. Test Verification

Implemented in [`tests/prediction/PredictionEngineTest.cpp`](file:///d:/POC-FINAL-CPP/Realtime-Train/tests/prediction/PredictionEngineTest.cpp):
- Constant velocity trajectory prediction across multiple tracks.
- Train stopping mid-track due to service brake deceleration.
- Downhill vs uphill grade adjustments on braking curves.
- Correct clamping at route end (terminal buffer stop).
- Linear uncertainty expansion across all 5 standard horizons.
