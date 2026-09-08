# Module 4 — Physics & Braking Dynamics

## 1. Module Overview

Module 4 is the deterministic kinematics and braking dynamics engine of the Train Collision Avoidance System (TCAS).

In a railway network, trains possess immense kinetic energy due to their massive tonnage (up to 1,500 tonnes for freight) and high operating speeds (up to 162 km/h for express services). Unlike road vehicles, trains cannot swerve and require hundreds or thousands of metres to stop. Furthermore, track slope (gradient) significantly impacts stopping capability: uphill slopes assist braking, while downhill slopes oppose braking and can cause runaway conditions if miscalculated.

Module 4 encapsulates all kinematic motion equations, gradient-corrected braking physics, driver reaction modeling, and safe stopping distance envelopes into a stateless, high-performance C++23 calculation engine.

---

## 2. Objectives

1. **Deterministic Motion Calculations**: Provide closed-form equations of motion for updating train position and velocity at every simulation tick.
2. **Gradient-Aware Braking**: Adjust nominal braking deceleration based on the physical slope of the track using standard gravitational acceleration ($g = 9.80665\text{ m/s}^2$).
3. **Safe Distance Envelopes**: Compute the exact safety envelope ($d_{\text{safe}}$) consisting of reaction distance, service braking distance, and safety margin.
4. **Emergency Braking Distance**: Calculate absolute stopping distance under maximum emergency brake application.
5. **Speed-Distance Projections**: Compute the velocity of a train at any future distance point along its path.
6. **Thread Safety & Zero Side-Effects**: Implement all physics calculations as pure, stateless static methods (`[[nodiscard]]`), safe for concurrent evaluation.

---

## 3. Architectural Role & System Seams

The `KinematicsEngine` sits at the boundary between raw physical reality and high-level predictive safety:

```text
  Track Infrastructure              Train Entity
   (Gradient, Length)            (Mass, Vmax, a_brake, a_emerg)
            │                               │
            └───────────────┬───────────────┘
                            │
                            ▼
               ┌────────────────────────┐
               │    KinematicsEngine    │
               └────────────┬───────────┘
                            │
       ┌────────────────────┼────────────────────┐
       ▼                    ▼                    ▼
[ThreadOrchestrator] [PredictionEngine] [ResolutionEngine]
  (20ms Position /     (Multi-Horizon     (Braking Feasibility &
   Velocity Tick)       Trajectories)      Target Speed Commands)
```

- **Input**: Current train velocity, acceleration, track gradient, nominal deceleration rates, and time quantum $\Delta t$.
- **Output**: Updated state $(x, v)$, effective deceleration $a_{\text{eff}}$, braking distance $d_{\text{brake}}$, and safe stopping envelope $d_{\text{safe}}$.

---

## 4. Mathematical Physics & Formulas

The module implements the standard Newtonian kinematic equations of motion:

### 4.1 Position Update
Position along the current track is updated using the standard quadratic Taylor expansion:
$$x(t + \Delta t) = x_0 + v_0 \cdot \Delta t + \frac{1}{2} \cdot a \cdot (\Delta t)^2$$
*Rule*: If $\Delta t \le 0$, position is returned unchanged ($x_0$).

### 4.2 Velocity Update with Clamping
Velocity is updated linearly and strictly clamped within physical bounds:
$$v(t + \Delta t) = \text{clamp}(v_0 + a \cdot \Delta t, \; 0.0, \; v_{\text{max}})$$
*Rule*: Speed can never become negative (no reversing in normal running) and cannot exceed the train or track maximum allowed speed limit.

### 4.3 Gradient-Corrected Effective Deceleration
Track gradient is defined as the dimensionless ratio $h / L$ (rise over run):
- **Positive gradient ($+m$)**: Uphill track. Gravity pulls the train backward, assisting the brakes.
- **Negative gradient ($-m$)**: Downhill track. Gravity pulls the train forward, opposing the brakes.

$$a_{\text{eff}} = \max\left(a_{\text{nominal}} + g \cdot \text{gradient}, \; kMinDeceleration\right)$$
Where:
- $g = 9.80665\text{ m/s}^2$ (ISO 80000-3 standard gravity)
- $kMinDeceleration = 0.01\text{ m/s}^2$ prevents negative or zero deceleration on steep downgrades, ensuring a mathematical floor.

### 4.4 Braking Distance
From the Torricelli equation ($v^2 = v_0^2 + 2ad$ with final velocity $v = 0$):
$$d_{\text{brake}} = \frac{v_0^2}{2 \cdot a_{\text{eff}}}$$
*Rule*: If $v_0 \le 0$, $d_{\text{brake}} = 0$.

### 4.5 Driver Reaction Distance
Before braking begins, the train continues at its initial speed during the human or automated reaction window:
$$d_{\text{reaction}} = v_0 \cdot t_{\text{reaction}}$$
- Default reaction time: $t_{\text{reaction}} = 1.5\text{ s}$

### 4.6 Total Safe Stopping Envelope ($d_{\text{safe}}$)
The safety pipeline uses this envelope to determine whether a train is at risk:
$$d_{\text{safe}} = d_{\text{reaction}} + d_{\text{brake}} + d_{\text{margin}}$$
- Default safety margin: $d_{\text{margin}} = 50.0\text{ m}$

### 4.7 Speed After Distance
Calculates the velocity reached after covering distance $d$ under constant acceleration $a$:
$$v_f = \sqrt{\max(v_0^2 + 2 \cdot a \cdot d, \; 0.0)}$$
If $v_0^2 + 2ad \le 0$, the train has come to a complete stop prior to covering distance $d$, returning $0.0\text{ m/s}$.

---

## 5. Public API Reference

The `tcas::physics::KinematicsEngine` class contains only static methods:

```cpp
namespace tcas::physics {

class KinematicsEngine {
public:
    KinematicsEngine() = delete; // Stateless pure class

    static constexpr double kGravity = 9.80665;
    static constexpr double kDefaultReactionTime = 1.5;
    static constexpr double kDefaultSafetyMargin = 50.0;
    static constexpr double kMinDeceleration = 0.01;

    [[nodiscard]] static DistanceMeters updatePosition(
        DistanceMeters position, SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared acceleration, TimeSeconds dt);

    [[nodiscard]] static SpeedMetersPerSecond updateVelocity(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared acceleration,
        TimeSeconds dt, SpeedMetersPerSecond maximumSpeed);

    [[nodiscard]] static AccelerationMetersPerSecondSquared effectiveDeceleration(
        AccelerationMetersPerSecondSquared nominalDeceleration, double gradient);

    [[nodiscard]] static DistanceMeters brakingDistance(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared effectiveDecel);

    [[nodiscard]] static DistanceMeters reactionDistance(
        SpeedMetersPerSecond velocity,
        TimeSeconds reactionTime = kDefaultReactionTime);

    [[nodiscard]] static DistanceMeters safeDistance(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared nominalDeceleration,
        double gradient,
        TimeSeconds reactionTime = kDefaultReactionTime,
        DistanceMeters safetyMargin = kDefaultSafetyMargin);

    [[nodiscard]] static DistanceMeters emergencyStoppingDistance(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared emergencyDeceleration,
        double gradient);

    [[nodiscard]] static SpeedMetersPerSecond speedAfterDistance(
        SpeedMetersPerSecond initialVelocity,
        AccelerationMetersPerSecondSquared acceleration,
        DistanceMeters distance);
};

} // namespace tcas::physics
```

---

## 6. Execution & Data Flow

In each physics tick ($\Delta t = 20\text{ ms}$), the Thread Orchestrator calls the physics routines for every train in the fleet:

```text
   [20ms Physics Tick Triggered]
                 │
                 ▼
   For each Train in TrainManager:
                 │
                 ├─► Read current (pos, vel, acc, trackId)
                 ├─► Fetch Track from RailwayNetwork (get gradient)
                 │
                 ├─► KinematicsEngine::updatePosition(pos, vel, acc, 0.020)
                 ├─► KinematicsEngine::updateVelocity(vel, acc, 0.020, v_max)
                 │
                 ├─► Did train reach the end of the track?
                 │     ├── YES: Advance to next track on planned route
                 │     └── NO:  Update train position on current track
                 │
                 └─► Write updated state back to Train entity
```

---

## 7. Edge Cases Handled

1. **Downhill Runaway Guard**: On steep downgrades (e.g., $-5\%$), $g \cdot (-0.05) = -0.49\text{ m/s}^2$. If nominal braking is $0.5\text{ m/s}^2$, effective braking would drop to $0.01\text{ m/s}^2$. The engine enforces `kMinDeceleration = 0.01` to guard against zero or negative deceleration values.
2. **Zero or Negative Time Steps**: If $\Delta t \le 0.0$, the methods return current positions and velocities unchanged without numerical instability.
3. **Speed Zero Clamping**: Braking deceleration cannot cause the train to travel backward. Once velocity reaches $0$, it is strictly clamped.
4. **Imaginary Speed Prevention**: In `speedAfterDistance`, if braking brings the train to rest before the specified distance is reached ($v_0^2 + 2ad < 0$), the formula returns `0.0` rather than taking the square root of a negative number.
5. **Non-Finite Number Rejection**: Validated against NaN and infinite inputs.

---

## 8. Automated Test Verification

The physics engine is validated by comprehensive GoogleTest suites:

- **`tests/physics/KinematicsEngineTest.cpp`**: Tests constant velocity, positive acceleration, service brake deceleration, emergency stopping distance, uphill/downhill gradient corrections, driver reaction distance, safe distance calculations, and extreme boundary cases.
- **`tests/integration/PhysicsNavigationIntegrationTest.cpp`**: Tests the integration seam where trains physically traverse tracks, decelerate across track boundaries, and respect varying speed limits along a route.

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Have you ever tried stopping a loaded shopping cart versus a bicycle versus a heavy car? 

A heavy car takes much longer to stop than a bicycle, and if you are driving downhill on a steep slope, it takes even longer because gravity is pulling you forward.

**Module 4 is the "Laws of Physics & Brakes" Calculator for the trains.**

It calculates exactly what happens to a train every 20 milliseconds:
1. **How far did the train move?** It calculates the new position based on speed and acceleration.
2. **What are the brakes capable of?** A giant freight train weighing 1,500 tonnes cannot stop on a dime. Module 4 calculates how many metres the train will glide before it can come to a full stop.
3. **Is the track sloping up or down?** If the train is climbing a mountain track, gravity helps it stop faster. If it is rolling downhill into a valley, gravity fights the brakes, so Module 4 warns that a much longer stopping distance is required.
4. **The Safety Bubble ($d_{\text{safe}}$)**: It calculates a "safety cushion" in front of the train (driver reaction time + full braking distance + 50 metres of empty buffer space). If anything enters that cushion, alarms must go off!
