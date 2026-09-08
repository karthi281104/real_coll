# Module 6 — Sensor Modeling & State Estimation

## 1. Module Overview

Module 6 is the sensor modeling, uncertainty quantification, and state estimation engine of the Train Collision Avoidance System (TCAS).

In real-world rail operations, a train's true position and velocity can never be measured with 100% precision. Train wheels experience wheel slip (during acceleration) and wheel slide (during braking), causing wheel-revolution tachometers and odometers to continuously drift away from truth. Furthermore, electrical sensor noise, mechanical vibrations, and occasional hardware dropouts corrupt raw measurements.

Module 6 implements a discrete-time 1D Kalman Filter that fuses high-frequency, drifting odometry with discrete, high-precision track balise (transponder) fixes. It performs statistical outlier rejection gating and continuously computes state covariance ($\sigma_{\text{pos}}, \sigma_{\text{vel}}$), providing safety-critical modules with reliable estimates of train position and its uncertainty envelope.

---

## 2. Objectives

1. **Continuous State Estimation**: Maintain accurate, filtered estimates of position, velocity, and acceleration across continuous operational cycles.
2. **Wheel Slip Drift Simulation**: Model realistic wheel slip and slide errors that accumulate over distance and time.
3. **Discrete Absolute Balise Anchoring**: Integrate discrete track transponders (Eurobalises) that provide millimeter-accurate position anchors, resetting accumulated drift.
4. **Statistical Outlier Gating**: Reject anomalous sensor spikes or faulty readings using a $3.5\sigma$ innovation gate.
5. **Sensor Health & Degradation Detection**: Monitor consecutive outlier occurrences and sensor validity flags, raising `isDegraded` alarms when tracking integrity degrades.
6. **Uncertainty Propagation**: Output standard deviation bounds ($\sigma_{\text{pos}}, \sigma_{\text{vel}}$) so predictive collision algorithms can expand safety buffers under degraded sensing.

---

## 3. Architectural Role & System Seams

```text
    Physical Train Motion
    (True pos, vel, acc)
              │
      ┌───────┴───────────────────┐
      ▼                           ▼
[Odometer Hardware]     [Balise Transponder]
(1% wheel slip drift,     (Trackside radio beacon,
 electrical noise)        exact anchor position)
      │                           │
      └─────────────┬─────────────┘
                    │
                    ▼
          ┌───────────────────┐
          │   StateEstimator  │
          │   (Kalman Filter) │
          └─────────┬─────────┘
                    │
                    ▼
              EstimatedState
    ├── position (filtered)
    ├── velocity (filtered)
    ├── acceleration (filtered)
    ├── positionUncertainty (std dev)
    ├── velocityUncertainty (std dev)
    └── isDegraded (health flag)
                    │
      ┌─────────────┴─────────────┐
      ▼                           ▼
[PredictionEngine]       [ThreadOrchestrator]
(Expands trajectory       (Detects degraded sensors
 safety envelopes)         and alerts operator)
```

---

## 4. Kalman Filter Mathematical Formulation

The estimator tracks a 3-dimensional state vector $\mathbf{x}$ and a $3 \times 3$ error covariance matrix $\mathbf{P}$:
$$\mathbf{x} = \begin{bmatrix} x \\ v \\ a \end{bmatrix} = \begin{bmatrix} \text{position} \\ \text{velocity} \\ \text{acceleration} \end{bmatrix}, \quad \mathbf{P} \in \mathbb{R}^{3 \times 3}$$

### 4.1 Time Update (Prediction Step)
At each time step $\Delta t$, the state is propagated forward using Newtonian kinematics:
$$\hat{\mathbf{x}}^- = \mathbf{F} \hat{\mathbf{x}}, \quad \mathbf{P}^- = \mathbf{F} \mathbf{P} \mathbf{F}^T + \mathbf{Q}$$

State transition matrix $\mathbf{F}$:
$$\mathbf{F} = \begin{bmatrix} 1 & \Delta t & \frac{1}{2}(\Delta t)^2 \\ 0 & 1 & \Delta t \\ 0 & 0 & 1 \end{bmatrix}$$

Process noise covariance matrix $\mathbf{Q}$:
$$\mathbf{Q} = \begin{bmatrix} q_{\text{pos}} \Delta t & 0 & 0 \\ 0 & q_{\text{vel}} \Delta t & 0 \\ 0 & 0 & q_{\text{acc}} \Delta t \end{bmatrix}$$
- Default values: $q_{\text{pos}} = 0.1\text{ m}^2/\text{s}, \; q_{\text{vel}} = 0.2\text{ (m/s)}^2/\text{s}, \; q_{\text{acc}} = 0.5\text{ (m/s}^2)^2/\text{s}$.

### 4.2 Measurement Update (Odometry)
When an odometer reading $\mathbf{z} = [z_{\text{pos}}, z_{\text{vel}}]^T$ arrives:
1. **Measurement Matrix**:
   $$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}, \quad \mathbf{R} = \begin{bmatrix} \sigma_{\text{pos\_meas}}^2 & 0 \\ 0 & \sigma_{\text{vel\_meas}}^2 \end{bmatrix}$$
2. **Innovation (Residual)**:
   $$\mathbf{y} = \mathbf{z} - \mathbf{H} \hat{\mathbf{x}}^-$$
3. **Innovation Covariance**:
   $$\mathbf{S} = \mathbf{H} \mathbf{P}^- \mathbf{H}^T + \mathbf{R}$$
4. **Statistical Outlier Gating**:
   $$\text{NIS} = \mathbf{y}^T \mathbf{S}^{-1} \mathbf{y}$$
   If $\text{NIS} > (3.5)^2 = 12.25$, the measurement is rejected as an anomaly or sensor glitch.
5. **Kalman Gain & Correction**:
   $$\mathbf{K} = \mathbf{P}^- \mathbf{H}^T \mathbf{S}^{-1}$$
   $$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K} \mathbf{y}$$
   $$\mathbf{P} = (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}^-$$

### 4.3 Measurement Update (Track Balise Anchor)
When a train crosses an absolute track balise transponder, an absolute 1D position measurement $z_{\text{balise}}$ is provided with near-zero noise ($R_{\text{balise}} = 0.01\text{ m}^2$):
$$\mathbf{H}_{\text{balise}} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$$
Fusing the balise instantly collapses position variance $P_{00}$ down to $0.01\text{ m}^2$, completely eliminating accumulated wheel slip drift.

---

## 5. Public API Reference

```cpp
namespace tcas::sensor {

struct EstimatedState {
    DistanceMeters position{ 0.0 };
    SpeedMetersPerSecond velocity{ 0.0 };
    AccelerationMetersPerSecondSquared acceleration{ 0.0 };
    double positionUncertainty{ 1.0 }; // standard deviation in metres
    double velocityUncertainty{ 0.5 }; // standard deviation in m/s
    SimTimeTick timestamp{ 0 };
    bool isDegraded{ false };
};

class StateEstimator {
public:
    explicit StateEstimator(
        const SensorNoiseConfig& config = {},
        DistanceMeters initialPosition = 0.0,
        SpeedMetersPerSecond initialVelocity = 0.0
    );

    void predict(TimeSeconds dt, SimTimeTick timestamp) noexcept;
    bool updateOdometry(const OdometerMeasurement& measurement) noexcept;
    bool updateBalise(const BaliseTransponder& balise) noexcept;

    [[nodiscard]] EstimatedState estimatedState() const noexcept;
    [[nodiscard]] DistanceMeters position() const noexcept;
    [[nodiscard]] SpeedMetersPerSecond velocity() const noexcept;
    [[nodiscard]] double positionUncertainty() const noexcept;
    [[nodiscard]] bool isDegraded() const noexcept;
    void reset(DistanceMeters initialPosition = 0.0,
               SpeedMetersPerSecond initialVelocity = 0.0) noexcept;
};

class Odometer {
public:
    OdometerMeasurement measure(
        DistanceMeters truePos, SpeedMetersPerSecond trueVel,
        AccelerationMetersPerSecondSquared trueAcc,
        TimeSeconds dt, SimTimeTick tick);
    void calibrate(DistanceMeters exactPosition);
    void setFaulty(bool faulty);
    [[nodiscard]] DistanceMeters accumulatedDrift() const;
};

} // namespace tcas::sensor
```

---

## 6. Execution Flow

```text
[Simulation Tick dt = 0.020s]
              │
              ├─► 1. StateEstimator::predict(dt, tick)
              │      Propagates state forward via F, increases covariance by Q
              │
              ├─► 2. Odometer::measure(trueState)
              │      Adds 1% drift and Gaussian measurement noise
              │
              ├─► 3. StateEstimator::updateOdometry(measurement)
              │      Compute innovation y = z - Hx
              │      Is y^T S^-1 y <= (3.5)^2?
              │         ├── YES: Compute Kalman gain K, update state & covariance
              │         └── NO:  Reject measurement, increment outlier count
              │
              └─► 4. Train crosses Balise Transponder?
                        └── YES: StateEstimator::updateBalise(exactPos)
                                 Resets position uncertainty, clears drift
```

---

## 7. Edge Cases Handled

1. **Persistent Sensor Dropout / Fault**: If a sensor hardware failure is injected (`setFaulty(true)`), incoming measurements are marked invalid. The estimator skips measurement updates, relies on dead reckoning prediction, and flags `isDegraded = true`.
2. **Wild Measurement Spikes**: Outlier gating rejects random voltage spikes or slip bursts exceeding $3.5\sigma$.
3. **Sustained Outliers (>5 Consecutive)**: If 5 consecutive readings fail the innovation gate, the filter recognizes that either the physical model or the sensor has diverged and flags `isDegraded = true`.
4. **Covariance Symmetry & Positive-Definiteness**: The covariance matrix $\mathbf{P}$ is bounded from below to prevent numerical degeneracy.

---

## 8. Automated Test Verification

Validated through `tests/sensor/OdometerTest.cpp` and `tests/sensor/StateEstimatorTest.cpp`:

- Steady-state convergence under Gaussian noise
- Wheel slip drift accumulation over 1,000 metres of travel
- Instantaneous drift cancellation upon balise transponder reception
- Innovation gate outlier rejection of $10\sigma$ anomalous inputs
- Degradation state transitions and health reporting

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Have you ever walked through a dark room with your eyes closed, counting your footsteps to guess where you are? 

Each step you take is a little bit uncertain. By step 20, you think you are in the kitchen, but you might be off by a few feet. Then, your hand touches the refrigerator door—bam! You instantly know your exact location again.

**Module 6 is the "Smart Navigation Brain" that keeps track of the train's true location.**

1. **The Slippery Wheels**: When a train accelerates or slams on the brakes, the steel wheels slip slightly on the steel rails. If the computer only counted wheel spins, it would gradually get confused about where the train actually is (wheel slip drift).
2. **The Kalman Filter (Noise Cleaner)**: Like a smart noise-cancelling headphone, it strips away the jitter, vibrations, and noisy readings to find the smooth, true speed and position of the train.
3. **The Track Balises (Reality Check Beacons)**: Every few kilometres along the tracks, there is an electronic transponder box bolted between the rails. When the train zooms over it, the beacon broadcasts: *"You are at exactly kilometre 14.500!"* Module 6 immediately resets any wheel drift to zero.
4. **The Uncertainty Bubble**: Module 6 doesn't just say *"The train is at 500 metres."* It says *"The train is at 500 metres, plus or minus 2 metres."* If a sensor breaks, the bubble expands to *"plus or minus 50 metres,"* warning the safety system to keep a much wider distance from other trains.
