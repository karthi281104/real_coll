# Module 3 — Simulation & Clock

## 1. Module Overview

The Simulation & Clock module provides the **deterministic time backbone** of the TCAS system.

All TCAS modules that need to know the current time query the simulation clock rather than any real-time wall-clock source. This ensures the system is fully reproducible, testable, and deterministic.

---

## 2. Objectives

1. Provide a discrete-step simulation clock (`SimClock`) driven by explicit `tick()` calls.
2. Provide a configuration value type (`SimulationConfig`) holding all timing parameters.
3. Provide a periodic timer helper (`SimulationTimer`) to tell subsystems when to fire.
4. Ensure all time-related code is fully testable without threading or real-time dependencies.
5. Establish the time foundation required by Module 4 (Physics) and all later modules.

---

## 3. Module Responsibilities

Module 3 is responsible for:

- Simulation time tracking
- Tick-based clock advancement
- Timing parameter configuration
- Periodic subsystem fire scheduling

Module 3 is NOT responsible for:

- Real-time scheduling (Module 11 — Thread Orchestrator)
- Train physics (Module 4)
- Any I/O or sensor data (Module 6)

---

## 4. Classes

### 4.1 SimClock

Discrete-step deterministic simulation clock.

| Method | Description |
|---|---|
| `SimClock(TimeSeconds dt)` | Construct with given time step (default 0.020 s) |
| `void tick()` | Advance by one time step |
| `void reset()` | Reset to t=0, tick=0 |
| `TimeSeconds elapsed()` | Current simulation time in seconds |
| `SimTimeTick tickCount()` | Number of ticks since construction or reset |
| `TimeSeconds dt()` | Fixed time step |

### 4.2 SimulationConfig

Plain aggregate value type holding timing parameters.

| Field | Default | Description |
|---|---|---|
| `physicsPeriodMs` | 20 | Physics update interval (ms) |
| `safetyPeriodMs` | 100 | Safety check interval (ms) |
| `communicationPeriodMs` | 100 | Communication update interval (ms) |
| `hmiPeriodMs` | 200 | HMI telemetry interval (ms) |
| `predictionHorizonSeconds` | 60.0 | Prediction look-ahead window (s) |

### 4.3 SimulationTimer

Periodic fire helper for subsystem scheduling.

| Method | Description |
|---|---|
| `SimulationTimer(periodMs, physicsPeriodMs)` | Construct with period and physics tick size |
| `bool shouldFire(SimTimeTick)` | True when tick is a multiple of intervalTicks |
| `SimTimeTick intervalTicks()` | Computed interval in ticks |

---

## 5. Design Decisions

### No Wall-Clock Dependency

`SimClock` uses only `double` arithmetic internally. It has zero dependency on `std::chrono`, `time()`, or any system call. This is intentional — the system clock must be deterministic for testing.

### Tick-Driven Advancement

Time advances only when `tick()` is called explicitly. There is no background thread or automatic advancement. Module 11 will call `tick()` in its main loop.

### SimulationTimer Floor Division

If `periodMs` is not an exact multiple of `physicsPeriodMs`, the interval is rounded down. For example, 110 ms / 20 ms = 5.5 → 5 ticks (100 ms effective period). This is documented in the header.

---

---

## 6. Execution Flow

The simulation clock orchestrates time deterministically across the entire simulator. Time advances in discrete quantum ticks ($\Delta t = 20\text{ ms}$):

```text
[External Loop / Orchestrator]
               │
               ▼
        SimClock::tick()
               │
      ┌────────┴────────────────────────┐
      │  tickCount++                    │
      │  elapsedTime += dt (0.020 s)    │
      └────────┬────────────────────────┘
               │
      ┌────────▼────────────────────────┐
      │ SimulationTimer Queries         │
      ├─────────────────────────────────┤
      │ timerPhysics.shouldFire(tick)   │ ──► Every 1 tick (20 ms)  ──► Kinematics update
      │ timerSafety.shouldFire(tick)    │ ──► Every 5 ticks (100 ms)──► Conflict detection & resolution
      │ timerComms.shouldFire(tick)     │ ──► Every 5 ticks (100 ms)──► Wireless channel step
      │ timerHmi.shouldFire(tick)       │ ──► Every 10 ticks (200 ms)─► Dashboard render & telemetry
      └─────────────────────────────────┘
```

Because time only advances when `tick()` is called, tests can advance time by 100 ticks instantaneously without sleeping for 2 real seconds.

---

## 7. Automated Test Verification

Module 3 is validated by 3 comprehensive test suites under `tests/simulation/`:

1. **`SimClockTest.cpp`**: Tests clock construction, step advancement, reset behavior, custom dt handling, and precision over 1,000,000 ticks.
2. **`SimulationConfigTest.cpp`**: Validates default parameter structures, horizon validity, and custom configuration propagation.
3. **`SimulationTimerTest.cpp`**: Validates periodic trigger intervals, exact zero-tick behavior, sub-cycle floor division, and multi-timer cadence alignment.

All tests pass with 100% success rate under GoogleTest.

---

## 8. Module Status

**STATUS: COMPLETE — FULLY INTEGRATED & VERIFIED**

---

## 9. Layman's Terms Description (What Does This Module Do?)

Think of a movie projector or a musical metronome:

If you are watching a movie, each frame of film clicks forward 24 times every second. If you pause the projector, everything in the movie freezes. If you speed up the film, the actors move faster, but the story happens in the exact same sequence every single time.

**Module 3 is the Metronome of the Train Simulator.**

Instead of relying on the computer's wall clock (which can stutter, lag, or vary between fast and slow computers), Module 3 creates a steady, rock-solid "tick... tick... tick..." every 20 milliseconds.
- At **Tick 1**, it tells the physics engine: "Move the train forward by 0.02 seconds of travel."
- Every **5 Ticks**, it tells the safety brain: "Look ahead down the tracks for any collisions."
- Every **10 Ticks**, it updates the operator's computer screen.

Because the clock is completely controllable, engineers can test the safety system in "fast forward" (simulating an hour of train traffic in 3 seconds) or "slow motion" (stepping through a near-miss millisecond by millisecond) with 100% mathematical reproducibility.

