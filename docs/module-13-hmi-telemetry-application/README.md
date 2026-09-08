# Module 13 — Human-Machine Interface (HMI), Telemetry & Application Layer

## 1. Module Overview

Module 13 is the operator interface, telemetry logging, performance analysis, and interactive application layer of the Train Collision Avoidance System (TCAS).

A sophisticated automated safety system cannot operate as a black box: human railway operators, dispatch supervisors, and safety inspectors must have clear, real-time visibility into the system's operational state. They need to monitor train locations, observe collision predictions as they unfold, inspect safety commands, inject test faults, and review historical telemetry logs for accident investigation and performance certification.

Module 13 provides:
1. **`HmiDisplay`**: An ANSI-formatted real-time terminal dashboard visualizing fleet status, active conflicts, reservations, and system health.
2. **`TelemetryLogger`**: A high-performance CSV and event logging engine persisting operational state to disk for post-mission analysis.
3. **`PerformanceMetrics`**: A real-time statistical analyzer tracking safety KPIs (near-miss count, collision count, minimum separation, minimum TTC, and rendering latency).
4. **`TcasApplication`**: A menu-driven interactive terminal application allowing operators to dispatch trains, view live radar tracking, trigger scenarios, and inject live sensor/communication faults.

---

## 2. Objectives

1. **Intuitive Operator Dashboard**: Render live ASCII/ANSI consoles displaying train velocities, positions, operational states, active conflicts, and right-of-way reservations.
2. **Persistent Telemetry Logging**: Stream timestamped CSV telemetry logs to the `logs/` directory for historical analysis and regulatory compliance.
3. **Safety Performance Tracking**: Continuously compute safety KPIs, including collision count (zero tolerance), near misses, and minimum separation distance.
4. **Interactive Control Console**: Provide a hierarchical terminal menu system for fleet management, scenario triggering, live radar observation, and manual intervention.
5. **Real-Time Fault Injection**: Enable operators to dynamically toggle sensor dropouts or wireless communication blackouts to observe fail-safe system behavior live.

---

## 3. Architectural Role & System Seams

```text
               ThreadOrchestrator (Module 12)
                             │
            WorldState Snapshot (every 200 ms)
                             │
       ┌─────────────────────┼─────────────────────┐
       ▼                     ▼                     ▼
   HmiDisplay         TelemetryLogger     PerformanceMetrics
(ANSI Terminal        (CSV Data Stream      (Safety KPIs, TTC,
  Visualizer)           to Disk)             Latency Counters)
       │                     │                     │
       └─────────────────────┼─────────────────────┘
                             │
                             ▼
                      TcasApplication
           ├── Main Control Menu
           ├── Fleet Management (Dispatch R-01..R-10 / Custom)
           ├── Safety & Interlocking Monitor
           ├── Fault Injection Console
           └── Live Radar View Loop
```

---

## 4. Implementation Details

### 4.1 HMI Terminal Dashboard (`HmiDisplay`)
Renders formatted tabular views of the entire railway world state:
- **System Header**: Displays simulation time, active status (`READY`, `RUNNING`, `PAUSED`, `DEGRADED`), cycle counts across all 4 loops, and operator notices.
- **Fleet Table**: Shows each train's ID, Type (Express, Passenger, Freight), Current Track, Position along track ($m$), Velocity ($m/s$), State (`RUNNING`, `SLOWING`, `BRAKING`, `HOLD (SIG)`, `EMERGENCY`), and Sensor Health.
- **Active Conflicts Table**: Highlights detected hazards (`HEAD-ON`, `REAR-END`, `JUNCTION`, `PLATFORM`), involved trains, predicted collision time, and minimum separation.
- **Active Resource Reservations Table**: Displays locked junctions and track segments with owning train IDs and reservation expiration times.
- **Safety Decisions & Commands Table**: Shows real-time commands issued by the `SafetyPipeline`.

### 4.2 Telemetry & Event Logging (`TelemetryLogger`)
- Writes chronological records into `logs/telemetry_<timestamp>.csv`.
- Logs timestamp, train ID, position, velocity, acceleration, state, active track, and active safety commands at every HMI tick.
- Flushes buffers cleanly upon shutdown to guarantee zero data loss.

### 4.3 Performance Metrics & Safety KPIs (`PerformanceMetrics`)
Monitors system performance metrics across every cycle:
- `collisionCount`: Number of times physical distance between two trains dropped below $0.0\text{ m}$ (target: strictly $0$).
- `nearMissCount`: Number of times separation dropped below $50.0\text{ m}$ buffer without active braking.
- `emergencyBrakeCount`: Total emergency brake interventions.
- `minimumSeparation`: The closest physical proximity recorded between any two trains during the run.
- `minimumTtc`: The lowest Time-To-Collision observed.
- `maximumHmiLatencyMs`: Maximum measured rendering latency in milliseconds.

### 4.4 Interactive Application Shell (`TcasApplication`)
Features a hierarchical menu structure:
- **`MainMenu`**:
  - `[1] Fleet Management`: Dispatch catalog routes, custom Dijkstra paths, list active trains, remove trains.
  - `[2] Safety & Interlocking`: View live active conflicts, space-time reservations, and arbitration decisions.
  - `[3] Fault Injection`: Toggle sensor degradation on individual trains, toggle wireless packet loss, clear all faults.
  - `[4] Simulation Controls`: Start, pause, resume, reset, or enter the Live Radar loop.
  - `[5] Live Radar View`: Full-screen auto-refreshing dashboard running at 5 Hz with single-keystroke escape.

---

## 5. Public API Reference

```cpp
namespace tcas::hmi {

struct PerformanceSnapshot {
    std::size_t collisionCount{ 0 };
    std::size_t nearMissCount{ 0 };
    std::size_t emergencyBrakeCount{ 0 };
    std::size_t conflictObservations{ 0 };
    double minimumSeparation{ 0.0 };
    double minimumTtc{ 0.0 };
    double maximumHmiLatencyMs{ 0.0 };
};

class HmiDisplay {
public:
    [[nodiscard]] static std::string format(const orchestrator::WorldState& state);
    static void render(const orchestrator::WorldState& state, std::ostream& output);
};

class TelemetryLogger {
public:
    explicit TelemetryLogger(std::string logDirectory = "logs");
    void log(const orchestrator::WorldState& state);
    void flush();
};

class PerformanceMetrics {
public:
    void observe(const orchestrator::WorldState& state,
                 std::chrono::steady_clock::duration hmiDuration);
    [[nodiscard]] PerformanceSnapshot snapshot() const;
};

} // namespace tcas::hmi

namespace tcas::app {

class TcasApplication {
public:
    TcasApplication();
    ~TcasApplication();
    int run();
};

} // namespace tcas::app
```

---

## 6. Execution Flow

```text
[Operator launches ./tcas]
           │
           ▼
TcasApplication::run()
           │
  ┌────────┴──────────────────────────────┐
  ▼                                       ▼
Interactive Menu Loop              Background Threads
├── 1. Dispatch Catalog Route R-07 ├── Physics Loop (20ms)
├── 2. Dispatch Catalog Route R-08 ├── Safety Pipeline Loop (100ms)
├── 3. Enter Live Radar View       ├── Comms Channel Loop (100ms)
│      (Refreshes at 5 Hz)         └── HMI Loop (200ms)
│                                         │
▼                                         ▼
Operator watches Train 1 cross       HmiDisplay formats frame
Junction while Train 2 halts         TelemetryLogger writes CSV
at red signal and auto-resumes!      Metrics updates KPIs
```

---

## 7. Edge Cases Handled

1. **Terminal Flicker Prevention**: HMI renders through double-buffering via `std::ostringstream` before streaming to `std::cout`, eliminating terminal screen flickering.
2. **Graceful Console Resizing**: Tables use fixed-width column clamping with automatic string truncation for long route or train names.
3. **Directory Creation Failures**: If `logs/` directory does not exist, `TelemetryLogger` automatically creates the path recursively using `std::filesystem`.
4. **Thread-Safe Snapshot Extraction**: Uses `std::shared_lock` so that rendering never blocks high-frequency physical train movement.

---

## 8. Automated Test Verification

Validated through 3 test suites under `tests/hmi/`:

- **`HmiDisplayTest.cpp`**: Tests formatting logic, column header alignment, empty fleet rendering, active conflict highlighting, and ANSI color escape generation.
- **`TelemetryLoggerTest.cpp`**: Tests file creation, CSV header correctness, continuous row appending, and clean file closure.
- **`PerformanceMetricsTest.cpp`**: Tests near-miss thresholds, minimum distance tracking, zero-collision accounting, and latency timing accuracy.

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Imagine the NASA Mission Control room in Houston or the big glass-walled dispatch center at a major central station:

The flight controllers and station masters don't look at lines of raw C++ code. They look at giant, colorful digital screens that show:
- Where is every train right now? (Train 1: Moving at 120 km/h; Train 2: Stopped at Red Signal).
- Are there any flashing red conflict warnings?
- Which tracks are reserved and locked?
- What are the flight controllers' buttons? (*"Dispatch Train"*, *"Pause Simulation"*, *"Test Radio Failure"*).

**Module 13 is the "Cockpit Dashboard, Mission Control Screen & Black Box Recorder" for TCAS.**

1. **The Dashboard (`HmiDisplay`)**: Puts a beautiful, easy-to-read command center right on your computer screen.
2. **The Black Box (`TelemetryLogger`)**: Like the flight data recorder on an airliner, it writes every single move, speed change, and brake command into a permanent log file on disk.
3. **The Scorecard (`PerformanceMetrics`)**: Keeps score of safety—proving that zero collisions occurred and measuring how close trains got to each other.
4. **The Pilot's Controls (`TcasApplication`)**: Gives the user an interactive menu to dispatch trains, create emergencies, and watch the system save the day in real time!
