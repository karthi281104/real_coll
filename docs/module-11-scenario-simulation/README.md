# Module 11 — Scenario Simulation & Route Catalog

## 1. Module Overview

Module 11 is the test harness, standardized route catalog, and operational scenario simulation engine of the Train Collision Avoidance System (TCAS).

To validate that the TCAS safety pipeline works reliably under every conceivable real-world condition, the system needs reproducible, standardized operating scenarios. These range from benign disjoint train movements to catastrophic edge cases, such as two trains rushing head-on along a single line, a fast express overtaking a stalled freight train, dual trains converging at an interlocking junction, platform contention, sensor dropouts, wireless interference, and trains moving too fast to stop before a red signal.

Module 11 provides two key components:
1. **`RouteCatalog`**: A curated library of 10 pre-engineered operational routes across an 8-node railway network topology, enabling one-click train dispatching with known conflict properties.
2. **`ScenarioManager`**: An automated scenario orchestrator that instantiates, configures, and resets the railway network and fleet for 8 critical safety test scenarios.

---

## 2. Objectives

1. **Standardized 8-Node Benchmark Topology**: Maintain a realistic railway network topology comprising stations, mainline junctions, freight yards, and passenger platforms.
2. **Predefined Route Catalog (R-01 to R-10)**: Provide 10 catalog routes with documented conflict pairings (head-on, rear-end, junction, platform, and benign disjoint paths).
3. **Dynamic Dispatching Support**: Allow dispatching trains by catalog ID or custom point-to-point Dijkstra routing via a single unified API.
4. **Automated Conflict Scenario Generator**: Instantly configure 8 standard operational conflict scenarios with precise initial positions, velocities, and fault injections.
5. **Deterministic Reset & Teardown**: Cleanly reset network tracks, train fleets, and route tables between consecutive test runs without memory leaks.

---

## 3. Architectural Role & System Seams

```text
               User / CLI / Test Suite
                         │
        ┌────────────────┴────────────────┐
        ▼                                 ▼
  RouteCatalog                     ScenarioManager
  ├── 10 Catalog Routes            ├── 8 Conflict Scenarios
  ├── Dynamic Train Creation       ├── Network Topology Builder
  └── Route Table Setup            └── Live Fault Injection
        │                                 │
        └────────────────┬────────────────┘
                         │
                         ▼
           ┌───────────────────────────┐
           │    RailwayNetwork (Net)   │
           │    TrainManager (Fleet)   │
           │    SafetyPipeline (Routes)│
           └─────────────┬─────────────┘
                         │
                         ▼
        [ThreadOrchestrator Simulation Engine]
```

---

## 4. Benchmark Network Topology & Catalog Routes

### 4.1 The 8-Node Railway Topology
The benchmark topology connects 8 strategic railway locations:
- **Node 1**: `Central Station` (Major terminus)
- **Node 2**: `Alpha Junction` (Mainline convergent switch)
- **Node 3**: `Beta Junction` (Crossover switch)
- **Node 4**: `North Terminal` (Secondary terminus)
- **Node 5**: `South Harbor` (Harbor station)
- **Node 6**: `Freight Approach` (Yard entry branch)
- **Node 7**: `Freight Yard` (Cargo depot)
- **Node 8**: `Platform A` (Shared passenger platform)

### 4.2 Standard Route Catalog (R-01 to R-10)

| Route Code | Name | Default Train | Path (Nodes) | Initial Track & Pos | Initial Speed | Conflict Property |
|---|---|---|---|---|---|---|
| **R-01** | South Shore Express | `Express` | Central (1) $\to$ South Harbor (5) | T101 @ 100m | 30 m/s | Safe disjoint path (no collision) |
| **R-02** | North Corridor Commuter | `Passenger` | Beta (3) $\to$ North (4) | T103 @ 100m | 25 m/s | Safe disjoint path (no collision) |
| **R-03** | Eastbound Mainline Express | `Express` | Central (1) $\to$ Alpha (2) | T101 @ 200m | 30 m/s | **Head-On Conflict** with R-04 on Track 101/201 |
| **R-04** | Westbound Counter-Flow | `Passenger` | Alpha (2) $\to$ Central (1) | T201 @ 300m | 30 m/s | **Head-On Conflict** with R-03 on Track 101/201 |
| **R-05** | Slow Heavy Freight Lead | `Freight` | Central (1) $\to$ North (4) | T101 @ 700m | 15 m/s | **Rear-End Conflict** with R-06 (Lead train) |
| **R-06** | High-Speed Overtaker | `Express` | Central (1) $\to$ North (4) | T101 @ 100m | 35 m/s | **Rear-End Conflict** with R-05 (Overtaker) |
| **R-07** | Approach Yard Freight | `Freight` | Freight App (6) $\to$ Yard (7) | T105 @ 1500m | 20 m/s | **Junction Conflict** at Alpha Jct ($t=25\text{s}$) with R-08 |
| **R-08** | Alpha Converging Passenger | `Passenger` | Central (1) $\to$ North (4) | T101 @ 1375m | 25 m/s | **Junction Conflict** at Alpha Jct ($t=25\text{s}$) with R-07 |
| **R-09** | South Harbor Platform Feeder | `Passenger` | South Harbor (5) $\to$ Plat A (8) | T107 @ 100m | 16 m/s | **Platform Conflict** at Platform A ($t=25\text{s}$) with R-10 |
| **R-10** | Express Platform Arrival | `Express` | Central (1) $\to$ Plat A (8) | T101 @ 1200m | 32 m/s | **Platform Conflict** at Platform A ($t=25\text{s}$) with R-09 |

---

## 5. Standard Scenario Suite (`ScenarioManager`)

The `ScenarioManager` provides 8 pre-configured full operational scenarios:

1. **`JunctionConflict`**: Express and Freight trains converging on Alpha Junction. Tests priority arbitration (Express granted right-of-way, Freight held at signal).
2. **`RearEndConflict`**: High-speed Express trailing behind a slow Freight train on Track 101. Tests distance closing detection, speed reduction, and headway maintenance.
3. **`HeadOnConflict`**: Two trains dispatched in opposite directions along a single track corridor. Tests immediate high-risk detection, service braking, and emergency stop escalation.
4. **`PlatformConflict`**: Two commuter trains arriving at the same terminal platform at $t=25\text{s}$. Tests platform resource interlocking and station holding.
5. **`MultipleConflicts`**: Three trains operating simultaneously with overlapping conflicts (Junction conflict at Alpha + Rear-End conflict approaching North). Tests `ConflictPriorityQueue` multi-threat ordering.
6. **`SensorFailure`**: Degraded odometry and high noise injected into Train 1. Tests Kalman Filter health flagging and safety buffer inflation.
7. **`CommunicationFailure`**: Severe packet loss ($80\%$) injected into wireless channel. Tests degraded communications hysteresis and fail-safe cautionary speed reduction.
8. **`UnsafeStopping`**: Train dispatched at excessive speed with insufficient physical track distance remaining before an occupied junction. Tests feasibility check failure and immediate `EmergencyBrake`.

---

## 6. Public API Reference

```cpp
namespace tcas::scenario {

enum class ScenarioType {
    JunctionConflict,
    RearEndConflict,
    HeadOnConflict,
    PlatformConflict,
    MultipleConflicts,
    SensorFailure,
    CommunicationFailure,
    UnsafeStopping
};

struct ScenarioResult {
    std::vector<orchestrator::SafetyPipeline::TrainRoute> routes;
    std::string description;
    bool injectSensorFailure{ false };
    bool injectCommunicationFailure{ false };
};

class ScenarioManager {
public:
    ScenarioManager(
        infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager);

    [[nodiscard]] ScenarioResult load(ScenarioType scenario);
    [[nodiscard]] static const char* scenarioName(ScenarioType scenario) noexcept;
};

class RouteCatalog {
public:
    [[nodiscard]] static const std::vector<CatalogRouteInfo>& allRoutes() noexcept;
    [[nodiscard]] static const CatalogRouteInfo* findRoute(int catalogId) noexcept;
    [[nodiscard]] static DispatchedTrainResult dispatchCatalogRoute(
        int catalogId,
        const infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager,
        TrainId suggestedTrainId = 0);
    [[nodiscard]] static DispatchedTrainResult dispatchCustomRoute(
        NodeId sourceNode, NodeId destinationNode,
        TrainType type, SpeedMetersPerSecond speed,
        const infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager,
        TrainId suggestedTrainId = 0);
};

} // namespace tcas::scenario
```

---

## 7. Execution Flow

```text
User selects Menu Option: "Load Scenario: Junction Conflict"
                               │
                               ▼
            ScenarioManager::load(ScenarioType::JunctionConflict)
                               │
       ┌───────────────────────┴───────────────────────┐
       │ 1. Clear fleet from TrainManager              │
       │ 2. Build benchmark 8-node RailwayNetwork      │
       │ 3. Create Express Train 1 (v=25 m/s) on T101  │
       │ 4. Create Freight Train 2 (v=20 m/s) on T105  │
       │ 5. Calculate routes converging at Alpha Jct   │
       └───────────────────────┬───────────────────────┘
                               │
                               ▼
             Construct ScenarioResult:
             - routes: [TrainRoute 1, TrainRoute 2]
             - description: "Express and Freight converging at Alpha Jct"
                               │
                               ▼
        SafetyPipeline::setRoutes(result.routes)
        ThreadOrchestrator::start()
```

---

## 8. Automated Test Verification

Validated through `tests/scenario/RouteCatalogTest.cpp` and `tests/integration/Module8To12IntegrationTest.cpp`:

- Catalog lookup validity across all 10 catalog entries
- Automated dispatching of Express, Passenger, and Freight trains
- Custom route planning and parameter validation
- Full scenario initialization and state reset verification

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Imagine a flight simulator or a driving test center:

To get a driver's license, the test examiner doesn't just watch you drive down an empty street on a sunny afternoon. They test you on:
- Parallel parking between two cars
- An emergency stop when an obstacle pops up
- Merging onto a busy highway during rush hour
- Driving through heavy rain or fog

**Module 11 is the "Flight Simulator & Scenario Director" for TCAS.**

It comes pre-loaded with an entire city railway network and 10 ready-to-go train routes. With a single click, it can set up dramatic test scenarios:
- *"What if two trains head straight toward each other on Track 1?"*
- *"What if a fast bullet train speeds up behind a slow cargo train?"*
- *"What if two trains arrive at the same switch at the exact same second?"*
- *"What if a storm knocks out the radio antennas?"*

By running these standardized test simulations over and over, engineers prove beyond any shadow of a doubt that the safety software will protect human lives in every emergency.
