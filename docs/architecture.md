# TCAS Master Architecture Directory

## Real-Time Train Collision Avoidance System (C++23)

Welcome to the technical architecture documentation for the **TCAS (Train Collision Avoidance System)** project.

---

## 1. Master System Specification

For an in-depth, end-to-end breakdown of the complete system workflow, real-time multi-threading, mathematical formulations, fault tolerance, and operational guide, please read:

👉 **[Integrated System Architecture & Operational Guide](integrated-system-architecture.md)**

---

## 2. The 12-Module Documentation Index

Each individual subsystem maintains its own dedicated technical documentation covering its mathematical models, component design, public API specifications, and testing verification:

1. **[Module 1 — Railway Infrastructure](module-01-railway-infrastructure/README.md)**
   - Topological network graph, Nodes (Stations, Junctions, Platforms), Directed Tracks (Gradients, Speed Limits).
2. **[Module 2 — Train Management](module-02-train-management/README.md)**
   - Train entity hierarchy, physical train parameters, Express/Passenger/Freight classes, and fleet management.
3. **[Module 3 — Simulation & Clock](module-03-simulation-clock/README.md)**
   - Discrete-step simulation clock (`SimClock`), fixed timesteps ($\Delta t = 20\text{ ms}$), and timing configuration.
4. **[Module 4 — Physics & Braking Dynamics](module-04-physics-braking/README.md)**
   - Numerical integration of motion, gradient-induced gravitational acceleration, and stopping distance formulas.
5. **[Module 5 — Route & Navigation](module-05-route-navigation/README.md)**
   - Dijkstra shortest-path route planning, route connectivity validation, and travel time estimation.
6. **[Module 6 — Sensor & State Estimation](module-06-sensor-state-estimation/README.md)**
   - Wheel-odometer simulation, sensor noise modeling, and fused position/velocity estimation with uncertainty envelopes.
7. **[Module 7 — Communication Simulation](module-07-communication-simulation/README.md)**
   - V2V and V2I wireless channel simulation, transmission delays, physical distance attenuation, and packet loss drop rates.
8. **[Module 8 — Trajectory Prediction](module-08-trajectory-prediction/README.md)**
   - Kinematic lookahead projection across multiple tracks, speed limit compliance, and linear uncertainty growth ($\sigma(t) = \sigma_0 + 0.5t$).
9. **[Module 9 — Conflict Detection & Resource Reservation](module-09-conflict-detection/README.md)**
   - Continuous analytical quadratic interval solver, Head-on, Rear-end, Junction, and Platform conflict detection, and temporal resource reservation leases.
10. **[Module 10 — Risk Assessment, Priority Engine & Resolution](module-10-risk-priority-resolution/README.md)**
    - Multi-factor risk scoring ($0-100$), operational priority hierarchy, max-heap conflict priority queue, and safety braking commands.
11. **[Module 11 — Operational Scenarios & Simulation Testing](module-11-scenario-simulation/README.md)**
    - The 8 standardized TCAS operational scenarios, benchmark topologies, fault injection, and automated testing.
12. **[Module 12 — Real-Time Orchestrator, HMI & Telemetry](module-12-realtime-orchestrator-hmi/README.md)**
    - Multi-threaded real-time executive (Physics, Safety, Comm, HMI, Input), thread-safe command queues, single-writer ANSI VT100 dashboard, and CSV telemetry streaming.

---

## 3. Technology Stack & Verification
- **Language Standard**: C++23 (`set(CMAKE_CXX_STANDARD 23)`)
- **Build System**: CMake 3.20+ / GNU Make / Ninja
- **Threading**: Native C++ `std::thread`, `std::shared_mutex`, POSIX `Threads::Threads`
- **Testing**: Google Test (293/293 tests passing, 100% pass rate)
- **Static Analysis**: Cppcheck C++23 (0 errors across 73 files)
