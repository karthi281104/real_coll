# Module 1 — Railway Infrastructure

## 1. Module Overview

The Railway Infrastructure module forms the foundation of the Real-Time Train Collision Avoidance System (TCAS).

The module models the physical railway network as a directed graph. Railway locations are represented as nodes, while railway tracks are represented as directed edges connecting those nodes.

This module provides the infrastructure representation required by later TCAS modules such as train navigation, predictive position projection, conflict detection, junction reservation, and collision avoidance.

The module also provides graph-analysis algorithms for validating the railway network:

- Breadth-First Search (BFS) for connectivity/reachability validation.
- Depth-First Search (DFS) for cycle detection.

---

## 2. Objectives

The objectives of Module 1 are:

1. Represent railway infrastructure using object-oriented C++ classes.
2. Model railway locations as graph nodes.
3. Model railway tracks as directed graph edges.
4. Represent stations, junctions, and platforms.
5. Maintain railway nodes and tracks in a central `RailwayNetwork`.
6. Validate track references before adding tracks.
7. Prevent duplicate node and track identifiers.
8. Validate railway network connectivity.
9. Detect cycles in the railway graph.
10. Provide a reliable foundation for subsequent TCAS modules.
11. Establish GoogleTest-based automated testing for the module.

---

## 3. Module Responsibilities

Module 1 is responsible for:

- Node management
- Track management
- Station representation
- Junction representation
- Platform representation
- Graph construction
- Adjacency-list maintenance
- Network reachability validation
- Cycle detection
- Basic infrastructure data validation

The module is not responsible for:

- Train movement
- Train physics
- Braking calculations
- Train route optimization
- Dijkstra's algorithm
- Sensor simulation
- Communication simulation
- Future position prediction
- Collision detection between trains
- Risk assessment
- Conflict resolution
- Multithreading
- HMI

Those responsibilities belong to later modules.

---

## 4. Infrastructure Model

The railway network is represented as a directed graph:

```text
Node ── Track ──> Node
```

For example:

```text
Station A
    │
    │ Track T1
    ▼
Junction J1
    │
    │ Track T2
    ▼
Station B
```

The corresponding graph is:

```text
A → J1 → B
```

A track is not automatically treated as bidirectional.

If a physical railway track supports movement in both directions, the directionality will be explicitly represented in the infrastructure model rather than being assumed.

This design is important for later head-on conflict detection and train route planning.

---

# 5. Node

The `Node` class represents a location in the railway graph.

A node contains:

- Node ID
- Node name
- Node type

Supported node types are:

```text
Generic
Station
Junction
Platform
```

### Node Interface

```cpp
Node(
    NodeId id,
    std::string name,
    NodeType type
);
```

The node identifier must uniquely identify the node within the railway network.

---

# 6. Track

The `Track` class represents a directed railway connection between two nodes.

A track contains:

- Track ID
- Source node
- Destination node
- Track length
- Speed limit
- Gradient

Conceptually:

```text
Source Node ───────────────> Destination Node
             Track
```

### Track Properties

| Property | Description |
|---|---|
| ID | Unique track identifier |
| Source | Starting node |
| Destination | Ending node |
| Length | Track length in metres |
| Speed Limit | Maximum permitted speed |
| Gradient | Track gradient |

A track can only be added when both its source and destination nodes already exist.

---

# 7. Station

The `Station` class represents a railway station.

A station contains:

- Station/node identifier
- Station name

Stations are associated with infrastructure nodes and will later be used by the navigation and train-management modules.

---

# 8. Junction

The `Junction` class represents a railway junction.

A junction is an important infrastructure resource because multiple train routes may converge on or pass through the same junction.

Example:

```text
              Track A
                 │
                 ▼
Track B ─────► J1 ─────► Track C
```

Junctions become particularly important in later modules for:

- Conflict-zone identification
- Junction reservation
- Train priority
- Predictive conflict resolution

Module 1 only models the junction. It does not perform train arbitration.

---

# 9. Platform

The `Platform` class represents a station platform.

A platform contains:

- Platform ID
- Associated station/node ID
- Platform name

Platform resources will later be used when detecting situations where two trains are predicted to occupy the same platform or approach the same platform through a shared junction.

---

# 10. RailwayNetwork

`RailwayNetwork` is the central class of Module 1.

It maintains:

```text
Nodes
Tracks
Adjacency List
```

The implementation uses:

```cpp
std::unordered_map<NodeId, Node>
std::unordered_map<TrackId, Track>
std::unordered_map<NodeId, std::vector<NodeId>>
```

The adjacency list represents outgoing railway connections.

For:

```text
A → B
B → C
```

the adjacency representation is conceptually:

```text
A → [B]
B → [C]
C → []
```

---

# 11. Node Validation

When a node is added, its identifier must be unique.

For example:

```text
Node 1 → accepted
Node 2 → accepted
Node 1 → rejected
```

Duplicate node identifiers are therefore prevented.

---

# 12. Track Validation

A track is accepted only when:

1. The track ID does not already exist.
2. The source node exists.
3. The destination node exists.
4. The track length is greater than zero.
5. The speed limit is not negative.

Example:

```text
Track 100
Source: 1
Destination: 2
Length: 1000 m
Speed Limit: 30 m/s
```

is valid when nodes `1` and `2` exist.

---

# 13. BFS — Connectivity Validation

Breadth-First Search is used to determine whether all railway nodes are reachable from the selected starting node.

Conceptually:

```text
Start
  │
  ▼
Queue
  │
  ▼
Visit Node
  │
  ▼
Visit Neighbours
  │
  ▼
Continue Until Queue Empty
```

The implementation maintains a visited set to prevent repeated traversal.

The number of visited nodes is compared against the total number of nodes.

If all nodes are reached:

```text
Network → Connected
```

Otherwise:

```text
Network → Disconnected
```

### Important Design Note

The current implementation evaluates reachability from one starting node. Therefore, this is an operational reachability check rather than a mathematical strong-connectivity test.

Strong connectivity may be considered separately if required by the final railway-network specification.

---

# 14. DFS — Cycle Detection

Depth-First Search is used to detect cycles in the directed railway graph.

The implementation maintains:

```text
Visited
Recursion Stack
```

A cycle is detected when DFS encounters a node that is already present in the current recursion stack.

Example:

```text
A → B
↑   │
│   ▼
└── C
```

This produces:

```text
A → B → C → A
```

Therefore:

```text
Cycle detected = true
```

---

# 15. Algorithm Complexity

For a graph containing:

- `V` nodes
- `E` tracks

the graph traversal algorithms have the following expected complexity:

| Algorithm | Time Complexity | Space Complexity |
|---|---:|---:|
| BFS | O(V + E) | O(V) |
| DFS | O(V + E) | O(V) |

Node and track lookup using `std::unordered_map` has expected constant-time lookup:

```text
O(1) average
```

---

# 16. Error Handling Strategy

Module 1 uses boolean return values for insertion operations.

Example:

```cpp
bool addNode(const Node& node);
bool addTrack(const Track& track);
```

A successful operation returns:

```text
true
```

An invalid or duplicate operation returns:

```text
false
```

This provides a simple interface for the current PoC.

Later modules can build higher-level validation and diagnostic reporting on top of this foundation.

---

# 17. GoogleTest Strategy

GoogleTest is the mandatory testing framework for the TCAS project.

Module 1 tests cover:

- Node creation
- Node properties
- Node types
- Track properties
- Empty network
- Node insertion
- Duplicate node rejection
- Track insertion
- Missing source node
- Missing destination node
- Duplicate track rejection
- Node lookup
- Missing node lookup
- Track lookup
- Empty network connectivity
- Single-node connectivity
- Connected graph
- Disconnected graph
- Acyclic graph
- Cyclic graph

---

# 18. Test Results

Module 1 currently contains:

```text
17 automated tests
```

Test execution:

```text
17/17 tests passed
100% tests passed
```

CTest execution time:

```text
0.40 seconds
```

Build environment:

```text
Compiler: Microsoft Visual C++ 19.42.34435.0
IDE/Build Tools: Visual Studio 2022 Build Tools
CMake: 4.4
C++ Standard: C++23
Test Framework: GoogleTest 1.17.0
Operating System: Windows
```

---

# 19. Module 1 Test Summary

| Test Area | Result |
|---|---|
| Node creation | PASS |
| Node types | PASS |
| Track creation | PASS |
| Empty network | PASS |
| Node insertion | PASS |
| Duplicate node handling | PASS |
| Track insertion | PASS |
| Invalid source handling | PASS |
| Invalid destination handling | PASS |
| Duplicate track handling | PASS |
| Node lookup | PASS |
| Track lookup | PASS |
| BFS connected graph | PASS |
| BFS disconnected graph | PASS |
| DFS acyclic graph | PASS |
| DFS cyclic graph | PASS |
| Single-node network | PASS |

---

# 20. Module 1 Limitations

The following functionality is intentionally deferred:

### Dijkstra

Route calculation is part of Module 5.

### Train-aware infrastructure occupancy

Train occupancy will be introduced by later modules.

### Dynamic conflict zones

Conflict zones involving trains will be implemented in the safety subsystem.

### Junction reservation

Junction reservation is implemented conceptually in the architecture but actual train-resource arbitration belongs to the later safety modules.

### Strong connectivity

The current BFS implementation performs reachability validation from one starting node. Strongly connected component analysis is not currently implemented.

---

# 21. Future Integration

Module 1 provides infrastructure information to later modules.

The expected dependency chain is:

```text
Module 1
Railway Infrastructure
        │
        ├──────────────► Module 2
        │               Train Management
        │
        └──────────────► Module 5
                        Route & Navigation
                              │
                              ▼
                         Module 8
                    Prediction Engine
                              │
                              ▼
                         Module 9
                    Conflict Detection
                              │
                              ▼
                        Module 10
                  Risk & Resolution
```

---

# 22. Completion Criteria

Module 1 is considered complete when:

- [x] Node implemented
- [x] Track implemented
- [x] Station implemented
- [x] Junction implemented
- [x] Platform implemented
- [x] RailwayNetwork implemented
- [x] Adjacency list implemented
- [x] BFS implemented
- [x] DFS implemented
- [x] GoogleTest integrated
- [x] Unit tests implemented
- [x] Edge-case tests implemented
- [x] Project builds successfully
- [x] 17/17 tests pass
- [x] Module documentation created

---

# 23. Module Status

**STATUS: COMPLETE — BASELINE IMPLEMENTATION**

Module 1 successfully establishes the railway infrastructure foundation required by the subsequent TCAS modules.

The implementation has been compiled using C++23 and validated using GoogleTest/CTest with all current tests passing.

---

# 24. Layman's Terms Description (What Does This Module Do?)

Imagine you are building a giant model train set or drawing a map for an entire national railway system. Before any train can run, you need the actual tracks, stations, junctions (where tracks split or merge), and station platforms laid down on the ground.

**Module 1 is the "Digital Blueprint" of the railway network.**

- **Nodes** are the dots on the map: towns, stations, junctions, or platforms.
- **Tracks** are the one-way steel rails connecting those dots, with rules like length and maximum safe speed limits.
- **The Railway Network** is the master map that keeps track of every single piece of steel and every station, ensuring no two tracks have the same ID and that tracks only connect to stations that actually exist.
- **The Path Checkers (BFS & DFS)** are like safety inspectors walking the line before opening day. They check: "Can a train actually get from Station A to Station B?" (reachability) and "Are there any infinite circular loops that could trap a train or confuse navigation?" (cycle detection).

Without this module, the trains would have nowhere to run, no speed limits to obey, and no physical map to follow.