# Module 5 — Route Planning & Graph Navigation

## 1. Module Overview

Module 5 is the route planning and pathfinding engine of the Train Collision Avoidance System (TCAS).

In a complex railway topology comprising interconnected main lines, passing sidings, crossover junctions, and terminus stations, trains must find valid, optimal paths from their dispatch origin to their target destination. Railway networks are strictly directed graphs: trains cannot reverse direction arbitrarily on one-way mainlines, and tracks possess fixed physical distances.

Module 5 implements Dijkstra's shortest-path algorithm operating directly on the `RailwayNetwork` graph, returning an ordered sequence of directed tracks and computing the total travel distance in metres.

---

## 2. Objectives

1. **Shortest Path Computation**: Implement Dijkstra's algorithm to determine the minimum physical distance route between any two nodes.
2. **Directed Graph Adherence**: Strictly respect track directionality (a track from Node A to Node B cannot be traversed backwards from B to A unless a corresponding reverse track exists).
3. **Turn-by-Turn Track Reconstruction**: Produce an ordered sequence of `TrackId` identifiers that subsequent predictive and simulation engines can follow segment by segment.
4. **Failure Diagnostics**: Provide explicit, actionable failure reasons (`SameNode`, `NodeNotFound`, `NoPathExists`) when a route cannot be established.
5. **Stateless & Thread-Safe**: Design `RouteNavigator` as a stateless, reentrant engine safe for concurrent queries across multiple train dispatch threads.

---

## 3. Architectural Role & System Seams

The `RouteNavigator` connects network infrastructure with train dispatching and trajectory projection:

```text
  RailwayNetwork (Graph) + Origin Node + Destination Node
                             │
                             ▼
                   RouteNavigator (Dijkstra)
                             │
                             ▼
                        RouteResult
             ├── success (bool)
             ├── tracks (std::vector<TrackId>)
             ├── totalDistance (DistanceMeters)
             └── reason (FailReason)
                             │
       ┌─────────────────────┼─────────────────────┐
       ▼                     ▼                     ▼
[Train Dispatcher]    [PredictionEngine]     [SafetyPipeline]
 (Assigns itinerary   (Projects trajectory   (Detects spatial/temporal
  to new train)        along track sequence)  overlap along route)
```

---

## 4. Algorithm & Implementation Details

### 4.1 Dijkstra's Shortest Path Algorithm
`RouteNavigator::findRoute` executes a min-heap Dijkstra search:

1. **Initialization**:
   - Maintains a distance map `dist[NodeId] = infinity` (with `dist[origin] = 0.0`).
   - Maintains a predecessor map `parent[NodeId] = {prevNodeId, connectingTrackId}`.
   - Uses `std::priority_queue` storing pairs of `(distance, NodeId)` ordered by smallest distance.
2. **Exploration**:
   - Pops the node $u$ with minimal tentative distance.
   - If $u == \text{destination}$, early exit is triggered.
   - For each outgoing track $(u, v)$ from $u$'s adjacency list in `RailwayNetwork`:
     $$\text{newDist} = \text{dist}[u] + \text{track.length()}$$
     If $\text{newDist} < \text{dist}[v]$:
     - Update $\text{dist}[v] = \text{newDist}$.
     - Record predecessor $\text{parent}[v] = \{u, \text{track.id()}\}$.
     - Push `(newDist, v)` into the priority queue.
3. **Path Reconstruction**:
   - Backtracks from destination node to origin node using the `parent` map.
   - Reverses the accumulated list of `TrackId`s to produce the forward-ordered path.
   - Sums track lengths to obtain `totalDistance`.

### 4.2 Time & Space Complexity
For a railway network with $V$ nodes and $E$ tracks:
- **Time Complexity**: $O((V + E) \log V)$ using binary min-heap.
- **Space Complexity**: $O(V)$ auxiliary storage for distance and predecessor maps.

---

## 5. Public API Reference

```cpp
namespace tcas::navigation {

struct RouteResult {
    enum class FailReason {
        None,         // Path successfully found
        SameNode,     // Origin == Destination
        NodeNotFound, // Origin or destination does not exist in RailwayNetwork
        NoPathExists  // Graph is disconnected or destination unreachable
    };

    bool success{ false };
    std::vector<TrackId> tracks{};
    DistanceMeters totalDistance{ 0.0 };
    FailReason reason{ FailReason::None };
};

class RouteNavigator {
public:
    RouteNavigator() = default;

    [[nodiscard]] static RouteResult findRoute(
        const infrastructure::RailwayNetwork& network,
        NodeId origin,
        NodeId destination
    );
};

} // namespace tcas::navigation
```

---

## 6. Execution & Data Flow

```text
User / Catalog / Dispatcher requests Route: Node 1 -> Node 6
                           │
                           ▼
             RouteNavigator::findRoute(net, 1, 6)
                           │
                 [Validate Origin & Dest]
                 ├── Node 1 or 6 missing? ──► Return FailReason::NodeNotFound
                 └── Node 1 == Node 6?    ──► Return FailReason::SameNode
                           │
                 [Execute Dijkstra Search]
                 ├── Min-heap priority queue
                 ├── Relax track distances
                 └── Destination reached?
                           ├── NO  ──► Return FailReason::NoPathExists
                           └── YES ──► Backtrack parent map
                                            │
                                            ▼
                           Construct RouteResult:
                           - success: true
                           - tracks: [T101, T104, T107]
                           - totalDistance: 3200.0 m
                           - reason: FailReason::None
```

---

## 7. Edge Cases Handled

1. **Same Node Origin & Destination**: If `origin == destination`, the search immediately returns `success = false` with `FailReason::SameNode`, preventing 0-length circular routes.
2. **Missing Nodes**: If either origin or destination ID is not found in the `RailwayNetwork`, returns `FailReason::NodeNotFound` without throwing exceptions or causing null pointer dereferences.
3. **Unreachable / Disconnected Destination**: When no directed track sequence connects origin to destination (e.g., disconnected island or opposite one-way line), the algorithm cleanly drains the queue and returns `FailReason::NoPathExists`.
4. **Parallel Tracks**: If two distinct tracks connect the same two nodes, Dijkstra naturally selects the shorter track.
5. **Directed One-Way Enforcement**: Tracks with opposite directions are treated strictly as distinct edges.

---

## 8. Automated Test Verification

Validated through `tests/navigation/RouteNavigatorTest.cpp` and `tests/integration/PhysicsNavigationIntegrationTest.cpp`:

- Single-hop and multi-hop route finding
- Shortest route selection when multiple alternative paths exist
- Error detection for non-existent origin/destination
- Disconnected network handling
- Directed track enforcement (verifying reverse traversal is prohibited)
- Route reconstruction order correctness

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Think of Google Maps or Waze on your smartphone:

When you enter your current location and where you want to go, Google Maps scans the road network, checks one-way streets, measures the distance along every street, and gives you the shortest, turn-by-turn route: *"Take Main Street for 2 km, turn right onto Industrial Way for 1 km, then enter Station Yard."*

**Module 5 is the "GPS Navigation App" for every train on the tracks.**

When a train is dispatched from Central Station to North Port:
1. It looks at the master railway map (Module 1).
2. It checks every possible connection, strictly obeying one-way track signs.
3. It finds the absolute shortest track path to reach the destination.
4. It hands the train driver a numbered list of track segments to follow (`Track 101 -> Track 104 -> Track 107`) and tells them the total trip distance (e.g., 3,200 metres).
5. If someone requests a route between two stations that aren't connected by tracks, it politely replies: *"No path exists!"*
