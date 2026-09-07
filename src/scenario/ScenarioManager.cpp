#include "scenario/ScenarioManager.hpp"

#include "infrastructure/Node.hpp"
#include "infrastructure/Track.hpp"
#include "navigation/RouteNavigator.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"

#include <memory>

namespace tcas::scenario
{

// -----------------------------------------------------------------------
// Common network topology
//
//  [1] Central ──T101──> [2] Alpha Jct ──T102──> [3] Beta Jct ──T103──> [4] North
//                             |                        |
//                            T104                    T106
//                             |                        |
//                            [5] South              [7] Freight Yard
//                            T105                   T107
//                             |                        |
//                         (reverse)                 [6] Freight Approach ──T108──> [2]
// -----------------------------------------------------------------------

ScenarioManager::ScenarioManager(
    infrastructure::RailwayNetwork& network,
    train::TrainManager& trainManager)
    : network_(network),
      trainManager_(trainManager)
{
}

void ScenarioManager::buildBaseNetwork()
{
    using namespace tcas::infrastructure;

    // Clear everything
    network_ = RailwayNetwork{};
    trainManager_.clear();

    // Nodes
    network_.addNode(Node(1, "Central Station",   NodeType::Station));
    network_.addNode(Node(2, "Alpha Junction",    NodeType::Junction));
    network_.addNode(Node(3, "Beta Junction",     NodeType::Junction));
    network_.addNode(Node(4, "North Terminal",    NodeType::Station));
    network_.addNode(Node(5, "South Harbor",      NodeType::Station));
    network_.addNode(Node(6, "Freight Approach",  NodeType::Generic));
    network_.addNode(Node(7, "Freight Yard",      NodeType::Generic));
    network_.addNode(Node(8, "Platform A",        NodeType::Platform));

    // Directed tracks
    //  Central -> Alpha
    network_.addTrack(infrastructure::Track(101, 1, 2, 2000.0, 35.0, 0.000));
    //  Alpha -> Beta
    network_.addTrack(infrastructure::Track(102, 2, 3, 1500.0, 30.0, 0.020));
    //  Beta -> North
    network_.addTrack(infrastructure::Track(103, 3, 4, 2500.0, 40.0, -0.015));
    //  Alpha -> South
    network_.addTrack(infrastructure::Track(104, 2, 5, 3000.0, 25.0, 0.010));
    //  Freight Approach -> Alpha
    network_.addTrack(infrastructure::Track(105, 6, 2, 2000.0, 25.0, 0.000));
    //  Alpha -> Freight Yard
    network_.addTrack(infrastructure::Track(106, 2, 7, 1800.0, 25.0, 0.000));
    //  South -> Platform A (for platform conflict scenario)
    network_.addTrack(infrastructure::Track(107, 5, 8, 500.0, 20.0, 0.000));
    //  Beta -> Platform A (second approach to same platform)
    network_.addTrack(infrastructure::Track(108, 3, 8, 600.0, 20.0, 0.000));
}

// -----------------------------------------------------------------------
// Junction Conflict
// -----------------------------------------------------------------------
//  Express:  Central(1) -> Alpha(2) -> North(4)   on T101, position near end
//  Freight:  Freight Approach(6) -> Alpha(2) -> Yard(7)  near end of T105
//  Both converge on Alpha Junction node 2 simultaneously.
ScenarioResult ScenarioManager::loadJunctionConflict()
{
    buildBaseNetwork();

    trainManager_.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    trainManager_.addTrain(std::make_unique<train::FreightTrain>(
        3, 120000.0, 22.2, 0.5, 0.8));

    auto* express = trainManager_.getTrain(1);
    auto* freight = trainManager_.getTrain(3);

    // Place both trains near their junction approach boundary
    express->setPosition(1800.0);   // 200 m before Alpha on T101
    express->setVelocity(20.0);
    express->setAcceleration(0.0);

    freight->setPosition(1800.0);   // 200 m before Alpha on T105
    freight->setVelocity(15.0);
    freight->setAcceleration(0.0);

    const auto expressRoute =
        navigation::RouteNavigator::findRoute(network_, 1, 4);
    const auto freightRoute =
        navigation::RouteNavigator::findRoute(network_, 6, 7);

    ScenarioResult result;
    result.routes = {
        { 1, 101, expressRoute },
        { 3, 105, freightRoute }
    };
    result.description =
        "JUNCTION CONFLICT: Express #1 (Central->North) and Freight #3\n"
        "(Freight Approach->Yard) both converge on Alpha Junction (Node 2).\n"
        "Express has priority. Freight should REDUCE SPEED or HOLD.";
    return result;
}

// -----------------------------------------------------------------------
// Rear-End Conflict
// -----------------------------------------------------------------------
//  Train 1 (Passenger): T101, position 1000 m, slower
//  Train 2 (Express):   T101, position 600 m, faster — closing from behind
ScenarioResult ScenarioManager::loadRearEndConflict()
{
    buildBaseNetwork();

    trainManager_.addTrain(std::make_unique<train::PassengerTrain>(
        1, 60000.0, 33.3, 0.8, 1.2));
    trainManager_.addTrain(std::make_unique<train::ExpressTrain>(
        2, 45000.0, 45.0, 0.9, 1.4));

    auto* passenger = trainManager_.getTrain(1);
    auto* express   = trainManager_.getTrain(2);

    passenger->setPosition(1200.0);   // ahead on T101
    passenger->setVelocity(10.0);     // slow
    passenger->setAcceleration(0.0);

    express->setPosition(600.0);      // behind on T101
    express->setVelocity(25.0);       // fast — will close
    express->setAcceleration(0.0);

    const auto route = navigation::RouteNavigator::findRoute(network_, 1, 4);

    ScenarioResult result;
    result.routes = {
        { 1, 101, route },
        { 2, 101, route }
    };
    result.description =
        "REAR-END CONFLICT: Express #2 (25 m/s) closing on Passenger #1\n"
        "(10 m/s) on Track 101. Express has higher priority but will close\n"
        "the gap. Risk engine should detect and resolve.";
    return result;
}

// -----------------------------------------------------------------------
// Head-On Conflict
// -----------------------------------------------------------------------
//  Two trains approaching each other on the SAME track (T101) from opposite
//  directions: Express #1 moving forward (+25 m/s) from position 200m,
//  and Freight #3 moving in reverse (-15 m/s) from position 1800m.
//  The ConflictDetector classifies opposite velocities / closing displacements
//  as HEAD-ON, triggering critical emergency braking.
ScenarioResult ScenarioManager::loadHeadOnConflict()
{
    buildBaseNetwork();

    trainManager_.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    trainManager_.addTrain(std::make_unique<train::FreightTrain>(
        3, 120000.0, 22.2, 0.5, 0.8));

    auto* express = trainManager_.getTrain(1);
    auto* freight = trainManager_.getTrain(3);

    // Express far behind on T101, moving at high speed
    express->setPosition(100.0);
    express->setVelocity(35.0);
    express->setAcceleration(0.0);

    // Freight ahead on T101, moving slowly — critical closing conflict
    freight->setPosition(700.0);
    freight->setVelocity(5.0);
    freight->setAcceleration(0.0);

    const auto expressRoute = navigation::RouteNavigator::findRoute(network_, 1, 4);
    const auto freightRoute = navigation::RouteNavigator::findRoute(network_, 1, 4);

    ScenarioResult result;
    result.routes = {
        { 1, 101, expressRoute },
        { 3, 101, freightRoute }
    };
    result.description =
        "CRITICAL CLOSING CONFLICT: Express #1 (35 m/s) closing rapidly on Freight #3\n"
        "(5 m/s) on Track 101 — relative closing speed 30 m/s. Stopping distance exceeded;\n"
        "immediate emergency braking commanded.";
    return result;
}

// -----------------------------------------------------------------------
// Platform Conflict
// -----------------------------------------------------------------------
//  Two trains approaching Platform A (Node 8) from different tracks:
//  Train 1 via T102->T108, Train 2 via T104->T107
ScenarioResult ScenarioManager::loadPlatformConflict()
{
    buildBaseNetwork();

    trainManager_.addTrain(std::make_unique<train::PassengerTrain>(
        1, 60000.0, 33.3, 0.8, 1.2));
    trainManager_.addTrain(std::make_unique<train::PassengerTrain>(
        2, 60000.0, 33.3, 0.8, 1.2));

    auto* p1 = trainManager_.getTrain(1);
    auto* p2 = trainManager_.getTrain(2);

    p1->setPosition(1400.0);   // on T102 (Alpha->Beta, length 1500m, leads to T108->Node 8)
    p1->setVelocity(15.0);
    p1->setAcceleration(0.0);

    p2->setPosition(2500.0);   // on T104 (Alpha->South, length 3000m, leads to T107->Node 8)
    p2->setVelocity(15.0);
    p2->setAcceleration(0.0);

    navigation::RouteResult route1;
    route1.success = true;
    route1.tracks = { 102, 108 };
    route1.totalDistance = 2100.0;
    route1.reason = navigation::RouteResult::FailReason::None;

    navigation::RouteResult route2;
    route2.success = true;
    route2.tracks = { 104, 107 };
    route2.totalDistance = 3500.0;
    route2.reason = navigation::RouteResult::FailReason::None;

    ScenarioResult result;
    result.routes = {
        { 1, 102, route1 },
        { 2, 104, route2 }
    };
    result.description =
        "PLATFORM CONFLICT: Two Passenger trains approaching Platform A (Node 8)\n"
        "simultaneously via independent approach tracks (T102->T108 vs T104->T107).\n"
        "Train #1 has lower ID and priority; Train #2 is held at signal.";
    return result;
}

// -----------------------------------------------------------------------
// Multiple Simultaneous Conflicts
// -----------------------------------------------------------------------
//  Three trains create two overlapping conflicts:
//    Express(1) vs Freight(3) at Alpha Junction
//    Passenger(2) vs Freight(3) at Alpha Junction
//  ConflictPriorityQueue processes highest risk first.
ScenarioResult ScenarioManager::loadMultipleConflicts()
{
    buildBaseNetwork();

    trainManager_.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    trainManager_.addTrain(std::make_unique<train::PassengerTrain>(
        2, 60000.0, 33.3, 0.8, 1.2));
    trainManager_.addTrain(std::make_unique<train::FreightTrain>(
        3, 120000.0, 22.2, 0.5, 0.8));

    auto* express   = trainManager_.getTrain(1);
    auto* passenger = trainManager_.getTrain(2);
    auto* freight   = trainManager_.getTrain(3);

    express->setPosition(1900.0);   // 100 m before Alpha on T101
    express->setVelocity(22.0);
    express->setAcceleration(0.0);

    passenger->setPosition(1750.0); // 250 m before Alpha on T101 (behind express)
    passenger->setVelocity(18.0);
    passenger->setAcceleration(0.0);

    freight->setPosition(1900.0);   // 100 m before Alpha on T105
    freight->setVelocity(15.0);
    freight->setAcceleration(0.0);

    const auto expressRoute   = navigation::RouteNavigator::findRoute(network_, 1, 4);
    const auto passengerRoute = navigation::RouteNavigator::findRoute(network_, 1, 4);
    const auto freightRoute   = navigation::RouteNavigator::findRoute(network_, 6, 7);

    ScenarioResult result;
    result.routes = {
        { 1, 101, expressRoute   },
        { 2, 101, passengerRoute },
        { 3, 105, freightRoute   }
    };
    result.description =
        "MULTIPLE CONFLICTS: Express #1 + Passenger #2 + Freight #3.\n"
        "Two junction conflicts at Alpha (Node 2) detected simultaneously.\n"
        "ConflictPriorityQueue processes Express vs Freight first (higher TTC risk),\n"
        "then Passenger vs Freight. Priority: Express > Passenger > Freight.";
    return result;
}

// -----------------------------------------------------------------------
// Sensor Failure
// -----------------------------------------------------------------------
//  Same as junction conflict, but with sensor failure flag set.
//  Risk engine applies degraded sensor confidence -> higher risk score.
ScenarioResult ScenarioManager::loadSensorFailure()
{
    auto result = loadJunctionConflict();
    result.injectSensorFailure = true;
    result.description =
        "SENSOR FAILURE: Junction conflict scenario (Express vs Freight at Alpha)\n"
        "with sensor failure injected on Train #3.\n"
        "Risk engine applies degraded sensor confidence (0.5).\n"
        "Expected: higher risk score, more conservative resolution.";
    return result;
}

// -----------------------------------------------------------------------
// Communication Failure
// -----------------------------------------------------------------------
//  Same as junction conflict, but with communication failure flag set.
ScenarioResult ScenarioManager::loadCommunicationFailure()
{
    auto result = loadJunctionConflict();
    result.injectCommunicationFailure = true;
    result.description =
        "COMMUNICATION FAILURE: Junction conflict scenario with high packet loss.\n"
        "Risk engine applies degraded communication confidence (0.4).\n"
        "Expected: higher risk score, emergency braking more likely.";
    return result;
}

// -----------------------------------------------------------------------
// Unsafe Stopping Distance
// -----------------------------------------------------------------------
//  Express at very high speed, very close to junction — cannot stop in time.
//  brakingDistance > availableDistance -> brakingFeasible = false
//  -> ResolutionEngine issues EmergencyBrake regardless of priority.
ScenarioResult ScenarioManager::loadUnsafeStopping()
{
    buildBaseNetwork();

    trainManager_.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    trainManager_.addTrain(std::make_unique<train::FreightTrain>(
        3, 120000.0, 22.2, 0.5, 0.8));

    auto* express = trainManager_.getTrain(1);
    auto* freight = trainManager_.getTrain(3);

    // Express fast approach to junction with insufficient stopping distance
    // (at 40 m/s, emergency stopping distance ~571m >> 200m available)
    express->setPosition(1800.0);   // 200 m before Alpha on T101
    express->setVelocity(40.0);     // near max — braking distance >> 200 m
    express->setAcceleration(0.0);

    freight->setPosition(1900.0);   // 100 m before Alpha on T105
    freight->setVelocity(15.0);
    freight->setAcceleration(0.0);

    const auto expressRoute = navigation::RouteNavigator::findRoute(network_, 1, 4);
    const auto freightRoute = navigation::RouteNavigator::findRoute(network_, 6, 7);

    ScenarioResult result;
    result.routes = {
        { 1, 101, expressRoute },
        { 3, 105, freightRoute }
    };
    result.description =
        "UNSAFE STOPPING: Express #1 at 40 m/s with only 200 m before Alpha.\n"
        "Emergency braking distance (571m) >> available distance (200m).\n"
        "brakingFeasible = false -> EMERGENCY BRAKE regardless of priority.\n"
        "Safety overrides operational priority in all cases.";
    return result;
}

// -----------------------------------------------------------------------
// Public interface
// -----------------------------------------------------------------------

ScenarioResult ScenarioManager::load(ScenarioType scenario)
{
    switch (scenario)
    {
    case ScenarioType::JunctionConflict:
        return loadJunctionConflict();
    case ScenarioType::RearEndConflict:
        return loadRearEndConflict();
    case ScenarioType::HeadOnConflict:
        return loadHeadOnConflict();
    case ScenarioType::PlatformConflict:
        return loadPlatformConflict();
    case ScenarioType::MultipleConflicts:
        return loadMultipleConflicts();
    case ScenarioType::SensorFailure:
        return loadSensorFailure();
    case ScenarioType::CommunicationFailure:
        return loadCommunicationFailure();
    case ScenarioType::UnsafeStopping:
        return loadUnsafeStopping();
    }
    return {};
}

const char* ScenarioManager::scenarioName(ScenarioType scenario) noexcept
{
    switch (scenario)
    {
    case ScenarioType::JunctionConflict:      return "Junction Conflict";
    case ScenarioType::RearEndConflict:       return "Rear-End Conflict";
    case ScenarioType::HeadOnConflict:        return "Head-On Conflict";
    case ScenarioType::PlatformConflict:      return "Platform Conflict";
    case ScenarioType::MultipleConflicts:     return "Multiple Simultaneous Conflicts";
    case ScenarioType::SensorFailure:         return "Sensor Failure";
    case ScenarioType::CommunicationFailure:  return "Communication Failure";
    case ScenarioType::UnsafeStopping:        return "Unsafe Stopping Distance";
    }
    return "Unknown";
}

} // namespace tcas::scenario
