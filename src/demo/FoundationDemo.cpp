#include "demo/FoundationDemo.hpp"

#include "common/Types.hpp"
#include "infrastructure/Node.hpp"
#include "infrastructure/Track.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "train/ExpressTrain.hpp"
#include "train/PassengerTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/TrainManager.hpp"

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace tcas::demo
{

namespace
{

const char* nodeTypeToString(
    infrastructure::NodeType type
)
{
    switch (type)
    {
    case infrastructure::NodeType::Generic:
        return "Generic";

    case infrastructure::NodeType::Station:
        return "Station";

    case infrastructure::NodeType::Junction:
        return "Junction";

    case infrastructure::NodeType::Platform:
        return "Platform";
    }

    return "Unknown";
}

const char* trainTypeToString(TrainType type)
{
    switch (type)
    {
    case TrainType::Express:
        return "Express";

    case TrainType::Passenger:
        return "Passenger";

    case TrainType::Freight:
        return "Freight";
    }

    return "Unknown";
}

const char* trainStateToString(TrainState state)
{
    switch (state)
    {
    case TrainState::Idle:
        return "Idle";

    case TrainState::Running:
        return "Running";

    case TrainState::Slowing:
        return "Slowing";

    case TrainState::Braking:
        return "Braking";

    case TrainState::Stopped:
        return "Stopped";

    case TrainState::EmergencyBrake:
        return "EmergencyBrake";

    case TrainState::Completed:
        return "Completed";
    }

    return "Unknown";
}

void printSeparator()
{
    std::cout
        << "============================================================\n";
}

void printHeader(const std::string& title)
{
    std::cout << '\n';
    printSeparator();
    std::cout << "  " << title << '\n';
    printSeparator();
}

} // namespace


void runFoundationDemo()
{
    printHeader("TCAS - MODULE 1 + MODULE 2 FOUNDATION DEMO");

    // ============================================================
    // MODULE 1 - Railway Infrastructure
    // ============================================================

    printHeader("MODULE 1: RAILWAY INFRASTRUCTURE");

    infrastructure::RailwayNetwork network;

    // ------------------------------------------------------------
    // Create nodes
    // ------------------------------------------------------------

    infrastructure::Node chennai(
        1,
        "Chennai Central",
        infrastructure::NodeType::Station
    );

    infrastructure::Node arakkonam(
        2,
        "Arakkonam",
        infrastructure::NodeType::Station
    );

    infrastructure::Node junction(
        3,
        "Railway Junction",
        infrastructure::NodeType::Junction
    );

    infrastructure::Node platform(
        4,
        "Platform 1",
        infrastructure::NodeType::Platform
    );

    network.addNode(chennai);
    network.addNode(arakkonam);
    network.addNode(junction);
    network.addNode(platform);

    std::cout << "\n[Infrastructure] Nodes created\n";

    std::cout
        << "  Node 1: "
        << chennai.name()
        << " ("
        << nodeTypeToString(chennai.type())
        << ")\n";

    std::cout
        << "  Node 2: "
        << arakkonam.name()
        << " ("
        << nodeTypeToString(arakkonam.type())
        << ")\n";

    std::cout
        << "  Node 3: "
        << junction.name()
        << " ("
        << nodeTypeToString(junction.type())
        << ")\n";

    std::cout
        << "  Node 4: "
        << platform.name()
        << " ("
        << nodeTypeToString(platform.type())
        << ")\n";

    // ------------------------------------------------------------
    // Create tracks
    // ------------------------------------------------------------

    infrastructure::Track track1(
        101,
        1,
        2,
        70'000.0,
        44.44,
        0.5
    );

    infrastructure::Track track2(
        102,
        2,
        3,
        40'000.0,
        33.33,
        0.2
    );

    infrastructure::Track track3(
        103,
        3,
        4,
        5'000.0,
        22.22,
        0.0
    );

    network.addTrack(track1);
    network.addTrack(track2);
    network.addTrack(track3);

    std::cout << "\n[Infrastructure] Tracks created\n";

    std::cout
        << "  Track 101: Node "
        << track1.source()
        << " -> Node "
        << track1.destination()
        << ", length = "
        << track1.length()
        << " m, limit = "
        << track1.speedLimit()
        << " m/s\n";

    std::cout
        << "  Track 102: Node "
        << track2.source()
        << " -> Node "
        << track2.destination()
        << ", length = "
        << track2.length()
        << " m, limit = "
        << track2.speedLimit()
        << " m/s\n";

    std::cout
        << "  Track 103: Node "
        << track3.source()
        << " -> Node "
        << track3.destination()
        << ", length = "
        << track3.length()
        << " m, limit = "
        << track3.speedLimit()
        << " m/s\n";

    // ------------------------------------------------------------
    // Network summary
    // ------------------------------------------------------------

    std::cout << "\n[Network] Summary\n";

    std::cout
        << "  Nodes       : "
        << network.nodeCount()
        << '\n';

    std::cout
        << "  Tracks      : "
        << network.trackCount()
        << '\n';

    std::cout
        << "  Connected   : "
        << (network.isWeaklyConnected() ? "YES" : "NO")
        << '\n';

    std::cout
        << "  Has cycle   : "
        << (network.hasCycle() ? "YES" : "NO")
        << '\n';

    // ------------------------------------------------------------
    // Node / Track lookup
    // ------------------------------------------------------------

    std::cout << "\n[Network] Lookup demonstration\n";

    const auto* foundNode = network.getNode(1);

    if (foundNode != nullptr)
    {
        std::cout
            << "  Found node 1: "
            << foundNode->name()
            << '\n';
    }

    const auto* foundTrack = network.getTrack(101);

    if (foundTrack != nullptr)
    {
        std::cout
            << "  Found track 101: "
            << foundTrack->source()
            << " -> "
            << foundTrack->destination()
            << '\n';
    }

    // ============================================================
    // MODULE 2 - Train Domain
    // ============================================================

    printHeader("MODULE 2: TRAIN DOMAIN");

    train::TrainManager manager;

    // ------------------------------------------------------------
    // Create different train types
    // ------------------------------------------------------------

    auto express = std::make_unique<train::ExpressTrain>(
        1001,
        450'000.0,
        55.56,
        1.2,
        2.5
    );

    auto passenger = std::make_unique<train::PassengerTrain>(
        1002,
        350'000.0,
        38.89,
        1.0,
        2.0
    );

    auto freight = std::make_unique<train::FreightTrain>(
        1003,
        900'000.0,
        27.78,
        0.8,
        1.8
    );

    manager.addTrain(std::move(express));
    manager.addTrain(std::move(passenger));
    manager.addTrain(std::move(freight));

    std::cout << "\n[TrainManager] Trains registered\n";

    std::cout
        << "  Train count: "
        << manager.trainCount()
        << '\n';

    // ------------------------------------------------------------
    // Display train information
    // ------------------------------------------------------------

    const TrainId trainIds[] =
    {
        1001,
        1002,
        1003
    };

    for (const TrainId id : trainIds)
    {
        const auto* train = manager.getTrain(id);

        if (train == nullptr)
        {
            continue;
        }

        std::cout << "\n  Train ID: " << train->id() << '\n';

        std::cout
            << "    Type             : "
            << trainTypeToString(train->type())
            << '\n';

        std::cout
            << "    Mass             : "
            << train->mass()
            << " kg\n";

        std::cout
            << "    Maximum speed    : "
            << train->maximumSpeed()
            << " m/s\n";

        std::cout
            << "    Service braking  : "
            << train->serviceBraking()
            << " m/s^2\n";

        std::cout
            << "    Emergency braking: "
            << train->emergencyBraking()
            << " m/s^2\n";

        std::cout
            << "    Position         : "
            << train->position()
            << " m\n";

        std::cout
            << "    Velocity         : "
            << train->velocity()
            << " m/s\n";

        std::cout
            << "    Acceleration     : "
            << train->acceleration()
            << " m/s^2\n";

        std::cout
            << "    State            : "
            << trainStateToString(train->state())
            << '\n';
    }

    // ------------------------------------------------------------
    // Train state / motion update
    // ------------------------------------------------------------

    printHeader("TRAIN MOTION UPDATE");

    auto* expressTrain = manager.getTrain(1001);

    if (expressTrain != nullptr)
    {
        std::cout << "[Before update]\n";

        std::cout
            << "  Position     : "
            << expressTrain->position()
            << " m\n";

        std::cout
            << "  Velocity     : "
            << expressTrain->velocity()
            << " m/s\n";

        std::cout
            << "  Acceleration : "
            << expressTrain->acceleration()
            << " m/s^2\n";

        std::cout
            << "  State        : "
            << trainStateToString(expressTrain->state())
            << '\n';

        // Simulate movement.
        expressTrain->setPosition(12'500.0);
        expressTrain->setVelocity(42.0);
        expressTrain->setAcceleration(1.5);
        expressTrain->setState(TrainState::Running);

        std::cout << "\n[After update]\n";

        std::cout
            << "  Position     : "
            << expressTrain->position()
            << " m\n";

        std::cout
            << "  Velocity     : "
            << expressTrain->velocity()
            << " m/s\n";

        std::cout
            << "  Acceleration : "
            << expressTrain->acceleration()
            << " m/s^2\n";

        std::cout
            << "  State        : "
            << trainStateToString(expressTrain->state())
            << '\n';
    }

    // ============================================================
    // Train Manager operations
    // ============================================================

    printHeader("TRAIN MANAGER OPERATIONS");

    std::cout
        << "Contains train 1001: "
        << (manager.contains(1001) ? "YES" : "NO")
        << '\n';

    std::cout
        << "Contains train 9999: "
        << (manager.contains(9999) ? "YES" : "NO")
        << '\n';

    std::cout << "\nRemoving train 1003...\n";

    const bool removed = manager.removeTrain(1003);

    std::cout
        << "  Remove result: "
        << (removed ? "SUCCESS" : "FAILED")
        << '\n';

    std::cout
        << "  Remaining trains: "
        << manager.trainCount()
        << '\n';

    std::cout
        << "  Contains train 1003: "
        << (manager.contains(1003) ? "YES" : "NO")
        << '\n';

    // ============================================================
    // Final demonstration summary
    // ============================================================

    printHeader("FOUNDATION DEMO COMPLETE");

    std::cout
        << "Module 1 - Infrastructure : OPERATIONAL\n";

    std::cout
        << "Module 2 - Train Domain   : OPERATIONAL\n";

    std::cout
        << "Railway nodes             : "
        << network.nodeCount()
        << '\n';

    std::cout
        << "Railway tracks            : "
        << network.trackCount()
        << '\n';

    std::cout
        << "Managed trains             : "
        << manager.trainCount()
        << '\n';

    std::cout << '\n';
}

} // namespace tcas::demo
