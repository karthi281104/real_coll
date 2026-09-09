#include "application/TcasApplication.hpp"

#include "hmi/HmiDisplay.hpp"
#include "navigation/RouteNavigator.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace tcas::app
{

namespace
{

const char* trainTypeName(TrainType t) noexcept
{
    switch (t)
    {
    case TrainType::Express:   return "Express";
    case TrainType::Passenger: return "Passenger";
    case TrainType::Freight:   return "Freight";
    }
    return "Unknown";
}

const char* trainStateName(TrainState s) noexcept
{
    switch (s)
    {
    case TrainState::Idle:           return "IDLE";
    case TrainState::Running:        return "RUNNING";
    case TrainState::Slowing:        return "SLOWING";
    case TrainState::Braking:        return "BRAKING";
    case TrainState::Stopped:        return "HOLD (SIG)";
    case TrainState::EmergencyBrake: return "EMERGENCY";
    case TrainState::Completed:      return "ARRIVED";
    }
    return "UNKNOWN";
}

const char* conflictTypeName(conflict::ConflictType t) noexcept
{
    switch (t)
    {
    case conflict::ConflictType::RearEnd:  return "REAR-END";
    case conflict::ConflictType::HeadOn:   return "HEAD-ON";
    case conflict::ConflictType::Junction: return "JUNCTION";
    case conflict::ConflictType::Platform: return "PLATFORM";
    }
    return "UNKNOWN";
}

const char* commandName(safety::SafetyCommandType t) noexcept
{
    switch (t)
    {
    case safety::SafetyCommandType::NoAction:       return "NO ACTION";
    case safety::SafetyCommandType::ReduceSpeed:    return "REDUCE SPEED";
    case safety::SafetyCommandType::HoldAtSignal:   return "HOLD AT SIGNAL";
    case safety::SafetyCommandType::EmergencyBrake: return "EMERGENCY BRAKE";
    }
    return "UNKNOWN";
}

void waitForEnter()
{
    std::cout << "\n[Press ENTER to return to menu] > ";
    std::string dummy;
    std::getline(std::cin, dummy);
}

int readInteger(const std::string& prompt, int defaultValue = 0)
{
    std::cout << prompt;
    std::string line;
    if (!std::getline(std::cin, line) || line.empty())
    {
        return defaultValue;
    }
    try
    {
        return std::stoi(line);
    }
    catch (...)
    {
        return defaultValue;
    }
}

double readDoubleValue(const std::string& prompt, double defaultValue = 0.0)
{
    std::cout << prompt;
    std::string line;
    if (!std::getline(std::cin, line) || line.empty())
    {
        return defaultValue;
    }
    try
    {
        return std::stod(line);
    }
    catch (...)
    {
        return defaultValue;
    }
}

} // namespace

TcasApplication::TcasApplication()
{
    buildNetwork();
    initDefaultScenario();
}

void TcasApplication::initDefaultScenario()
{
    auto r1 = scenario::RouteCatalog::dispatchCatalogRoute(1, network_, trainManager_, 101);
    if (r1.success)
    {
        currentRoutes_.push_back(r1.trainRoute);
    }

    auto r2 = scenario::RouteCatalog::dispatchCatalogRoute(2, network_, trainManager_, 102);
    if (r2.success)
    {
        currentRoutes_.push_back(r2.trainRoute);
    }

    startSimulation();
}

TcasApplication::~TcasApplication()
{
    if (orchestrator_)
    {
        orchestrator_->stop();
    }
}

void TcasApplication::buildNetwork()
{
    using namespace tcas::infrastructure;
    network_ = RailwayNetwork{};
    trainManager_.clear();
    currentRoutes_.clear();

    network_.addNode(Node(1, "Central Station",   NodeType::Station));
    network_.addNode(Node(2, "Alpha Junction",    NodeType::Junction));
    network_.addNode(Node(3, "Beta Junction",     NodeType::Junction));
    network_.addNode(Node(4, "North Terminal",    NodeType::Station));
    network_.addNode(Node(5, "South Harbor",      NodeType::Station));
    network_.addNode(Node(6, "Freight Approach",  NodeType::Generic));
    network_.addNode(Node(7, "Freight Yard",      NodeType::Generic));
    network_.addNode(Node(8, "Platform A",        NodeType::Platform));

    network_.addTrack(Track(101, 1, 2, 2000.0, 35.0,  0.000));
    network_.addTrack(Track(201, 2, 1, 2000.0, 35.0,  0.000));
    network_.addTrack(Track(102, 2, 3, 1500.0, 30.0,  0.020));
    network_.addTrack(Track(202, 3, 2, 1500.0, 30.0, -0.020));
    network_.addTrack(Track(103, 3, 4, 2500.0, 40.0, -0.015));
    network_.addTrack(Track(203, 4, 3, 2500.0, 40.0,  0.015));
    network_.addTrack(Track(104, 2, 5, 3000.0, 25.0,  0.010));
    network_.addTrack(Track(204, 5, 2, 3000.0, 25.0, -0.010));
    network_.addTrack(Track(105, 6, 2, 2000.0, 25.0,  0.000));
    network_.addTrack(Track(205, 2, 6, 2000.0, 25.0,  0.000));
    network_.addTrack(Track(106, 2, 7, 1800.0, 25.0,  0.000));
    network_.addTrack(Track(206, 7, 2, 1800.0, 25.0,  0.000));
    network_.addTrack(Track(107, 5, 8,  500.0, 20.0,  0.000));
    network_.addTrack(Track(207, 8, 5, 1000.0, 20.0,  0.000));
    network_.addTrack(Track(108, 3, 8,  600.0, 20.0,  0.000));
    network_.addTrack(Track(208, 8, 3, 1100.0, 20.0,  0.000));
}

int TcasApplication::run()
{
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut != INVALID_HANDLE_VALUE)
    {
        DWORD dwMode = 0;
        if (GetConsoleMode(hOut, &dwMode))
        {
            SetConsoleMode(hOut, dwMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
        }
    }
#endif

    std::string inputLine;
    while (!shutdown_)
    {
        switch (currentMenu_)
        {
        case MenuState::MainMenu:
            printMainMenu();
            break;
        case MenuState::FleetMenu:
            printFleetMenu();
            break;
        case MenuState::SafetyMenu:
            printSafetyMenu();
            break;
        case MenuState::FaultMenu:
            printFaultMenu();
            break;
        case MenuState::SimControlMenu:
            printSimControlMenu();
            break;
        }

        std::cout << "Select option > ";
        if (!std::getline(std::cin, inputLine))
        {
            break;
        }

        while (!inputLine.empty() && std::isspace(static_cast<unsigned char>(inputLine.front())))
        {
            inputLine.erase(inputLine.begin());
        }
        while (!inputLine.empty() && std::isspace(static_cast<unsigned char>(inputLine.back())))
        {
            inputLine.pop_back();
        }

        switch (currentMenu_)
        {
        case MenuState::MainMenu:
            handleMainMenuInput(inputLine);
            break;
        case MenuState::FleetMenu:
            handleFleetMenuInput(inputLine);
            break;
        case MenuState::SafetyMenu:
            handleSafetyMenuInput(inputLine);
            break;
        case MenuState::FaultMenu:
            handleFaultMenuInput(inputLine);
            break;
        case MenuState::SimControlMenu:
            handleSimControlMenuInput(inputLine);
            break;
        }
    }

    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }
    std::cout << "\n[OK] TCAS Application shut down cleanly. Goodbye.\n";
    return 0;
}

void TcasApplication::printSeparator() const
{
    std::cout << "======================================================================\n";
}

void TcasApplication::printHeader() const
{
    printSeparator();
    std::cout << "                 TCAS CONTROL CENTER — LINUX CONSOLE\n"
              << "       Real-Time Predictive Train Collision Avoidance System\n";
    printSeparator();
    if (orchestrator_)
    {
        const auto snap = orchestrator_->snapshot();
        const std::string safeStr = snap.activeConflicts.empty() ? "[SAFE: ALL CLEAR]" : "[ALERT: CONFLICT DETECTED]";
        std::cout << " SYSTEM: " << (orchestrator_->isRunning() ? "RUNNING" : "STOPPED")
                  << " | TIME: " << std::fixed << std::setprecision(1) << snap.simulationTime << "s"
                  << " | SAFETY: " << safeStr
                  << " | FLEET: " << snap.trains.size() << " Trains\n";
        if (!snap.operatorMessage.empty())
        {
            std::cout << " STATUS: " << snap.operatorMessage << "\n";
        }
        printSeparator();
    }
}

void TcasApplication::printMainMenu() const
{
    printHeader();
    std::cout << "\n  [1] Fleet Dispatcher & Route Catalog  (Dispatch R-01..R-10, Remove, Speed)\n"
              << "  [2] Live Simulation Radar             (Interactive real-time observation)\n"
              << "  [3] Safety, Conflicts & Interlocking  (Active conflicts, Priority decisions)\n"
              << "  [4] Fault Injection Laboratory        (Sensor drift, Radio blackout)\n"
              << "  [5] Simulation Clock Controls         (Pause, Resume, Reset)\n"
              << "  [0] Exit / Shutdown\n\n";
}

void TcasApplication::handleMainMenuInput(const std::string& input)
{
    if (input == "1") { currentMenu_ = MenuState::FleetMenu; }
    else if (input == "2") { runLiveRadar(); }
    else if (input == "3") { currentMenu_ = MenuState::SafetyMenu; }
    else if (input == "4") { currentMenu_ = MenuState::FaultMenu; }
    else if (input == "5") { currentMenu_ = MenuState::SimControlMenu; }
    else if (input == "0" || input == "q" || input == "quit" || input == "exit")
    {
        shutdown_ = true;
    }
    else if (!input.empty())
    {
        std::cout << "[ERR] Invalid choice '" << input << "'. Enter 1-5 or 0 to exit.\n";
    }
}

void TcasApplication::printFleetMenu() const
{
    printSeparator();
    std::cout << " [1] FLEET DISPATCHER & ROUTE CATALOG\n";
    printSeparator();
    std::cout << "  [1] Dispatch Train from Route Catalog (10 Predefined Paths)\n"
              << "  [2] Quick Custom Dispatch (Dijkstra: Source Node -> Destination Node)\n"
              << "  [3] Remove Train by ID\n"
              << "  [4] Set Train Target Speed / Schedule\n"
              << "  [5] List Active Fleet & Assigned Routes\n"
              << "  [0] <-- Back to Main Menu\n\n";
}

void TcasApplication::handleFleetMenuInput(const std::string& input)
{
    if (input == "1") { dispatchCatalogInteractive(); }
    else if (input == "2") { dispatchCustomInteractive(); }
    else if (input == "3") { removeTrainInteractive(); }
    else if (input == "4") { setSpeedInteractive(); }
    else if (input == "5") { listFleet(); }
    else if (input == "0" || input == "b" || input == "back")
    {
        currentMenu_ = MenuState::MainMenu;
    }
    else if (!input.empty())
    {
        std::cout << "[ERR] Invalid choice. Enter 1-5 or 0.\n";
    }
}

void TcasApplication::dispatchCatalogInteractive()
{
    printSeparator();
    std::cout << " PREDEFINED ROUTE CATALOG (10 Realistic Commercial Runs)\n";
    printSeparator();
    const auto& routes = scenario::RouteCatalog::allRoutes();
    for (const auto& r : routes)
    {
        std::cout << " [" << std::setw(2) << r.catalogId << "] "
                  << r.code << " : " << r.name << "\n"
                  << "      Type: " << trainTypeName(r.defaultType)
                  << " | Speed: " << static_cast<int>(r.initialSpeed) << " m/s\n"
                  << "      Note: " << r.conflictNotice << "\n\n";
    }
    printSeparator();
    int choice = 0;
    while (true)
    {
        std::cout << "Enter Route ID to dispatch (1..10, 0=Cancel) > ";
        std::string line;
        if (!std::getline(std::cin, line)) { return; }
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line.empty())
        {
            std::cout << "  [ERROR] Input cannot be empty. Enter a route number (1..10) or 0 to cancel.\n";
            continue;
        }
        try
        {
            std::size_t idx = 0;
            choice = std::stoi(line, &idx);
            if (idx != line.size())
            {
                std::cout << "  [ERROR] Invalid characters detected. Please enter digits only (1..10, 0=Cancel).\n";
                continue;
            }
        }
        catch (...)
        {
            std::cout << "  [ERROR] Invalid integer. Please enter a valid number (1..10, 0=Cancel).\n";
            continue;
        }

        if (choice == 0)
        {
            std::cout << "  [INFO] Route dispatch canceled.\n";
            waitForEnter();
            return;
        }
        if (choice < 1 || choice > 10)
        {
            std::cout << "  [ERROR] Route ID " << choice << " not found. Available routes are 1 through 10. Please try again.\n";
            continue;
        }
        break;
    }

    auto result = scenario::RouteCatalog::dispatchCatalogRoute(
        choice, network_, trainManager_);
    if (result.success)
    {
        currentRoutes_.push_back(result.trainRoute);
        if (pipeline_) { pipeline_->addOrUpdateRoute(result.trainRoute); }
        if (orchestrator_)
        {
            orchestrator_->setTrainRoute(
                result.trainId, result.trainRoute.currentTrackId, result.trainRoute.route);
            orchestrator_->addTrain(result.trainId);
            orchestrator_->setOperatorMessage(result.message);
        }
        std::cout << "\n[SUCCESS] " << result.message << "\n";
    }
    else
    {
        std::cout << "\n[ERROR] Dispatch failed: " << result.message << "\n";
    }
    waitForEnter();
}

void TcasApplication::dispatchCustomInteractive()
{
    printSeparator();
    std::cout << " QUICK CUSTOM DISPATCH (Dijkstra Shortest Path)\n";
    printSeparator();
    std::cout << " Available Nodes:\n"
              << "  [1] Central Station  [2] Alpha Junction    [3] Beta Junction\n"
              << "  [4] North Terminal   [5] South Harbor      [6] Freight Approach\n"
              << "  [7] Freight Yard     [8] Platform A\n";
    printSeparator();

    // Step 1: Source Node validation loop
    int src = 0;
    while (true)
    {
        std::cout << "Enter Source Node (1..8, 0=Cancel) > ";
        std::string line;
        if (!std::getline(std::cin, line)) { return; }
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line.empty())
        {
            std::cout << "  [ERROR] Input cannot be empty. Please enter a node number (1..8) or 0 to cancel.\n";
            continue;
        }
        try
        {
            std::size_t idx = 0;
            src = std::stoi(line, &idx);
            if (idx != line.size())
            {
                std::cout << "  [ERROR] Invalid characters detected. Please enter digits only (1..8, 0=Cancel).\n";
                continue;
            }
        }
        catch (...)
        {
            std::cout << "  [ERROR] Invalid integer. Please enter a valid number (1..8, 0=Cancel).\n";
            continue;
        }

        if (src == 0)
        {
            std::cout << "  [INFO] Custom dispatch canceled.\n";
            waitForEnter();
            return;
        }
        if (src < 1 || src > 8)
        {
            std::cout << "  [ERROR] Node " << src << " does not exist. Available nodes are 1 through 8. Please try again.\n";
            continue;
        }
        break;
    }

    // Step 2: Destination Node validation loop
    int dst = 0;
    while (true)
    {
        std::cout << "Enter Destination Node (1..8, 0=Cancel) > ";
        std::string line;
        if (!std::getline(std::cin, line)) { return; }
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line.empty())
        {
            std::cout << "  [ERROR] Input cannot be empty. Please enter a node number (1..8) or 0 to cancel.\n";
            continue;
        }
        try
        {
            std::size_t idx = 0;
            dst = std::stoi(line, &idx);
            if (idx != line.size())
            {
                std::cout << "  [ERROR] Invalid characters detected. Please enter digits only (1..8, 0=Cancel).\n";
                continue;
            }
        }
        catch (...)
        {
            std::cout << "  [ERROR] Invalid integer. Please enter a valid number (1..8, 0=Cancel).\n";
            continue;
        }

        if (dst == 0)
        {
            std::cout << "  [INFO] Custom dispatch canceled.\n";
            waitForEnter();
            return;
        }
        if (dst < 1 || dst > 8)
        {
            std::cout << "  [ERROR] Node " << dst << " does not exist. Available nodes are 1 through 8. Please try again.\n";
            continue;
        }
        if (dst == src)
        {
            std::cout << "  [ERROR] Destination node cannot be the same as Source node (" << src << "). Please try again.\n";
            continue;
        }

        // Validate Dijkstra track connectivity immediately
        const auto testRoute = navigation::RouteNavigator::findRoute(
            network_, static_cast<NodeId>(src), static_cast<NodeId>(dst));
        if (!testRoute.success || testRoute.tracks.empty())
        {
            std::cout << "  [ERROR] No reachable track path exists from Node " << src << " to Node " << dst << ".\n"
                      << "          Please choose a different destination node.\n";
            continue;
        }
        break;
    }

    // Step 3: Train Type validation loop
    TrainType tType = TrainType::Passenger;
    while (true)
    {
        std::cout << "Select Train Type (1=Express, 2=Passenger, 3=Freight, 0=Cancel) > ";
        std::string line;
        if (!std::getline(std::cin, line)) { return; }
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line.empty())
        {
            std::cout << "  [ERROR] Input cannot be empty. Please enter a value for train type (1=Express, 2=Passenger, 3=Freight, 0=Cancel).\n";
            continue;
        }
        try
        {
            std::size_t idx = 0;
            int typeInt = std::stoi(line, &idx);
            if (idx != line.size())
            {
                std::cout << "  [ERROR] Invalid characters. Please enter 1, 2, or 3 (or 0 to cancel).\n";
                continue;
            }
            if (typeInt == 0)
            {
                std::cout << "  [INFO] Custom dispatch canceled.\n";
                waitForEnter();
                return;
            }
            if (typeInt == 1) { tType = TrainType::Express; break; }
            if (typeInt == 2) { tType = TrainType::Passenger; break; }
            if (typeInt == 3) { tType = TrainType::Freight; break; }
            std::cout << "  [ERROR] Invalid option " << typeInt << ". Please enter 1 (Express), 2 (Passenger), or 3 (Freight).\n";
        }
        catch (...)
        {
            std::cout << "  [ERROR] Invalid train type selection. Please enter 1, 2, or 3.\n";
        }
    }

    // Step 4: Initial Speed validation loop
    const double maxAllowed = (tType == TrainType::Express) ? 45.0 :
                              ((tType == TrainType::Freight) ? 22.2 : 33.3);
    double spd = 0.0;
    while (true)
    {
        std::cout << "Enter Initial Speed in m/s (1.0.." << maxAllowed << ", 0=Cancel) > ";
        std::string line;
        if (!std::getline(std::cin, line)) { return; }
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line.empty())
        {
            std::cout << "  [ERROR] Input cannot be empty. Please enter a value for initial speed (1.0.."
                      << maxAllowed << " m/s, 0=Cancel).\n";
            continue;
        }
        try
        {
            std::size_t idx = 0;
            spd = std::stod(line, &idx);
            if (idx != line.size())
            {
                std::cout << "  [ERROR] Invalid characters in speed. Please enter a numeric value (e.g. 25.0).\n";
                continue;
            }
            if (spd == 0.0)
            {
                std::cout << "  [INFO] Custom dispatch canceled.\n";
                waitForEnter();
                return;
            }
            if (spd < 1.0 || spd > maxAllowed)
            {
                std::cout << "  [ERROR] Speed " << spd << " m/s is out of range. Must be between 1.0 and "
                          << maxAllowed << " m/s. Please try again.\n";
                continue;
            }
            break;
        }
        catch (...)
        {
            std::cout << "  [ERROR] Invalid number format for speed. Please try again.\n";
        }
    }

    auto result = scenario::RouteCatalog::dispatchCustomRoute(
        static_cast<NodeId>(src), static_cast<NodeId>(dst),
        tType, spd, network_, trainManager_);

    if (result.success)
    {
        currentRoutes_.push_back(result.trainRoute);
        if (pipeline_) { pipeline_->addOrUpdateRoute(result.trainRoute); }
        if (orchestrator_)
        {
            orchestrator_->setTrainRoute(
                result.trainId, result.trainRoute.currentTrackId, result.trainRoute.route);
            orchestrator_->addTrain(result.trainId);
            orchestrator_->setOperatorMessage(result.message);
        }
        std::cout << "\n[SUCCESS] " << result.message << "\n";
    }
    else
    {
        std::cout << "\n[ERROR] " << result.message << "\n";
    }
    waitForEnter();
}

void TcasApplication::removeTrainInteractive()
{
    listFleet();
    int tid = readInteger("Enter Train ID to delete (0=Cancel) > ", 0);
    if (tid > 0)
    {
        // Check that the train actually exists (read-only, safe before posting command)
        const auto snap = orchestrator_ ? orchestrator_->snapshot() : orchestrator::WorldState{};
        const bool found = std::any_of(snap.trains.begin(), snap.trains.end(),
            [tid](const auto& t) { return t.id == static_cast<TrainId>(tid); });
        if (found)
        {
            if (pipeline_)  { pipeline_->removeRoute(static_cast<TrainId>(tid)); }
            std::erase_if(currentRoutes_, [tid](const auto& r) {
                return r.trainId == static_cast<TrainId>(tid);
            });
            if (orchestrator_)
            {
                // All TrainManager mutation is routed through the physics lock
                orchestrator_->postCommand({orchestrator::UserCommandType::RemoveTrain,
                    static_cast<TrainId>(tid)});
                orchestrator_->setOperatorMessage("[OK] Train #" + std::to_string(tid) +
                    " removed from fleet.");
            }
            std::cout << "\n[OK] Train #" << tid << " removal queued.\n";
        }
        else
        {
            std::cout << "\n[ERROR] Train #" << tid << " not found in active fleet.\n";
        }
    }
    waitForEnter();
}

void TcasApplication::setSpeedInteractive()
{
    listFleet();
    int tid = readInteger("Enter Train ID (0=Cancel) > ", 0);
    if (tid <= 0) return;
    double spd = readDoubleValue("Enter New Target Speed in m/s > ", 20.0);
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::SetSpeed, static_cast<TrainId>(tid), spd});
        orchestrator_->setOperatorMessage("[OK] Target speed for Train #" + std::to_string(tid) +
            " set to " + std::to_string(static_cast<int>(spd)) + " m/s");
        std::cout << "\n[OK] Speed command posted for Train #" << tid << " -> " << spd << " m/s\n";
    }
    waitForEnter();
}

void TcasApplication::listFleet()
{
    printSeparator();
    std::cout << " ACTIVE TRAIN FLEET\n";
    printSeparator();
    if (!orchestrator_)
    {
        std::cout << "Simulation orchestrator not running.\n";
        return;
    }
    const auto snap = orchestrator_->snapshot();
    if (snap.trains.empty())
    {
        std::cout << "No trains currently active on the network.\n";
    }
    else
    {
        std::cout << "  ID     TYPE        TRACK   POSITION(m)   SPEED(m/s)  LIMIT   STATE\n"
                  << " --------------------------------------------------------------------\n";
        for (const auto& t : snap.trains)
        {
            const double effectiveLimit = snap.communicationFailure
                ? std::min(t.maximumSpeed, 10.0)
                : t.maximumSpeed;
            std::string stateStr = trainStateName(t.state);
            if (t.state == TrainState::Stopped)
            {
                for (const auto& cmd : snap.commands)
                {
                    if (cmd.trainId == t.id && cmd.type == safety::SafetyCommandType::HoldAtSignal)
                    {
                        stateStr = "HOLD (SIG)";
                        break;
                    }
                }
            }
            if (t.sensorFailure)
            {
                stateStr += " [SENS-FAULT]";
            }
            std::cout << "  #" << std::setw(4) << std::left << t.id << ' '
                      << std::setw(11) << trainTypeName(t.type) << ' '
                      << std::setw(7) << t.trackId << ' '
                      << std::setw(13) << std::fixed << std::setprecision(1) << t.position << ' '
                      << std::setw(11) << t.velocity << ' '
                      << std::setw(7) << effectiveLimit << ' '
                      << stateStr << '\n';
        }
    }
    printSeparator();
}

void TcasApplication::printSafetyMenu() const
{
    printSeparator();
    std::cout << " [3] SAFETY, CONFLICTS & INTERLOCKING CENTER\n";
    printSeparator();
    std::cout << "  [1] Conflict Prediction & Resolution History (Active & Full Session History)\n"
              << "  [2] Interlocking & Resource Reservation History (Active & Past Zone Locks)\n"
              << "  [3] Safety Command & Arbitration Audit Log (All Decisions & Resulting Outcomes)\n"
              << "  [4] Full Session Safety Narrative (End-to-End Prediction -> Action -> Resolution)\n"
              << "  [0] <-- Back to Main Menu\n\n";
}

void TcasApplication::handleSafetyMenuInput(const std::string& input)
{
    if (input == "1") { showActiveConflicts(); }
    else if (input == "2") { showReservations(); }
    else if (input == "3") { showDecisions(); }
    else if (input == "4") { showSafetyLifecycleNarrative(); }
    else if (input == "0" || input == "b" || input == "back")
    {
        currentMenu_ = MenuState::MainMenu;
    }
    else if (!input.empty())
    {
        std::cout << "[ERR] Invalid choice. Enter 1-4 or 0.\n";
    }
}

void TcasApplication::showActiveConflicts()
{
    printSeparator();
    std::cout << " CONFLICT PREDICTION & RESOLUTION HISTORY (Current Session)\n";
    printSeparator();
    if (!orchestrator_) return;

    const auto snap = orchestrator_->snapshot();
    const auto history = orchestrator_->conflictHistory();

    std::cout << " >> CURRENTLY ACTIVE CONFLICTS (" << snap.activeConflicts.size() << "):\n";
    if (snap.activeConflicts.empty())
    {
        std::cout << "    [ALL CLEAR] No active conflicts across prediction horizons.\n";
    }
    else
    {
        for (const auto& c : snap.activeConflicts)
        {
            std::cout << "    * [ACTIVE] " << conflictTypeName(c.type)
                      << " | Trains: #" << c.trainA << " <-> #" << c.trainB
                      << " | Track: " << c.trackId
                      << (c.resourceNodeId != 0 ? (" | Node: " + std::to_string(c.resourceNodeId)) : "")
                      << "\n      Time to Collision (TTC): " << std::fixed << std::setprecision(2)
                      << c.firstConflictTime << " s | Min Separation: " << c.minimumSeparation << " m\n";
        }
    }

    std::cout << "\n >> SESSION CONFLICT LIFECYCLE AUDIT (Total: " << history.size() << "):\n";
    if (history.empty())
    {
        std::cout << "    No conflicts have occurred in this session so far.\n";
    }
    else
    {
        for (const auto& h : history)
        {
            std::cout << "  --------------------------------------------------------------------\n"
                      << "  Conflict #" << h.id << ": " << conflictTypeName(h.type)
                      << " | Trains: #" << h.trainA << " <-> #" << h.trainB
                      << " | Track " << h.trackId
                      << (h.resourceNodeId != 0 ? (" | Node #" + std::to_string(h.resourceNodeId)) : "") << "\n"
                      << "  Detected At : t = " << std::fixed << std::setprecision(1) << h.detectedTime << " s"
                      << " | Initial TTC: " << std::setprecision(2) << h.initialTtc << " s"
                      << " | Initial Sep: " << h.initialSeparation << " m\n"
                      << "  Arbitration : Priority -> Train #" << h.priorityTrain
                      << " | Yielding -> Train #" << h.yieldingTrain
                      << " (Risk Score: " << h.riskScore << ")\n"
                      << "  System Action: Command -> " << commandName(h.commandType)
                      << " (Target: " << std::setprecision(1) << h.targetSpeed << " m/s)";
            if (h.reservationMade)
            {
                std::cout << " | Interlocking: Exclusive Lock Node #" << h.reservedNodeId;
            }
            std::cout << "\n  Status       : ";
            if (h.isResolved)
            {
                std::cout << "[RESOLVED at t = " << h.resolvedTime << " s]\n"
                          << "  Resolution   : " << h.resolutionOutcome << "\n";
            }
            else
            {
                std::cout << "[ACTIVE / RESOLVING] System actively enforcing safety separation.\n";
            }
        }
    }
    printSeparator();
    waitForEnter();
}

void TcasApplication::showReservations()
{
    printSeparator();
    std::cout << " INTERLOCKING & RESOURCE RESERVATION HISTORY (Current Session)\n";
    printSeparator();
    if (!orchestrator_) return;

    const auto snap = orchestrator_->snapshot();
    const auto history = orchestrator_->reservationHistory();

    std::cout << " >> CURRENTLY ACTIVE EXCLUSIVE RESERVATIONS (" << snap.reservations.size() << "):\n";
    if (snap.reservations.empty())
    {
        std::cout << "    No exclusive junction/platform reservations currently active.\n";
    }
    else
    {
        for (const auto& r : snap.reservations)
        {
            std::cout << "    * [LOCKED] Zone: Node " << r.zone.nodeId
                      << " | Reserved For: Train #" << r.trainId
                      << " | Window: [" << std::fixed << std::setprecision(1)
                      << r.startTime << "s - " << r.endTime << "s]\n";
        }
    }

    std::cout << "\n >> SESSION RESERVATION AUDIT LOG (Total: " << history.size() << "):\n";
    if (history.empty())
    {
        std::cout << "    No junction or platform reservations recorded in this session.\n";
    }
    else
    {
        for (const auto& r : history)
        {
            std::cout << "  * Res #" << r.id << " | Node #" << r.nodeId
                      << " | Granted to Train #" << r.trainId
                      << " | Window: [" << std::fixed << std::setprecision(1)
                      << r.startTime << "s - " << r.endTime << "s] | Requested: t=" << r.requestedTime << "s"
                      << " | Status: " << (r.isReleased ? ("[RELEASED at t=" + std::to_string(static_cast<int>(r.releasedTime)) + "s]") : "[ACTIVE]")
                      << "\n";
        }
    }
    printSeparator();
    waitForEnter();
}

void TcasApplication::showDecisions()
{
    printSeparator();
    std::cout << " SAFETY ARBITRATION & COMMAND AUDIT LOG (Current Session)\n";
    printSeparator();
    if (!orchestrator_) return;

    const auto snap = orchestrator_->snapshot();
    const auto history = orchestrator_->commandHistory();

    std::cout << " >> CURRENT CYCLE DECISIONS (" << snap.decisions.size() << "):\n";
    if (snap.decisions.empty())
    {
        std::cout << "    No active arbitration decisions in current cycle.\n";
    }
    else
    {
        for (const auto& d : snap.decisions)
        {
            std::cout << "    * Priority Train: #" << d.priorityTrain
                      << " (granted passage)\n"
                      << "      Yielding Train: #" << d.yieldingTrain
                      << " -> Command: " << commandName(d.commandType)
                      << " | Risk Score: " << std::fixed << std::setprecision(2) << d.riskScore << "\n";
        }
    }

    std::cout << "\n >> SESSION SAFETY COMMAND AUDIT LOG (Total: " << history.size() << "):\n";
    if (history.empty())
    {
        std::cout << "    No safety intervention commands recorded in this session.\n";
    }
    else
    {
        for (const auto& c : history)
        {
            std::cout << "  * Cmd #" << c.id << " [t = " << std::fixed << std::setprecision(1) << c.timestamp << " s] "
                      << "Train #" << c.trainId << " -> " << commandName(c.type)
                      << " (Target: " << c.targetSpeed << " m/s, Risk: " << std::setprecision(1) << c.riskScore << ")\n"
                      << "    Trigger: " << c.triggerReason << "\n"
                      << "    Outcome: " << c.outcome << "\n";
        }
    }
    printSeparator();
    waitForEnter();
}

void TcasApplication::showSafetyLifecycleNarrative()
{
    printSeparator();
    std::cout << " COMPREHENSIVE SESSION SAFETY NARRATIVE (End-to-End)\n";
    std::cout << " Conflict Prediction -> Arbitration -> Interlocking & Commands -> Resolution\n";
    printSeparator();
    if (!orchestrator_) return;

    const auto conflicts = orchestrator_->conflictHistory();
    const auto reservations = orchestrator_->reservationHistory();
    const auto commands = orchestrator_->commandHistory();

    if (conflicts.empty() && reservations.empty() && commands.empty())
    {
        std::cout << " [ALL CLEAR] No safety events or conflicts have occurred in this session.\n"
                  << " All trains operated under nominal line conditions.\n";
    }
    else
    {
        std::cout << " SESSION SUMMARY:\n"
                  << "  * Total Conflicts Detected    : " << conflicts.size() << "\n"
                  << "  * Interlocking Reservations   : " << reservations.size() << "\n"
                  << "  * Safety Commands Issued      : " << commands.size() << "\n\n";

        std::cout << " CHRONOLOGICAL SAFETY EVENT LIFECYCLES:\n";
        for (const auto& h : conflicts)
        {
            std::cout << " ====================================================================\n"
                      << " [EVENT #" << h.id << "] " << conflictTypeName(h.type) << " Conflict\n"
                      << "  1. INCIDENT PREDICTION:\n"
                      << "     - Trains Involved: Train #" << h.trainA << " and Train #" << h.trainB << "\n"
                      << "     - Location       : Track " << h.trackId
                      << (h.resourceNodeId != 0 ? (" approaching Node #" + std::to_string(h.resourceNodeId)) : "") << "\n"
                      << "     - Time Detected  : t = " << std::fixed << std::setprecision(1) << h.detectedTime << " s\n"
                      << "     - Initial TTC    : " << std::setprecision(2) << h.initialTtc << " s (Min Separation: " << h.initialSeparation << " m)\n\n"
                      << "  2. SYSTEM ARBITRATION & INTERLOCKING DECISION:\n"
                      << "     - Right-of-Way   : Granted to Priority Train #" << h.priorityTrain << "\n"
                      << "     - Yielding Train : Train #" << h.yieldingTrain << " (Risk Assessment: " << h.riskScore << ")\n";
            if (h.reservationMade)
            {
                std::cout << "     - Interlocking   : Exclusive passage reservation granted on Node #" << h.reservedNodeId
                          << " for Train #" << h.priorityTrain << "\n";
            }
            std::cout << "\n  3. SAFETY COMMAND & KINEMATIC INTERVENTION:\n"
                      << "     - Issued Command : " << commandName(h.commandType)
                      << " (Regulated Target Speed: " << std::setprecision(1) << h.targetSpeed << " m/s)\n"
                      << "     - Action Result  : Train #" << h.yieldingTrain << " applied service brakes to establish safe headway.\n\n"
                      << "  4. CONFLICT RESOLUTION & OUTCOME:\n"
                      << "     - Final State    : " << (h.isResolved ? ("[RESOLVED at t = " + std::to_string(static_cast<int>(h.resolvedTime)) + " s]") : "[ACTIVE / UNDER MANAGEMENT]") << "\n"
                      << "     - Resolution Log : " << (h.isResolved ? h.resolutionOutcome : "Separation actively being regulated.") << "\n";
        }
    }
    printSeparator();
    waitForEnter();
}

void TcasApplication::printFaultMenu() const
{
    printSeparator();
    std::cout << " [4] FAULT INJECTION LABORATORY\n";
    printSeparator();
    if (orchestrator_)
    {
        const auto snap = orchestrator_->snapshot();
        std::cout << " Current Status:\n"
                  << "  Sensors:       " << (snap.sensorFailure ? "[DEGRADED / FAULT]" : "[NOMINAL]") << "\n"
                  << "  Communication: " << (snap.communicationFailure ? "[FAIL-SAFE BLACKOUT (Speed <= 10m/s)]" : "[NOMINAL]") << "\n";
        printSeparator();
    }
    std::cout << "  [1] Toggle Sensor Fault on Train (Expands uncertainty -> earlier braking)\n"
              << "  [2] Toggle Wireless Comm Blackout (Enforces fail-safe speed cap 10 m/s)\n"
              << "  [3] Clear All Active Faults\n"
              << "  [0] <-- Back to Main Menu\n\n";
}

void TcasApplication::handleFaultMenuInput(const std::string& input)
{
    if (input == "1") { toggleSensorFaultInteractive(); }
    else if (input == "2") { toggleCommFaultInteractive(); }
    else if (input == "3") { clearAllFaults(); }
    else if (input == "0" || input == "b" || input == "back")
    {
        currentMenu_ = MenuState::MainMenu;
    }
    else if (!input.empty())
    {
        std::cout << "[ERR] Invalid choice. Enter 1-3 or 0.\n";
    }
}

void TcasApplication::toggleSensorFaultInteractive()
{
    listFleet();
    int tid = readInteger("Enter Train ID to toggle sensor fault (0=All Trains) > ", 0);
    if (!orchestrator_) return;
    const bool current = orchestrator_->snapshot().sensorFailure;
    const bool next = !current;
    if (tid == 0)
    {
        orchestrator_->setSensorFault(next);
        orchestrator_->postCommand({
            next ? orchestrator::UserCommandType::InjectSensorFailure : orchestrator::UserCommandType::RecoverSensor
        });
    }
    else
    {
        orchestrator_->setSensorFault(static_cast<TrainId>(tid), next);
        orchestrator_->postCommand({
            next ? orchestrator::UserCommandType::InjectSensorFailure : orchestrator::UserCommandType::RecoverSensor,
            static_cast<TrainId>(tid)
        });
    }
    std::cout << "\n[OK] Sensor fault on " << (tid == 0 ? "ALL trains" : ("Train #" + std::to_string(tid)))
              << " set to " << (next ? "ACTIVE (Uncertainty expanded)" : "CLEARED (Nominal)") << ".\n";
    waitForEnter();
}

void TcasApplication::toggleCommFaultInteractive()
{
    if (!orchestrator_) return;
    const bool current = orchestrator_->snapshot().communicationFailure;
    const bool next = !current;
    orchestrator_->setCommFault(next);
    orchestrator_->postCommand({
        next ? orchestrator::UserCommandType::InjectCommFailure : orchestrator::UserCommandType::RecoverComm
    });
    std::cout << "\n[OK] Wireless communication link "
              << (next ? "SEVERED! Fail-safe speed cap (10 m/s) active." : "RESTORED! Nominal link active.") << "\n";
    waitForEnter();
}

void TcasApplication::clearAllFaults()
{
    if (!orchestrator_) return;
    orchestrator_->setSensorFault(false);
    orchestrator_->setCommFault(false);
    orchestrator_->postCommand({ orchestrator::UserCommandType::RecoverSensor });
    orchestrator_->postCommand({ orchestrator::UserCommandType::RecoverComm });
    std::cout << "\n[OK] All sensor and communication faults cleared.\n";
    waitForEnter();
}

void TcasApplication::printSimControlMenu() const
{
    printSeparator();
    std::cout << " [5] SIMULATION CLOCK CONTROLS\n";
    printSeparator();
    std::cout << "  [1] Pause Simulation\n"
              << "  [2] Resume Simulation\n"
              << "  [3] Reset Simulation (Clear all trains)\n"
              << "  [0] <-- Back to Main Menu\n\n";
}

void TcasApplication::handleSimControlMenuInput(const std::string& input)
{
    if (input == "1") { pauseSimulation(); }
    else if (input == "2") { resumeSimulation(); }
    else if (input == "3") { resetSimulation(); }
    else if (input == "0" || input == "b" || input == "back")
    {
        currentMenu_ = MenuState::MainMenu;
    }
    else if (!input.empty())
    {
        std::cout << "[ERR] Invalid choice. Enter 1-3 or 0.\n";
    }
}

void TcasApplication::runLiveRadar()
{
    while (!shutdown_)
    {
        printSeparator();
        std::cout << "               TCAS LIVE SIMULATION RADAR\n";
        printSeparator();
        if (!orchestrator_)
        {
            std::cout << "Simulation not running.\n";
            waitForEnter();
            return;
        }

        const auto snap = orchestrator_->snapshot();
        std::cout << " TIME: " << std::fixed << std::setprecision(1) << snap.simulationTime << "s"
                  << " | STATUS: " << (orchestrator_->isPaused() ? "PAUSED" : "RUNNING")
                  << " | SAFETY: " << (snap.activeConflicts.empty() ? "SAFE" : "CONFLICT ACTIVE") << "\n";
        if (!snap.operatorMessage.empty())
        {
            std::cout << " NOTE: " << snap.operatorMessage << "\n";
        }
        printSeparator();

        std::cout << "  ID     TYPE        TRACK   POSITION(m)   SPEED(m/s)  LIMIT   STATE\n"
                  << " --------------------------------------------------------------------\n";
        for (const auto& t : snap.trains)
        {
            const double effectiveLimit = snap.communicationFailure
                ? std::min(t.maximumSpeed, 10.0)
                : t.maximumSpeed;
            std::string stateStr = trainStateName(t.state);
            if (t.state == TrainState::Stopped)
            {
                for (const auto& cmd : snap.commands)
                {
                    if (cmd.trainId == t.id && cmd.type == safety::SafetyCommandType::HoldAtSignal)
                    {
                        stateStr = "HOLD (SIG)";
                        break;
                    }
                }
            }
            if (t.sensorFailure)
            {
                stateStr += " [SENS-FAULT]";
            }
            std::cout << "  #" << std::setw(4) << std::left << t.id << ' '
                      << std::setw(11) << trainTypeName(t.type) << ' '
                      << std::setw(7) << t.trackId << ' '
                      << std::setw(13) << std::fixed << std::setprecision(1) << t.position << ' '
                      << std::setw(11) << t.velocity << ' '
                      << std::setw(7) << effectiveLimit << ' '
                      << stateStr << '\n';
        }
        printSeparator();

        if (!snap.activeConflicts.empty())
        {
            std::cout << " ACTIVE CONFLICTS DETECTED:\n";
            for (const auto& c : snap.activeConflicts)
            {
                std::cout << "  >> " << conflictTypeName(c.type)
                          << " @ Node " << c.resourceNodeId
                          << " between Train #" << c.trainA << " and Train #" << c.trainB
                          << " (TTC: " << std::fixed << std::setprecision(1) << c.firstConflictTime << "s)\n";
            }
        }
        bool hasInterventions = false;
        for (const auto& cmd : snap.commands)
        {
            if (cmd.type != safety::SafetyCommandType::NoAction)
            {
                hasInterventions = true;
                break;
            }
        }
        if (hasInterventions)
        {
            std::cout << " ACTIVE TCAS INTERVENTIONS:\n";
            std::unordered_set<TrainId> shownTrains;
            for (const auto& cmd : snap.commands)
            {
                if (cmd.type != safety::SafetyCommandType::NoAction && !shownTrains.contains(cmd.trainId))
                {
                    shownTrains.insert(cmd.trainId);
                    std::cout << "  >> Train #" << cmd.trainId << " : "
                              << commandName(cmd.type)
                              << " -> Target Speed " << std::fixed << std::setprecision(1)
                              << cmd.targetSpeed << " m/s\n";
                }
            }
        }
        printSeparator();

        std::cout << " [Press ENTER to refresh | 's' to stream 5s live | '0' to return to menu] > ";
        std::string opt;
        if (!std::getline(std::cin, opt))
        {
            break;
        }
        if (opt == "0" || opt == "q" || opt == "b" || opt == "back")
        {
            break;
        }
        if (opt == "s" || opt == "stream")
        {
            std::cout << "\n  --- Streaming 5 seconds of live telemetry ---\n";
            for (int i = 0; i < 5; ++i)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                const auto liveSnap = orchestrator_->snapshot();
                std::cout << "  [t=" << std::fixed << std::setprecision(1) << liveSnap.simulationTime << "s]\n";
                if (liveSnap.trains.empty())
                {
                    std::cout << "    (No active trains in simulation)\n";
                }
                else
                {
                    for (const auto& tr : liveSnap.trains)
                    {
                        const double effLimit = liveSnap.communicationFailure
                            ? std::min(tr.maximumSpeed, 10.0)
                            : tr.maximumSpeed;
                        std::string st = trainStateName(tr.state);
                        if (tr.state == TrainState::Stopped)
                        {
                            for (const auto& cmd : liveSnap.commands)
                            {
                                if (cmd.trainId == tr.id && cmd.type == safety::SafetyCommandType::HoldAtSignal)
                                {
                                    st = "HOLD (SIG)";
                                    break;
                                }
                            }
                        }
                        if (tr.sensorFailure) st += " [SENS-FAULT]";

                        std::cout << "    >> Train #" << tr.id
                                  << " [" << std::setw(9) << std::left << trainTypeName(tr.type) << "]"
                                  << " Track " << std::setw(3) << tr.trackId
                                  << " @ " << std::setw(6) << std::right << static_cast<int>(tr.position) << "m"
                                  << " | Speed: " << std::setw(4) << static_cast<int>(tr.velocity) << " m/s (Limit: "
                                  << static_cast<int>(effLimit) << ")"
                                  << " | " << st << "\n";
                    }
                }
            }
            std::cout << "  ---------------------------------------------\n\n";
        }
    }
}

void TcasApplication::startSimulation()
{
    if (orchestrator_ && orchestrator_->isRunning())
    {
        return;
    }

    pipeline_ = std::make_unique<orchestrator::SafetyPipeline>(
        network_, trainManager_, currentRoutes_);

    std::vector<TrainId> trainIds;
    for (const auto& r : currentRoutes_)
    {
        trainIds.push_back(r.trainId);
    }

    orchestrator::OrchestratorConfig cfg;
    cfg.printHmi = false; // Decoupled: do not blind-print over terminal prompt
    cfg.physicsPeriod = std::chrono::milliseconds(20);
    cfg.safetyPeriod = std::chrono::milliseconds(50);
    cfg.communicationPeriod = std::chrono::milliseconds(100);

    orchestrator_ = std::make_unique<orchestrator::ThreadOrchestrator>(
        network_, trainManager_, commChannel_, trainIds, cfg, pipeline_->makeStep());

    for (const auto& r : currentRoutes_)
    {
        orchestrator_->setTrainRoute(r.trainId, r.currentTrackId, r.route);
    }

    orchestrator_->start();
}

void TcasApplication::pauseSimulation()
{
    if (orchestrator_)
    {
        orchestrator_->pause();
        std::cout << "\n[OK] Simulation paused.\n";
    }
    waitForEnter();
}

void TcasApplication::resumeSimulation()
{
    if (orchestrator_)
    {
        orchestrator_->resume();
        std::cout << "\n[OK] Simulation resumed.\n";
    }
    waitForEnter();
}

void TcasApplication::resetSimulation()
{
    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }
    buildNetwork();
    initDefaultScenario();
    std::cout << "\n[OK] Simulation reset complete. Default routes (R-01 and R-02) re-initialized.\n";
    waitForEnter();
}

} // namespace tcas::app
