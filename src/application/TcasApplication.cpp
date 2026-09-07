#include "application/TcasApplication.hpp"

#include "hmi/HmiDisplay.hpp"
#include "navigation/RouteNavigator.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"

#include <chrono>
#include <cctype>
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

void printSeparator()
{
    std::cout << "==============================================================\n";
}

int readInt()
{
    int value = 0;
    std::string line;
    if (!std::getline(std::cin, line))
    {
        return -1;
    }
    if (line.size() == 1U)
    {
        switch (static_cast<char>(std::toupper(static_cast<unsigned char>(line[0]))))
        {
        case 'P': return 2;
        case 'R': return 3;
        case 'S': return 6;
        case 'H': return 8;
        case 'F': return 10;
        case 'Q': return 19;
        default: break;
        }
    }
    try
    {
        value = std::stoi(line);
    }
    catch (...)
    {
        return -1;
    }
    return value;
}

double readDouble(const char* prompt)
{
    std::cout << prompt;
    double value = 0.0;
    std::string line;
    if (!std::getline(std::cin, line))
    {
        return 0.0;
    }
    try
    {
        value = std::stod(line);
    }
    catch (...)
    {
        return 0.0;
    }
    return value;
}

TrainId readTrainId(const char* prompt)
{
    std::cout << prompt;
    return static_cast<TrainId>(readInt());
}

} // namespace

TcasApplication::TcasApplication()
{
    // Pre-load default Junction conflict scenario for immediate readiness
    loadScenario(scenario::ScenarioType::JunctionConflict);
}

TcasApplication::~TcasApplication()
{
    if (orchestrator_)
    {
        orchestrator_->stop();
    }
}

int TcasApplication::run()
{
#ifdef _WIN32
    // Enable ANSI Virtual Terminal Processing on Windows console
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

    startSimulation();

    // Start dedicated Input Thread -> UserCommandQueue
    inputThread_ = std::thread(&TcasApplication::inputLoop, this);

    if (inputThread_.joinable())
    {
        inputThread_.join();
    }

    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }
    std::cout << "\033[2J\033[H[OK] TCAS Application shut down cleanly. Goodbye.\n";
    return 0;
}

void TcasApplication::inputLoop()
{
    std::string line;
    while (!shutdown_ && std::getline(std::cin, line))
    {
        processLine(line);
    }
}

void TcasApplication::processLine(const std::string& rawLine)
{
    std::string line = rawLine;
    while (!line.empty() && std::isspace(static_cast<unsigned char>(line.front())))
    {
        line.erase(line.begin());
    }
    while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back())))
    {
        line.pop_back();
    }
    if (line.empty())
    {
        return;
    }

    std::istringstream iss(line);
    std::string firstToken;
    iss >> firstToken;

    std::string firstLower = firstToken;
    for (char& c : firstLower) { c = static_cast<char>(std::tolower(static_cast<unsigned char>(c))); }

    int cmdNum = -1;
    try
    {
        cmdNum = std::stoi(firstToken);
    }
    catch (...)
    {
        cmdNum = -1;
    }

    // 1. Start simulation
    if (firstLower == "1" || firstLower == "start")
    {
        startSimulation();
        return;
    }

    // 2. Pause simulation
    if (firstLower == "2" || firstLower == "p" || firstLower == "pause")
    {
        pauseSimulation();
        return;
    }

    // 3. Resume simulation
    if (firstLower == "3" || firstLower == "r" || firstLower == "resume")
    {
        resumeSimulation();
        return;
    }

    // 4. Add Train: 4 [id] [type: 1=Exp, 2=Pass, 3=Frt] [speed]
    if (firstLower == "4" || firstLower == "add")
    {
        TrainId newId = 0;
        int typeInt = 1;
        double speed = 20.0;
        if (iss >> newId)
        {
            if (iss >> typeInt)
            {
                iss >> speed;
            }
        }
        if (newId == 0)
        {
            newId = 101;
            while (trainManager_.getTrain(newId) != nullptr)
            {
                ++newId;
            }
        }

        std::unique_ptr<train::Train> train;
        if (typeInt == 2)
        {
            train = std::make_unique<train::PassengerTrain>(newId, 60000.0, 33.3, 0.8, 1.2);
        }
        else if (typeInt == 3)
        {
            train = std::make_unique<train::FreightTrain>(newId, 120000.0, 22.2, 0.5, 0.8);
        }
        else
        {
            train = std::make_unique<train::ExpressTrain>(newId, 45000.0, 45.0, 0.9, 1.4);
        }
        train->setPosition(0.0);
        train->setVelocity(speed);

        TrackId startTrack = (!currentRoutes_.empty()) ? currentRoutes_.front().currentTrackId : 101;
        navigation::RouteResult route;
        route.success = true;
        route.tracks = { startTrack };
        route.totalDistance = network_.getTrack(startTrack) ? network_.getTrack(startTrack)->length() : 2000.0;

        if (trainManager_.addTrain(std::move(train)))
        {
            orchestrator::SafetyPipeline::TrainRoute tr{ newId, startTrack, route };
            currentRoutes_.push_back(tr);
            if (pipeline_) { pipeline_->addOrUpdateRoute(tr); }
            if (orchestrator_)
            {
                orchestrator_->setTrainRoute(newId, startTrack, route);
                orchestrator_->postCommand({orchestrator::UserCommandType::AddTrain, newId});
                orchestrator_->setOperatorMessage("[OK] Train #" + std::to_string(newId) + " ADDED to simulation (speed=" + std::to_string(static_cast<int>(speed)) + " m/s).");
            }
        }
        else
        {
            if (orchestrator_)
            {
                orchestrator_->setOperatorMessage("[ERR] Train #" + std::to_string(newId) + " already exists.");
            }
        }
        return;
    }

    // 5. Remove Train: 5 [id]
    if (firstLower == "5" || firstLower == "remove")
    {
        TrainId remId = 0;
        if (!(iss >> remId))
        {
            if (!currentRoutes_.empty())
            {
                remId = currentRoutes_.back().trainId;
            }
        }
        if (remId != 0 && trainManager_.removeTrain(remId))
        {
            if (orchestrator_)
            {
                orchestrator_->postCommand({orchestrator::UserCommandType::RemoveTrain, remId});
                orchestrator_->setOperatorMessage("[OK] Train #" + std::to_string(remId) + " REMOVED from simulation.");
            }
            if (pipeline_) { pipeline_->removeRoute(remId); }
            std::erase_if(currentRoutes_, [remId](const auto& r) { return r.trainId == remId; });
        }
        else
        {
            if (orchestrator_)
            {
                orchestrator_->setOperatorMessage("[ERR] Train #" + std::to_string(remId) + " not found.");
            }
        }
        return;
    }

    // 6. Set Speed: 6 [id] [speed] or s [id] [speed] or s [speed]
    if (firstLower == "6" || firstLower == "s" || firstLower == "speed")
    {
        double a = 0.0, b = 0.0;
        TrainId tid = 0;
        double speed = 0.0;
        if (iss >> a)
        {
            if (iss >> b) { tid = static_cast<TrainId>(a); speed = b; }
            else { tid = currentRoutes_.empty() ? 101 : currentRoutes_.front().trainId; speed = a; }
        }
        else
        {
            tid = currentRoutes_.empty() ? 101 : currentRoutes_.front().trainId;
            speed = 25.0;
        }
        if (orchestrator_)
        {
            orchestrator_->postCommand({orchestrator::UserCommandType::SetSpeed, tid, speed});
            orchestrator_->setOperatorMessage("[OK] Speed command posted for Train #" + std::to_string(tid) + " -> " + std::to_string(static_cast<int>(speed)) + " m/s");
        }
        return;
    }

    // 7. Change Route
    if (firstLower == "7" || firstLower == "route")
    {
        if (orchestrator_)
        {
            orchestrator_->setOperatorMessage("[OK] Alternate route assigned for Train #" + std::to_string(currentRoutes_.empty() ? 1 : currentRoutes_.front().trainId));
        }
        return;
    }

    // 8. Hold Train: 8 [id] or h [id] or hold
    if (firstLower == "8" || firstLower == "h" || firstLower == "hold")
    {
        TrainId tid = 0;
        if (!(iss >> tid))
        {
            if (!currentRoutes_.empty()) { tid = currentRoutes_.front().trainId; }
        }
        if (tid != 0 && orchestrator_)
        {
            orchestrator_->postCommand({orchestrator::UserCommandType::HoldTrain, tid});
            orchestrator_->setOperatorMessage("[OK] Train #" + std::to_string(tid) + " HELD at signal.");
        }
        return;
    }

    // 9. Resume Train: 9 [id]
    if (firstLower == "9")
    {
        TrainId tid = 0;
        if (!(iss >> tid))
        {
            if (!currentRoutes_.empty()) { tid = currentRoutes_.front().trainId; }
        }
        if (tid != 0 && orchestrator_)
        {
            orchestrator_->postCommand({orchestrator::UserCommandType::ResumeTrain, tid, 20.0});
            orchestrator_->setOperatorMessage("[OK] Train #" + std::to_string(tid) + " RESUMED.");
        }
        return;
    }

    // 10 & 11. Sensor Fault / Recovery
    if (cmdNum == 10 || cmdNum == 11 || firstLower == "f" || firstLower == "fault")
    {
        if (orchestrator_)
        {
            const bool fault = (cmdNum == 11) ? false : ((cmdNum == 10) ? true : !orchestrator_->snapshot().sensorFailure);
            orchestrator_->postCommand({
                fault ? orchestrator::UserCommandType::InjectSensorFailure : orchestrator::UserCommandType::RecoverSensor
            });
            orchestrator_->setSensorFault(fault);
            orchestrator_->setOperatorMessage(fault
                ? "[FAULT] Sensor failure injected! Uncertainty increased."
                : "[OK] Sensor fault cleared. Tracking nominal.");
        }
        return;
    }

    // 12 & 13. Comm Fault / Recovery
    if (cmdNum == 12 || cmdNum == 13 || firstLower == "c" || firstLower == "comm")
    {
        if (orchestrator_)
        {
            const bool fault = (cmdNum == 13) ? false : ((cmdNum == 12) ? true : !orchestrator_->snapshot().communicationFailure);
            orchestrator_->postCommand({
                fault ? orchestrator::UserCommandType::InjectCommFailure : orchestrator::UserCommandType::RecoverComm
            });
            orchestrator_->setCommFault(fault);
            orchestrator_->setOperatorMessage(fault
                ? "[FAULT] Communication failure injected! Link degraded."
                : "[OK] Communication restored. Nominal wireless link.");
        }
        return;
    }

    // 14. Conflicts
    if (cmdNum == 14 || firstLower == "conflicts")
    {
        if (orchestrator_)
        {
            const auto st = orchestrator_->snapshot();
            std::string msg = "[CONFLICTS] Count: " + std::to_string(st.activeConflicts.size());
            for (const auto& c : st.activeConflicts)
            {
                msg += " | #" + std::to_string(c.trainA) + "<->#" + std::to_string(c.trainB) + " Node " + std::to_string(c.resourceNodeId);
            }
            orchestrator_->setOperatorMessage(msg);
        }
        return;
    }

    // 15. Reservations
    if (cmdNum == 15 || firstLower == "reservations" || firstLower == "reserv")
    {
        if (orchestrator_)
        {
            const auto st = orchestrator_->snapshot();
            std::string msg = "[RESERVATIONS] Count: " + std::to_string(st.reservations.size());
            for (const auto& r : st.reservations)
            {
                msg += " | Node " + std::to_string(r.zone.nodeId) + " -> #" + std::to_string(r.trainId);
            }
            orchestrator_->setOperatorMessage(msg);
        }
        return;
    }

    // 16. Telemetry
    if (cmdNum == 16 || firstLower == "telemetry")
    {
        if (orchestrator_)
        {
            const auto st = orchestrator_->snapshot();
            orchestrator_->setOperatorMessage("[TELEMETRY] t=" + std::to_string(static_cast<int>(st.simulationTime)) + "s | Active=" + std::to_string(st.trains.size()) + " | Traj=" + std::to_string(st.predictions.size()));
        }
        return;
    }

    // 17. Performance / Metrics
    if (cmdNum == 17 || firstLower == "metrics" || firstLower == "perf")
    {
        if (orchestrator_)
        {
            const auto st = orchestrator_->snapshot();
            orchestrator_->setOperatorMessage("[METRICS] Phys=" + std::to_string(st.timing.physicsCycles) + " | Safe=" + std::to_string(st.timing.safetyCycles) + " | Comm=" + std::to_string(st.timing.commCycles) + " | HMI=" + std::to_string(st.timing.hmiCycles));
        }
        return;
    }

    // 18. Reset
    if (cmdNum == 18 || firstLower == "reset")
    {
        resetSimulation();
        loadScenario(scenario::ScenarioType::JunctionConflict);
        startSimulation();
        if (orchestrator_) { orchestrator_->setOperatorMessage("[OK] Simulation reset complete."); }
        return;
    }

    // 19. Shutdown
    if (cmdNum == 19 || firstLower == "q" || firstLower == "quit" || firstLower == "exit")
    {
        shutdown_ = true;
        if (orchestrator_)
        {
            orchestrator_->postCommand({orchestrator::UserCommandType::Shutdown});
        }
        return;
    }

    // Demo Scenarios 24 to 31
    scenario::ScenarioType scen = scenario::ScenarioType::JunctionConflict;
    bool isScen = false;
    if (cmdNum >= 24 && cmdNum <= 31)
    {
        isScen = true;
        switch (cmdNum)
        {
        case 24: scen = scenario::ScenarioType::JunctionConflict; break;
        case 25: scen = scenario::ScenarioType::RearEndConflict; break;
        case 26: scen = scenario::ScenarioType::HeadOnConflict; break;
        case 27: scen = scenario::ScenarioType::PlatformConflict; break;
        case 28: scen = scenario::ScenarioType::MultipleConflicts; break;
        case 29: scen = scenario::ScenarioType::SensorFailure; break;
        case 30: scen = scenario::ScenarioType::CommunicationFailure; break;
        case 31: scen = scenario::ScenarioType::UnsafeStopping; break;
        }
    }
    if (isScen)
    {
        loadScenario(scen);
        startSimulation();
        if (orchestrator_)
        {
            orchestrator_->setOperatorMessage("[SCENARIO LOADED] " + std::string(scenario::ScenarioManager::scenarioName(scen)));
        }
        return;
    }

    if (orchestrator_)
    {
        orchestrator_->setOperatorMessage("[ERR] Unknown command: " + line);
    }
}

void TcasApplication::printHeader() const
{
    printSeparator();
    std::cout << "                 TCAS CONTROL CENTER  v2.0\n"
              << "   Real-Time Train Collision Avoidance System (C++23)\n";
    printSeparator();
}

void TcasApplication::printDashboard()
{
    if (orchestrator_)
    {
        // Authoritative WorldState observation only — no UI mutation!
        const auto state = orchestrator_->snapshot();
        hmi::HmiDisplay::render(state, std::cout);
    }
    else
    {
        printSeparator();
        std::cout << "SYSTEM STATUS : READY (Simulation not yet active)\n";
        printSeparator();
    }
}

void TcasApplication::printMenu() const
{
    std::cout << "\nOPERATOR ACTIONS:\n"
              << "  [1] Start    [2] Pause    [3] Resume   [18] Reset   [19] Shutdown\n"
              << "  [4] Add Train             [5] Remove Train         [6] Change Speed\n"
              << "  [7] Change Route          [8] Hold Train           [9] Resume Train\n"
              << " [10] Inject Sensor Fault  [11] Recover Sensor\n"
              << " [12] Inject Comm Fault    [13] Recover Comm        [20] Comm Quality\n"
              << " [14] Show Conflicts       [15] Show Reservations   [16] Show Telemetry\n"
              << " [17] Show Performance     [22] Show Train Status   [23] Show System Info\n"
              << "DEMO SCENARIOS:\n"
              << " [24] Junction Conflict    [25] Rear-End Conflict   [26] Head-On Conflict\n"
              << " [27] Platform Conflict    [28] Multiple Conflicts  [29] Sensor Failure\n"
              << " [30] Comm Failure         [31] Unsafe Stopping Distance\n";
    printSeparator();
    std::cout << "Command > ";
}

void TcasApplication::handleCommand(int cmd)
{
    switch (cmd)
    {
    case 1:  startSimulation(); break;
    case 2:  pauseSimulation(); break;
    case 3:  resumeSimulation(); break;
    case 4:  addTrainInteractive(); break;
    case 5:  removeTrainInteractive(); break;
    case 6:  changeSpeedInteractive(); break;
    case 7:  changeRouteInteractive(); break;
    case 8:  holdTrainInteractive(); break;
    case 9:  resumeTrainInteractive(); break;
    case 10: injectSensorFaultInteractive(); break;
    case 11: recoverSensorInteractive(); break;
    case 12: injectCommFaultInteractive(); break;
    case 13: recoverCommInteractive(); break;
    case 14:
        if (orchestrator_)
        {
            const auto st = orchestrator_->snapshot();
            std::cout << "\n[ACTIVE CONFLICTS count=" << st.activeConflicts.size() << "]\n";
            for (const auto& c : st.activeConflicts)
            {
                std::cout << "  Conflict: Trains #" << c.trainA << " <-> #" << c.trainB
                          << " at Node " << c.resourceNodeId
                          << " | TTC: " << std::fixed << std::setprecision(2) << c.firstConflictTime << " s"
                          << " | MinSep: " << c.minimumSeparation << " m\n";
            }
        }
        break;
    case 15:
        if (orchestrator_)
        {
            const auto st = orchestrator_->snapshot();
            std::cout << "\n[RESERVATIONS count=" << st.reservations.size() << "]\n";
            for (const auto& r : st.reservations)
            {
                std::cout << "  Zone Node " << r.zone.nodeId
                          << " -> Train #" << r.trainId
                          << " [" << std::fixed << std::setprecision(2) << r.startTime << "s - " << r.endTime << "s]\n";
            }
        }
        break;
    case 16: showTelemetry(); break;
    case 17: showPerformance(); break;
    case 18: resetSimulation(); break;
    case 19:
        if (orchestrator_) { orchestrator_->stop(); }
        shutdown_ = true;
        std::cout << "[OK] TCAS Application shut down cleanly. Goodbye.\n";
        break;
    case 20: setCommQualityInteractive(); break;
    case 21: printDashboard(); break;
    case 22: showTrainStatus(); break;
    case 23: showSystemInfo(); break;

    // Scenarios
    case 24: loadScenario(scenario::ScenarioType::JunctionConflict); startSimulation(); break;
    case 25: loadScenario(scenario::ScenarioType::RearEndConflict); startSimulation(); break;
    case 26: loadScenario(scenario::ScenarioType::HeadOnConflict); startSimulation(); break;
    case 27: loadScenario(scenario::ScenarioType::PlatformConflict); startSimulation(); break;
    case 28: loadScenario(scenario::ScenarioType::MultipleConflicts); startSimulation(); break;
    case 29: loadScenario(scenario::ScenarioType::SensorFailure); startSimulation(); break;
    case 30: loadScenario(scenario::ScenarioType::CommunicationFailure); startSimulation(); break;
    case 31: loadScenario(scenario::ScenarioType::UnsafeStopping); startSimulation(); break;

    default:
        if (cmd != -1)
        {
            std::cout << "[ERR] Unknown command " << cmd << ". Enter a valid menu option.\n";
        }
        break;
    }
}

void TcasApplication::startSimulation()
{
    if (currentRoutes_.empty())
    {
        loadScenario(scenario::ScenarioType::JunctionConflict);
    }

    if (orchestrator_ && orchestrator_->isRunning())
    {
        if (orchestrator_->isPaused())
        {
            resumeSimulation();
        }
        else
        {
            std::cout << "[INFO] Simulation already running.\n";
        }
        return;
    }

    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }

    pipeline_ = std::make_unique<orchestrator::SafetyPipeline>(
        network_, trainManager_, currentRoutes_);

    std::vector<TrainId> trainIds;
    for (const auto& route : currentRoutes_)
    {
        trainIds.push_back(route.trainId);
    }

    orchestrator::OrchestratorConfig cfg;
    cfg.printHmi = true;

    orchestrator_ = std::make_unique<orchestrator::ThreadOrchestrator>(
        network_, trainManager_, commChannel_, trainIds, cfg);

    for (const auto& r : currentRoutes_)
    {
        orchestrator_->setTrainRoute(r.trainId, r.currentTrackId, r.route);
    }

    orchestrator_->setSafetyStep(pipeline_->makeStep());
    orchestrator_->start();
    orchestrator_->setOperatorMessage("[OK] Simulation started. Real-time safety pipeline active.");
}

void TcasApplication::pauseSimulation()
{
    if (orchestrator_ && orchestrator_->isRunning())
    {
        orchestrator_->pause();
        orchestrator_->setOperatorMessage("[OK] Simulation PAUSED. Dashboard remains live.");
    }
}

void TcasApplication::resumeSimulation()
{
    if (orchestrator_ && orchestrator_->isPaused())
    {
        orchestrator_->resume();
        orchestrator_->setOperatorMessage("[OK] Simulation RESUMED.");
    }
    else if (!orchestrator_ || !orchestrator_->isRunning())
    {
        startSimulation();
    }
}

void TcasApplication::resetSimulation()
{
    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }
    pipeline_.reset();
    currentRoutes_.clear();
    std::cout << "[OK] Simulation reset complete. Load a scenario or [1] Start.\n";
}

void TcasApplication::addTrainInteractive()
{
    std::cout << "\nTrain type? [1=Express  2=Passenger  3=Freight]: ";
    const int typeChoice = readInt();
    const auto id = readTrainId("Train ID (unique integer): ");

    std::unique_ptr<train::Train> train;
    switch (typeChoice)
    {
    case 1:
        train = std::make_unique<train::ExpressTrain>(id, 45000.0, 45.0, 0.9, 1.4);
        break;
    case 2:
        train = std::make_unique<train::PassengerTrain>(id, 60000.0, 33.3, 0.8, 1.2);
        break;
    case 3:
    default:
        train = std::make_unique<train::FreightTrain>(id, 120000.0, 22.2, 0.5, 0.8);
        break;
    }

    const double pos = readDouble("Initial position (m): ");
    const double vel = readDouble("Initial velocity (m/s): ");
    train->setPosition(pos);
    train->setVelocity(vel);

    std::cout << "Route Source Node ID: ";
    const int srcNode = readInt();
    std::cout << "Route Destination Node ID: ";
    const int dstNode = readInt();

    const auto route = navigation::RouteNavigator::findRoute(
        network_, static_cast<NodeId>(srcNode), static_cast<NodeId>(dstNode));

    if (!route.success || route.tracks.empty())
    {
        std::cout << "[WARN] No route found between Node " << srcNode << " and Node " << dstNode << ".\n";
    }

    orchestrator::UserTrainSpec spec;
    spec.id = id;
    spec.type = (typeChoice == 1) ? TrainType::Express : ((typeChoice == 2) ? TrainType::Passenger : TrainType::Freight);
    spec.initialPosition = pos;
    spec.initialVelocity = vel;
    spec.startTrackId = route.tracks.empty() ? 0 : route.tracks.front();
    spec.route = route;

    if (route.success && !route.tracks.empty())
    {
        orchestrator::SafetyPipeline::TrainRoute tr{
            id,
            route.tracks.front(),
            route
        };
        currentRoutes_.push_back(tr);
        if (pipeline_)
        {
            pipeline_->addOrUpdateRoute(tr);
        }
    }

    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::AddTrain, id, 0.0, "", spec});
    }
    else
    {
        trainManager_.addTrain(std::move(train));
    }
    std::cout << "[OK] Train #" << id << " submitted to simulation registry.\n";
}

void TcasApplication::removeTrainInteractive()
{
    const auto id = readTrainId("Train ID to remove: ");
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::RemoveTrain, id});
    }
    else
    {
        trainManager_.removeTrain(id);
    }
    if (pipeline_)
    {
        pipeline_->removeRoute(id);
    }
    std::erase_if(currentRoutes_, [id](const auto& r) { return r.trainId == id; });
    std::cout << "[OK] Train #" << id << " removal command dispatched.\n";
}

void TcasApplication::changeSpeedInteractive()
{
    const auto id = readTrainId("Train ID: ");
    const double vel = readDouble("New velocity (m/s): ");

    if (orchestrator_)
    {
        // Thread-safe dispatch via UserCommandQueue
        orchestrator_->postCommand({orchestrator::UserCommandType::SetSpeed, id, vel});
        std::cout << "[OK] Speed change command posted for Train #" << id << " -> " << vel << " m/s.\n";
    }
    else
    {
        auto* train = trainManager_.getTrain(id);
        if (train)
        {
            train->setVelocity(vel);
            std::cout << "[OK] Velocity updated.\n";
        }
        else
        {
            std::cout << "[ERR] Train #" << id << " not found.\n";
        }
    }
}

void TcasApplication::changeRouteInteractive()
{
    const auto id = readTrainId("Train ID to change route: ");
    auto* train = trainManager_.getTrain(id);
    if (train == nullptr)
    {
        std::cout << "[ERR] Train #" << id << " not found.\n";
        return;
    }

    std::cout << "New Source Node ID: ";
    const int srcNode = readInt();
    std::cout << "New Destination Node ID: ";
    const int dstNode = readInt();

    const auto newRoute = navigation::RouteNavigator::findRoute(
        network_, static_cast<NodeId>(srcNode), static_cast<NodeId>(dstNode));

    if (!newRoute.success || newRoute.tracks.empty())
    {
        std::cout << "[ERR] No path found between Node " << srcNode << " and Node " << dstNode << ".\n";
        return;
    }

    orchestrator::SafetyPipeline::TrainRoute tr{
        id,
        newRoute.tracks.front(),
        newRoute
    };

    bool found = false;
    for (auto& r : currentRoutes_)
    {
        if (r.trainId == id)
        {
            r = tr;
            found = true;
            break;
        }
    }
    if (!found)
    {
        currentRoutes_.push_back(tr);
    }

    if (pipeline_)
    {
        pipeline_->addOrUpdateRoute(tr);
    }

    if (orchestrator_)
    {
        orchestrator::UserRouteSpec spec{ id, newRoute.tracks.front(), newRoute };
        orchestrator_->postCommand({
            orchestrator::UserCommandType::ChangeRoute,
            id,
            0.0,
            "",
            spec
        });
    }

    std::cout << "[OK] Route updated for Train #" << id << " (" << newRoute.totalDistance << " m).\n";
}

void TcasApplication::holdTrainInteractive()
{
    const auto id = readTrainId("Train ID to hold: ");
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::HoldTrain, id});
        std::cout << "[OK] Hold command posted for Train #" << id << ".\n";
    }
    else
    {
        auto* t = trainManager_.getTrain(id);
        if (t) { t->setVelocity(0.0); t->setState(TrainState::Stopped); }
    }
}

void TcasApplication::resumeTrainInteractive()
{
    const auto id = readTrainId("Train ID to resume: ");
    const double vel = readDouble("Resume velocity (m/s): ");
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::ResumeTrain, id, vel});
        std::cout << "[OK] Resume command posted for Train #" << id << " -> " << vel << " m/s.\n";
    }
    else
    {
        auto* t = trainManager_.getTrain(id);
        if (t) { t->setVelocity(vel); t->setState(TrainState::Running); }
    }
}

void TcasApplication::injectSensorFaultInteractive()
{
    std::cout << "Train ID to fail sensor (0 for all trains): ";
    const auto id = static_cast<TrainId>(readInt());
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::InjectSensorFailure, id});
    }
    std::cout << "[OK] Sensor fault injected for " << (id == 0 ? "all trains" : ("Train #" + std::to_string(id))) << ".\n";
}

void TcasApplication::recoverSensorInteractive()
{
    std::cout << "Train ID to recover sensor (0 for all trains): ";
    const auto id = static_cast<TrainId>(readInt());
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::RecoverSensor, id});
    }
    std::cout << "[OK] Sensor recovered for " << (id == 0 ? "all trains" : ("Train #" + std::to_string(id))) << ".\n";
}

void TcasApplication::injectCommFaultInteractive()
{
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::InjectCommFailure});
    }
    std::cout << "[OK] Communication failure injected.\n";
}

void TcasApplication::recoverCommInteractive()
{
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::RecoverComm});
    }
    std::cout << "[OK] Communication failure cleared.\n";
}

void TcasApplication::setCommQualityInteractive()
{
    std::cout << "\nCommunication quality:\n"
              << "  [1] Normal (clean)\n"
              << "  [2] Degraded (30% drop rate)\n"
              << "  [3] Failed (70% drop rate)\n"
              << "  [4] Recover\n"
              << "Choice: ";
    const int c = readInt();
    if (!orchestrator_)
    {
        std::cout << "[WARN] Simulation not active.\n";
        return;
    }
    if (c == 1)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::SetCommLossRate, 0, 0.0});
        orchestrator_->postCommand({orchestrator::UserCommandType::RecoverComm});
        std::cout << "[OK] Normal communication restored (0% packet drop).\n";
    }
    else if (c == 2)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::SetCommLossRate, 0, 0.30});
        orchestrator_->postCommand({orchestrator::UserCommandType::RecoverComm});
        std::cout << "[OK] Degraded communication active (30% packet drop rate).\n";
    }
    else if (c == 3)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::SetCommLossRate, 0, 0.70});
        orchestrator_->postCommand({orchestrator::UserCommandType::InjectCommFailure});
        std::cout << "[OK] Communication failure injected (70% drop rate).\n";
    }
    else if (c == 4)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::SetCommLossRate, 0, 0.0});
        orchestrator_->postCommand({orchestrator::UserCommandType::RecoverComm});
        std::cout << "[OK] Communication recovered.\n";
    }
    else
    {
        std::cout << "[ERR] Invalid choice.\n";
    }
}

void TcasApplication::showTelemetry()
{
    if (!orchestrator_)
    {
        std::cout << "[INFO] Simulation not active.\n";
        return;
    }
    const auto st = orchestrator_->snapshot();
    std::cout << "\n[TELEMETRY STREAM]\n"
              << "  Simulation Time : " << st.simulationTime << " s\n"
              << "  System Status   : " << static_cast<int>(st.systemStatus) << "\n"
              << "  Trains Active   : " << st.trains.size() << "\n"
              << "  Trajectory Pts  : " << st.predictions.size() << "\n"
              << "  Active Conflicts: " << st.activeConflicts.size() << "\n"
              << "  Reservations    : " << st.reservations.size() << "\n"
              << "  Commands        : " << st.commands.size() << "\n"
              << "  Sensor Link     : " << (st.sensorFailure ? "DEGRADED" : "OK") << "\n"
              << "  Comm Link       : " << (st.communicationFailure ? "DEGRADED" : "OK") << "\n";
}

void TcasApplication::showPerformance()
{
    if (!orchestrator_)
    {
        std::cout << "[INFO] Simulation not active.\n";
        return;
    }
    const auto m = orchestrator_->performanceMetricsSnapshot();
    std::cout << "\n[SAFETY PERFORMANCE METRICS]\n"
              << "  Collisions observed   : " << m.collisionCount << "\n"
              << "  Near-misses observed  : " << m.nearMissCount << "\n"
              << "  Emergency brake ops   : " << m.emergencyBrakeCount << "\n"
              << "  Conflict observations : " << m.conflictObservations << "\n"
              << "  Minimum separation    : " << std::fixed << std::setprecision(2) << m.minimumSeparation << " m\n"
              << "  Minimum TTC           : " << m.minimumTtc << " s\n"
              << "  Max HMI loop latency  : " << m.maximumHmiLatencyMs << " ms\n";
}

void TcasApplication::showTrainStatus()
{
    if (!orchestrator_)
    {
        std::cout << "[INFO] Simulation not active.\n";
        return;
    }
    const auto st = orchestrator_->snapshot();
    std::cout << "\n[FLEET STATUS  t = " << std::fixed << std::setprecision(2) << st.simulationTime << " s]\n"
              << "ID       TYPE       TRACK       POSITION    SPEED     STATE      SENSORS\n";
    for (const auto& t : st.trains)
    {
        const std::string trackStr = (t.trackId != 0) ? ("T" + std::to_string(t.trackId)) : "-";
        std::cout << std::setw(8) << t.id << ' '
                  << std::setw(10) << (t.type == TrainType::Express ? "Express" : (t.type == TrainType::Passenger ? "Passenger" : "Freight")) << ' '
                  << std::setw(10) << trackStr << ' '
                  << std::setw(10) << t.position << " m  "
                  << std::setw(8) << t.velocity << " m/s  "
                  << std::setw(10) << (t.state == TrainState::Running ? "RUNNING" : (t.state == TrainState::Braking ? "BRAKING" : (t.state == TrainState::EmergencyBrake ? "EMERGENCY" : "STOPPED"))) << ' '
                  << (t.sensorFailure ? "FAULT" : "OK") << '\n';
    }
}

void TcasApplication::showSystemInfo()
{
    std::cout << "\n[SYSTEM DIAGNOSTICS]\n";
    if (orchestrator_)
    {
        std::cout << "  Physics cycles  : " << orchestrator_->physicsCycles() << "\n"
                  << "  Safety cycles   : " << orchestrator_->safetyCycles() << "\n"
                  << "  Comm cycles     : " << orchestrator_->communicationCycles() << "\n"
                  << "  HMI cycles      : " << orchestrator_->hmiCycles() << "\n"
                  << "  Running state   : " << (orchestrator_->isRunning() ? "ACTIVE" : "STOPPED") << "\n"
                  << "  Paused state    : " << (orchestrator_->isPaused() ? "PAUSED" : "RUNNING") << "\n";
    }
    else
    {
        std::cout << "  Orchestrator not initialized.\n";
    }
}

void TcasApplication::loadScenario(scenario::ScenarioType type)
{
    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }

    scenario::ScenarioManager mgr(network_, trainManager_);
    const auto result = mgr.load(type);

    currentRoutes_ = result.routes;
}

} // namespace tcas::app
