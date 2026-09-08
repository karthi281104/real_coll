#pragma once

#include "communication/CommunicationChannel.hpp"
#include "hmi/PerformanceMetrics.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "orchestrator/SafetyPipeline.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "scenario/RouteCatalog.hpp"
#include "scenario/ScenarioManager.hpp"
#include "train/TrainManager.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace tcas::app
{

enum class MenuState
{
    MainMenu,
    FleetMenu,
    SafetyMenu,
    FaultMenu,
    SimControlMenu
};

class TcasApplication
{
public:
    TcasApplication();
    ~TcasApplication();

    TcasApplication(const TcasApplication&) = delete;
    TcasApplication& operator=(const TcasApplication&) = delete;

    int run();

private:
    void printSeparator() const;
    void printHeader() const;

    // Menu views
    void printMainMenu() const;
    void printFleetMenu() const;
    void printSafetyMenu() const;
    void printFaultMenu() const;
    void printSimControlMenu() const;

    // Menu handlers
    void handleMainMenuInput(const std::string& input);
    void handleFleetMenuInput(const std::string& input);
    void handleSafetyMenuInput(const std::string& input);
    void handleFaultMenuInput(const std::string& input);
    void handleSimControlMenuInput(const std::string& input);

    // Fleet operations
    void dispatchCatalogInteractive();
    void dispatchCustomInteractive();
    void removeTrainInteractive();
    void setSpeedInteractive();
    void listFleet();

    // Safety & Interlocking views
    void showActiveConflicts();
    void showReservations();
    void showDecisions();

    // Fault injection
    void toggleSensorFaultInteractive();
    void toggleCommFaultInteractive();
    void clearAllFaults();

    // Live Radar observation loop
    void runLiveRadar();

    // Simulation controls
    void startSimulation();
    void pauseSimulation();
    void resumeSimulation();
    void resetSimulation();

    void buildNetwork();
    void initDefaultScenario();

    infrastructure::RailwayNetwork network_;
    train::TrainManager trainManager_;
    communication::CommunicationChannel commChannel_;
    std::unique_ptr<orchestrator::SafetyPipeline> pipeline_;
    std::unique_ptr<orchestrator::ThreadOrchestrator> orchestrator_;
    hmi::PerformanceMetrics perfMetrics_;
    std::vector<orchestrator::SafetyPipeline::TrainRoute> currentRoutes_;

    MenuState currentMenu_{ MenuState::MainMenu };
    std::atomic<bool> shutdown_{ false };
};

} // namespace tcas::app
