#pragma once

#include "communication/CommunicationChannel.hpp"
#include "hmi/HmiDisplay.hpp"
#include "hmi/PerformanceMetrics.hpp"
#include "hmi/TelemetryLogger.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "orchestrator/CommandQueue.hpp"
#include "orchestrator/UserCommand.hpp"
#include "orchestrator/WorldState.hpp"
#include "train/TrainManager.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace tcas::orchestrator
{

struct OrchestratorConfig
{
    std::chrono::milliseconds physicsPeriod{ 20 };
    std::chrono::milliseconds safetyPeriod{ 100 };
    std::chrono::milliseconds communicationPeriod{ 100 };
    std::chrono::milliseconds hmiPeriod{ 200 };
    bool printHmi{ false };
    std::string telemetryDirectory{ "logs" };
    TimeSeconds completedTrainDwellSeconds{ 15.0 };
};

struct SafetyCycleResult
{
    std::vector<prediction::FutureState> predictions;
    std::vector<conflict::Conflict> activeConflicts;
    std::vector<conflict::ResourceReservation> reservations;
    std::vector<safety::SafetyCommand> commands;
    std::vector<SafetyDecision> decisions;
};

using SafetyStep = std::function<SafetyCycleResult(const WorldState&)>;

struct TrainNavigationState
{
    TrainId trainId{ 0 };
    TrackId currentTrackId{ 0 };
    std::size_t routeTrackIndex{ 0 };
    navigation::RouteResult route;
};

class ThreadOrchestrator
{
public:
    ThreadOrchestrator(
        const infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager,
        communication::CommunicationChannel& communicationChannel,
        std::vector<TrainId> trainIds,
        OrchestratorConfig config = {},
        SafetyStep safetyStep = {}
    );

    ~ThreadOrchestrator();

    ThreadOrchestrator(const ThreadOrchestrator&) = delete;
    ThreadOrchestrator& operator=(const ThreadOrchestrator&) = delete;

    void start();
    void stop();
    void pause();
    void resume();

    [[nodiscard]] bool isRunning() const noexcept;
    [[nodiscard]] bool isPaused() const noexcept;
    [[nodiscard]] WorldState snapshot() const;
    [[nodiscard]] std::size_t physicsCycles() const noexcept;
    [[nodiscard]] std::size_t safetyCycles() const noexcept;
    [[nodiscard]] std::size_t communicationCycles() const noexcept;
    [[nodiscard]] std::size_t hmiCycles() const noexcept;

    void setSafetyStep(SafetyStep safetyStep);

    // Thread-safe command dispatching from UI / external controllers
    void postCommand(UserCommand command);

    // Live fault injection
    void setSensorFault(bool fault);
    void setSensorFault(TrainId trainId, bool fault);
    void setCommFault(bool fault);

    // Dynamic train & route management
    void addTrain(TrainId trainId);
    void removeTrain(TrainId trainId);
    void setTrainRoute(TrainId trainId, TrackId startTrackId, navigation::RouteResult route);

    void setOperatorMessage(std::string message);
    void setCompletedTrainDwellSeconds(TimeSeconds seconds) noexcept { config_.completedTrainDwellSeconds = seconds; }
    [[nodiscard]] TimeSeconds completedTrainDwellSeconds() const noexcept { return config_.completedTrainDwellSeconds; }

    [[nodiscard]] std::vector<ConflictLifecycleRecord> conflictHistory() const;
    [[nodiscard]] std::vector<ReservationLifecycleRecord> reservationHistory() const;
    [[nodiscard]] std::vector<CommandLifecycleRecord> commandHistory() const;
    void clearHistory();

private:
    void physicsLoop();
    void safetyLoop();
    void communicationLoop();
    void hmiLoop();
    void updateWorldSnapshotLocked();
    void processUserCommandsLocked();
    void waitUntil(std::chrono::steady_clock::time_point next);

    const infrastructure::RailwayNetwork& network_;
    train::TrainManager& trainManager_;
    communication::CommunicationChannel& communicationChannel_;
    std::vector<TrainId> trainIds_;
    OrchestratorConfig config_;
    hmi::TelemetryLogger telemetryLogger_;
    hmi::PerformanceMetrics performanceMetrics_;

    mutable std::shared_mutex worldMutex_;
    WorldState worldState_;
    CommandQueue commandQueue_;

    mutable std::mutex userCommandMutex_;
    std::queue<UserCommand> userCommandQueue_;

    std::unordered_map<TrainId, TrainNavigationState> navStates_;
    std::unordered_map<TrainId, SpeedMetersPerSecond> operatorSpeedLimits_;
    std::unordered_map<TrainId, SpeedMetersPerSecond> safetySpeedLimits_;
    /// Stores the dispatch/scheduled speed for each train so auto-resume can
    /// restore the correct speed rather than using a hardcoded constant.
    std::unordered_map<TrainId, SpeedMetersPerSecond> dispatchSpeeds_;
    std::unordered_set<TrainId> failedSensors_;
    std::unordered_set<TrainId> trainsHeldBySafety_;
    /// Trains in EmergencyBrake state require explicit operator reset before
    /// they may move again — they are NOT auto-resumed by the safety loop.
    std::unordered_set<TrainId> emergencyBrakeSet_;
    std::unordered_map<TrainId, TimeSeconds> arrivalTimes_;
    std::atomic<bool> userCommFault_{ false };
    std::atomic<bool> commChannelDegraded_{ false };
    /// Previous-cycle degradation state for hysteresis (LOGIC-7 fix).
    bool commDegradedPrev_{ false };
    std::atomic<bool> safetyFailure_{ false };

    std::vector<ConflictLifecycleRecord> conflictHistory_;
    std::vector<ReservationLifecycleRecord> reservationHistory_;
    std::vector<CommandLifecycleRecord> commandHistory_;
    std::unordered_map<std::uint64_t, std::size_t> activeConflictRecordMap_;
    std::unordered_map<std::uint64_t, std::size_t> activeReservationRecordMap_;
    std::unordered_map<TrainId, safety::SafetyCommand> lastRecordedCommand_;
    std::size_t conflictCounter_{ 0 };
    std::size_t reservationCounter_{ 0 };
    std::size_t commandCounter_{ 0 };

    mutable std::mutex safetyStepMutex_;
    SafetyStep safetyStep_;

    std::atomic<bool> running_{ false };
    std::atomic<bool> paused_{ false };
    std::thread physicsThread_;
    std::thread safetyThread_;
    std::thread communicationThread_;
    std::thread hmiThread_;

    mutable std::mutex shutdownMutex_;
    std::condition_variable shutdownCondition_;

    std::atomic<std::size_t> physicsCycles_{ 0 };
    std::atomic<std::size_t> safetyCycles_{ 0 };
    std::atomic<std::size_t> communicationCycles_{ 0 };
    std::atomic<std::size_t> hmiCycles_{ 0 };
};

} // namespace tcas::orchestrator
