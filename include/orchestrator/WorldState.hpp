#pragma once

#include "common/Types.hpp"
#include "conflict/Conflict.hpp"
#include "conflict/ResourceReservation.hpp"
#include "prediction/FutureState.hpp"
#include "safety/SafetyCommand.hpp"

#include <string>
#include <vector>

namespace tcas::orchestrator
{

enum class SystemStatus
{
    Ready,
    Running,
    Paused,
    Degraded,
    Shutdown
};

struct SafetyDecision
{
    TrainId yieldingTrain{ 0 };
    TrainId priorityTrain{ 0 };
    double riskScore{ 0.0 };
    safety::SafetyCommandType commandType{
        safety::SafetyCommandType::NoAction };
};

struct ConflictLifecycleRecord
{
    std::size_t id{ 0 };
    TimeSeconds detectedTime{ 0.0 };
    TimeSeconds resolvedTime{ 0.0 };
    bool isResolved{ false };

    TrainId trainA{ 0 };
    TrainId trainB{ 0 };
    conflict::ConflictType type{ conflict::ConflictType::RearEnd };
    TrackId trackId{ 0 };
    NodeId resourceNodeId{ 0 };
    TimeSeconds initialTtc{ 0.0 };
    DistanceMeters initialSeparation{ 0.0 };

    TrainId priorityTrain{ 0 };
    TrainId yieldingTrain{ 0 };
    double riskScore{ 0.0 };
    safety::SafetyCommandType commandType{ safety::SafetyCommandType::NoAction };
    double targetSpeed{ 0.0 };
    bool reservationMade{ false };
    NodeId reservedNodeId{ 0 };

    std::string resolutionOutcome;
};

struct ReservationLifecycleRecord
{
    std::size_t id{ 0 };
    TrainId trainId{ 0 };
    NodeId nodeId{ 0 };
    conflict::ConflictZoneType zoneType{ conflict::ConflictZoneType::Junction };
    TimeSeconds requestedTime{ 0.0 };
    TimeSeconds startTime{ 0.0 };
    TimeSeconds endTime{ 0.0 };
    TimeSeconds releasedTime{ 0.0 };
    bool isReleased{ false };
};

struct CommandLifecycleRecord
{
    std::size_t id{ 0 };
    TimeSeconds timestamp{ 0.0 };
    TrainId trainId{ 0 };
    safety::SafetyCommandType type{ safety::SafetyCommandType::NoAction };
    double targetSpeed{ 0.0 };
    double riskScore{ 0.0 };
    std::string triggerReason;
    std::string outcome;
};

struct TrainSnapshot
{
    TrainId id{ 0 };
    TrainType type{ TrainType::Passenger };
    TrackId trackId{ 0 };
    double mass{ 0.0 };
    double maximumSpeed{ 0.0 };
    double serviceBraking{ 0.0 };
    double emergencyBraking{ 0.0 };
    TrainState state{ TrainState::Idle };
    DistanceMeters position{ 0.0 };
    SpeedMetersPerSecond velocity{ 0.0 };
    AccelerationMetersPerSecondSquared acceleration{ 0.0 };
    bool sensorFailure{ false };
    double positionUncertainty{ 1.0 };
    std::size_t routeTrackIndex{ 0 };
    bool commFailure{ false };

    TrainSnapshot() = default;

    TrainSnapshot(
        TrainId id_, TrainType type_, TrackId trackId_,
        double mass_, double maxSpeed_, double sBrake_, double eBrake_,
        TrainState state_, DistanceMeters pos_, SpeedMetersPerSecond vel_, AccelerationMetersPerSecondSquared acc_,
        bool sensorFault_ = false, double uncert_ = 1.0, std::size_t routeIdx_ = 0, bool commFail_ = false)
        : id(id_), type(type_), trackId(trackId_), mass(mass_), maximumSpeed(maxSpeed_),
          serviceBraking(sBrake_), emergencyBraking(eBrake_), state(state_), position(pos_),
          velocity(vel_), acceleration(acc_), sensorFailure(sensorFault_),
          positionUncertainty(uncert_), routeTrackIndex(routeIdx_), commFailure(commFail_) {}

    TrainSnapshot(
        TrainId id_, TrainType type_, TrackId trackId_,
        TrainState state_, DistanceMeters pos_, SpeedMetersPerSecond vel_, AccelerationMetersPerSecondSquared acc_,
        bool sensorFault_ = false)
        : id(id_), type(type_), trackId(trackId_), state(state_), position(pos_),
          velocity(vel_), acceleration(acc_), sensorFailure(sensorFault_) {}
};

struct ThreadTimingMetrics
{
    double physicsPeriodTargetMs{ 20.0 };
    double physicsActualPeriodMs{ 20.0 };
    double physicsExecutionMs{ 0.0 };
    double physicsMaxExecutionMs{ 0.0 };
    std::size_t physicsCycles{ 0 };
    std::size_t physicsDeadlineMisses{ 0 };

    double safetyPeriodTargetMs{ 100.0 };
    double safetyActualPeriodMs{ 100.0 };
    double safetyExecutionMs{ 0.0 };
    double safetyMaxExecutionMs{ 0.0 };
    std::size_t safetyCycles{ 0 };
    std::size_t safetyDeadlineMisses{ 0 };

    double commPeriodTargetMs{ 100.0 };
    std::size_t commCycles{ 0 };
    std::size_t packetsSent{ 0 };
    std::size_t packetsDelivered{ 0 };
    std::size_t packetsDropped{ 0 };
    double packetDropRatePct{ 0.0 };
    double commLatencyMs{ 0.0 };

    double hmiPeriodTargetMs{ 200.0 };
    std::size_t hmiCycles{ 0 };
    double hmiLatencyMs{ 0.0 };
};

struct WorldState
{
    TimeSeconds simulationTime{ 0.0 };
    std::vector<TrainSnapshot> trains;
    std::vector<prediction::FutureState> predictions;
    std::vector<conflict::Conflict> activeConflicts;
    std::vector<conflict::ResourceReservation> reservations;
    std::vector<safety::SafetyCommand> commands;
    bool sensorFailure{ false };
    bool communicationFailure{ false };
    SystemStatus systemStatus{ SystemStatus::Ready };
    std::vector<SafetyDecision> decisions;

    ThreadTimingMetrics timing;
    std::string operatorMessage;
    std::string safetyError;
};

} // namespace tcas::orchestrator
