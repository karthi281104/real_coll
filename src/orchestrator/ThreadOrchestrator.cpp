#include "orchestrator/ThreadOrchestrator.hpp"

#include "physics/KinematicsEngine.hpp"
#include "train/ExpressTrain.hpp"
#include "train/PassengerTrain.hpp"
#include "train/FreightTrain.hpp"

#include <algorithm>
#include <iostream>

namespace tcas::orchestrator
{

ThreadOrchestrator::ThreadOrchestrator(
    const infrastructure::RailwayNetwork& network,
    train::TrainManager& trainManager,
    communication::CommunicationChannel& communicationChannel,
    std::vector<TrainId> trainIds,
    OrchestratorConfig config,
    SafetyStep safetyStep)
    : network_(network),
      trainManager_(trainManager),
      communicationChannel_(communicationChannel),
      trainIds_(std::move(trainIds)),
      config_(config),
    telemetryLogger_(config_.telemetryDirectory),
      safetyStep_(std::move(safetyStep))
{
    updateWorldSnapshotLocked();
}

ThreadOrchestrator::~ThreadOrchestrator()
{
    stop();
}

void ThreadOrchestrator::start()
{
    if (running_.exchange(true))
    {
        return;
    }
    paused_.store(false);

    {
        std::unique_lock lock(worldMutex_);
        worldState_.systemStatus = SystemStatus::Running;
    }

    physicsThread_ = std::thread(&ThreadOrchestrator::physicsLoop, this);
    safetyThread_ = std::thread(&ThreadOrchestrator::safetyLoop, this);
    communicationThread_ = std::thread(&ThreadOrchestrator::communicationLoop, this);
    hmiThread_ = std::thread(&ThreadOrchestrator::hmiLoop, this);
}

void ThreadOrchestrator::stop()
{
    if (!running_.exchange(false))
    {
        return;
    }
    paused_.store(false);

    {
        std::unique_lock lock(worldMutex_);
        worldState_.systemStatus = SystemStatus::Shutdown;
    }

    shutdownCondition_.notify_all();

    if (physicsThread_.joinable())
    {
        physicsThread_.join();
    }
    if (safetyThread_.joinable())
    {
        safetyThread_.join();
    }
    if (communicationThread_.joinable())
    {
        communicationThread_.join();
    }
    if (hmiThread_.joinable())
    {
        hmiThread_.join();
    }
}

void ThreadOrchestrator::pause()
{
    paused_.store(true);
    std::unique_lock lock(worldMutex_);
    worldState_.systemStatus = SystemStatus::Paused;
}

void ThreadOrchestrator::resume()
{
    paused_.store(false);
    std::unique_lock lock(worldMutex_);
    worldState_.systemStatus = SystemStatus::Running;
}

bool ThreadOrchestrator::isRunning() const noexcept
{
    return running_.load();
}

bool ThreadOrchestrator::isPaused() const noexcept
{
    return paused_.load();
}

WorldState ThreadOrchestrator::snapshot() const
{
    std::shared_lock lock(worldMutex_);
    return worldState_;
}

std::size_t ThreadOrchestrator::physicsCycles() const noexcept
{
    return physicsCycles_.load();
}

std::size_t ThreadOrchestrator::safetyCycles() const noexcept
{
    return safetyCycles_.load();
}

std::size_t ThreadOrchestrator::communicationCycles() const noexcept
{
    return communicationCycles_.load();
}

std::size_t ThreadOrchestrator::hmiCycles() const noexcept
{
    return hmiCycles_.load();
}

void ThreadOrchestrator::setSafetyStep(SafetyStep safetyStep)
{
    std::lock_guard lock(safetyStepMutex_);
    safetyStep_ = std::move(safetyStep);
}

void ThreadOrchestrator::postCommand(UserCommand command)
{
    std::lock_guard lock(userCommandMutex_);
    userCommandQueue_.push(std::move(command));
}

void ThreadOrchestrator::setSensorFault(TrainId trainId, bool fault)
{
    std::unique_lock lock(worldMutex_);
    if (fault)
    {
        failedSensors_.insert(trainId);
    }
    else
    {
        failedSensors_.erase(trainId);
    }
    updateWorldSnapshotLocked();
}

void ThreadOrchestrator::setSensorFault(bool fault)
{
    std::unique_lock lock(worldMutex_);
    if (fault)
    {
        for (const auto tid : trainIds_)
        {
            failedSensors_.insert(tid);
        }
    }
    else
    {
        failedSensors_.clear();
    }
    updateWorldSnapshotLocked();
}

void ThreadOrchestrator::setCommFault(bool fault)
{
    userCommFault_.store(fault);
    std::unique_lock lock(worldMutex_);
    worldState_.communicationFailure = userCommFault_.load() || commChannelDegraded_.load();
}

void ThreadOrchestrator::addTrain(TrainId trainId)
{
    std::unique_lock lock(worldMutex_);
    if (std::find(trainIds_.begin(), trainIds_.end(), trainId) == trainIds_.end())
    {
        trainIds_.push_back(trainId);
        updateWorldSnapshotLocked();
    }
}

void ThreadOrchestrator::removeTrain(TrainId trainId)
{
    std::unique_lock lock(worldMutex_);
    const auto it = std::find(trainIds_.begin(), trainIds_.end(), trainId);
    if (it != trainIds_.end())
    {
        trainIds_.erase(it);
        navStates_.erase(trainId);
        failedSensors_.erase(trainId);
        updateWorldSnapshotLocked();
    }
}

void ThreadOrchestrator::setTrainRoute(
    TrainId trainId,
    TrackId startTrackId,
    navigation::RouteResult route)
{
    std::unique_lock lock(worldMutex_);
    TrainNavigationState nav;
    nav.trainId = trainId;
    nav.currentTrackId = startTrackId;
    nav.routeTrackIndex = 0;
    nav.route = std::move(route);

    for (std::size_t i = 0; i < nav.route.tracks.size(); ++i)
    {
        if (nav.route.tracks[i] == startTrackId)
        {
            nav.routeTrackIndex = i;
            break;
        }
    }
    navStates_[trainId] = std::move(nav);
    updateWorldSnapshotLocked();
}

void ThreadOrchestrator::setOperatorMessage(std::string message)
{
    std::unique_lock lock(worldMutex_);
    worldState_.operatorMessage = std::move(message);
}

void ThreadOrchestrator::waitUntil(
    const std::chrono::steady_clock::time_point next)
{
    std::unique_lock lock(shutdownMutex_);
    shutdownCondition_.wait_until(
        lock,
        next,
        [this]
        {
            return !running_.load();
        });
}

void ThreadOrchestrator::updateWorldSnapshotLocked()
{
    worldState_.trains.clear();
    worldState_.trains.reserve(trainIds_.size());

    for (const TrainId trainId : trainIds_)
    {
        const auto* train = trainManager_.getTrain(trainId);
        if (train == nullptr)
        {
            continue;
        }

        TrackId trackId = 0;
        const auto it = navStates_.find(trainId);
        if (it != navStates_.end())
        {
            trackId = it->second.currentTrackId;
        }

        const bool hasSensorFault = failedSensors_.contains(trainId);
        const double uncert = hasSensorFault ? 15.0 : 1.0;

        worldState_.trains.push_back({
            train->id(),
            train->type(),
            trackId,
            train->mass(),
            train->maximumSpeed(),
            train->serviceBraking(),
            train->emergencyBraking(),
            train->state(),
            train->position(),
            train->velocity(),
            train->acceleration(),
            hasSensorFault,
            uncert
        });
    }

    worldState_.sensorFailure = !failedSensors_.empty();
    worldState_.communicationFailure = userCommFault_.load() || commChannelDegraded_.load();
    worldState_.systemStatus = paused_.load()
        ? SystemStatus::Paused
        : ((safetyFailure_.load() || worldState_.sensorFailure ||
            worldState_.communicationFailure)
            ? SystemStatus::Degraded
            : (running_.load() ? SystemStatus::Running : SystemStatus::Shutdown));

    worldState_.timing.physicsCycles = physicsCycles_.load();
    worldState_.timing.safetyCycles = safetyCycles_.load();
    worldState_.timing.commCycles = communicationCycles_.load();
    worldState_.timing.hmiCycles = hmiCycles_.load();
    worldState_.timing.physicsPeriodTargetMs = static_cast<double>(config_.physicsPeriod.count());
    worldState_.timing.safetyPeriodTargetMs = static_cast<double>(config_.safetyPeriod.count());
    worldState_.timing.commPeriodTargetMs = static_cast<double>(config_.communicationPeriod.count());
    worldState_.timing.hmiPeriodTargetMs = static_cast<double>(config_.hmiPeriod.count());
    worldState_.timing.packetsSent = communicationChannel_.totalSent();
    worldState_.timing.packetsDelivered = communicationChannel_.totalDelivered();
    worldState_.timing.packetsDropped = communicationChannel_.totalDropped();
    if (worldState_.timing.packetsSent > 0)
    {
        worldState_.timing.packetDropRatePct = (100.0 * static_cast<double>(worldState_.timing.packetsDropped)) / static_cast<double>(worldState_.timing.packetsSent);
    }
}

void ThreadOrchestrator::processUserCommandsLocked()
{
    std::queue<UserCommand> commands;
    {
        std::lock_guard lock(userCommandMutex_);
        std::swap(commands, userCommandQueue_);
    }

    while (!commands.empty())
    {
        const auto cmd = std::move(commands.front());
        commands.pop();

        switch (cmd.type)
        {
        case UserCommandType::Start:
            paused_.store(false);
            worldState_.systemStatus = SystemStatus::Running;
            worldState_.operatorMessage = "[OK] Simulation RUNNING.";
            break;

        case UserCommandType::Pause:
            paused_.store(true);
            worldState_.systemStatus = SystemStatus::Paused;
            worldState_.operatorMessage = "[OK] Simulation PAUSED.";
            break;

        case UserCommandType::Resume:
            paused_.store(false);
            worldState_.systemStatus = SystemStatus::Running;
            worldState_.operatorMessage = "[OK] Simulation RESUMED.";
            break;

        case UserCommandType::SetSpeed:
            if (auto* train = trainManager_.getTrain(cmd.trainId))
            {
                if (train->state() == TrainState::EmergencyBrake)
                {
                    worldState_.operatorMessage = "[REJECTED] Train #" + std::to_string(cmd.trainId) +
                        " is in EMERGENCY BRAKE. Safety constraint active.";
                    break;
                }
                const double safetyLimit = safetySpeedLimits_.contains(cmd.trainId)
                    ? safetySpeedLimits_[cmd.trainId]
                    : train->maximumSpeed();
                const double targetSpd = std::clamp(cmd.numericValue, 0.0, train->maximumSpeed());
                operatorSpeedLimits_[cmd.trainId] = targetSpd;
                train->setVelocity(std::min(targetSpd, safetyLimit));
                worldState_.operatorMessage = "[OK] Speed for Train #" + std::to_string(cmd.trainId) +
                    " set to " + std::to_string(static_cast<int>(targetSpd)) + " m/s";
            }
            break;

        case UserCommandType::HoldTrain:
            if (auto* train = trainManager_.getTrain(cmd.trainId))
            {
                train->setVelocity(0.0);
                train->setAcceleration(0.0);
                train->setState(TrainState::Stopped);
                worldState_.operatorMessage = "[OK] Train #" + std::to_string(cmd.trainId) + " HELD at signal.";
            }
            break;

        case UserCommandType::ResumeTrain:
            if (auto* train = trainManager_.getTrain(cmd.trainId))
            {
                operatorSpeedLimits_[cmd.trainId] = std::max(0.0, cmd.numericValue);
                train->setState(TrainState::Running);
                worldState_.operatorMessage = "[OK] Train #" + std::to_string(cmd.trainId) + " RESUMED.";
            }
            break;

        case UserCommandType::InjectSensorFailure:
            if (cmd.trainId != 0)
            {
                failedSensors_.insert(cmd.trainId);
            }
            else
            {
                for (auto tid : trainIds_) { failedSensors_.insert(tid); }
            }
            worldState_.sensorFailure = !failedSensors_.empty();
            break;

        case UserCommandType::RecoverSensor:
            if (cmd.trainId != 0)
            {
                failedSensors_.erase(cmd.trainId);
            }
            else
            {
                failedSensors_.clear();
            }
            worldState_.sensorFailure = !failedSensors_.empty();
            break;

        case UserCommandType::InjectCommFailure:
            userCommFault_.store(true);
            worldState_.communicationFailure = true;
            break;

        case UserCommandType::RecoverComm:
            userCommFault_.store(false);
            worldState_.communicationFailure = commChannelDegraded_.load();
            break;

        case UserCommandType::SetCommLossRate:
            communicationChannel_.setPacketLossRate(cmd.numericValue);
            commChannelDegraded_.store(cmd.numericValue >= 0.25);
            worldState_.communicationFailure = userCommFault_.load() || (cmd.numericValue >= 0.60);
            break;

        case UserCommandType::AddTrain:
            if (cmd.payload.has_value())
            {
                try
                {
                    const auto spec = std::any_cast<UserTrainSpec>(cmd.payload);
                    std::unique_ptr<train::Train> train;
                    switch (spec.type)
                    {
                    case TrainType::Express:
                        train = std::make_unique<train::ExpressTrain>(spec.id, 45000.0, 45.0, 0.9, 1.4);
                        break;
                    case TrainType::Passenger:
                        train = std::make_unique<train::PassengerTrain>(spec.id, 60000.0, 33.3, 0.8, 1.2);
                        break;
                    case TrainType::Freight:
                    default:
                        train = std::make_unique<train::FreightTrain>(spec.id, 120000.0, 22.2, 0.5, 0.8);
                        break;
                    }
                    train->setPosition(spec.initialPosition);
                    train->setVelocity(spec.initialVelocity);

                    const TrainId tid = spec.id;
                    if (trainManager_.addTrain(std::move(train)))
                    {
                        if (std::find(trainIds_.begin(), trainIds_.end(), tid) == trainIds_.end())
                        {
                            trainIds_.push_back(tid);
                        }
                        if (!spec.route.tracks.empty())
                        {
                            TrainNavigationState nav;
                            nav.trainId = tid;
                            nav.currentTrackId = spec.startTrackId;
                            nav.routeTrackIndex = 0;
                            nav.route = spec.route;
                            for (std::size_t i = 0; i < nav.route.tracks.size(); ++i)
                            {
                                if (nav.route.tracks[i] == spec.startTrackId)
                                {
                                    nav.routeTrackIndex = i;
                                    break;
                                }
                            }
                            navStates_[tid] = std::move(nav);
                        }
                        worldState_.operatorMessage = "[OK] Train #" + std::to_string(tid) + " ADDED.";
                    }
                }
                catch (...) {}
            }
            else
            {
                if (std::find(trainIds_.begin(), trainIds_.end(), cmd.trainId) == trainIds_.end())
                {
                    trainIds_.push_back(cmd.trainId);
                }
            }
            break;

        case UserCommandType::RemoveTrain:
            trainManager_.removeTrain(cmd.trainId);
            std::erase(trainIds_, cmd.trainId);
            navStates_.erase(cmd.trainId);
            failedSensors_.erase(cmd.trainId);
            operatorSpeedLimits_.erase(cmd.trainId);
            safetySpeedLimits_.erase(cmd.trainId);
            break;

        case UserCommandType::ChangeRoute:
            if (cmd.payload.has_value())
            {
                try
                {
                    const auto spec = std::any_cast<UserRouteSpec>(cmd.payload);
                    TrainNavigationState nav;
                    nav.trainId = spec.trainId;
                    nav.currentTrackId = spec.startTrackId;
                    nav.routeTrackIndex = 0;
                    nav.route = spec.route;
                    for (std::size_t i = 0; i < nav.route.tracks.size(); ++i)
                    {
                        if (nav.route.tracks[i] == spec.startTrackId)
                        {
                            nav.routeTrackIndex = i;
                            break;
                        }
                    }
                    navStates_[spec.trainId] = std::move(nav);
                }
                catch (const std::exception& e)
                {
                    worldState_.operatorMessage = "[ERR] Route change failed: " + std::string(e.what());
                }
                catch (...)
                {
                    worldState_.operatorMessage = "[ERR] Route change failed: Unknown error";
                }
            }
            break;

        default:
            break;
        }
    }
}

void ThreadOrchestrator::physicsLoop()
{
    auto next = std::chrono::steady_clock::now();
    const double dt = static_cast<double>(config_.physicsPeriod.count()) / 1000.0;

    while (running_.load())
    {
        next += config_.physicsPeriod;

        std::unique_lock lock(worldMutex_);

        // 1. Drain user commands synchronously inside physics tick
        processUserCommandsLocked();

        // 2. If paused, do not advance kinematics or simulation time
        if (paused_.load())
        {
            updateWorldSnapshotLocked();
            lock.unlock();
            waitUntil(next);
            continue;
        }

        // 3. Process safety commands from SafetyPipeline
        safety::SafetyCommand command;
        while (commandQueue_.tryPop(&command))
        {
            auto* train = trainManager_.getTrain(command.trainId);
            if (train == nullptr)
            {
                continue;
            }

            switch (command.type)
            {
            case safety::SafetyCommandType::ReduceSpeed:
                safetySpeedLimits_[command.trainId] = std::min(
                    safetySpeedLimits_.contains(command.trainId)
                        ? safetySpeedLimits_[command.trainId]
                        : train->maximumSpeed(),
                    std::max(0.0, command.targetSpeed));
                train->setVelocity(std::min(
                    train->velocity(), safetySpeedLimits_[command.trainId]));
                train->setAcceleration(std::min(train->acceleration(), 0.0));
                train->setState(TrainState::Braking);
                break;
            case safety::SafetyCommandType::HoldAtSignal:
            case safety::SafetyCommandType::EmergencyBrake:
                train->setVelocity(0.0);
                train->setAcceleration(0.0);
                train->setState(command.isEmergency()
                    ? TrainState::EmergencyBrake
                    : TrainState::Braking);
                break;
            case safety::SafetyCommandType::NoAction:
                break;
            }
        }

        // 4. Update kinematics & track transitions
        for (const TrainId trainId : trainIds_)
        {
            auto* train = trainManager_.getTrain(trainId);
            if (train == nullptr)
            {
                continue;
            }

            const auto newPosition = physics::KinematicsEngine::updatePosition(
                train->position(), train->velocity(), train->acceleration(), dt);
            const auto newVelocity = physics::KinematicsEngine::updateVelocity(
                train->velocity(), train->acceleration(), dt, train->maximumSpeed());

            // Track boundary checking
            auto navIt = navStates_.find(trainId);
            if (navIt != navStates_.end() && navIt->second.currentTrackId != 0)
            {
                auto& nav = navIt->second;
                const auto* curTrack = network_.getTrack(nav.currentTrackId);
                if (curTrack != nullptr)
                {
                    if (newPosition >= curTrack->length())
                    {
                        if (nav.routeTrackIndex + 1 < nav.route.tracks.size())
                        {
                            const double excess = newPosition - curTrack->length();
                            ++nav.routeTrackIndex;
                            nav.currentTrackId = nav.route.tracks[nav.routeTrackIndex];
                            train->setPosition(excess);
                        }
                        else
                        {
                            train->setPosition(curTrack->length());
                            train->setVelocity(0.0);
                            train->setAcceleration(0.0);
                            train->setState(TrainState::Stopped);
                        }
                    }
                    else
                    {
                        train->setPosition(newPosition);
                    }
                }
                else
                {
                    train->setPosition(newPosition);
                }
            }
            else
            {
                train->setPosition(newPosition);
            }

            const double operatorLimit = operatorSpeedLimits_.contains(trainId)
                ? operatorSpeedLimits_[trainId]
                : train->maximumSpeed();
            const double safetyLimit = safetySpeedLimits_.contains(trainId)
                ? safetySpeedLimits_[trainId]
                : train->maximumSpeed();
            train->setVelocity(std::min({
                newVelocity,
                operatorLimit,
                safetyLimit,
                train->maximumSpeed()}));
        }

        worldState_.simulationTime += dt;
        updateWorldSnapshotLocked();
        ++physicsCycles_;
        lock.unlock();
        waitUntil(next);
    }
}

void ThreadOrchestrator::safetyLoop()
{
    auto next = std::chrono::steady_clock::now();

    while (running_.load())
    {
        next += config_.safetyPeriod;

        if (paused_.load())
        {
            waitUntil(next);
            continue;
        }

        const WorldState state = snapshot();
        SafetyStep step;
        {
            std::lock_guard lock(safetyStepMutex_);
            step = safetyStep_;
        }
        if (step)
        {
            try
            {
                const SafetyCycleResult result = step(state);
                {
                    std::unique_lock lock(worldMutex_);
                    worldState_.safetyError.clear();
                    worldState_.predictions = result.predictions;
                    worldState_.activeConflicts = result.activeConflicts;
                    worldState_.reservations = result.reservations;
                    worldState_.commands = result.commands;
                    worldState_.decisions = result.decisions;
                }
                for (const auto& command : result.commands)
                {
                    commandQueue_.push(command);
                }
            }
            catch (const std::exception& e)
            {
                safetyFailure_.store(true);
                std::unique_lock lock(worldMutex_);
                worldState_.safetyError = e.what();
                worldState_.systemStatus = SystemStatus::Degraded;
            }
            catch (...)
            {
                safetyFailure_.store(true);
                std::unique_lock lock(worldMutex_);
                worldState_.safetyError = "Unknown safety execution exception";
                worldState_.systemStatus = SystemStatus::Degraded;
            }
        }
        ++safetyCycles_;
        waitUntil(next);
    }
}

void ThreadOrchestrator::communicationLoop()
{
    auto next = std::chrono::steady_clock::now();
    while (running_.load())
    {
        next += config_.communicationPeriod;

        if (paused_.load())
        {
            waitUntil(next);
            continue;
        }

        const auto state = snapshot();
        const auto tick = static_cast<SimTimeTick>(
            std::max(0.0, state.simulationTime) * 1000.0);
        const auto sentBefore = communicationChannel_.totalSent();
        const auto deliveredBefore = communicationChannel_.totalDelivered();
        const auto droppedBefore = communicationChannel_.totalDropped();
        communicationChannel_.step(tick);
        {
            std::unique_lock lock(worldMutex_);
            const auto sent = communicationChannel_.totalSent() - sentBefore;
            [[maybe_unused]] const auto delivered = communicationChannel_.totalDelivered() - deliveredBefore;
            const auto dropped = communicationChannel_.totalDropped() - droppedBefore;
            const double dropRate = (sent > 0U) ? (static_cast<double>(dropped) / static_cast<double>(sent)) : 0.0;
            const bool degraded = (sent > 0U && (dropRate >= 0.25 || communicationChannel_.packetLossRate() >= 0.25));
            commChannelDegraded_.store(degraded);
            worldState_.communicationFailure = userCommFault_.load() || (dropRate >= 0.60 || communicationChannel_.packetLossRate() >= 0.60);
        }
        ++communicationCycles_;
        waitUntil(next);
    }
}

void ThreadOrchestrator::hmiLoop()
{
    auto next = std::chrono::steady_clock::now();

    while (running_.load())
    {
        next += config_.hmiPeriod;
        const auto started = std::chrono::steady_clock::now();
        const WorldState state = snapshot();
        telemetryLogger_.logSnapshot(state);
        telemetryLogger_.logConflicts(state);
        performanceMetrics_.observe(
            state,
            std::chrono::steady_clock::now() - started);
        if (config_.printHmi)
        {
            hmi::HmiDisplay::render(state, std::cout);
        }
        ++hmiCycles_;
        waitUntil(next);
    }
}

hmi::PerformanceSnapshot ThreadOrchestrator::performanceMetricsSnapshot() const
{
    std::shared_lock lock(worldMutex_);
    return performanceMetrics_.snapshot();
}

} // namespace tcas::orchestrator
